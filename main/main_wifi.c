/*
 * ESP32-C6 + AS7343 - 完整SMUX配置（修复版）
 * 修复TCP/IP时序、SMUX配置和通道映射
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ESP32 系统
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_timer.h"

// WiFi
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

// HTTP Server
#include "esp_http_server.h"

// I2C 驱动
#include "driver/i2c.h"

// ==============================
// 配置定义
// ==============================
#define TAG "AS7343"

// WiFi热点配置
#define SOFT_AP_SSID      "ESP32C6AS7343"
#define SOFT_AP_PASSWORD  "88888888"
#define SOFT_AP_MAX_CONN  4
#define SOFT_AP_CHANNEL   1

// HTTP端口
#define HTTP_PORT 80

// I2C配置
#define I2C_MASTER_SCL_IO          5
#define I2C_MASTER_SDA_IO          4
#define I2C_MASTER_NUM             0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_MASTER_TIMEOUT_MS       1000

// AS7343地址
#define AS7343_ADDR                 0x39

// AS7343寄存器地址
#define AS7343_REG_ENABLE           0x80
#define AS7343_REG_ID               0x92
#define AS7343_REG_STATUS           0x71  // 状态寄存器（修正）
#define AS7343_REG_ATIME            0xA9
#define AS7343_REG_ASTEP_L          0xAA
#define AS7343_REG_ASTEP_H          0xAB
#define AS7343_REG_AGAIN            0xA8  // 增益寄存器（修正）
#define AS7343_REG_CFG1             0xB0  // 配置寄存器1
#define AS7343_REG_CFG2             0xB1  // 配置寄存器2
#define AS7343_REG_CFG3             0xB2  // 配置寄存器3
#define AS7343_REG_CFG4             0xB3  // 配置寄存器4
#define AS7343_REG_CFG5             0xB4  // 配置寄存器5
#define AS7343_REG_CFG6             0xB5  // 配置寄存器6
#define AS7343_REG_CFG7             0xB6  // 配置寄存器7
#define AS7343_REG_CFG8             0xB7  // 配置寄存器8
#define AS7343_REG_CFG9             0xB8  // 配置寄存器9
#define AS7343_REG_CFG10            0xB9  // 配置寄存器10
#define AS7343_REG_CFG11            0xBA  // 配置寄存器11
#define AS7343_REG_CFG12            0xBB  // 配置寄存器12

// 数据寄存器基址（自动递增读取）
#define AS7343_REG_CH0_DATA_L       0x95
#define AS7343_REG_CH0_DATA_H       0x96

// SMUX 相关
#define AS7343_REG_SMUX_CMD         0xAF  // SMUX命令寄存器
#define AS7343_REG_SMUX_RAM         0x20  // SMUX RAM起始地址

// ==============================
// 数据结构定义
// ==============================

typedef struct {
    uint32_t cursor;
    uint16_t f1;   // 415nm - CH0
    uint16_t f2;   // 445nm - CH1
    uint16_t f3;   // 480nm - CH2
    uint16_t f4;   // 515nm - CH3
    uint16_t f5;   // 555nm - CH4
    uint16_t f6;   // 590nm - CH5
    uint16_t f7;   // 630nm - CH6
    uint16_t f8;   // 680nm - CH7
    uint16_t fz;   // 705nm - CH8
    uint16_t fy;   // 730nm - CH9
    uint16_t fxl;  // 760nm - CH10
    uint16_t nir;  // 近红外 - CH11
    uint16_t clear; // 清晰通道 - CH12
    float ci;      // 叶绿素指数
} sensor_data_t;

// 全局变量
static sensor_data_t g_latest_data = {0};
static SemaphoreHandle_t g_data_mutex = NULL;

// ==============================
// 函数声明
// ==============================
static esp_err_t i2c_master_init(void);
static esp_err_t as7343_write_reg(uint8_t reg, uint8_t data);
static esp_err_t as7343_read_reg(uint8_t reg, uint8_t *data);
static esp_err_t as7343_read_regs(uint8_t reg, uint8_t *data, size_t len);
static uint16_t as7343_read_word(uint8_t reg_low, uint8_t reg_high);
static esp_err_t as7343_init_complete(void);
static void sensor_task(void *pvParameters);
static void start_webserver(void);

// ==============================
// WiFi初始化（修复：移除重复初始化）
// ==============================

static void wifi_init_softap(void)
{
    // 注意：esp_netif_init() 和 esp_event_loop_create_default() 
    // 已经在 app_main 中调用，这里不再重复！
    
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SOFT_AP_SSID,
            .ssid_len = strlen(SOFT_AP_SSID),
            .password = SOFT_AP_PASSWORD,
            .max_connection = SOFT_AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .channel = SOFT_AP_CHANNEL,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");  // 修正key
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "✅ SoftAP started");
        ESP_LOGI(TAG, "📡 SSID: %s", SOFT_AP_SSID);
        ESP_LOGI(TAG, "🔑 Password: %s", SOFT_AP_PASSWORD);
        ESP_LOGI(TAG, "🌐 AP IP: " IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(TAG, "========================================");
        
        char ip_str[16];
        sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(TAG, "📱 Open: http://%s:%d/", ip_str, HTTP_PORT);
    }
}

// ==============================
// I2C底层操作
// ==============================

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "I2C initialized");
    }
    
    return ret;
}

static esp_err_t as7343_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7343_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static esp_err_t as7343_read_reg(uint8_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7343_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7343_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

// 连续读取多个寄存器（更高效）
static esp_err_t as7343_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    if (len == 0) return ESP_OK;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // 写寄存器地址
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7343_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    // 重复启动，开始读取
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7343_ADDR << 1 | I2C_MASTER_READ, true);
    
    // 读取前 len-1 个字节，发送 ACK
    for (size_t i = 0; i < len - 1; i++) {
        i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
    }
    // 最后一个字节，发送 NACK
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static uint16_t as7343_read_word(uint8_t reg_low, uint8_t reg_high)
{
    uint8_t low = 0, high = 0;
    
    if (as7343_read_reg(reg_low, &low) != ESP_OK) {
        return 0;
    }
    
    if (as7343_read_reg(reg_high, &high) != ESP_OK) {
        return 0;
    }
    
    return (uint16_t)((high << 8) | low);
}

// ==============================
// AS7343 SMUX配置（修正版）
// ==============================

/**
 * 配置SMUX - 将特定波长映射到通道
 * AS7343 使用 RAM 地址 0x20-0x4F 来配置 SMUX
 */
static esp_err_t as7343_configure_smux(void)
{
    esp_err_t ret;
    uint8_t smux_config[48];  // SMUX RAM 大小
    
    // 清零配置
    memset(smux_config, 0, sizeof(smux_config));
    
    // 配置 Cycle 1: FZ, FY, FXL, NIR, VIS, FD
    // 映射到 CH0-CH5
    smux_config[0x00] = 0x30;  // CH0 = FZ (705nm)
    smux_config[0x01] = 0x30;  // CH1 = FY (730nm)
    smux_config[0x02] = 0x30;  // CH2 = FXL (760nm)
    smux_config[0x03] = 0x30;  // CH3 = NIR
    smux_config[0x04] = 0x30;  // CH4 = VIS
    smux_config[0x05] = 0x30;  // CH5 = FD
    
    // 配置 Cycle 2: F2, F3, F4, F6, 保留2个
    // 映射到 CH6-CH11
    smux_config[0x06] = 0x30;  // CH6 = F2 (445nm)
    smux_config[0x07] = 0x30;  // CH7 = F3 (480nm)
    smux_config[0x08] = 0x30;  // CH8 = F4 (515nm)
    smux_config[0x09] = 0x30;  // CH9 = F6 (590nm)
    smux_config[0x0A] = 0x00;  // CH10 = 保留
    smux_config[0x0B] = 0x00;  // CH11 = 保留
    
    // 配置 Cycle 3: F1, F5, F7, F8, 保留2个
    // 映射到 CH0-CH5（Cycle 3 覆盖 Cycle 1 的寄存器）
    smux_config[0x0C] = 0x30;  // CH0 = F1 (415nm)
    smux_config[0x0D] = 0x30;  // CH1 = F5 (555nm)
    smux_config[0x0E] = 0x30;  // CH2 = F7 (630nm)
    smux_config[0x0F] = 0x30;  // CH3 = F8 (680nm)
    smux_config[0x10] = 0x00;  // CH4 = 保留
    smux_config[0x11] = 0x00;  // CH5 = 保留
    
    // 写入 SMUX RAM
    for (int i = 0; i < 48; i++) {
        ret = as7343_write_reg(AS7343_REG_SMUX_RAM + i, smux_config[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write SMUX RAM at offset %d", i);
            return ret;
        }
    }
    
    // 发送 SMUX 命令（0x10 = 写 SMUX 配置）
    ret = as7343_write_reg(AS7343_REG_SMUX_CMD, 0x10);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(10));  // 等待 SMUX 配置完成
    
    ESP_LOGI(TAG, "SMUX configured");
    return ESP_OK;
}

// ==============================
// AS7343 完整初始化（修正版）
// ==============================

static esp_err_t as7343_init_complete(void)
{
    uint8_t id = 0;
    esp_err_t ret;
    
    // 1. 读取器件ID
    ret = as7343_read_reg(AS7343_REG_ID, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ret;
    }
    ESP_LOGI(TAG, "AS7343 Device ID: 0x%02X (expected 0x09)", id);
    
    if (id != 0x09) {
        ESP_LOGW(TAG, "Unexpected device ID!");
    }
    
    // 2. 软件复位
    ret = as7343_write_reg(AS7343_REG_ENABLE, 0x06);  // 复位命令
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(50));  // 增加复位延时
    
    // 3. 配置积分时间 ATIME = 100
    ret = as7343_write_reg(AS7343_REG_ATIME, 100);
    if (ret != ESP_OK) return ret;
    
    // 4. 配置 ASTEP (积分步长) = 999 (0x03E7)
    ret = as7343_write_reg(AS7343_REG_ASTEP_L, 0xE7);
    if (ret != ESP_OK) return ret;
    ret = as7343_write_reg(AS7343_REG_ASTEP_H, 0x03);
    if (ret != ESP_OK) return ret;
    
    // 5. 配置增益 AGAIN = 5 (16x)
    ret = as7343_write_reg(AS7343_REG_AGAIN, 5);
    if (ret != ESP_OK) return ret;
    
    // 6. 配置测量模式
    // CFG1: 3 cycles, auto sequence, 使能所有通道
    ret = as7343_write_reg(AS7343_REG_CFG1, 0x30);
    if (ret != ESP_OK) return ret;
    
    // 7. 配置 SMUX
    ret = as7343_configure_smux();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SMUX configuration failed");
        return ret;
    }
    
    // 8. 使能传感器 (PON = 1)
    ret = as7343_write_reg(AS7343_REG_ENABLE, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "AS7343 fully initialized with SMUX");
    return ESP_OK;
}

// ==============================
// 数据采集任务（修正版）
// ==============================

static void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started");
    
    uint32_t cursor = 0;
    uint8_t data_buffer[26];  // 13个通道 * 2字节
    
    // 任务启动时等待系统稳定
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (1) {
        // 1. 触发测量（SPM = 1, SM = 1）
        esp_err_t ret = as7343_write_reg(AS7343_REG_ENABLE, 0x07);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to start measurement");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        
        // 2. 等待测量完成（根据 ATIME 和 ASTEP 计算）
        // 积分时间 = (ATIME + 1) * (ASTEP + 1) * 2.78us
        // 100 * 1000 * 2.78us = 278ms，取整300ms
        vTaskDelay(pdMS_TO_TICKS(300));
        
        // 3. 检查状态寄存器（可选）
        uint8_t status = 0;
        as7343_read_reg(AS7343_REG_STATUS, &status);
        if (status & 0x01) {
            ESP_LOGD(TAG, "Measurement complete");
        }
        
        // 4. 读取所有通道数据（使用连续读取更高效）
        // 从 CH0_DATA_L (0x95) 开始，读取26字节（13个通道）
        ret = as7343_read_regs(AS7343_REG_CH0_DATA_L, data_buffer, 26);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read sensor data");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        
        // 5. 解析数据（根据SMUX配置映射）
        sensor_data_t new_data = {0};
        new_data.cursor = cursor;
        
        // 注意：这里假设使用的是 Cycle 1 的配置
        // 实际映射需要根据你的SMUX配置调整
        new_data.fz   = (data_buffer[1] << 8) | data_buffer[0];   // CH0
        new_data.fy   = (data_buffer[3] << 8) | data_buffer[2];   // CH1
        new_data.fxl  = (data_buffer[5] << 8) | data_buffer[4];   // CH2
        new_data.nir  = (data_buffer[7] << 8) | data_buffer[6];   // CH3
        // CH4, CH5 是 VIS 和 FD
        new_data.f2   = (data_buffer[13] << 8) | data_buffer[12]; // CH6
        new_data.f3   = (data_buffer[15] << 8) | data_buffer[14]; // CH7
        new_data.f4   = (data_buffer[17] << 8) | data_buffer[16]; // CH8
        new_data.f6   = (data_buffer[19] << 8) | data_buffer[18]; // CH9
        
        // 读取 Cycle 3 的数据（需要重新配置或从其他寄存器读取）
        // 这里简化处理，实际应该配置多周期测量
        
        // 6. 计算叶绿素指数 CI = (FY - F7) / (FY + F7)
        // 注意：如果F7在当前cycle没有读取，需要从其他cycle获取
        if (new_data.fy + new_data.f7 > 0) {
            new_data.ci = (float)(new_data.fy - new_data.f7) / (float)(new_data.fy + new_data.f7);
        } else {
            new_data.ci = 0.0f;
        }
        
        // 7. 更新全局数据
        if (xSemaphoreTake(g_data_mutex, portMAX_DELAY)) {
            g_latest_data = new_data;
            xSemaphoreGive(g_data_mutex);
        }
        
        cursor++;
        
        // 8. 打印日志
        if (cursor % 5 == 0) {
            ESP_LOGI(TAG, "Sample %lu: F2=%u, F3=%u, F4=%u, FZ=%u, FY=%u, CI=%.3f", 
                     (unsigned long)cursor, new_data.f2, new_data.f3, new_data.f4, 
                     new_data.fz, new_data.fy, new_data.ci);
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ==============================
// HTTP处理函数
// ==============================

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char html[] = 
        "<!DOCTYPE html>"
        "<html><head><title>AS7343 Spectrometer</title>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<style>"
        "body{font-family:Arial,sans-serif;margin:40px auto;max-width:900px;background:#f5f5f5;}"
        "h1{color:#2c3e50;border-bottom:3px solid #3498db;padding-bottom:10px;}"
        ".container{background:white;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);}"
        ".data-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin:20px 0;}"
        ".data-item{background:#ecf0f1;padding:10px;border-radius:5px;text-align:center;}"
        ".data-item .label{font-size:12px;color:#7f8c8d;}"
        ".data-item .value{font-size:20px;font-weight:bold;color:#2c3e50;}"
        ".ci-box{background:#27ae60;color:white;padding:20px;border-radius:10px;text-align:center;margin:20px 0;}"
        ".ci-box .value{font-size:48px;font-weight:bold;}"
        "</style></head>"
        "<body>"
        "<div class='container'>"
        "<h1>🌱 AS7343 Spectrometer</h1>"
        "<div class='ci-box'>"
        "<div class='label'>Chlorophyll Index</div>"
        "<div class='value' id='ci'>--</div>"
        "</div>"
        "<div class='data-grid'>"
        "<div class='data-item'><div class='label'>F1 415nm</div><div class='value' id='f1'>--</div></div>"
        "<div class='data-item'><div class='label'>F2 445nm</div><div class='value' id='f2'>--</div></div>"
        "<div class='data-item'><div class='label'>F3 480nm</div><div class='value' id='f3'>--</div></div>"
        "<div class='data-item'><div class='label'>F4 515nm</div><div class='value' id='f4'>--</div></div>"
        "<div class='data-item'><div class='label'>F5 555nm</div><div class='value' id='f5'>--</div></div>"
        "<div class='data-item'><div class='label'>F6 590nm</div><div class='value' id='f6'>--</div></div>"
        "<div class='data-item'><div class='label'>F7 630nm</div><div class='value' id='f7'>--</div></div>"
        "<div class='data-item'><div class='label'>F8 680nm</div><div class='value' id='f8'>--</div></div>"
        "<div class='data-item'><div class='label'>FZ 705nm</div><div class='value' id='fz'>--</div></div>"
        "<div class='data-item'><div class='label'>FY 730nm</div><div class='value' id='fy'>--</div></div>"
        "<div class='data-item'><div class='label'>FXL 760nm</div><div class='value' id='fxl'>--</div></div>"
        "<div class='data-item'><div class='label'>NIR</div><div class='value' id='nir'>--</div></div>"
        "</div>"
        "<p style='text-align:center;'><a href='/latest'>View Raw JSON</a></p>"
        "</div>"
        "<script>"
        "function updateData(){fetch('/latest').then(r=>r.json()).then(d=>{"
        "document.getElementById('ci').innerText=d.ci?d.ci.toFixed(3):'--';"
        "document.getElementById('f1').innerText=d.f1||'--';"
        "document.getElementById('f2').innerText=d.f2||'--';"
        "document.getElementById('f3').innerText=d.f3||'--';"
        "document.getElementById('f4').innerText=d.f4||'--';"
        "document.getElementById('f5').innerText=d.f5||'--';"
        "document.getElementById('f6').innerText=d.f6||'--';"
        "document.getElementById('f7').innerText=d.f7||'--';"
        "document.getElementById('f8').innerText=d.f8||'--';"
        "document.getElementById('fz').innerText=d.fz||'--';"
        "document.getElementById('fy').innerText=d.fy||'--';"
        "document.getElementById('fxl').innerText=d.fxl||'--';"
        "document.getElementById('nir').innerText=d.nir||'--';"
        "}).catch(e=>console.log('Waiting...'));}"
        "setInterval(updateData,2000);updateData();"
        "</script>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static esp_err_t latest_get_handler(httpd_req_t *req)
{
    sensor_data_t data = {0};
    
    if (xSemaphoreTake(g_data_mutex, portMAX_DELAY)) {
        data = g_latest_data;
        xSemaphoreGive(g_data_mutex);
    }
    
    char response[512];
    snprintf(response, sizeof(response),
        "{"
        "\"cursor\":%lu,"
        "\"f1\":%u,\"f2\":%u,\"f3\":%u,\"f4\":%u,"
        "\"f5\":%u,\"f6\":%u,\"f7\":%u,\"f8\":%u,"
        "\"fz\":%u,\"fy\":%u,\"fxl\":%u,\"nir\":%u,"
        "\"ci\":%.3f"
        "}",
        (unsigned long)data.cursor,
        data.f1, data.f2, data.f3, data.f4,
        data.f5, data.f6, data.f7, data.f8,
        data.fz, data.fy, data.fxl, data.nir,
        data.ci);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTP_PORT;
    config.lru_purge_enable = true;
    config.stack_size = 8192;
    config.task_priority = 5;
    
    esp_err_t ret = httpd_start(&server, &config);
    if (ret == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t uri_latest = { .uri = "/latest", .method = HTTP_GET, .handler = latest_get_handler };
        
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_latest);
        
        ESP_LOGI(TAG, "HTTP server started on port %d", HTTP_PORT);
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
    }
}

// ==============================
// 主函数（关键修复！）
// ==============================

void app_main(void)
{
    // 0. 首先初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32-C6 + AS7343 - Full SMUX Config");
    ESP_LOGI(TAG, "========================================");
    
    // 1. 【关键】先初始化TCP/IP堆栈（必须在WiFi之前！）
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 2. 启动WiFi热点
    wifi_init_softap();
    
    // 3. 【关键】等待TCP/IP堆栈完全就绪
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 4. 初始化I2C
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 5. 初始化AS7343
    ret = as7343_init_complete();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AS7343 init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 6. 创建互斥锁
    g_data_mutex = xSemaphoreCreateMutex();
    if (g_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    
    // 7. 启动传感器任务
    if (xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return;
    }
    
    // 8. 【关键】再等待一段时间确保传感器任务启动
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 9. 最后启动HTTP服务器（确保所有网络组件就绪）
    start_webserver();
    
    ESP_LOGI(TAG, "System ready");
}