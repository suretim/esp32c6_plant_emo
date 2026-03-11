/*
 * ESP32-C6 + AS7343 - 真正的12通道读取
 * 使用3个cycles分别读取所有通道
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "esp_http_server.h"

#include "driver/i2c.h"

#define TAG "AS7343"

// WiFi配置
#define SOFT_AP_SSID      "ESP32C6AS7343"
#define SOFT_AP_PASSWORD  "88888888"
#define SOFT_AP_MAX_CONN  4
#define SOFT_AP_CHANNEL   1
#define HTTP_PORT 80

// I2C配置
#define I2C_MASTER_SCL_IO          4
#define I2C_MASTER_SDA_IO          5
#define I2C_MASTER_NUM             0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_MASTER_TIMEOUT_MS      1000

// AS7343地址
#define AS7343_ADDR                0x39

// AS7343寄存器
#define AS7343_REG_ENABLE          0x80
#define AS7343_REG_ID              0x92
#define AS7343_REG_STATUS          0x71
#define AS7343_REG_CH0_DATA_L      0x95
#define AS7343_REG_CFG1            0xB0  // 配置1: 周期数
#define AS7343_REG_CFG6            0xB5  // 配置6: SMUX
#define AS7343_REG_CFG9            0xB8  // 配置9: 模式
#define AS7343_REG_AGAIN           0xAA  // 增益
#define AS7343_REG_ATIME           0x81  // 积分时间低
#define AS7343_REG_ASTEP_L         0x83  // 步长低
#define AS7343_REG_ASTEP_H         0x84  // 步长高

// 控制位
#define AS7343_ENABLE_PON          0x01
#define AS7343_ENABLE_SPEN         0x02
#define AS7343_ENABLE_SMUX         0x10
#define AS7343_ENABLE_FDEN         0x40

// 数据结构
typedef struct {
    uint32_t cursor;
    uint16_t f1;   // 415nm - Cycle 3
    uint16_t f2;   // 445nm - Cycle 2
    uint16_t f3;   // 480nm - Cycle 2
    uint16_t f4;   // 515nm - Cycle 2
    uint16_t f5;   // 555nm - Cycle 3
    uint16_t f6;   // 590nm - Cycle 2
    uint16_t f7;   // 630nm - Cycle 3
    uint16_t f8;   // 680nm - Cycle 3
    uint16_t fz;   // 705nm - Cycle 1
    uint16_t fy;   // 730nm - Cycle 1
    uint16_t fxl;  // 760nm - Cycle 1
    uint16_t nir;  // 900nm - Cycle 1
    uint16_t clear; // Clear - Cycle 1
    float ci;      // 叶绿素指数
} sensor_data_t;

static sensor_data_t g_latest_data = {0};
static SemaphoreHandle_t g_data_mutex = NULL;

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
    
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
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

// 连续读取多个字节
static esp_err_t as7343_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    if (len == 0) return ESP_OK;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7343_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7343_ADDR << 1 | I2C_MASTER_READ, true);
    
    for (size_t i = 0; i < len - 1; i++) {
        i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ==============================
// AS7343 初始化（正确的3-cycle配置）
// ==============================

static esp_err_t as7343_init(void)
{
    // 启用自动序列模式
as7341_write_reg(AS7341_REG_CFG9, 0x10);  // AUTO_SMUX

// 然后只需启动一次，AS7341 会自动完成 2-3 个周期
as7341_write_reg(AS7341_REG_ENABLE, AS7341_ENABLE_PON | AS7341_ENABLE_SPEN | AS7341_ENABLE_SMUXEN);
    return ESP_OK;
}

/**
 * @brief 读取所有11通道（正确的双周期流程）
 */
static esp_err_t as7341_read_all_channels(sensor_data_t *data)
{
    uint8_t buffer[12];  // 6通道×2字节
    esp_err_t ret;
    
    // ---- Cycle 1: F1, F2, F3, F4, CLEAR, NIR ----
    ret = as7341_configure_smux_cycle(true);  // true = F1-F4
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SMUX config Cycle 1 failed");
        return ret;
    }
    
    // 启动光谱测量（SP_EN=1）
    uint8_t enable_val;
    ret = as7341_read_reg(AS7341_REG_ENABLE, &enable_val);
    if (ret != ESP_OK) return ret;
    enable_val |= AS7341_ENABLE_SPEN;
    ret = as7341_write_reg(AS7341_REG_ENABLE, enable_val);
    if (ret != ESP_OK) return ret;
    
    // 等待数据就绪（AVALID=1）
    ret = as7341_wait_for_data(500);
    if (ret != ESP_OK) return ret;
    
    // 读取6通道数据
    ret = as7341_read_regs(AS7341_REG_CH0_DATA_L, buffer, 12);
    if (ret != ESP_OK) return ret;
    
    // 解析Cycle 1
    data->f1    = buffer[0]  | (buffer[1]  << 8);   // ADC0
    data->f2    = buffer[2]  | (buffer[3]  << 8);   // ADC1
    data->f3    = buffer[4]  | (buffer[5]  << 8);   // ADC2
    data->f4    = buffer[6]  | (buffer[7]  << 8);   // ADC3
    data->clear = buffer[8]  | (buffer[9]  << 8);   // ADC4
    data->nir   = buffer[10] | (buffer[11] << 8);   // ADC5
    
    // 关闭SP_EN（为下一次SMUX配置做准备）
    enable_val &= ~AS7341_ENABLE_SPEN;
    ret = as7341_write_reg(AS7341_REG_ENABLE, enable_val);
    if (ret != ESP_OK) return ret;
    
    // ---- Cycle 2: F5, F6, F7, F8, CLEAR, NIR ----
    ret = as7341_configure_smux_cycle(false);  // false = F5-F8
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SMUX config Cycle 2 failed");
        return ret;
    }
    
    // 再次启动光谱测量
    ret = as7341_read_reg(AS7341_REG_ENABLE, &enable_val);
    if (ret != ESP_OK) return ret;
    enable_val |= AS7341_ENABLE_SPEN;
    ret = as7341_write_reg(AS7341_REG_ENABLE, enable_val);
    if (ret != ESP_OK) return ret;
    
    // 等待数据就绪
    ret = as7341_wait_for_data(500);
    if (ret != ESP_OK) return ret;
    
    // 读取Cycle 2数据
    ret = as7341_read_regs(AS7341_REG_CH0_DATA_L, buffer, 12);
    if (ret != ESP_OK) return ret;
    
    // 解析Cycle 2
    data->f5    = buffer[0]  | (buffer[1]  << 8);   // ADC0
    data->f6    = buffer[2]  | (buffer[3]  << 8);   // ADC1
    data->f7    = buffer[4]  | (buffer[5]  << 8);   // ADC2
    data->f8    = buffer[6]  | (buffer[7]  << 8);   // ADC3
    // ADC4=CLEAR, ADC5=NIR（与Cycle 1相同，可验证一致性）
    uint16_t clear2 = buffer[8]  | (buffer[9]  << 8);
    uint16_t nir2   = buffer[10] | (buffer[11] << 8);
    
    // 验证CLEAR和NIR的一致性（可选）
    if (abs((int)data->clear - (int)clear2) > 1000) {
        ESP_LOGW(TAG, "Clear mismatch: %u vs %u", data->clear, clear2);
    }
    
    // 关闭测量
    enable_val &= ~AS7341_ENABLE_SPEN;
    as7341_write_reg(AS7341_REG_ENABLE, enable_val);
    
    return ESP_OK;
}

// ==============================
// 【关键】读取指定cycle的数据
// ==============================

static esp_err_t as7343_read_cycle(uint8_t cycle_num, uint16_t *ch0, uint16_t *ch1, uint16_t *ch2, uint16_t *ch3, uint16_t *ch4, uint16_t *ch5)
{
    uint8_t buffer[12];
    esp_err_t ret;
    
    // 如果cycle_num > 0，需要手动切换cycle（某些版本需要）
    // 这里假设auto-sequence已经配置好了
    
    // 启动测量
    ret = as7343_write_reg(AS7343_REG_ENABLE, 0x03);  // PON + SPEN
    if (ret != ESP_OK) return ret;
    
    // 等待测量完成（每个cycle约140ms，3个cycles约420ms）
    vTaskDelay(pdMS_TO_TICKS(150));  // 等待当前cycle完成
    
    // 读取6个通道
    ret = as7343_read_regs(AS7343_REG_CH0_DATA_L, buffer, 12);
    if (ret != ESP_OK) return ret;
    
    *ch0 = buffer[0] | (buffer[1] << 8);
    *ch1 = buffer[2] | (buffer[3] << 8);
    *ch2 = buffer[4] | (buffer[5] << 8);
    *ch3 = buffer[6] | (buffer[7] << 8);
    *ch4 = buffer[8] | (buffer[9] << 8);
    *ch5 = buffer[10] | (buffer[11] << 8);
    
    return ESP_OK;
}

// ==============================
// 【关键】3-cycle完整读取
// ==============================

static void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started (3-cycle mode)");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    uint32_t cursor = 0;
    
    while (1) {
        uint16_t c1[6], c2[6], c3[6];  // 3个cycles，每个6通道
        
        // 【方案A】连续启动3次测量，分别读取（简单但慢）
        // Cycle 1: FZ, FY, FXL, NIR, Clear, FD
        esp_err_t ret = as7343_read_cycle(0, &c1[0], &c1[1], &c1[2], &c1[3], &c1[4], &c1[5]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Cycle 1 read failed");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        
        // 注意：如果CFG1配置了auto-sequence，下一次启动会自动进入Cycle 2
        // 否则需要手动配置
        
        // 再次启动，读取Cycle 2: F2, F3, F4, F6, reserved, reserved
        ret = as7343_read_cycle(1, &c2[0], &c2[1], &c2[2], &c2[3], &c2[4], &c2[5]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Cycle 2 read failed");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        
        // 再次启动，读取Cycle 3: F1, F5, F7, F8, reserved, reserved
        ret = as7343_read_cycle(2, &c3[0], &c3[1], &c3[2], &c3[3], &c3[4], &c3[5]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Cycle 3 read failed");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        
        // 合并数据（根据AS7343 datasheet的SMUX映射）
        sensor_data_t new_data = {
            .cursor = cursor++,
            // Cycle 1
            .fz  = c1[0],   // FZ (705nm)
            .fy  = c1[1],   // FY (730nm)
            .fxl = c1[2],   // FXL (760nm)
            .nir = c1[3],   // NIR
            .clear = c1[4], // Clear
            // Cycle 2
            .f2  = c2[0],   // F2 (445nm)
            .f3  = c2[1],   // F3 (480nm)
            .f4  = c2[2],   // F4 (515nm)
            .f6  = c2[3],   // F6 (590nm)
            // Cycle 3
            .f1  = c3[0],   // F1 (415nm)
            .f5  = c3[1],   // F5 (555nm)
            .f7  = c3[2],   // F7 (630nm)
            .f8  = c3[3],   // F8 (680nm)
        };
        
        // 计算CI（使用F7和FY）
        if (new_data.f7 + new_data.fy > 0) {
            new_data.ci = (float)((int)new_data.fy - (int)new_data.f7) / 
                         (float)(new_data.f7 + new_data.fy);
        } else {
            new_data.ci = 0.0f;
        }
        
        // 更新全局数据
        if (xSemaphoreTake(g_data_mutex, portMAX_DELAY)) {
            g_latest_data = new_data;
            xSemaphoreGive(g_data_mutex);
        }
        
        // 打印关键通道
        if (cursor % 3 == 0) {
            ESP_LOGI(TAG, "C1[FZ=%4u,FY=%4u] C2[F2=%4u,F3=%4u,F4=%4u] C3[F7=%4u,F8=%4u] CI=%.3f",
                     c1[0], c1[1], c2[0], c2[1], c2[2], c3[2], c3[3], new_data.ci);
        }
        
        // 总周期约450ms * 3 = 1.35s，加上2s间隔
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ==============================
// HTTP服务器
// ==============================

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
        "\"f1_415nm\":%u,\"f2_445nm\":%u,\"f3_480nm\":%u,\"f4_515nm\":%u,"
        "\"f5_555nm\":%u,\"f6_590nm\":%u,\"f7_630nm\":%u,\"f8_680nm\":%u,"
        "\"fz_705nm\":%u,\"fy_730nm\":%u,\"fxl_760nm\":%u,\"nir\":%u,"
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

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char html[] = 
        "<!DOCTYPE html>"
        "<html><head><title>AS7343 12-Channel</title>"
        "<meta charset='UTF-8'>"
        "<meta http-equiv='refresh' content='3'>"
        "<style>"
        "body{font-family:Arial;margin:20px;background:#f0f0f0;}"
        ".container{max-width:900px;margin:0 auto;background:white;padding:20px;border-radius:10px;}"
        "h1{color:#2c3e50;border-bottom:3px solid #3498db;}"
        ".cycle{margin:15px 0;padding:10px;background:#ecf0f1;border-radius:5px;}"
        ".cycle h3{margin:0 0 10px 0;color:#7f8c8d;}"
        ".channels{display:grid;grid-template-columns:repeat(6,1fr);gap:5px;}"
        ".ch{background:white;padding:8px;text-align:center;border-radius:3px;}"
        ".ch .nm{font-size:10px;color:#7f8c8d;}"
        ".ch .val{font-size:16px;font-weight:bold;color:#2c3e50;}"
        ".ci-box{background:#27ae60;color:white;padding:15px;border-radius:10px;text-align:center;}"
        ".ci-box .val{font-size:36px;font-weight:bold;}"
        "</style></head>"
        "<body>"
        "<div class='container'>"
        "<h1>🌱 AS7343 12-Channel Spectrometer</h1>"
        "<div class='ci-box'>Chlorophyll Index: <span class='val' id='ci'>--</span></div>"
        "<div class='cycle'>"
        "<h3>Cycle 1 (NIR/Red Edge)</h3>"
        "<div class='channels'>"
        "<div class='ch'><div class='nm'>FZ 705</div><div class='val' id='fz'>--</div></div>"
        "<div class='ch'><div class='nm'>FY 730</div><div class='val' id='fy'>--</div></div>"
        "<div class='ch'><div class='nm'>FXL 760</div><div class='val' id='fxl'>--</div></div>"
        "<div class='ch'><div class='nm'>NIR</div><div class='val' id='nir'>--</div></div>"
        "</div></div>"
        "<div class='cycle'>"
        "<h3>Cycle 2 (Blue/Green)</h3>"
        "<div class='channels'>"
        "<div class='ch'><div class='nm'>F2 445</div><div class='val' id='f2'>--</div></div>"
        "<div class='ch'><div class='nm'>F3 480</div><div class='val' id='f3'>--</div></div>"
        "<div class='ch'><div class='nm'>F4 515</div><div class='val' id='f4'>--</div></div>"
        "<div class='ch'><div class='nm'>F6 590</div><div class='val' id='f6'>--</div></div>"
        "</div></div>"
        "<div class='cycle'>"
        "<h3>Cycle 3 (VIS)</h3>"
        "<div class='channels'>"
        "<div class='ch'><div class='nm'>F1 415</div><div class='val' id='f1'>--</div></div>"
        "<div class='ch'><div class='nm'>F5 555</div><div class='val' id='f5'>--</div></div>"
        "<div class='ch'><div class='nm'>F7 630</div><div class='val' id='f7'>--</div></div>"
        "<div class='ch'><div class='nm'>F8 680</div><div class='val' id='f8'>--</div></div>"
        "</div></div>"
        "<p style='text-align:center;'><a href='/latest'>View JSON</a></p>"
        "</div>"
        "<script>"
        "function updateData(){fetch('/latest').then(r=>r.json()).then(d=>{"
        "document.getElementById('ci').innerText=d.ci?d.ci.toFixed(3):'--';"
        "['f1','f2','f3','f4','f5','f6','f7','f8','fz','fy','fxl','nir'].forEach(k=>{"
        "let el=document.getElementById(k);if(el)el.innerText=d[k+'_415nm']||d[k+'_445nm']||d[k+'_480nm']||d[k+'_515nm']||d[k+'_555nm']||d[k+'_590nm']||d[k+'_630nm']||d[k+'_680nm']||d[k+'_705nm']||d[k+'_730nm']||d[k+'_760nm']||d[k]||'--';});"
        "}).catch(e=>console.log('Waiting...'));}"
        "setInterval(updateData,3000);updateData();"
        "</script>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, strlen(html));
    return ESP_OK;
}

static void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = HTTP_PORT;
    config.stack_size = 8192;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t uri_latest = { .uri = "/latest", .method = HTTP_GET, .handler = latest_get_handler };
        
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_latest);
        
        ESP_LOGI(TAG, "HTTP server started on port %d", HTTP_PORT);
    }
}

static void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
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
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "📡 WiFi: %s", SOFT_AP_SSID);
        ESP_LOGI(TAG, "🌐 IP: " IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI(TAG, "========================================");
    }
}

// ==============================
// 主函数
// ==============================

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP32-C6 + AS7343 - True 12-Channel Mode");
    
    wifi_init_softap();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }
    
    ret = as7343_init_complete();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AS7343 init failed");
        return;
    }
    
    g_data_mutex = xSemaphoreCreateMutex();
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    
    vTaskDelay(pdMS_TO_TICKS(500));
    start_webserver();
    
    ESP_LOGI(TAG, "System ready");




    //extern void task_stepper_motor(void *pvParameters);
    //xTaskCreate(task_stepper_motor, "run_stepper_motor", 4096, NULL, 5, NULL);
     


    
}