/*
 * ESP32-C6 + AS7343 光谱传感器
 * 修复网络初始化 - 不使用 WiFi
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
#include "esp_netif.h"      // 网络接口
#include "esp_event.h"       // 事件循环
#include "lwip/err.h"        // lwIP 错误码
#include "lwip/sockets.h"    // socket API
#include "lwip/sys.h"        // lwIP 系统
#include "lwip/netdb.h"      // 网络数据库

// I2C 驱动
#include "driver/i2c.h"

// HTTP Server
#include "esp_http_server.h"

// ==============================
// 配置定义
// ==============================
#define TAG "AS7343"

// 缓冲区配置
#define BUFFER_SIZE 100
#define SAMPLE_INTERVAL_MS 2000
#define HTTP_PORT 80

// I2C 配置
#define I2C_MASTER_SCL_IO          5   // GPIO5
#define I2C_MASTER_SDA_IO          4   // GPIO4
#define I2C_MASTER_NUM             0
#define I2C_MASTER_FREQ_HZ         100000

// AS7343 7-bit I2C 地址
#define AS7343_ADDR                 0x39

// AS7343 寄存器地址
#define AS7343_REG_ID               0x92
#define AS7343_REG_STATUS           0x73
#define AS7343_REG_ENABLE           0x80
#define AS7343_REG_CONFIG           0x70

// 通道数据寄存器
#define AS7343_REG_ADDR0            0x95  // F1 低字节
#define AS7343_REG_ADDR1            0x97  // F2 低字节
#define AS7343_REG_ADDR2            0x99  // F3 低字节
#define AS7343_REG_ADDR3            0x9B  // F4 低字节
#define AS7343_REG_ADDR4            0x9D  // F5 低字节
#define AS7343_REG_ADDR5            0x9F  // F6 低字节
#define AS7343_REG_ADDR6            0xA1  // F7 低字节
#define AS7343_REG_ADDR7            0xA3  // F8 低字节
#define AS7343_REG_ADDR8            0xA5  // FZ 低字节
#define AS7343_REG_ADDR9            0xA7  // FY 低字节
#define AS7343_REG_ADDR10           0xA9  // FXL 低字节
#define AS7343_REG_ADDR11           0xAB  // NIR 低字节
#define AS7343_REG_ADDR12           0xAF  // Clear 低字节

// 预期器件ID
#define AS7343_EXPECTED_ID          0x09

// ==============================
// 数据结构定义
// ==============================

typedef struct {
    uint32_t cursor;
    uint64_t timestamp;
    uint16_t f1; uint16_t f2; uint16_t f3; uint16_t f4;
    uint16_t f5; uint16_t f6; uint16_t f7; uint16_t f8;
    uint16_t fz; uint16_t fy; uint16_t fxl; uint16_t nir;
    uint16_t clear;
    float ci;
} sensor_data_t;

typedef struct {
    sensor_data_t data[BUFFER_SIZE];
    uint32_t write_index;
    SemaphoreHandle_t mutex;
} circular_buffer_t;

static circular_buffer_t g_sensor_buffer = {
    .write_index = 0,
    .mutex = NULL
};
// WiFi 配置（可选）

 
// ==============================
// I2C 初始化
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
        ESP_LOGI(TAG, "I2C initialized on SDA=%d, SCL=%d", 
                 I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    }
    
    return ret;
}

// ==============================
// AS7343 通信函数
// ==============================

static esp_err_t as7343_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7343_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "写寄存器 0x%02X 失败: %s", reg, esp_err_to_name(ret));
    }
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
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static uint16_t as7343_read_word(uint8_t reg_low)
{
    uint8_t low = 0, high = 0;
    if (as7343_read_reg(reg_low, &low) != ESP_OK) return 0;
    if (as7343_read_reg(reg_low + 1, &high) != ESP_OK) return 0;
    return (uint16_t)((high << 8) | low);
}

// ==============================
// AS7343 初始化
// ==============================

static esp_err_t as7343_init(void)
{
    uint8_t id = 0;
    esp_err_t ret;
    
    // 读取器件ID
    ret = as7343_read_reg(AS7343_REG_ID, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "读取ID失败");
        return ret;
    }
    
    ESP_LOGI(TAG, "AS7343 器件ID: 0x%02X", id);
    
    if (id != AS7343_EXPECTED_ID) {
        ESP_LOGW(TAG, "器件ID可能不匹配: 期望 0x%02X, 得到 0x%02X", 
                 AS7343_EXPECTED_ID, id);
    }
    
    // 使能传感器
    ret = as7343_write_reg(AS7343_REG_ENABLE, 0x03);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "AS7343 初始化成功");
    return ESP_OK;
}

// ==============================
// 数据采集
// ==============================

static void as7343_read_all_channels(sensor_data_t *data)
{
    // 触发测量
    as7343_write_reg(AS7343_REG_ENABLE, 0x07);
    
    vTaskDelay(pdMS_TO_TICKS(150));
    
    data->f1 = as7343_read_word(AS7343_REG_ADDR0);
    data->f2 = as7343_read_word(AS7343_REG_ADDR1);
    data->f3 = as7343_read_word(AS7343_REG_ADDR2);
    data->f4 = as7343_read_word(AS7343_REG_ADDR3);
    data->f5 = as7343_read_word(AS7343_REG_ADDR4);
    data->f6 = as7343_read_word(AS7343_REG_ADDR5);
    data->f7 = as7343_read_word(AS7343_REG_ADDR6);
    data->f8 = as7343_read_word(AS7343_REG_ADDR7);
    data->fz = as7343_read_word(AS7343_REG_ADDR8);
    data->fy = as7343_read_word(AS7343_REG_ADDR9);
    data->fxl = as7343_read_word(AS7343_REG_ADDR10);
    data->nir = as7343_read_word(AS7343_REG_ADDR11);
    data->clear = as7343_read_word(AS7343_REG_ADDR12);
    
    // 计算叶绿素指数
    if (data->f7 + data->fy > 0) {
        data->ci = (float)(data->fy - data->f7) / (float)(data->f7 + data->fy);
    }
}

static void sensor_collection_task(void *pvParameters)
{
    ESP_LOGI(TAG, "传感器采集任务启动，间隔 %d ms", SAMPLE_INTERVAL_MS);
    
    uint32_t cursor = 0;
    
    while (1) {
        sensor_data_t new_data = {
            .cursor = cursor,
            .timestamp = esp_timer_get_time() / 1000
        };
        
        as7343_read_all_channels(&new_data);
        
        if (xSemaphoreTake(g_sensor_buffer.mutex, portMAX_DELAY)) {
            uint32_t idx = cursor % BUFFER_SIZE;
            g_sensor_buffer.data[idx] = new_data;
            g_sensor_buffer.write_index = cursor + 1;
            xSemaphoreGive(g_sensor_buffer.mutex);
        }
        
        cursor++;
        
        if (cursor % 5 == 0) {
            ESP_LOGI(TAG, "样本 %" PRIu32 ": F2=%u, F6=%u, F7=%u, FY=%u, CI=%.3f", 
                    cursor, new_data.f2, new_data.f6, new_data.f7, new_data.fy, new_data.ci);
        }
        
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

// ==============================
// HTTP 服务器处理函数
// ==============================

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char response[] = 
        "<!DOCTYPE html>"
        "<html><head><title>AS7343 Spectrometer</title>"
        "<meta charset='UTF-8'><meta http-equiv='refresh' content='2'>"
        "<style>"
        "body{font-family:Arial;margin:40px;background:#f0f0f0;}"
        "h1{color:#2c3e50;}"
        ".data{background:white;padding:20px;border-radius:10px;margin:20px 0;}"
        "</style></head>"
        "<body>"
        "<h1>🌱 AS7343 Spectrometer</h1>"
        "<div class='data'>"
        "<p><a href='/latest'>View JSON Data</a></p>"
        "</div>"
        "</body></html>";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t latest_get_handler(httpd_req_t *req)
{
    if (g_sensor_buffer.write_index == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No data");
        return ESP_FAIL;
    }
    
    uint32_t latest_idx = g_sensor_buffer.write_index - 1;
    uint32_t buffer_idx = latest_idx % BUFFER_SIZE;
    
    sensor_data_t data = {0};
    
    if (xSemaphoreTake(g_sensor_buffer.mutex, portMAX_DELAY)) {
        data = g_sensor_buffer.data[buffer_idx];
        xSemaphoreGive(g_sensor_buffer.mutex);
    }
    
    char response[2048];
    snprintf(response, sizeof(response),
        "{"
        "\"cursor\":%" PRIu32 ","
        "\"timestamp\":%llu,"
        "\"f1_415\":%u,"
        "\"f2_445\":%u,"
        "\"f3_480\":%u,"
        "\"f4_515\":%u,"
        "\"f5_555\":%u,"
        "\"f6_590\":%u,"
        "\"f7_630\":%u,"
        "\"f8_680\":%u,"
        "\"fz_705\":%u,"
        "\"fy_730\":%u,"
        "\"fxl_760\":%u,"
        "\"nir\":%u,"
        "\"clear\":%u,"
        "\"ci\":%.3f"
        "}",
        data.cursor, (unsigned long long)data.timestamp,
        data.f1, data.f2, data.f3, data.f4,
        data.f5, data.f6, data.f7, data.f8,
        data.fz, data.fy, data.fxl, data.nir,
        data.clear, data.ci);
    
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
    config.max_uri_handlers = 10;
    
    esp_err_t ret = httpd_start(&server, &config);
    if (ret == ESP_OK) {
        httpd_uri_t uri_root = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler };
        httpd_uri_t uri_latest = { .uri = "/latest", .method = HTTP_GET, .handler = latest_get_handler };
        
        httpd_register_uri_handler(server, &uri_root);
        httpd_register_uri_handler(server, &uri_latest);
        
        ESP_LOGI(TAG, "HTTP server started on port %d", HTTP_PORT);
        ESP_LOGI(TAG, "Connect to http://[esp32-ip]:%d/", HTTP_PORT);
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
    }
}

// ==============================
// 网络初始化 - 简化版本
// ==============================

static void network_init(void)
{
    // 初始化网络接口
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init failed: %s", esp_err_to_name(ret));
        return;
    }
    
    // 创建默认事件循环
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "Network initialized (TCP/IP stack only)");
}

// ==============================
// 主函数
// ==============================

void app_main(void)
{
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32-C6 + AS7343 Spectrometer");
    ESP_LOGI(TAG, "========================================");
    
    // 初始化网络协议栈 (只需要TCP/IP，不需要WiFi)
    network_init();
    
    // 初始化I2C
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C初始化失败: %s", esp_err_to_name(ret));
        return;
    }
    
    // 初始化AS7343
    ret = as7343_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AS7343初始化失败");
        return;
    }
    
    // 创建互斥锁
    g_sensor_buffer.mutex = xSemaphoreCreateMutex();
    if (g_sensor_buffer.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }
    
    // 启动数据采集任务
    BaseType_t task_created = xTaskCreate(
        sensor_collection_task, 
        "sensor_collection", 
        4096, 
        NULL, 
        5, 
        NULL
    );
    
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        return;
    }
    
    // 启动HTTP服务器
    start_webserver();
    
    ESP_LOGI(TAG, "系统初始化完成");
}