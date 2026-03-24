/*
 * ESP32-C6 + AS7341 - 11通道光谱传感器
 * 8个可见光通道 + NIR + Clear
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
#include "sensor_data.h"
#define TAG "AS7341"

// WiFi配置
#define SOFT_AP_SSID      "ESP32C6AS7341"
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

// AS7341地址
//#define AS7341_ADDR                0x39


//#define DEBUG
#ifdef DEBUG
#define AS_LOGI(TAG, format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#define AS_LOGD(TAG, format, ...) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#else
#define AS_LOGI(TAG, format, ...)
#define AS_LOGD(TAG, format, ...)
#endif 


static sensor_data_t g_latest_data = {0};
static SemaphoreHandle_t g_data_mutex = NULL;
static ppfd_config_t g_ppfd_config;
sensor_data_t data = {0};
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

static esp_err_t as7341_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7341_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t as7341_read_reg(uint8_t reg, uint8_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7341_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7341_ADDR << 1 | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t as7341_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    if (len == 0) return ESP_OK;
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7341_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, AS7341_ADDR << 1 | I2C_MASTER_READ, true);
    
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
// SMUX配置
// ==============================

static const uint8_t smux_table[][2] = {
    {0x00, 0x30},  // F1 to ADC0
    {0x01, 0x31},  // F2 to ADC1
    {0x02, 0x32},  // F3 to ADC2
    {0x03, 0x33},  // F4 to ADC3
    {0x04, 0x34},  // F5 to ADC4
    {0x05, 0x35},  // F6 to ADC5
    {0x06, 0x30},  // F7 to ADC0 (Cycle 2)
    {0x07, 0x31},  // F8 to ADC1 (Cycle 2)
    {0x08, 0x32},  // NIR to ADC2 (Cycle 2)
    {0x09, 0x33},  // Clear to ADC3 (Cycle 2)
    {0xFF, 0x00}
};

static esp_err_t as7341_configure_smux(void)
{
    esp_err_t ret;
    
    ret = as7341_write_reg(AS7341_REG_ENABLE, AS7341_ENABLE_PON);
    if (ret != ESP_OK) return ret;
    
    ret = as7341_write_reg(AS7341_REG_CFG6, 0x00);
    if (ret != ESP_OK) return ret;
    
    for (int i = 0; smux_table[i][0] != 0xFF; i++) {
        ret = as7341_write_reg(0x00 + smux_table[i][0], smux_table[i][1]);
        if (ret != ESP_OK) return ret;
    }
    
    ret = as7341_write_reg(AS7341_REG_CFG8, 0x10);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    AS_LOGI(TAG, "SMUX configured");
    return ESP_OK;
}

// ==============================
// AS7341 初始化
// ==============================

static esp_err_t as7341_init(void)
{
    uint8_t id;
    esp_err_t ret;
    
    ret = as7341_read_reg(AS7341_REG_ID, &id);
    if (ret != ESP_OK) {
        AS_LOGD(TAG, "Failed to read ID");
        //return ret;
    }
    
    AS_LOGI(TAG, "AS7341 ID: 0x%02X", id);
    
    // 软件复位
    ret = as7341_write_reg(AS7341_REG_ENABLE, 0x06);
    //if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 配置增益 (1x)
    ret = as7341_write_reg(AS7341_REG_AGAIN, 0x00);
    if (ret != ESP_OK) return ret;
    
    // 配置积分时间
    ret = as7341_write_reg(AS7341_REG_ATIME, 20);
    if (ret != ESP_OK) return ret;
    
    ret = as7341_write_reg(AS7341_REG_ASTEP_L, 0xE7);
    if (ret != ESP_OK) return ret;
    
    ret = as7341_write_reg(AS7341_REG_ASTEP_H, 0x03);
    if (ret != ESP_OK) return ret;
    
    // 2-cycle模式
    ret = as7341_write_reg(AS7341_REG_CFG0, 0x02);
    if (ret != ESP_OK) return ret;
    
    // 配置SMUX
    ret = as7341_configure_smux();
    if (ret != ESP_OK) return ret;
    
    // 使能
    ret = as7341_write_reg(AS7341_REG_ENABLE, AS7341_ENABLE_PON);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    
    ret = as7341_write_reg(AS7341_REG_ENABLE, AS7341_ENABLE_PON | AS7341_ENABLE_SMUXEN);
    if (ret != ESP_OK) return ret;
    
    AS_LOGI(TAG, "AS7341 initialized");
    return ESP_OK;
}

// ==============================
// 读取所有通道
// ==============================

static esp_err_t as7341_read_all_channels(sensor_data_t *data)
{
    uint8_t buffer[12];
    esp_err_t ret;
    
    // Cycle 1
    ret = as7341_write_reg(AS7341_REG_ENABLE, AS7341_ENABLE_PON | AS7341_ENABLE_SPEN | AS7341_ENABLE_SMUXEN);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ret = as7341_read_regs(AS7341_REG_CH0_DATA_L, buffer, 12);
    if (ret != ESP_OK) return ret;
    
    data->f1 = buffer[0] | (buffer[1] << 8);
    data->f2 = buffer[2] | (buffer[3] << 8);
    data->f3 = buffer[4] | (buffer[5] << 8);
    data->f4 = buffer[6] | (buffer[7] << 8);
    data->f5 = buffer[8] | (buffer[9] << 8);
    data->f6 = buffer[10] | (buffer[11] << 8);
    
    // Cycle 2
    ret = as7341_write_reg(AS7341_REG_ENABLE, AS7341_ENABLE_PON | AS7341_ENABLE_SPEN | AS7341_ENABLE_SMUXEN);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(200));
    
    ret = as7341_read_regs(AS7341_REG_CH0_DATA_L, buffer, 12);
    if (ret != ESP_OK) return ret;
    
    data->f7 = buffer[0] | (buffer[1] << 8);
    data->f8 = buffer[2] | (buffer[3] << 8);
    data->nir = buffer[4] | (buffer[5] << 8);
    data->clear = buffer[6] | (buffer[7] << 8);
    
    return ESP_OK;
}

// ==============================
// PPFD 计算函数
// ==============================

void ppfd_init(ppfd_config_t *config)
{
    const float DEFAULT_PAR_WEIGHTS[8] = {
        0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125, 0.125
    };
    
    const float DEFAULT_PLANT_WEIGHTS[8] = {
        0.20, 0.45, 0.60, 0.70, 0.85, 0.95, 1.00, 0.90
    };
    
    for (int i = 0; i < 8; i++) {
        config->weight_par[i] = DEFAULT_PAR_WEIGHTS[i];
        config->weight_plant[i] = DEFAULT_PLANT_WEIGHTS[i];
    }
    config->calibration_k = 0.05f;  // 增加默认值
    config->reference_ppfd = 0;
}

static void ppfd_calculate(sensor_data_t *data)
{
    // 植物响应权重
    const float PLANT_WEIGHTS[8] = {
        0.20f, 0.45f, 0.60f, 0.70f, 0.85f, 0.95f, 1.00f, 0.90f
    };
    
    // 计算加权和
    float weighted_sum = 0.0f;
    weighted_sum += (float)data->f1 * PLANT_WEIGHTS[0];
    weighted_sum += (float)data->f2 * PLANT_WEIGHTS[1];
    weighted_sum += (float)data->f3 * PLANT_WEIGHTS[2];
    weighted_sum += (float)data->f4 * PLANT_WEIGHTS[3];
    weighted_sum += (float)data->f5 * PLANT_WEIGHTS[4];
    weighted_sum += (float)data->f6 * PLANT_WEIGHTS[5];
    weighted_sum += (float)data->f7 * PLANT_WEIGHTS[6];
    weighted_sum += (float)data->f8 * PLANT_WEIGHTS[7];
    
    // 调试输出
    static int count = 0;
    if (++count % 5 == 0) {
        AS_LOGI(TAG, "PPFD weighted sum = %.1f", weighted_sum);
    }
    
    // 计算 PPFD
    const float CAL_K = 0.05f;
    data->ppfd_plant = weighted_sum * CAL_K;
    
    // 计算 PAR
    float par_sum = (float)data->f1 + data->f2 + data->f3 + data->f4 +
                    data->f5 + data->f6 + data->f7 + data->f8;
    data->ppfd_par = par_sum * 0.125f;
}

// ==============================
// 传感器任务
// ==============================

static void sensor_task(void *pvParameters)
{
    AS_LOGI(TAG, "Sensor task started");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    uint32_t cursor = 0;
    
    
    while (1) {
        esp_err_t ret = as7341_read_all_channels(&data);
        
        if (ret == ESP_OK) {
            data.cursor = cursor++;
            
            // 计算 CI
            if (data.f8 + data.f7 > 0) {
                data.ci = (float)((int)data.f8 - (int)data.f7) / 
                         (float)(data.f8 + data.f7);
            } else {
                data.ci = 0.0f;
            }
            
            // 计算 PPFD
            ppfd_calculate(&data);
            
            // 更新全局数据
            if (xSemaphoreTake(g_data_mutex, portMAX_DELAY)) {
                g_latest_data = data;
                xSemaphoreGive(g_data_mutex);
            }
            
            // 打印数据
            if (cursor % 3 == 0) {
                AS_LOGI(TAG, "F1=%4u F2=%4u F3=%4u F4=%4u F5=%4u F6=%4u", 
                         data.f1, data.f2, data.f3, data.f4, data.f5, data.f6);
                AS_LOGI(TAG, "F7=%4u F8=%4u NIR=%4u Clear=%4u CI=%.3f PPFD=%.1f", 
                         data.f7, data.f8, data.nir, data.clear, data.ci, data.ppfd_plant);
                //ESP_LOGI(TAG, " CI=%.3f PPFD=%.1f", 
                //         data.ci, data.ppfd_plant);
            }
        } else {
            AS_LOGD(TAG, "Failed to read sensor");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
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
    
    char response[768];  // 增大缓冲区
    snprintf(response, sizeof(response),
        "{"
        "\"cursor\":%lu,"
        "\"f1_415nm\":%u,\"f2_445nm\":%u,\"f3_480nm\":%u,\"f4_515nm\":%u,"
        "\"f5_555nm\":%u,\"f6_590nm\":%u,\"f7_630nm\":%u,\"f8_680nm\":%u,"
        "\"nir\":%u,\"clear\":%u,"
        "\"ci\":%.3f,"
        "\"ppfd\":%.1f,\"par\":%.1f"
        "}",
        (unsigned long)data.cursor,
        data.f1, data.f2, data.f3, data.f4,
        data.f5, data.f6, data.f7, data.f8,
        data.nir, data.clear,
        data.ci,
        data.ppfd_plant, data.ppfd_par);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    const char html[] = 
        "<!DOCTYPE html>"
        "<html><head><title>AS7341 11-Channel Spectrometer</title>"
        "<meta charset='UTF-8'>"
        "<meta http-equiv='refresh' content='3'>"
        "<style>"
        "body{font-family:Arial;margin:20px;background:#f0f0f0;}"
        ".container{max-width:900px;margin:0 auto;background:white;padding:20px;border-radius:10px;}"
        "h1{color:#2c3e50;border-bottom:3px solid #3498db;}"
        ".cycle{margin:15px 0;padding:10px;background:#ecf0f1;border-radius:5px;}"
        ".channels{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;}"
        ".ch{background:white;padding:10px;text-align:center;border-radius:5px;}"
        ".ch .nm{font-size:12px;color:#7f8c8d;}"
        ".ch .val{font-size:18px;font-weight:bold;color:#2c3e50;}"
        ".ci-box{background:#27ae60;color:white;padding:20px;border-radius:10px;text-align:center;margin-bottom:20px;}"
        ".ci-box .val{font-size:48px;font-weight:bold;}"
        ".ppfd-box{background:#3498db;color:white;padding:15px;border-radius:10px;margin-bottom:20px;text-align:center;}"
        ".ppfd-box .val{font-size:36px;font-weight:bold;}"
        ".ppfd-box .unit{font-size:14px;}"
        "</style></head>"
        "<body>"
        "<div class='container'>"
        "<h1>🌱 AS7341 11-Channel Spectrometer</h1>"
        
        "<div class='ppfd-box'>"
        "<h3>☀️ 光合有效辐射 (PPFD)</h3>"
        "<div class='val' id='ppfd'>--</div>"
        "<div class='unit'>μmol/m²/s</div>"
        "</div>"
        
        "<div class='ci-box'>"
        "Chlorophyll Index (NDVI-like)<br>"
        "<span class='val' id='ci'>--</span>"
        "</div>"
        
        "<div class='cycle'>"
        "<h3>Cycle 1 - Visible Spectrum (415-590nm)</h3>"
        "<div class='channels'>"
        "<div class='ch'><div class='nm'>F1 415nm</div><div class='val' id='f1'>--</div></div>"
        "<div class='ch'><div class='nm'>F2 445nm</div><div class='val' id='f2'>--</div></div>"
        "<div class='ch'><div class='nm'>F3 480nm</div><div class='val' id='f3'>--</div></div>"
        "<div class='ch'><div class='nm'>F4 515nm</div><div class='val' id='f4'>--</div></div>"
        "<div class='ch'><div class='nm'>F5 555nm</div><div class='val' id='f5'>--</div></div>"
        "<div class='ch'><div class='nm'>F6 590nm</div><div class='val' id='f6'>--</div></div>"
        "</div></div>"
        
        "<div class='cycle'>"
        "<h3>Cycle 2 - Red/NIR</h3>"
        "<div class='channels'>"
        "<div class='ch'><div class='nm'>F7 630nm</div><div class='val' id='f7'>--</div></div>"
        "<div class='ch'><div class='nm'>F8 680nm</div><div class='val' id='f8'>--</div></div>"
        "<div class='ch'><div class='nm'>NIR ~910nm</div><div class='val' id='nir'>--</div></div>"
        "<div class='ch'><div class='nm'>Clear</div><div class='val' id='clear'>--</div></div>"
        "</div></div>"
        
        "<p style='text-align:center;'><a href='/latest'>View JSON</a></p>"
        "</div>"
        
        "<script>"
        "function updateData(){"
        "fetch('/latest').then(r=>r.json()).then(d=>{"
        "   document.getElementById('ci').innerText = d.ci ? d.ci.toFixed(3) : '--';"
        "   document.getElementById('ppfd').innerText = d.ppfd ? d.ppfd.toFixed(1) : '--';"
        "   const channels = ['f1','f2','f3','f4','f5','f6','f7','f8','nir','clear'];"
        "   channels.forEach(k => {"
        "       let el = document.getElementById(k);"
        "       if (el) {"
        "           let val = d[k + '_415nm'] || d[k + '_445nm'] || "
        "                    d[k + '_480nm'] || d[k + '_515nm'] || "
        "                    d[k + '_555nm'] || d[k + '_590nm'] || "
        "                    d[k + '_630nm'] || d[k + '_680nm'] || "
        "                    d[k] || '--';"
        "           el.innerText = val;"
        "       }"
        "   });"
        "}).catch(e => console.log('Waiting for data...'));"
        "}"
        "setInterval(updateData, 3000);"
        "updateData();"
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
            .authmode = WIFI_AUTH_WPA2_PSK, //   WIFI_AUTH_WPA_WPA2_PSK,
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
extern void task_stepper_motor(void *pvParameters);
extern void run_mqtt(void);
void app_main(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP32-C6 + AS7341 - 11-Channel Spectrometer");
    
    // 初始化PPFD配置
    ppfd_init(&g_ppfd_config);
    
    wifi_init_softap();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }
    
    ret = as7341_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AS7341 init failed");
        return;
    }
    
    g_data_mutex = xSemaphoreCreateMutex();
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    
    vTaskDelay(pdMS_TO_TICKS(500));
    start_webserver();
    xTaskCreate(task_stepper_motor, "task_stepper_motor", 4096, NULL, 5, NULL);

    run_mqtt();
    ESP_LOGI(TAG, "System ready - Connect to WiFi: %s", SOFT_AP_SSID);
}