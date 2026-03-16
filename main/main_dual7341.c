/*
 * ESP32-C + 双AS7341 - 入射光和反射光测量
 * 计算植被指数 (NDVI, CI, PRI等)
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_random.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

// ==============================
// 配置参数
// ==============================
#define TAG "DUAL_AS7341"

// I2C配置
#define I2C_MASTER_SCL_IO          4
#define I2C_MASTER_SDA_IO          5
#define I2C_MASTER_NUM             0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_MASTER_TIMEOUT_MS      1000

// 复用器控制引脚
#define MUX_PIN_A                  18
#define MUX_PIN_B                  19

// AS7341地址
#define AS7341_ADDR                0x39

// AS7341寄存器
#define AS7341_REG_ENABLE          0x80
#define AS7341_REG_ID               0x92
#define AS7341_REG_STATUS           0x71
#define AS7341_REG_CH0_DATA_L       0x95
#define AS7341_REG_CFG0             0xB0
#define AS7341_REG_CFG1             0xB1
#define AS7341_REG_CFG6             0xB5
#define AS7341_REG_CFG8             0xB7
#define AS7341_REG_CFG9             0xB8
#define AS7341_REG_AGAIN            0xA9
#define AS7341_REG_ATIME            0x81
#define AS7341_REG_ASTEP_L          0x83
#define AS7341_REG_ASTEP_H          0x84

// 控制位
#define AS7341_ENABLE_PON           0x01
#define AS7341_ENABLE_SPEN          0x02
#define AS7341_ENABLE_SMUXEN        0x10
#define AS7341_ENABLE_WEN           0x08

// 传感器数量
#define SENSOR_COUNT                2
#define SENSOR_INCIDENT             0  // 入射光传感器
#define SENSOR_REFLECTED            1  // 反射光传感器

// ==============================
// 数据结构
// ==============================

// 单传感器数据
typedef struct {
    uint32_t timestamp;      // 时间戳
    uint16_t f1;   // 415nm
    uint16_t f2;   // 445nm
    uint16_t f3;   // 480nm
    uint16_t f4;   // 515nm
    uint16_t f5;   // 555nm
    uint16_t f6;   // 590nm
    uint16_t f7;   // 630nm
    uint16_t f8;   // 680nm
    uint16_t nir;  // 近红外
    uint16_t clear; // 清除通道
} as7341_data_t;

// 植被指数
typedef struct {
    float ndvi;         // 归一化植被指数 (NIR - Red)/(NIR + Red)
    float ci;           // 叶绿素指数 (F8 - F7)/(F8 + F7)
    float pri;          // 光化学反射指数 (F5 - F6)/(F5 + F6) 近似
    float re_ndvi;      // 红边NDVI (NIR - F8)/(NIR + F8)
    float sr;           // 简单比值指数 NIR/Red
    float msr;          // 改进型简单比值指数 (NIR/Red - 1)/sqrt(NIR/Red + 1)
    float ndvi_fy;      // 使用FY的NDVI (FY - F7)/(FY + F7)
    float ndvi_nir;     // 使用NIR的NDVI (NIR - F7)/(NIR + F7)
} vegetation_indices_t;

// 双传感器系统数据
typedef struct {
    uint32_t cursor;
    as7341_data_t incident;   // 入射光
    as7341_data_t reflected;  // 反射光
    vegetation_indices_t indices;
    float ppfd;               // 光合有效辐射
} dual_sensor_data_t;

static dual_sensor_data_t g_latest_data = {0};
static SemaphoreHandle_t g_data_mutex = NULL;

// ==============================
// I2C和复用器控制
// ==============================

static void mux_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MUX_PIN_A) | (1ULL << MUX_PIN_B),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // 默认选择传感器0
    gpio_set_level(MUX_PIN_A, 0);
    gpio_set_level(MUX_PIN_B, 0);
    ESP_LOGI(TAG, "MUX initialized");
}

 
// 选择传感器（通过CD4052切换）
static void mux_select(uint8_t sensor_id)
{
    // sensor_id 0: A=0, B=0 → 通道 X0/Y0 (AS7341-0)
    // sensor_id 1: A=1, B=0 → 通道 X1/Y1 (AS7341-1)
    
    gpio_set_level(MUX_PIN_A, sensor_id & 0x01);      // 取最低位
    gpio_set_level(MUX_PIN_B, (sensor_id >> 1) & 0x01); // 取第二位
    
    // 等待复用器稳定（10微秒）
    esp_rom_delay_us(10);
}
// ==============================
// I2C操作
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
// AS7341配置
// ==============================

static esp_err_t as7341_configure_smux(uint8_t sensor_id)
{
    mux_select(sensor_id);
    
    esp_err_t ret;
    
    // SMUX配置表
    const uint8_t smux_table[][2] = {
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
    
    // 进入SMUX配置模式
    ret = as7341_write_reg(AS7341_REG_CFG6, 0x00);
    if (ret != ESP_OK) return ret;
    
    // 写入SMUX配置
    for (int i = 0; smux_table[i][0] != 0xFF; i++) {
        ret = as7341_write_reg(0x00 + smux_table[i][0], smux_table[i][1]);
        if (ret != ESP_OK) return ret;
    }
    
    // 执行SMUX命令
    ret = as7341_write_reg(AS7341_REG_CFG8, 0x10);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGD(TAG, "Sensor %d SMUX configured", sensor_id);
    return ESP_OK;
}

static esp_err_t as7341_init_sensor(uint8_t sensor_id)
{
    mux_select(sensor_id);
    
    uint8_t id;
    esp_err_t ret;
    
    // 读取ID
    ret = as7341_read_reg(AS7341_REG_ID, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor %d: Failed to read ID", sensor_id);
        return ret;
    }
    
    ESP_LOGI(TAG, "Sensor %d ID: 0x%02X", sensor_id, id);
    
    // 软件复位
    ret = as7341_write_reg(AS7341_REG_ENABLE, 0x06);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 配置增益 (1x)
    ret = as7341_write_reg(AS7341_REG_AGAIN, 0x00);
    if (ret != ESP_OK) return ret;
    
    // 配置积分时间
    ret = as7341_write_reg(AS7341_REG_ATIME, 30);
    if (ret != ESP_OK) return ret;
    
    ret = as7341_write_reg(AS7341_REG_ASTEP_L, 0xE7);
    if (ret != ESP_OK) return ret;
    
    ret = as7341_write_reg(AS7341_REG_ASTEP_H, 0x03);
    if (ret != ESP_OK) return ret;
    
    // 2-cycle模式
    ret = as7341_write_reg(AS7341_REG_CFG0, 0x02);
    if (ret != ESP_OK) return ret;
    
    // 配置SMUX
    ret = as7341_configure_smux(sensor_id);
    if (ret != ESP_OK) return ret;
    
    // 使能电源
    ret = as7341_write_reg(AS7341_REG_ENABLE, AS7341_ENABLE_PON);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(5));
    
    // 使能SMUX
    ret = as7341_write_reg(AS7341_REG_ENABLE, AS7341_ENABLE_PON | AS7341_ENABLE_SMUXEN);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "Sensor %d initialized", sensor_id);
    return ESP_OK;
}

esp_err_t as7341_init_all(void)
{
    esp_err_t ret;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        ret = as7341_init_sensor(i);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize sensor %d", i);
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return ESP_OK;
}

// ==============================
// 读取单个传感器数据
// ==============================

static esp_err_t as7341_read_sensor(uint8_t sensor_id, as7341_data_t *data)
{
    mux_select(sensor_id);
    
    uint8_t buffer[12];
    esp_err_t ret;
    
    // Cycle 1
    ret = as7341_write_reg(AS7341_REG_ENABLE, 
                           AS7341_ENABLE_PON | AS7341_ENABLE_SPEN | AS7341_ENABLE_SMUXEN);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(150));
    
    ret = as7341_read_regs(AS7341_REG_CH0_DATA_L, buffer, 12);
    if (ret != ESP_OK) return ret;
    
    data->f1 = buffer[0] | (buffer[1] << 8);
    data->f2 = buffer[2] | (buffer[3] << 8);
    data->f3 = buffer[4] | (buffer[5] << 8);
    data->f4 = buffer[6] | (buffer[7] << 8);
    data->f5 = buffer[8] | (buffer[9] << 8);
    data->f6 = buffer[10] | (buffer[11] << 8);
    
    // Cycle 2
    ret = as7341_write_reg(AS7341_REG_ENABLE, 
                           AS7341_ENABLE_PON | AS7341_ENABLE_SPEN | AS7341_ENABLE_SMUXEN);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(150));
    
    ret = as7341_read_regs(AS7341_REG_CH0_DATA_L, buffer, 12);
    if (ret != ESP_OK) return ret;
    
    data->f7 = buffer[0] | (buffer[1] << 8);
    data->f8 = buffer[2] | (buffer[3] << 8);
    data->nir = buffer[4] | (buffer[5] << 8);
    data->clear = buffer[6] | (buffer[7] << 8);
    
    data->timestamp = (uint32_t)(esp_timer_get_time() / 1000);
    
    return ESP_OK;
}

// ==============================
// 植被指数计算
// ==============================

static float safe_divide(float num, float den, float epsilon)
{
    if (fabs(den) < epsilon) return 0.0f;
    return num / den;
}

static vegetation_indices_t calculate_indices(as7341_data_t *incident, as7341_data_t *reflected)
{
    vegetation_indices_t idx = {0};
    float epsilon = 1e-6f;
    
    // 计算反射率
    float r_f7 = safe_divide(reflected->f7, incident->f7, epsilon);
    float r_f8 = safe_divide(reflected->f8, incident->f8, epsilon);
    float r_nir = safe_divide(reflected->nir, incident->nir, epsilon);
    float r_f5 = safe_divide(reflected->f5, incident->f5, epsilon);
    float r_f6 = safe_divide(reflected->f6, incident->f6, epsilon);
    
    // 1. NDVI (NIR - Red)/(NIR + Red)
    float ndvi_num = r_nir - r_f7;
    float ndvi_den = r_nir + r_f7;
    idx.ndvi = safe_divide(ndvi_num, ndvi_den, epsilon);
    
    // 2. CI (F8 - F7)/(F8 + F7)
    float ci_num = r_f8 - r_f7;
    float ci_den = r_f8 + r_f7;
    idx.ci = safe_divide(ci_num, ci_den, epsilon);
    
    // 3. PRI (F5 - F6)/(F5 + F6) 近似
    float pri_num = r_f5 - r_f6;
    float pri_den = r_f5 + r_f6;
    idx.pri = safe_divide(pri_num, pri_den, epsilon);
    
    // 4. 红边NDVI (NIR - F8)/(NIR + F8)
    float re_num = r_nir - r_f8;
    float re_den = r_nir + r_f8;
    idx.re_ndvi = safe_divide(re_num, re_den, epsilon);
    
    // 5. 简单比值指数 SR = NIR/Red
    idx.sr = safe_divide(r_nir, r_f7, epsilon);
    
    // 6. 改进型简单比值指数 MSR = (NIR/Red - 1)/sqrt(NIR/Red + 1)
    if (idx.sr >= 0) {
        idx.msr = safe_divide((idx.sr - 1.0f), sqrtf(idx.sr + 1.0f), epsilon);
    }
    
    // 7. NDVI using NIR and F8
    idx.ndvi_nir = safe_divide(r_nir - r_f8, r_nir + r_f8, epsilon);
    
    return idx;
}

static float calculate_ppfd(as7341_data_t *data)
{
    // 植物响应权重 (McCree曲线近似)
    const float weights[8] = {
        0.20f,  // F1 415nm
        0.45f,  // F2 445nm
        0.60f,  // F3 480nm
        0.70f,  // F4 515nm
        0.85f,  // F5 555nm
        0.95f,  // F6 590nm
        1.00f,  // F7 630nm
        0.90f   // F8 680nm
    };
    
    uint16_t channels[8] = {
        data->f1, data->f2, data->f3, data->f4,
        data->f5, data->f6, data->f7, data->f8
    };
    
    float weighted_sum = 0.0f;
    for (int i = 0; i < 8; i++) {
        weighted_sum += (float)channels[i] * weights[i];
    }
    
    // 校准系数 (需要实际标定)
    const float CAL_K = 0.001f;
    return weighted_sum * CAL_K;
}

// ==============================
// 传感器任务
// ==============================

static void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Dual sensor task started");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    uint32_t cursor = 0;
    dual_sensor_data_t data = {0};
    
    while (1) {
        // 读取入射光传感器
        esp_err_t ret = as7341_read_sensor(SENSOR_INCIDENT, &data.incident);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read incident sensor");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        // 读取反射光传感器
        ret = as7341_read_sensor(SENSOR_REFLECTED, &data.reflected);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read reflected sensor");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        data.cursor = cursor++;
        
        // 计算植被指数
        data.indices = calculate_indices(&data.incident, &data.reflected);
        
        // 计算PPFD (使用入射光)
        data.ppfd = calculate_ppfd(&data.incident);
        
        // 更新全局数据
        if (xSemaphoreTake(g_data_mutex, portMAX_DELAY)) {
            g_latest_data = data;
            xSemaphoreGive(g_data_mutex);
        }
        
        // 打印结果
        if (cursor % 3 == 0) {
            ESP_LOGI(TAG, "=== 植被指数 ===");
            ESP_LOGI(TAG, "NDVI: %.3f | CI: %.3f | PRI: %.3f", 
                     data.indices.ndvi, data.indices.ci, data.indices.pri);
            ESP_LOGI(TAG, "RE-NDVI: %.3f | SR: %.2f | MSR: %.2f", 
                     data.indices.re_ndvi, data.indices.sr, data.indices.msr);
            ESP_LOGI(TAG, "PPFD: %.1f μmol/m²/s", data.ppfd);
            
            ESP_LOGI(TAG, "入射光: F7=%4u F8=%4u NIR=%4u", 
                     data.incident.f7, data.incident.f8, data.incident.nir);
            ESP_LOGI(TAG, "反射光: F7=%4u F8=%4u NIR=%4u", 
                     data.reflected.f7, data.reflected.f8, data.reflected.nir);
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ==============================
// 简单的Web服务器 (可选)
// ==============================

#include "esp_http_server.h"

static esp_err_t data_get_handler(httpd_req_t *req)
{
    dual_sensor_data_t data = {0};
    
    if (xSemaphoreTake(g_data_mutex, portMAX_DELAY)) {
        data = g_latest_data;
        xSemaphoreGive(g_data_mutex);
    }
    
    char response[1024];
    snprintf(response, sizeof(response),
        "{"
        "\"cursor\":%lu,"
        "\"incident\":{\"f7\":%u,\"f8\":%u,\"nir\":%u},"
        "\"reflected\":{\"f7\":%u,\"f8\":%u,\"nir\":%u},"
        "\"indices\":{"
        "\"ndvi\":%.3f,\"ci\":%.3f,\"pri\":%.3f,"
        "\"re_ndvi\":%.3f,\"sr\":%.2f,\"msr\":%.2f"
        "},"
        "\"ppfd\":%.1f"
        "}",
        (unsigned long)data.cursor,
        data.incident.f7, data.incident.f8, data.incident.nir,
        data.reflected.f7, data.reflected.f8, data.reflected.nir,
        data.indices.ndvi, data.indices.ci, data.indices.pri,
        data.indices.re_ndvi, data.indices.sr, data.indices.msr,
        data.ppfd);
    
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));
    return ESP_OK;
}

static void start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uri_data = {
            .uri = "/data",
            .method = HTTP_GET,
            .handler = data_get_handler
        };
        httpd_register_uri_handler(server, &uri_data);
        ESP_LOGI(TAG, "Web server started on port 80");
    }
}

// ==============================
// 主函数
// ==============================
void test_hardware_timer(void)
{
    ESP_LOGI(TAG, "=== Testing timers ===");
    
    // 测试 esp_timer
    int64_t esp_time1 = esp_timer_get_time();
    ESP_LOGI(TAG, "esp_timer_get_time(): %lld", esp_time1);
    
    // 测试 FreeRTOS tick
    TickType_t tick1 = xTaskGetTickCount();
    ESP_LOGI(TAG, "xTaskGetTickCount(): %lu", tick1);
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    int64_t esp_time2 = esp_timer_get_time();
    TickType_t tick2 = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "After 100ms delay:");
    ESP_LOGI(TAG, "esp_timer delta: %lld us", esp_time2 - esp_time1);
    ESP_LOGI(TAG, "tick delta: %lu ticks", tick2 - tick1);
}
void app_main(void)
{
        ESP_ERROR_CHECK(esp_timer_init());
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP32-C + Dual AS7341 Vegetation Monitor");
    
    // 初始化复用器
    mux_init();
    
    // 初始化I2C
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }
    
    // 初始化两个传感器
    ret = as7341_init_all();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init failed");
        return;
    }
    
    // 创建互斥锁
    g_data_mutex = xSemaphoreCreateMutex();
    test_hardware_timer();

    // 启动传感器任务
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    
    // 启动Web服务器
    vTaskDelay(pdMS_TO_TICKS(1000));
    start_webserver();
    
    ESP_LOGI(TAG, "System ready");
}