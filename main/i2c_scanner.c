/*
 * I2C 扫描器 - 用于检测 AS7343 的 I2C 地址
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"

#define TAG "I2C_SCANNER"
#define I2C_MASTER_SCL_IO          8   // GPIO8
#define I2C_MASTER_SDA_IO          9   // GPIO9
#define I2C_MASTER_NUM             0
#define I2C_MASTER_FREQ_HZ         100000

static void i2c_scanner(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    
    ESP_LOGI(TAG, "开始扫描 I2C 设备...");
    ESP_LOGI(TAG, "SCL: GPIO%d, SDA: GPIO%d", I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO);
    
    int device_count = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "找到设备 at 地址 0x%02X", addr);
            device_count++;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    if (device_count == 0) {
        ESP_LOGE(TAG, "没有找到任何 I2C 设备！");
        ESP_LOGE(TAG, "请检查：");
        ESP_LOGE(TAG, "  1. 接线是否正确（VCC, GND, SDA, SCL）");
        ESP_LOGE(TAG, "  2. 上拉电阻是否连接（4.7kΩ 到 3.3V）");
        ESP_LOGE(TAG, "  3. 设备供电是否正常（3.3V）");
    } else {
        ESP_LOGI(TAG, "扫描完成，找到 %d 个设备", device_count);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "I2C 扫描器启动");
    i2c_scanner();
}