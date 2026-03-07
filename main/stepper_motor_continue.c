#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "STEPPER_MOTOR";

// 定义连接到ULN2003驱动板的GPIO引脚 (根据你的接线修改)
// ESP32-C6开发板上可用的GPIO引脚
#define IN1 GPIO_NUM_9   // 对应电机A相
#define IN2 GPIO_NUM_18   // 对应电机B相
#define IN3 GPIO_NUM_19   // 对应电机C相
#define IN4 GPIO_NUM_20   // 对应电机D相

// 四相八拍控制时序 (A-AB-B-BC-C-CD-D-DA)
// 这是28BYJ-48最常用的驱动方式，运行更平稳
const uint8_t STEP_SEQUENCE[8][4] = {
    {1, 0, 0, 0},  // A
    {1, 1, 0, 0},  // AB
    {0, 1, 0, 0},  // B
    {0, 1, 1, 0},  // BC
    {0, 0, 1, 0},  // C
    {0, 0, 1, 1},  // CD
    {0, 0, 0, 1},  // D
    {1, 0, 0, 1}   // DA
};

// 电机参数
#define STEP_ANGLE      5.625   // 标称步距角（度）
#define GEAR_RATIO      64      // 减速比 64:1
// 实际步距角 = 5.625 / 64 ≈ 0.0879度
// 一圈所需步数 = 360 / 0.0879 ≈ 4096步
#define STEPS_PER_REV   4096    // 电机转一圈所需的步数（八拍模式）

// 默认步间延迟(微秒) - 对应约500Hz的脉冲频率
// 更小的值提高转速，但低于800us可能导致失步
#define DEFAULT_STEP_DELAY_US 2000  // 2ms

// 初始化GPIO
void motor_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1) | (1ULL << IN2) | (1ULL << IN3) | (1ULL << IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // 初始化为低电平
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
    
    ESP_LOGI(TAG, "GPIO initialized for stepper motor");
}

// 同时设置四个引脚的电平
static inline void set_step(uint8_t w1, uint8_t w2, uint8_t w3, uint8_t w4)
{
    gpio_set_level(IN1, w1);
    gpio_set_level(IN2, w2);
    gpio_set_level(IN3, w3);
    gpio_set_level(IN4, w4);
}

// 执行一步
static inline void do_step(int step_index)
{
    set_step(STEP_SEQUENCE[step_index][0],
             STEP_SEQUENCE[step_index][1],
             STEP_SEQUENCE[step_index][2],
             STEP_SEQUENCE[step_index][3]);
}

// 停止电机（所有线圈断电）
void motor_stop(void)
{
    set_step(0, 0, 0, 0);
    ESP_LOGI(TAG, "Motor stopped");
}

/**
 * 旋转指定步数
 * @param steps 步数（正转一圈约需4096步）
 * @param direction 方向 (1: 正转, -1: 反转)
 * @param delay_us 步间延迟(微秒)，建议1000-3000us
 */
void motor_rotate_steps(int steps, int direction, uint32_t delay_us)
{
    if (steps == 0) return;
    
    // 确保方向参数正确
    if (direction >= 0) {
        direction = 1;
    } else {
        direction = -1;
    }
    
    ESP_LOGI(TAG, "Rotating %d steps, direction: %s, delay: %lu us", 
             steps, direction > 0 ? "CW" : "CCW", delay_us);
    
    int step_count = sizeof(STEP_SEQUENCE) / sizeof(STEP_SEQUENCE[0]);
    int step_index = 0;
    
    // 根据方向确定步进序列的移动方向
    for (int i = 0; i < steps; i++) {
        do_step(step_index);
        esp_rom_delay_us(delay_us);
        
        // 更新步进索引
        if (direction > 0) {
            step_index = (step_index + 1) % step_count;
        } else {
            step_index = (step_index - 1 + step_count) % step_count;
        }
    }
    
    // 停止电机
    motor_stop();
}

/**
 * 旋转指定角度
 * @param degrees 角度（0-360）
 * @param direction 方向 (1: 正转, -1: 反转)
 * @param delay_us 步间延迟(微秒)
 */
void motor_rotate_angle(float degrees, int direction, uint32_t delay_us)
{
    // 将角度转换为步数
    int steps = (int)(degrees * STEPS_PER_REV / 360.0);
    if (steps == 0 && degrees > 0) {
        steps = 1;  // 至少转一步
    }
    
    motor_rotate_steps(steps, direction, delay_us);
    ESP_LOGI(TAG, "Rotated %.1f degrees (%d steps)", degrees, steps);
}

/**
 * 连续旋转
 * @param direction 方向 (1: 正转, -1: 反转)
 * @param delay_us 步间延迟(微秒)
 * @param duration_ms 持续时间(毫秒)，0表示无限旋转
 */
void motor_rotate_continuous(int direction, uint32_t delay_us, uint32_t duration_ms)
{
    if (direction == 0) return;
    
    int direction_val = (direction > 0) ? 1 : -1;
    int step_count = sizeof(STEP_SEQUENCE) / sizeof(STEP_SEQUENCE[0]);
    int step_index = 0;
    
    ESP_LOGI(TAG, "Continuous rotation, direction: %s", direction > 0 ? "CW" : "CCW");
    
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t duration_ticks = (duration_ms > 0) ? pdMS_TO_TICKS(duration_ms) : 0;
    
    while (1) {
        do_step(step_index);
        esp_rom_delay_us(delay_us);
        
        // 更新步进索引
        if (direction_val > 0) {
            step_index = (step_index + 1) % step_count;
        } else {
            step_index = (step_index - 1 + step_count) % step_count;
        }
        
        // 检查是否达到持续时间
        if (duration_ms > 0) {
            if ((xTaskGetTickCount() - start_tick) >= duration_ticks) {
                break;
            }
        }
    }
    
    motor_stop();
    ESP_LOGI(TAG, "Continuous rotation finished");
} 
void run_stepper_motor(void *pvParameters)
{
    vTaskDelay(pdMS_TO_TICKS(2000));
    // 初始化GPIO
    motor_gpio_init();
    
    ESP_LOGI(TAG, "28BYJ-48 Stepper Motor Test Started");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    while (1) {
        // 测试1：正转1圈
        ESP_LOGI(TAG, "Test 1: Rotate 1 revolution clockwise");
        motor_rotate_steps(STEPS_PER_REV, 1, DEFAULT_STEP_DELAY_US);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 测试2：反转90度
        ESP_LOGI(TAG, "Test 2: Rotate 90 degrees counter-clockwise");
        motor_rotate_angle(90.0, -1, DEFAULT_STEP_DELAY_US);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 测试3：正转180度（快速）
        ESP_LOGI(TAG, "Test 3: Rotate 180 degrees clockwise (faster)");
        motor_rotate_angle(180.0, 1, 1500);  // 1.5ms/步，速度更快
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 测试4：连续旋转2秒（正转）
        ESP_LOGI(TAG, "Test 4: Continuous rotation for 2 seconds");
        motor_rotate_continuous(1, DEFAULT_STEP_DELAY_US, 2000);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // 测试5：连续旋转2秒（反转）
        ESP_LOGI(TAG, "Test 5: Continuous rotation for 2 seconds (reverse)");
        motor_rotate_continuous(-1, DEFAULT_STEP_DELAY_US, 2000);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}