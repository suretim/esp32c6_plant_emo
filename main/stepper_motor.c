#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
static const char *TAG = "STEPPER_MOTOR";

// 定义GPIO引脚
#define IN1 GPIO_NUM_18
#define IN2 GPIO_NUM_19
#define IN3 GPIO_NUM_20
#define IN4 GPIO_NUM_21

// 四相八拍控制时序
const uint8_t STEP_SEQUENCE[8][4] = {
    {1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
    {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}
};

#define STEPS_PER_REV 4096

// 电机控制结构体
typedef struct {
    int current_step;      // 当前步进索引
    int target_steps;      // 目标步数
    int direction;         // 方向
    uint32_t step_delay_us;// 步间延迟
    bool is_running;       // 是否正在运行
} stepper_motor_t;

static stepper_motor_t motor = {
    .current_step = 0,
    .target_steps = 0,
    .direction = 1,
    .step_delay_us = 2000,
    .is_running = false
};

static TimerHandle_t step_timer = NULL;

// 初始化GPIO
static void motor_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IN1) | (1ULL << IN2) | (1ULL << IN3) | (1ULL << IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
}

// 设置引脚电平
static inline void set_step(uint8_t w1, uint8_t w2, uint8_t w3, uint8_t w4)
{
    gpio_set_level(IN1, w1);
    gpio_set_level(IN2, w2);
    gpio_set_level(IN3, w3);
    gpio_set_level(IN4, w4);
}

// 执行一步
static void do_step(int step_index)
{
    set_step(STEP_SEQUENCE[step_index][0],
             STEP_SEQUENCE[step_index][1],
             STEP_SEQUENCE[step_index][2],
             STEP_SEQUENCE[step_index][3]);
}

// 定时器回调函数 - 每次定时器触发执行一步
void step_timer_callback(TimerHandle_t xTimer)
{
    if (!motor.is_running) return;
    
    // 执行一步
    do_step(motor.current_step);
    
    // 更新步进索引
    motor.current_step = (motor.current_step + motor.direction + 8) % 8;
    
    // 更新目标步数计数
    if (motor.direction > 0) {
        motor.target_steps--;
    } else {
        motor.target_steps++;
    }
    
    // 检查是否到达目标
    if (motor.target_steps == 0) {
        motor.is_running = false;
        set_step(0, 0, 0, 0);  // 停止电机
        xTimerStop(step_timer, 0);
        ESP_LOGI(TAG, "Motor stopped - target reached");
    }
}

/**
 * 启动电机旋转
 * @param steps 步数
 * @param direction 方向 (1: 正转, -1: 反转)
 * @param delay_us 步间延迟(微秒)
 */
void motor_start_rotation(int steps, int direction, uint32_t delay_us)
{
    if (motor.is_running) {
        ESP_LOGW(TAG, "Motor is already running");
        return;
    }
    
    motor.target_steps = steps;
    motor.direction = (direction > 0) ? 1 : -1;
    motor.step_delay_us = delay_us;
    motor.is_running = true;
    
    // 计算定时器周期（微秒转毫秒/滴答）
    uint32_t delay_ms = delay_us / 1000;
    if (delay_ms < 1) delay_ms = 1;
    
    // 启动定时器
    xTimerChangePeriod(step_timer, pdMS_TO_TICKS(delay_ms), 0);
    xTimerStart(step_timer, 0);
    
    ESP_LOGI(TAG, "Motor started: %d steps, direction: %s, delay: %lu us", 
             steps, direction > 0 ? "CW" : "CCW", delay_us);
}

/**
 * 旋转指定角度
 */
void motor_rotate_angle(float degrees, int direction, uint32_t delay_us)
{
    int steps = (int)(degrees * STEPS_PER_REV / 360.0);
    if (steps == 0 && degrees > 0) steps = 1;
    motor_start_rotation(steps, direction, delay_us);
}

// 立即停止电机
void motor_emergency_stop(void)
{
    if (motor.is_running) {
        motor.is_running = false;
        set_step(0, 0, 0, 0);
        xTimerStop(step_timer, 0);
        ESP_LOGI(TAG, "Motor emergency stopped");
    }
}

// 电机控制任务（用于处理用户输入和监控）
void motor_control_task(void *pvParameters)
{
    while (1) {
        // 这里可以添加命令解析、状态监控等
        
        // 每100ms检查一次状态并让出CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// 电机状态结构体
typedef struct {
    int current_step;           // 当前步进索引
    int total_steps;            // 总步数
    int steps_remaining;        // 剩余步数
    int direction;              // 方向
    uint32_t step_delay_ms;     // 步间延迟(ms)
    bool is_running;            // 是否运行中
    int64_t last_step_time;     // 上一步的时间戳(us)
} motor_state_t;

static motor_state_t motor_state = {0};

// 非阻塞的步进电机控制函数
void motor_rotate_non_blocking(int steps, int direction, uint32_t delay_ms)
{
    if (motor_state.is_running) {
        ESP_LOGW(TAG, "Motor already running");
        return;
    }
    
    motor_state.current_step = 0;
    motor_state.total_steps = steps;
    motor_state.steps_remaining = steps;
    motor_state.direction = (direction > 0) ? 1 : -1;
    motor_state.step_delay_ms = delay_ms;
    motor_state.is_running = true;
    motor_state.last_step_time = esp_timer_get_time();
    
    // 执行第一步
    set_step(STEP_SEQUENCE[0][0], STEP_SEQUENCE[0][1], 
             STEP_SEQUENCE[0][2], STEP_SEQUENCE[0][3]);
    
    motor_state.steps_remaining--;
    
    ESP_LOGI(TAG, "Motor started non-blocking: %d steps", steps);
}

// 在loop或任务中定期调用这个函数
void motor_update(void)
{
    if (!motor_state.is_running) return;
    
    // 检查是否该执行下一步
    int64_t now = esp_timer_get_time();
    int64_t time_since_last_step = now - motor_state.last_step_time;
    uint32_t delay_us = motor_state.step_delay_ms * 1000;
    
    if (time_since_last_step >= delay_us) {
        if (motor_state.steps_remaining > 0) {
            // 执行下一步
            int step_count = sizeof(STEP_SEQUENCE) / sizeof(STEP_SEQUENCE[0]);
            motor_state.current_step = (motor_state.current_step + motor_state.direction + step_count) % step_count;
            
            set_step(STEP_SEQUENCE[motor_state.current_step][0],
                     STEP_SEQUENCE[motor_state.current_step][1],
                     STEP_SEQUENCE[motor_state.current_step][2],
                     STEP_SEQUENCE[motor_state.current_step][3]);
            
            motor_state.steps_remaining--;
            motor_state.last_step_time = now;
            
            // 可选：打印进度
            if (motor_state.steps_remaining % 100 == 0) {
                ESP_LOGD(TAG, "Progress: %d/%d steps remaining", 
                         motor_state.steps_remaining, motor_state.total_steps);
            }
        } else {
            // 完成所有步骤
            motor_state.is_running = false;
            set_step(0, 0, 0, 0);
            ESP_LOGI(TAG, "Motor finished non-blocking rotation");
        }
    }
}
 
void task_stepper_motor(void *pvParameters)
{
    // 初始化GPIO
    motor_gpio_init();
    
    ESP_LOGI(TAG, "28BYJ-48 Stepper Motor Test Started (Non-blocking mode)");
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    int count = 0;
    
    while (1) {
        ESP_LOGI(TAG, "=== Test Cycle %d ===", ++count);
        
        // 正转一圈
        ESP_LOGI(TAG, "Rotate 1 revolution CW (non-blocking)");
        motor_rotate_non_blocking(STEPS_PER_REV, 1, 2);  // 2ms/步
        
        // 电机运行时，可以做其他事情
        while (motor_state.is_running) {
            motor_update();  // 更新电机状态
            //ESP_LOGI(TAG, "Motor running... main task working");
            vTaskDelay(pdMS_TO_TICKS(10));  // 每10ms检查一次
        }
        
        ESP_LOGI(TAG, "Motor finished, waiting 2 seconds...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // 反转90度
        //ESP_LOGI(TAG, "Rotate 90 degrees CCW");
        //motor_rotate_non_blocking(STEPS_PER_REV/4, -1, 2);
        
        //while (motor_state.is_running) {
        //    motor_update();
        //    vTaskDelay(pdMS_TO_TICKS(10));
        //}
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
 