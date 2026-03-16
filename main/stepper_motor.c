#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sensor_data.h"
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

#define STEPS_PER_REV 30000
 extern sensor_data_t data ;
// 电机控制结构体
 

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
 

uint32_t get_current_time_us(void)
{
    TickType_t ticks = xTaskGetTickCount();
    uint32_t ms = ticks * portTICK_PERIOD_MS;
    return ms * 1000;  // 转换为微秒
}

static motor_state_t motor_state  ;
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
 

static bool is_motor_running(void)
{
    static int cnt_ci = 0;
    static int cnt_dark = 0;
    static int cnt_direction = 0;
    float current_ci, current_ppfd;
     static bool first_run = true;  // 添加首次运行标志
     
    // 如果是首次运行，重置所有静态变量
    if (first_run) {
        cnt_ci = 0;
        cnt_dark = 0;
        cnt_direction = 1;
        first_run = false;
        ESP_LOGI(TAG, "Static variables reset: cnt_ci=%d", cnt_ci);
    }
    // 保护临界区
    portENTER_CRITICAL(&spinlock);
    current_ci = data.ci;
    current_ppfd = data.ppfd_plant;
    portEXIT_CRITICAL(&spinlock);
    static uint32_t debug_counter = 0;
    if (debug_counter++ % 1000 == 0){
    //if(cnt_direction != motor_state.direction){
    // int64_t current_time = esp_timer_get_time();
    
        //ESP_LOGI(TAG, "=== DEBUG INFO ===");
        //ESP_LOGI(TAG, "Current time (us): %lld", current_time);
        //ESP_LOGI(TAG, "Current time (ms): %lld", current_time / 1000);
        //ESP_LOGI(TAG, "last_step_time: %lld", motor_state.last_step_time);
        ESP_LOGI(TAG, "cnt_ci: %d", cnt_ci);
        ESP_LOGI(TAG, "current_step: %d", motor_state.current_step);
        ESP_LOGI(TAG, "direction: %d", motor_state.direction);
        ESP_LOGI(TAG, "ci: %f, ppfd: %f", current_ci, current_ppfd);
        //ESP_LOGI(TAG, "=== END DEBUG ===");
        cnt_direction = motor_state.direction;
    }
    
    if(motor_state.current_step <= 100){
        motor_state.current_step = motor_state.total_steps - 1000;
        motor_state.direction = 1; 
        return true;
    }
    
    if(motor_state.current_step >= motor_state.total_steps-100){
        motor_state.current_step =  1000 ;
        motor_state.direction = -1;
        return true;
    }
    
    // 使用局部变量 current_ppfd
    if(motor_state.direction != -1 && current_ppfd < 50.0){ 
        if(cnt_dark >= 5){
            motor_state.direction =-1;
            motor_state.current_step =1000;
            //cnt_dark = 0; 
            cnt_ci = 0;
            printf("ppfd %f \n", current_ppfd);
            return true; 
        }
        cnt_dark++;
        
    } 
    //else {
    //    cnt_dark = 0;  // 添加：条件不满足时重置
    //}
    
    // 使用局部变量 current_ppfd
    if(current_ppfd > 100.0){
        cnt_dark = 0;  
    }
    
    // 使用局部变量 current_ci
    if(current_ci > 0.5 && current_ci<0.99){ 
        if(cnt_ci >= 3){
            motor_state.final_step = motor_state.total_steps - 1000;
            motor_state.direction =0;// (motor_state.direction > 0) ? -1 : 1;
            motor_state.is_running = false; 
            printf("ci %f, cnt_ci %d, current_step:%d direction %d \n",
                current_ci,      // 使用局部变量
                cnt_ci,
                motor_state.current_step,
                motor_state.direction
            );
            cnt_ci = 0;
            return false;
        } 
        cnt_ci++;
    } else {
        cnt_ci = 0;  // 添加：条件不满足时重置
    }
    
    return true;
}

 
// 在文件开头定义
static portMUX_TYPE motor_spinlock = portMUX_INITIALIZER_UNLOCKED;

// 修改 motor_state 的访问
void motor_rotate_non_blocking(int steps, uint32_t delay_ms) {
    portENTER_CRITICAL(&motor_spinlock);
    motor_state.total_steps = steps;
    motor_state.current_step = motor_state.final_step;
    //motor_state.direction = (motor_state.direction > 0) ? 1 : -1;
    motor_state.step_delay_ms = delay_ms;
    motor_state.last_step_time = esp_timer_get_time();
    portEXIT_CRITICAL(&motor_spinlock);
    
    ESP_LOGI(TAG, "Start motor: total=%d, cur=%d, dir=%d", 
        steps, motor_state.current_step, motor_state.direction);
}

void motor_update(void) {
    int64_t now = esp_timer_get_time();
    int64_t time_since_last_step = now - motor_state.last_step_time;
    int64_t delay_us = motor_state.step_delay_ms * 1000LL;
    static int cur_step = 0;
    int step_count = sizeof(STEP_SEQUENCE) / sizeof(STEP_SEQUENCE[0]);
    if (time_since_last_step >= delay_us) {
        // 执行下一步
        
        
        int random_delay = (rand() % 1000) + 500; // 生成500到1500之间的随机数
        if( motor_state.direction == 0){
            if(random_delay <1000  ){
                motor_state.current_step -= 1;
                cur_step = (cur_step - 1 + step_count) % step_count;
            }
            else{
                motor_state.current_step += 1;
                cur_step = (cur_step + 1 + step_count) % step_count;
            }
        }
        else{
            motor_state.current_step += motor_state.direction;
            cur_step = (cur_step + motor_state.direction + step_count) % step_count;
        }
        
        
        
        set_step(STEP_SEQUENCE[cur_step][0],
                 STEP_SEQUENCE[cur_step][1],
                 STEP_SEQUENCE[cur_step][2],
                 STEP_SEQUENCE[cur_step][3]);

        motor_state.last_step_time = now;
        
        // 调试：记录步进变化
        static uint32_t debug_counter = 0;
        if (debug_counter++ % 1000 == 0){
        //if (motor_state.current_step % 1000 == 0) {
            ESP_LOGI(TAG, "motor_update: cur_step=%d, direction=%d, time_since=%lld", 
                motor_state.current_step, motor_state.direction, time_since_last_step);
        }
    }
}
// 在loop或任务中定期调用这个函数
void xmotor_update(void)
{
    
    // 检查是否该执行下一步
    //int64_t now = esp_timer_get_time(); 
    uint32_t now = get_current_time_us();
    uint32_t time_since_last_step = now - motor_state.last_step_time; 
    uint32_t delay_us = motor_state.step_delay_ms * 1000;
    static int cur_step=0;
    
    if (time_since_last_step >= delay_us) {
        
            // 执行下一步
            int step_count = sizeof(STEP_SEQUENCE) / sizeof(STEP_SEQUENCE[0]);
            cur_step = (cur_step + motor_state.direction + step_count) % step_count;
            
            set_step(STEP_SEQUENCE[cur_step][3],
                     STEP_SEQUENCE[cur_step][2],
                     STEP_SEQUENCE[cur_step][1],
                     STEP_SEQUENCE[cur_step][0]);
            
            motor_state.current_step+= motor_state.direction;
            motor_state.last_step_time = now; 
        
     }
    
}
 


void task_stepper_motor(void *pvParameters)
{
    motor_gpio_init();
    ESP_LOGI(TAG, "Stepper Motor Task Started");
    
    // 等待传感器数据就绪（避免初始0值）
    int wait_count = 0;
    float last_ci = 0, last_ppfd = 0;
    
    do {
        portENTER_CRITICAL(&spinlock);
        last_ci = data.ci;
        last_ppfd = data.ppfd_plant;
        portEXIT_CRITICAL(&spinlock);
        
        if (last_ci == 0.0 && last_ppfd == 0.0) {
            ESP_LOGW(TAG, "Waiting for sensor data... (attempt %d)", ++wait_count);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    } while (last_ci == 0.0 && last_ppfd == 0.0 && wait_count < 50); // 最多等待5秒
    
    if (wait_count >= 50) {
        ESP_LOGE(TAG, "Sensor data timeout! Using default values.");
    } else {
        ESP_LOGI(TAG, "Sensor data ready: ci=%.3f, ppfd=%.1f", last_ci, last_ppfd);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    motor_state.direction =  1; 
    motor_state.is_running = true;
    motor_state.total_steps=STEPS_PER_REV;
    motor_state.final_step = 1;
    while (1) {
        ESP_LOGI(TAG, "Starting motor rotation ci=%.3f, ppfd=%.1f", last_ci, last_ppfd);
        motor_rotate_non_blocking(STEPS_PER_REV, 1);
        
        while (motor_state.is_running == true) {   
            motor_update();
            motor_state.is_running = is_motor_running();           
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        set_step(0, 0, 0, 0);
        portENTER_CRITICAL(&spinlock);
        last_ci = data.ci;
        last_ppfd = data.ppfd_plant;
        portEXIT_CRITICAL(&spinlock);
        ESP_LOGI(TAG, "stop motor rotation ci=%.3f, ppfd=%.1f", last_ci, last_ppfd);
        vTaskDelay(pdMS_TO_TICKS(10000));
        int random_delay = (rand() % 1000) + 500; // 生成500到1500之间的随机数
        if( random_delay >600  ){
            motor_state.direction =  0; 
        }
        else{
            
            if( last_ci <0.6){
                 motor_state.direction =  1;
            }
        }
         
        motor_state.is_running = true;
    }
}









