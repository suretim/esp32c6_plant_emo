#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "esp_netif.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <errno.h>

// ==================== 配置区 ====================
 
 

#define MQTT_SERVER_PORT 1883
#define MAX_CLIENTS 10
#define BUFFER_SIZE 8192           // 处理缓冲区 8KB
#define MAX_SUBSCRIPTIONS 100
#define MAX_TOPIC_LEN 128
#define RECV_BUFFER_SIZE 8192      // 接收缓冲区 16KB（足够容纳最大报文）
#define MQTT_CLIENT_STACK_SIZE 16384 // 任务栈 8KB
 
// MQTT协议常量
#define MQTT_CONNECT     0x10
#define MQTT_CONNACK     0x20
#define MQTT_PUBLISH     0x30
#define MQTT_PUBACK      0x40
#define MQTT_PUBREC      0x50
#define MQTT_PUBREL      0x60
#define MQTT_PUBCOMP     0x70
#define MQTT_SUBSCRIBE   0x80
#define MQTT_SUBACK      0x90
#define MQTT_UNSUBSCRIBE 0xA0
#define MQTT_UNSUBACK    0xB0
#define MQTT_PINGREQ     0xC0
#define MQTT_PINGRESP    0xD0
#define MQTT_DISCONNECT  0xE0

static const char *TAG = "MQTT_SVER";
static int server_fd = -1;
static esp_netif_t *wifi_sta_netif = NULL; 
static bool wifi_connected = false;
static bool static_ip_configured = false;



// ==================== 客户端管理（接收缓冲区在堆上） ====================
typedef struct mqtt_client {
    int fd;
    bool connected;
    struct mqtt_client *next;
    // 接收缓冲区 - 改为指针，在堆上分配
    uint8_t *recv_buffer;
    int recv_buffer_len;
    int recv_buffer_capacity;
} mqtt_client_t;

static mqtt_client_t *client_list = NULL;
static portMUX_TYPE client_lock = portMUX_INITIALIZER_UNLOCKED;

// ==================== 订阅管理 ====================
typedef struct subscription {
    int client_fd;
    char topic[MAX_TOPIC_LEN];
    int qos;
    struct subscription *next;
} subscription_t;

static subscription_t *subscriptions = NULL;
static int subscription_count = 0;
static SemaphoreHandle_t subscriptions_mutex = NULL;

// ==================== 函数声明 ====================
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void set_static_ip(void);
static int mqtt_parse_remaining_length(uint8_t *buf, int len, int *offset);
static int mqtt_parse_connect(uint8_t *buf, int len);
static void mqtt_send_connack(int client_fd, uint8_t return_code);
static bool mqtt_topic_match(const char *filter, const char *topic);
static mqtt_client_t* add_client(int fd);
static void remove_client(int fd);
static void add_subscription(int client_fd, const char *topic, int qos);
static void remove_client_subscriptions(int client_fd);
static void forward_to_subscribers(int sender_fd, const char *topic, uint8_t *data, int len);
static void handle_subscribe(int client_fd, uint8_t *buffer, int len);
static char* parse_publish_topic(uint8_t *buffer, int len, int *topic_len_out);
static int read_mqtt_packet(mqtt_client_t *client, uint8_t *out_buffer, int max_len);

// ==================== MQTT主题匹配 ====================
 
static bool mqtt_topic_match(const char *filter, const char *topic) {
    if (filter == NULL || topic == NULL) {
        return false;
    }
    
    // 处理 # 通配符
    if (strcmp(filter, "#") == 0) {
        return true;
    }
    
    // 处理以 /# 结尾的过滤器
    size_t filter_len = strlen(filter);
    if (filter_len >= 2 && filter[filter_len - 1] == '#' && filter[filter_len - 2] == '/') {
        // 提取过滤器的基础部分 (去掉末尾的 /#)
        char base_filter[MAX_TOPIC_LEN];
        strncpy(base_filter, filter, filter_len - 2);
        base_filter[filter_len - 2] = '\0';
        
        // 检查主题是否以基础部分开头
        if (strncmp(topic, base_filter, filter_len - 2) == 0) {
            // 主题长度等于基础部分，或者后面还有内容
            return true;
        }
        return false;
    }
    
    const char *f = filter;
    const char *t = topic;
    
    while (*f && *t) {
        if (*f == '#') {
            return (*(f + 1) == '\0');
        }
        
        if (*f == '+') {
            f++;
            while (*t && *t != '/') {
                t++;
            }
            // 跳过 '/' 如果存在
            if (*f == '/' && *t == '/') {
                t++;
                f++;
            }
            continue;
        }
        
        if (*f != *t) {
            return false;
        }
        
        f++;
        t++;
    }
    
    // 处理过滤器结束的情况
    if (*f == '\0' && *t == '\0') {
        return true;
    }
    
    // 处理过滤器以 # 结尾的情况
    if (*f == '#' && *(f + 1) == '\0') {
        return true;
    }
    
    // 处理过滤器以 + 结尾的情况
    if (*f == '+' && *(f + 1) == '\0') {
        // + 匹配剩余部分
        return true;
    }
    
    return false;
}



// ==================== 客户端管理（带缓冲区） ====================
static void remove_client_subscriptions(int client_fd) {
    if (subscriptions_mutex == NULL) return;
    
    if (xSemaphoreTake(subscriptions_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "获取订阅锁失败");
        return;
    }
    
    subscription_t *curr = subscriptions;
    subscription_t *prev = NULL;
    
    while (curr) {
        if (curr->client_fd == client_fd) {
            if (prev) {
                prev->next = curr->next;
            } else {
                subscriptions = curr->next;
            }
            subscription_t *to_free = curr;
            curr = curr->next;
            free(to_free);
            subscription_count--;
            ESP_LOGI(TAG, "移除客户端 fd=%d 的订阅", client_fd);
        } else {
            prev = curr;
            curr = curr->next;
        }
    }
    
    xSemaphoreGive(subscriptions_mutex);
}
 

// ==================== 添加客户端（分配堆内存） ====================
static mqtt_client_t* add_client(int fd) {
    portENTER_CRITICAL(&client_lock);
    
    mqtt_client_t *curr = client_list;
    while (curr) {
        if (curr->fd == fd) {
            curr->connected = true;
            curr->recv_buffer_len = 0;
            portEXIT_CRITICAL(&client_lock);
            return curr;
        }
        curr = curr->next;
    }
    
    mqtt_client_t *new_client = (mqtt_client_t *)calloc(1, sizeof(mqtt_client_t));
    if (new_client) {
        new_client->fd = fd;
        new_client->connected = true;
        new_client->recv_buffer_len = 0;
        new_client->recv_buffer_capacity = RECV_BUFFER_SIZE;
        // 在堆上分配接收缓冲区
        new_client->recv_buffer = (uint8_t *)malloc(RECV_BUFFER_SIZE);
        if (!new_client->recv_buffer) {
            ESP_LOGE(TAG, "分配接收缓冲区失败");
            free(new_client);
            portEXIT_CRITICAL(&client_lock);
            return NULL;
        }
        new_client->next = client_list;
        client_list = new_client;
    }
    
    portEXIT_CRITICAL(&client_lock);
    return new_client;
}

// ==================== 移除客户端（释放堆内存） ====================
static void remove_client(int fd) {
    portENTER_CRITICAL(&client_lock);
    
    mqtt_client_t *curr = client_list;
    mqtt_client_t *prev = NULL;
    
    while (curr) {
        if (curr->fd == fd) {
            if (prev) {
                prev->next = curr->next;
            } else {
                client_list = curr->next;
            }
            // 释放接收缓冲区
            if (curr->recv_buffer) {
                free(curr->recv_buffer);
            }
            free(curr);
            break;
        }
        prev = curr;
        curr = curr->next;
    }
    
    portEXIT_CRITICAL(&client_lock);
    remove_client_subscriptions(fd);
}

// ==================== 读取完整 MQTT 报文（使用堆缓冲区） ====================
static int read_mqtt_packet(mqtt_client_t *client, uint8_t *out_buffer, int max_len) {
    int fd = client->fd;
    uint8_t *buffer = client->recv_buffer;
    int *buf_len = &client->recv_buffer_len;
    int capacity = client->recv_buffer_capacity;
    
    // 从 socket 读取数据
    uint8_t tmp[BUFFER_SIZE];
    ssize_t n = recv(fd, tmp, sizeof(tmp), 0);
    
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;  // 无数据，稍后重试
        }
        return -1;  // 错误
    }
    if (n == 0) {
        return -1;  // 连接关闭
    }
    
    // 追加到缓冲区
    if (*buf_len + n > capacity) {
        ESP_LOGW(TAG, "缓冲区溢出，清空 fd=%d", fd);
        *buf_len = 0;
        return 0;
    }
    memcpy(buffer + *buf_len, tmp, n);
    *buf_len += n;
    
    // 尝试解析完整报文
    int pos = 0;
    while (pos < *buf_len) {
        if (pos + 1 > *buf_len) break;
        
        int rem_len = 0;
        int multiplier = 1;
        int rem_pos = pos + 1;
        uint8_t byte;
        bool complete = false;
        
        while (rem_pos < *buf_len) {
            byte = buffer[rem_pos];
            rem_len += (byte & 0x7F) * multiplier;
            multiplier *= 128;
            rem_pos++;
            if ((byte & 0x80) == 0) {
                complete = true;
                break;
            }
            if (multiplier > 128*128*128) break;
        }
        
        if (!complete) break;
        
        int total_len = rem_pos + rem_len;
        if (total_len > *buf_len - pos) break;
        
        if (total_len <= max_len) {
            memcpy(out_buffer, buffer + pos, total_len);
            int remaining = *buf_len - (pos + total_len);
            if (remaining > 0) {
                memmove(buffer, buffer + pos + total_len, remaining);
            }
            *buf_len = remaining;
            return total_len;
        } else {
            ESP_LOGE(TAG, "报文太大: %d > %d", total_len, max_len);
            pos += total_len;
        }
    }
    
    return 0;
}
// ==================== PUBLISH 处理函数 ====================
static void handle_publish_with_base64_check(int client_fd, uint8_t *buffer, int recv_len) {
    char *topic = parse_publish_topic(buffer, recv_len, NULL);
    ESP_LOGI(TAG, "🔥 收到PUBLISH报文 - 发送者fd=%d, 主题='%s', 长度=%d", 
             client_fd, topic ? topic : "unknown", recv_len);
    
    if (topic) {
        // 转发数据
        forward_to_subscribers(client_fd, topic, buffer, recv_len);
        free(topic);
    } else {
        forward_to_subscribers(client_fd, "unknown", buffer, recv_len);
    }
    
    // 发送 PUBACK (QoS 1)
    uint8_t qos = (buffer[0] & 0x06) >> 1;
    if (qos == 1) {
        uint16_t packet_id = 0;
        int pos = 1;
        while (pos < recv_len && (buffer[pos] & 0x80)) pos++;
        pos++;
        if (pos + 2 <= recv_len) {
            uint16_t t_len = (buffer[pos] << 8) | buffer[pos + 1];
            pos += 2 + t_len;
        }
        if (pos + 2 <= recv_len) {
            packet_id = (buffer[pos] << 8) | buffer[pos + 1];
        }
        uint8_t puback[] = {MQTT_PUBACK, 2, (uint8_t)((packet_id >> 8) & 0xFF), (uint8_t)(packet_id & 0xFF)};
        send(client_fd, puback, sizeof(puback), 0);
    }
}

 
// ==================== 修改后的 mqtt_client_handler ====================

// ==================== 修改后的 mqtt_client_handler ====================
static void mqtt_client_handler(void *arg) {
    int client_fd = (int)(uintptr_t)arg;
    
    mqtt_client_t *client = add_client(client_fd);
    if (!client) {
        close(client_fd);
        vTaskDelete(NULL);
        return;
    }
    
    uint8_t *buffer = (uint8_t *)malloc(BUFFER_SIZE);
    if (!buffer) {
        ESP_LOGE(TAG, "分配处理缓冲区失败");
        remove_client(client_fd);
        close(client_fd);
        vTaskDelete(NULL);
        return;
    }
    
    int recv_len;
    ESP_LOGI(TAG, "🔌 新客户端 fd=%d", client_fd);
    
    // 设置 socket 为非阻塞
    int flags = fcntl(client_fd, F_GETFL, 0);
    fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
    
    // ========== 添加：等待数据到达 ==========
    int wait_count = 0;
    while (wait_count < 50) {  // 等待最多 5 秒
        recv_len = read_mqtt_packet(client, buffer, BUFFER_SIZE);
        if (recv_len > 0) break;
        if (recv_len < 0) {
            ESP_LOGI(TAG, "读取错误");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_count++;
    }
    // =======================================
    
    ESP_LOGI(TAG, "读取 CONNECT 报文结果: recv_len=%d", recv_len);
    
    if (recv_len <= 0) {
        // ========== 添加：打印接收到的原始数据（如果有） ==========
        if (client->recv_buffer_len > 0) {
            ESP_LOGI(TAG, "缓冲区有 %d 字节数据:", client->recv_buffer_len);
            ESP_LOG_BUFFER_HEX("RAW", client->recv_buffer, client->recv_buffer_len > 64 ? 64 : client->recv_buffer_len);
        }
        // ======================================================
        
        if (recv_len == 0) {
            ESP_LOGI(TAG, "客户端 fd=%d 未发送CONNECT就断开", client_fd);
        } else {
            ESP_LOGI(TAG, "客户端 fd=%d 读取错误, errno=%d", client_fd, errno);
        }
        free(buffer);
        remove_client(client_fd);
        close(client_fd);
        vTaskDelete(NULL);
        return;
    }
    
    // 打印接收到的 CONNECT 报文前 64 字节
    ESP_LOGI(TAG, "收到 CONNECT 报文 (长度=%d):", recv_len);
    ESP_LOG_BUFFER_HEX("CONNECT", buffer, recv_len > 64 ? 64 : recv_len);
    
    // 解析 CONNECT 报文
    int parse_result = mqtt_parse_connect(buffer, recv_len);
    ESP_LOGI(TAG, "CONNECT 解析结果: %s", parse_result == 0 ? "成功" : "失败");
    
    if (parse_result == 0) {
        // 发送 CONNACK
        mqtt_send_connack(client_fd, 0);
        ESP_LOGI(TAG, "✅ 认证成功 fd=%d, CONNACK已发送", client_fd);
        
        // 等待一下确保 CONNACK 被发送
        vTaskDelay(pdMS_TO_TICKS(100));
    } else {
        ESP_LOGE(TAG, "❌ 认证失败 fd=%d, 发送拒绝", client_fd);
        mqtt_send_connack(client_fd, 1);
        free(buffer);
        remove_client(client_fd);
        close(client_fd);
        vTaskDelete(NULL);
        return;
    }
    
    // 主循环
    while (1) {
        recv_len = read_mqtt_packet(client, buffer, BUFFER_SIZE);
        
        if (recv_len < 0) {
            ESP_LOGI(TAG, "客户端 fd=%d 断开, errno=%d", client_fd, errno);
            break;
        }
        
        if (recv_len == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        uint8_t msg_type = buffer[0] & 0xF0;
        ESP_LOGI(TAG, "收到报文类型: 0x%02X, 长度=%d", msg_type, recv_len);
        
        switch (msg_type) {
            case MQTT_PUBLISH:
                ESP_LOGI(TAG, "处理 PUBLISH 报文");
                handle_publish_with_base64_check(client_fd, buffer, recv_len);
                break;
                
            case MQTT_SUBSCRIBE:
                ESP_LOGI(TAG, "处理 SUBSCRIBE 报文");
                handle_subscribe(client_fd, buffer, recv_len);
                break;
                
            case MQTT_PINGREQ: {
                uint8_t pingresp[] = {MQTT_PINGRESP, 0};
                send(client_fd, pingresp, sizeof(pingresp), 0);
                ESP_LOGI(TAG, "发送 PINGRESP");
                break;
            }
                
            case MQTT_DISCONNECT:
                ESP_LOGI(TAG, "收到 DISCONNECT");
                goto cleanup;
                
            default:
                ESP_LOGW(TAG, "未处理报文类型: 0x%02X", msg_type);
                break;
        }
    }

cleanup:
    free(buffer);  // 释放处理缓冲区
    remove_client(client_fd);
    close(client_fd);
    ESP_LOGI(TAG, "关闭 fd=%d", client_fd);
    vTaskDelete(NULL);
} 

// ==================== MQTT服务器任务 ====================
static void mqtt_server_task(void *arg) {
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    int opt = 1;

    server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (server_fd < 0) {
        ESP_LOGE(TAG, "创建socket失败");
        vTaskDelete(NULL);
        return;
    }

    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(MQTT_SERVER_PORT);
    if (bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "绑定端口失败");
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }

    if (listen(server_fd, MAX_CLIENTS) < 0) {
        ESP_LOGE(TAG, "监听失败");
        close(server_fd);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "✅ MQTT服务器启动，监听端口 %d", MQTT_SERVER_PORT);

    while (1) {
        int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_addr_len);
        if (client_fd < 0) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "🔌 新客户端: %s:%d", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        // 使用默认栈大小（因为大缓冲区在堆上）
        if (xTaskCreate(mqtt_client_handler, "mqtt_client", MQTT_CLIENT_STACK_SIZE , 
                        (void*)(uintptr_t)client_fd, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG, "创建客户端任务失败");
            close(client_fd);
        }
    }
}


// ==================== 订阅管理 ====================
static void add_subscription(int client_fd, const char *topic, int qos) {
    if (subscriptions_mutex == NULL) return;
    
    if (xSemaphoreTake(subscriptions_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "获取订阅锁失败");
        return;
    }
    
    if (subscription_count >= MAX_SUBSCRIPTIONS) {
        ESP_LOGE(TAG, "订阅数量达到上限 %d", MAX_SUBSCRIPTIONS);
        xSemaphoreGive(subscriptions_mutex);
        return;
    }
    
    subscription_t *curr = subscriptions;
    while (curr) {
        if (curr->client_fd == client_fd && strcmp(curr->topic, topic) == 0) {
            curr->qos = qos;
            xSemaphoreGive(subscriptions_mutex);
            return;
        }
        curr = curr->next;
    }
    
    subscription_t *new_sub = (subscription_t *)malloc(sizeof(subscription_t));
    if (new_sub) {
        memset(new_sub, 0, sizeof(subscription_t));
        new_sub->client_fd = client_fd;
        strncpy(new_sub->topic, topic, MAX_TOPIC_LEN - 1);
        new_sub->topic[MAX_TOPIC_LEN - 1] = '\0';
        new_sub->qos = qos;
        new_sub->next = subscriptions;
        subscriptions = new_sub;
        subscription_count++;
        ESP_LOGI(TAG, "✅ 添加订阅: fd=%d, topic='%s', QoS=%d", client_fd, topic, qos);
    }
    
    xSemaphoreGive(subscriptions_mutex);
}


static void forward_to_subscribers(int sender_fd, const char *topic, uint8_t *data, int len) {
    if (subscriptions_mutex == NULL || topic == NULL) return;
    
    int forwarded = 0;
    
    if (xSemaphoreTake(subscriptions_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "获取订阅锁失败");
        return;
    }
    
    // 打印当前所有订阅
    ESP_LOGI(TAG, "=== 当前订阅列表 ===");
    subscription_t *curr = subscriptions;  // 使用不同的变量名
    while (curr) {
        ESP_LOGI(TAG, "  订阅: fd=%d, topic='%s'", curr->client_fd, curr->topic);
        curr = curr->next;
    }
    ESP_LOGI(TAG, "==================");
    
    // 打印收到的主题
    ESP_LOGI(TAG, "收到 PUBLISH 主题: '%s'", topic);
    
    // 重新开始遍历 - 使用新的变量
    curr = subscriptions;
    while (curr) {
        if (curr->client_fd != sender_fd && curr->client_fd > 0) {
            bool match = mqtt_topic_match(curr->topic, topic);
            ESP_LOGI(TAG, "检查 fd=%d, sub='%s', topic='%s', match=%d", 
                     curr->client_fd, curr->topic, topic, match);
            if (match) {
                ssize_t sent = send(curr->client_fd, data, len, 0);
                if (sent == len) {
                    forwarded++;
                    ESP_LOGI(TAG, "  ✅ 转发到 fd=%d (%d bytes)", curr->client_fd, len);
                } else {
                    ESP_LOGW(TAG, "  ❌ 转发失败 fd=%d, sent=%d, len=%d", curr->client_fd, sent, len);
                }
            }
        }
        curr = curr->next;
    }
    
    xSemaphoreGive(subscriptions_mutex);
    
    if (forwarded > 0) {
        ESP_LOGI(TAG, "✅ 转发 %d 个订阅者", forwarded);
    } else {
        ESP_LOGW(TAG, "⚠️ 没有订阅者匹配主题 '%s'", topic);
    }
}

// ==================== MQTT报文解析 ====================
static char* parse_publish_topic(uint8_t *buffer, int len, int *topic_len_out) {
    if (len < 2) return NULL;
    
    int pos = 1;
    uint8_t encoded_byte;
    
    do {
        if (pos >= len) return NULL;
        encoded_byte = buffer[pos];
        pos++;
    } while ((encoded_byte & 0x80) != 0);
    
    if (pos + 2 > len) return NULL;
    uint16_t topic_len = (buffer[pos] << 8) | buffer[pos + 1];
    pos += 2;
    
    if (pos + topic_len > len) return NULL;
    
    char *topic = (char *)malloc(topic_len + 1);
    if (topic) {
        memcpy(topic, buffer + pos, topic_len);
        topic[topic_len] = '\0';
        if (topic_len_out) *topic_len_out = topic_len;
    }
    return topic;
}

static void handle_subscribe(int client_fd, uint8_t *buffer, int len) {
    int pos = 1;
    int remaining_length = 0;
    int multiplier = 1;
    uint8_t encoded_byte;
    
    do {
        if (pos >= len) return;
        encoded_byte = buffer[pos];
        remaining_length += (encoded_byte & 0x7F) * multiplier;
        multiplier *= 128;
        pos++;
    } while ((encoded_byte & 0x80) != 0);
    
    if (pos + 2 > len) return;
    uint16_t packet_id = (buffer[pos] << 8) | buffer[pos + 1];
    pos += 2;
    
    int payload_len = len - pos;
    int sub_pos = 0;
    uint8_t return_codes[10] = {0};
    int return_count = 0;
    
    while (sub_pos < payload_len && return_count < 10) {
        if (pos + sub_pos + 2 > len) break;
        uint16_t topic_len = (buffer[pos + sub_pos] << 8) | buffer[pos + sub_pos + 1];
        sub_pos += 2;
        
        if (pos + sub_pos + topic_len > len) break;
        char topic[MAX_TOPIC_LEN] = {0};
        int copy_len = topic_len < MAX_TOPIC_LEN - 1 ? topic_len : MAX_TOPIC_LEN - 1;
        memcpy(topic, buffer + pos + sub_pos, copy_len);
        sub_pos += topic_len;
        
        if (pos + sub_pos >= len) break;
        uint8_t qos = buffer[pos + sub_pos] & 0x03;
        sub_pos++;
        
        ESP_LOGI(TAG, "📝 订阅: '%s' QoS=%d", topic, qos);
        add_subscription(client_fd, topic, qos);
        return_codes[return_count++] = qos;
    }
    
    uint8_t suback[100];
    suback[0] = MQTT_SUBACK;
    suback[1] = 2 + return_count;
    suback[2] = (uint8_t)((packet_id >> 8) & 0xFF);
    suback[3] = (uint8_t)(packet_id & 0xFF);
    memcpy(suback + 4, return_codes, return_count);
    
    send(client_fd, suback, 4 + return_count, 0);
    ESP_LOGI(TAG, "✅ SUBACK发送成功");
}

// ==================== MQTT报文解析 ====================
static int mqtt_parse_remaining_length(uint8_t *buf, int len, int *offset) {
    int multiplier = 1;
    int remaining_length = 0;
    uint8_t byte;
    *offset = 1;
    
    do {
        if (*offset >= len) return -1;
        byte = buf[*offset];
        remaining_length += (byte & 0x7F) * multiplier;
        multiplier *= 128;
        (*offset)++;
        if (multiplier > 128*128*128) return -1;
    } while ((byte & 0x80) != 0);
    
    return remaining_length;
}

static int mqtt_parse_connect(uint8_t *buf, int len) {
    if (len < 2) return -1;
    if ((buf[0] & 0xF0) != MQTT_CONNECT) return -1;

    int offset;
    int remaining_length = mqtt_parse_remaining_length(buf, len, &offset);
    if (remaining_length < 0 || (offset + remaining_length) > len) return -1;

    if (offset + 2 > len) return -1;
    uint16_t proto_len = (buf[offset] << 8) | buf[offset+1];
    offset += 2;

    if (offset + proto_len + 1 > len) return -1;

    bool is_mqtt311 = (proto_len == 4 && memcmp(buf+offset, "MQTT", 4) == 0 && buf[offset+4] == 4);
    
    if (is_mqtt311) {
        ESP_LOGI(TAG, "收到MQTT CONNECT报文");
        return 0;
    }
    return -1;
}

static void mqtt_send_connack(int client_fd, uint8_t return_code) {
    uint8_t connack[] = {MQTT_CONNACK, 2, 0, return_code};
    send(client_fd, connack, sizeof(connack), 0);
}

// ==================== Base64 检查函数 ====================
static bool is_valid_base64_char(char c) {
    return ((c >= 'A' && c <= 'Z') || 
            (c >= 'a' && c <= 'z') || 
            (c >= '0' && c <= '9') || 
            c == '+' || c == '/' || c == '=');
}






// ==================== 事件处理 ====================
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        static_ip_configured = false;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_wifi_connect();
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
        set_static_ip();
        wifi_connected = true;
    }
}

// ==================== 主函数 ====================
void run_mqtt(void) {
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    subscriptions_mutex = xSemaphoreCreateMutex();
    if (subscriptions_mutex == NULL) {
        ESP_LOGE(TAG, "创建订阅锁失败");
        return;
    }
    
    if (xTaskCreate(mqtt_server_task, "mqtt_server", 16384, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "创建服务器任务失败");
    }
}