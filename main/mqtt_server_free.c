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
// #define WIFI_SSID       "ESP32CAM_AP3"
// #define STATIC_IP_ADDR  "192.168.4.101" 
// #define GATEWAY_IP      "192.168.4.1"
// #define WIFI_PASS       "88888888"   
// #define NETMASK         "255.255.255.0" 

// MQTT 配置
#define MQTT_SERVER_PORT 1883
#define MAX_CLIENTS 10
#define BUFFER_SIZE 4096
#define MAX_SUBSCRIPTIONS 100
#define MAX_TOPIC_LEN 128

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

// ==================== 客户端管理 ====================
typedef struct mqtt_client {
    int fd;
    bool connected;
    struct mqtt_client *next;
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
static void add_client(int fd);
static void remove_client(int fd);
static void add_subscription(int client_fd, const char *topic, int qos);
static void remove_client_subscriptions(int client_fd);
static void forward_to_subscribers(int sender_fd, const char *topic, uint8_t *data, int len);
static void handle_subscribe(int client_fd, uint8_t *buffer, int len);
static char* parse_publish_topic(uint8_t *buffer, int len, int *topic_len_out);

// ==================== MQTT主题匹配 ====================
static bool mqtt_topic_match(const char *filter, const char *topic) {
    if (filter == NULL || topic == NULL) {
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
    
    if (*f == '#' && *(f + 1) == '\0') {
        return true;
    }
    
    if (strcmp(filter, "+") == 0 && *t == '\0') {
        return true;
    }
    
    while (*f == '/') f++;
    
    return (*f == '\0' && *t == '\0');
}

// ==================== 客户端管理 ====================
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

static void add_client(int fd) {
    portENTER_CRITICAL(&client_lock);
    
    mqtt_client_t *curr = client_list;
    while (curr) {
        if (curr->fd == fd) {
            curr->connected = true;
            portEXIT_CRITICAL(&client_lock);
            return;
        }
        curr = curr->next;
    }
    
    mqtt_client_t *new_client = (mqtt_client_t *)malloc(sizeof(mqtt_client_t));
    if (new_client) {
        memset(new_client, 0, sizeof(mqtt_client_t));
        new_client->fd = fd;
        new_client->connected = true;
        new_client->next = client_list;
        client_list = new_client;
    }
    
    portEXIT_CRITICAL(&client_lock);
}

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
            free(curr);
            break;
        }
        prev = curr;
        curr = curr->next;
    }
    
    portEXIT_CRITICAL(&client_lock);
    remove_client_subscriptions(fd);
}

// ==================== 订阅管理 ====================
static void add_subscription(int client_fd, const char *topic, int qos) {
    if (subscriptions_mutex == NULL) {
        ESP_LOGE(TAG, "订阅锁未初始化");
        return;
    }
    
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

// ==================== 转发函数 ====================

 static void forward_to_subscribers(int sender_fd, const char *topic, uint8_t *data, int len) {
    if (subscriptions_mutex == NULL || topic == NULL) return;
    
    // 验证这是一个完整的 MQTT 报文
    if (len < 2) {
        ESP_LOGW(TAG, "数据太短，跳过转发");
        return;
    }
    
    // 解析报文长度
    int pos = 1;
    int remaining = 0;
    int multiplier = 1;
    uint8_t byte;
    
    do {
        if (pos >= len) {
            ESP_LOGW(TAG, "报文不完整，跳过转发");
            return;
        }
        byte = data[pos];
        remaining += (byte & 0x7F) * multiplier;
        multiplier *= 128;
        pos++;
    } while ((byte & 0x80) != 0);
    
    int expected_len = pos + remaining;
    if (expected_len != len) {
        ESP_LOGW(TAG, "报文长度不匹配: 期望 %d, 实际 %d, 可能是多个报文合并", expected_len, len);
        // 只转发第一个完整报文
        len = expected_len;
    }
    
    int forwarded = 0;
    
    if (xSemaphoreTake(subscriptions_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "获取订阅锁失败");
        return;
    }
    
    subscription_t *sub = subscriptions;
    while (sub) {
        if (sub->client_fd != sender_fd && sub->client_fd > 0) {
            bool match = mqtt_topic_match(sub->topic, topic);
            if (match) {
                ssize_t sent = send(sub->client_fd, data, len, 0);
                if (sent == len) {
                    forwarded++;
                    ESP_LOGI(TAG, "  ✅ 转发到 fd=%d (%d bytes)", sub->client_fd, len);
                } else {
                    ESP_LOGW(TAG, "  ❌ 转发失败 fd=%d, sent=%d, len=%d", sub->client_fd, sent, len);
                }
            }
        }
        sub = sub->next;
    }
    
    xSemaphoreGive(subscriptions_mutex);
    
    if (forwarded > 0) {
        ESP_LOGI(TAG, "✅ 转发 %d 个订阅者", forwarded);
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
#if 0
// ==================== WiFi和IP配置 ====================
static void set_static_ip(void) {
    if (static_ip_configured) return;
    if (wifi_sta_netif == NULL) return;

    esp_netif_dhcpc_stop(wifi_sta_netif);
    esp_netif_ip_info_t ip_info;
    inet_pton(AF_INET, STATIC_IP_ADDR, &ip_info.ip);
    inet_pton(AF_INET, GATEWAY_IP, &ip_info.gw);
    inet_pton(AF_INET, NETMASK, &ip_info.netmask);
    esp_netif_set_ip_info(wifi_sta_netif, &ip_info);
    
    static_ip_configured = true;
    ESP_LOGI(TAG, "静态IP配置成功: %s", STATIC_IP_ADDR);
}

static void wifi_init_sta(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id, instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "WiFi初始化完成");
}
#endif
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
// 检查字符是否为合法的 Base64 字符
static bool is_valid_base64_char(char c) {
    return ((c >= 'A' && c <= 'Z') || 
            (c >= 'a' && c <= 'z') || 
            (c >= '0' && c <= '9') || 
            c == '+' || c == '/' || c == '=');
}

// 清理 Base64 数据，移除非法字符
static void clean_base64_data(uint8_t *data, int *len) {
    if (!data || !len || *len <= 0) return;
    
    int write_pos = 0;
    int removed_count = 0;
    
    for (int i = 0; i < *len; i++) {
        char c = (char)data[i];
        if (is_valid_base64_char(c)) {
            data[write_pos++] = data[i];
        } else {
            removed_count++;
            // 记录非法字符位置
            if (removed_count <= 10) {
                ESP_LOGW(TAG, "发现非法Base64字符 at pos %d: 0x%02x ('%c')", i, data[i], isprint(c) ? c : '.');
            }
        }
    }
    
    if (removed_count > 0) {
        *len = write_pos;
        ESP_LOGW(TAG, "✅ 清理了 %d 个非法Base64字符，新长度: %d", removed_count, *len);
    }
}

// ==================== JSON 和 Base64 处理函数 ====================
// 从 JSON 中提取并清理 Base64 数据
static int extract_and_clean_base64_from_json(uint8_t *json_data, int json_len, uint8_t **out_base64, int *out_base64_len) {
    if (!json_data || json_len <= 0) return -1;
    
    // 查找 "weights_b64" 字段
    const char *search_key = "\"weights_b64\":\"";
    int key_len = strlen(search_key);
    
    char *json_str = (char *)json_data;
    char *start = strstr(json_str, search_key);
    if (!start) {
        ESP_LOGE(TAG, "未找到 weights_b64 字段");
        return -1;
    }
    
    start += key_len;
    char *end = strchr(start, '"');
    if (!end) {
        ESP_LOGE(TAG, "未找到 Base64 数据的结束引号");
        return -1;
    }
    
    int base64_len = end - start;
    if (base64_len <= 0) {
        ESP_LOGE(TAG, "Base64 数据长度为0");
        return -1;
    }
    
    ESP_LOGI(TAG, "找到 Base64 数据，原始长度: %d", base64_len);
    
    // 分配内存并复制 Base64 数据
    *out_base64 = (uint8_t *)malloc(base64_len + 1);
    if (!*out_base64) {
        ESP_LOGE(TAG, "分配内存失败");
        return -1;
    }
    
    memcpy(*out_base64, start, base64_len);
    (*out_base64)[base64_len] = '\0';
    
    // 检查并清理非法字符
    clean_base64_data(*out_base64, &base64_len);
    *out_base64_len = base64_len;
    
    return 0;
}

// ==================== 修改 PUBLISH 处理函数 ====================


// ==================== 修改 handle_publish_with_base64_check ====================

static void handle_publish_with_base64_check(int client_fd, uint8_t *buffer, int recv_len) {
    char *topic = parse_publish_topic(buffer, recv_len, NULL);
    ESP_LOGI(TAG, "🔥 收到PUBLISH报文 - 发送者fd=%d, 主题='%s', 长度=%d", 
             client_fd, topic ? topic : "unknown", recv_len);
    
    if (topic) {
        // 只处理权重数据分片
        bool is_weights_topic = (strstr(topic, "weights") != NULL);
        bool is_ack_topic = (strstr(topic, "ack") != NULL);
        
        if (is_weights_topic && !is_ack_topic && strstr(topic, "frag") != NULL) {
            ESP_LOGI(TAG, "📦 权重数据主题，检查并清理 Base64");
            
            // 找到 payload 起始位置
            int payload_offset = 1;
            int remaining_len = 0;
            int multiplier = 1;
            while (payload_offset < recv_len && (buffer[payload_offset] & 0x80)) {
                remaining_len += (buffer[payload_offset] & 0x7F) * multiplier;
                multiplier *= 128;
                payload_offset++;
            }
            if (payload_offset < recv_len) {
                remaining_len += (buffer[payload_offset] & 0x7F) * multiplier;
                payload_offset++;
            }
            
            // 跳过主题长度和主题
            if (payload_offset + 2 <= recv_len) {
                uint16_t topic_len = (buffer[payload_offset] << 8) | buffer[payload_offset + 1];
                payload_offset += 2 + topic_len;
            }
            
            // 跳过 packet ID
            uint8_t qos = (buffer[0] & 0x06) >> 1;
            if (qos > 0 && payload_offset + 2 <= recv_len) {
                payload_offset += 2;
            }
            
            // 提取 payload
            if (payload_offset < recv_len) {
                int payload_len = recv_len - payload_offset;
                char *payload_str = (char *)(buffer + payload_offset);
                
                // 查找 weights_b64 字段
                char *b64_start = strstr(payload_str, "\"weights_b64\":\"");
                if (b64_start) {
                    b64_start += strlen("\"weights_b64\":\"");
                    char *b64_end = strchr(b64_start, '"');
                    if (b64_end) {
                        int b64_len = b64_end - b64_start;
                        ESP_LOGI(TAG, "原始 Base64 长度: %d", b64_len);
                        
                        // 清理非法字符
                        int write_pos = 0;
                        int invalid_count = 0;
                        
                        for (int i = 0; i < b64_len; i++) {
                            unsigned char c = (unsigned char)b64_start[i];
                            // 检查是否为合法 Base64 字符
                            if ((c >= 'A' && c <= 'Z') || 
                                (c >= 'a' && c <= 'z') || 
                                (c >= '0' && c <= '9') || 
                                c == '+' || c == '/' || c == '=') {
                                b64_start[write_pos++] = b64_start[i];
                            } else {
                                invalid_count++;
                                if (invalid_count <= 5) {
                                    ESP_LOGW(TAG, "  清理非法字符 at pos %d: 0x%02x", i, c);
                                }
                            }
                        }
                        
                        if (invalid_count > 0) {
                            // 更新 Base64 长度
                            int new_b64_len = write_pos;
                            ESP_LOGW(TAG, "✅ 清理了 %d 个非法字符，新长度: %d", invalid_count, new_b64_len);
                            
                            // 移动后续数据
                            int shift = b64_len - new_b64_len;
                            if (shift > 0) {
                                // 移动结束引号和后面的内容
                                memmove(b64_start + new_b64_len, 
                                        b64_end, 
                                        payload_len - (b64_end - payload_str));
                                // 更新总长度
                                recv_len -= shift;
                                payload_len -= shift;
                                // 更新 buffer 中的剩余长度字段
                                int remaining_pos = 1;
                                int new_remaining = recv_len - remaining_pos - 1;
                                int rem_bytes = 0;
                                uint8_t rem_buf[4];
                                do {
                                    uint8_t digit = new_remaining % 128;
                                    new_remaining /= 128;
                                    if (new_remaining > 0) digit |= 0x80;
                                    rem_buf[rem_bytes++] = digit;
                                } while (new_remaining > 0);
                                for (int i = 0; i < rem_bytes && 1 + i < recv_len; i++) {
                                    buffer[1 + i] = rem_buf[i];
                                }
                            }
                        } else {
                            ESP_LOGI(TAG, "✅ Base64 数据合法");
                        }
                    }
                }
            }
        }
        
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




// ==================== MQTT客户端处理（直接转发，不分片收集） ====================
static void mqtt_client_handler(void *arg) {
    int client_fd = (int)(uintptr_t)arg;
    uint8_t buffer[BUFFER_SIZE];
    ssize_t recv_len;

    ESP_LOGI(TAG, "新客户端 fd=%d", client_fd);
    add_client(client_fd);

    recv_len = recv(client_fd, buffer, BUFFER_SIZE, 0);
    if (recv_len <= 0) {
        ESP_LOGI(TAG, "客户端未发送CONNECT");
        remove_client(client_fd);
        close(client_fd);
        vTaskDelete(NULL);
        return;
    }

    if (mqtt_parse_connect(buffer, recv_len) == 0) {
        mqtt_send_connack(client_fd, 0);
        ESP_LOGI(TAG, "✅ 认证成功 fd=%d", client_fd);
    } else {
        mqtt_send_connack(client_fd, 1);
        remove_client(client_fd);
        close(client_fd);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        recv_len = recv(client_fd, buffer, BUFFER_SIZE, 0);
        if (recv_len <= 0) {
            ESP_LOGI(TAG, "客户端 fd=%d 断开", client_fd);
            break;
        }

        uint8_t msg_type = buffer[0] & 0xF0;

        switch (msg_type) {
            case MQTT_PUBLISH: {

                handle_publish_with_base64_check(client_fd, buffer, recv_len);
                // char *topic = parse_publish_topic(buffer, recv_len, NULL);
                // ESP_LOGI(TAG, "🔥 收到PUBLISH报文 - 发送者fd=%d, 主题='%s', 长度=%d", 
                //          client_fd, topic ? topic : "unknown", recv_len);
                
                // if (topic) {
                //     // 直接转发，不做分片收集 
                //     forward_to_subscribers(client_fd, topic, buffer, recv_len);
                //     free(topic);
                // }
                
                // // 发送 PUBACK (QoS 1)
                // uint8_t qos = (buffer[0] & 0x06) >> 1;
                // if (qos == 1) {
                //     uint16_t packet_id = 0;
                //     int pos = 1;
                //     while (pos < recv_len && (buffer[pos] & 0x80)) pos++;
                //     pos++;
                //     if (pos + 2 <= recv_len) {
                //         uint16_t t_len = (buffer[pos] << 8) | buffer[pos + 1];
                //         pos += 2 + t_len;
                //     }
                //     if (pos + 2 <= recv_len) {
                //         packet_id = (buffer[pos] << 8) | buffer[pos + 1];
                //     }
                //     uint8_t puback[] = {MQTT_PUBACK, 2, (uint8_t)((packet_id >> 8) & 0xFF), (uint8_t)(packet_id & 0xFF)};
                //     send(client_fd, puback, sizeof(puback), 0);
                // }
                break;
            }
                
            case MQTT_SUBSCRIBE:
                handle_subscribe(client_fd, buffer, recv_len);
                break;
                
            case MQTT_PINGREQ: {
                uint8_t pingresp[] = {MQTT_PINGRESP, 0};
                send(client_fd, pingresp, sizeof(pingresp), 0);
                break;
            }
                
            case MQTT_DISCONNECT:
                goto cleanup;
                
            default:
                ESP_LOGW(TAG, "未处理: 0x%02X", msg_type);
                break;
        }
    }

cleanup:
    remove_client(client_fd);
    close(client_fd);
    ESP_LOGI(TAG, "关闭 fd=%d", client_fd);
    vTaskDelete(NULL);
}

// ==================== MQTT服务器任务 ====================
static void mqtt_server_task(void *arg) {
    //while (!wifi_connected) {
    //    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //    printf("等待WiFi连接...\n");
    //}

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

    //ESP_LOGI(TAG, "✅ MQTT服务器启动: %s:%d", STATIC_IP_ADDR, MQTT_SERVER_PORT);

    while (1) {
        int client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_addr_len);
        if (client_fd < 0) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "🔌 新客户端: %s:%d", inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));

        if (xTaskCreate(mqtt_client_handler, "mqtt_client", 8192, (void*)(uintptr_t)client_fd, 5, NULL) != pdPASS) {
            close(client_fd);
        }
    }
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
    //wifi_init_sta();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    subscriptions_mutex = xSemaphoreCreateMutex();
    if (subscriptions_mutex == NULL) {
        ESP_LOGE(TAG, "创建订阅锁失败");
        return;
    }
    
    if (xTaskCreate(mqtt_server_task, "mqtt_server", 8192, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "创建服务器任务失败");
    }
}