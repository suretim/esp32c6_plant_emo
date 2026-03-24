# plant_sensor_esp32/main/CMakeLists.txt
idf_component_register(SRCS     
                        "main_as7341.c" 
                        "comprehensive_analysis.cc"  
                        "stepper_motor.c"
                        "mqtt_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES 
                        esp_http_server 
                        nvs_flash 
                        freertos
                        driver
                        esp_timer
                        esp_netif
                        esp_event
                        esp_wifi
                        lwip
                    )

# 禁用警告
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format)