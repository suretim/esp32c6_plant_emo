# ESP32 植物传感器数据采集系统

## 功能特点

- 12通道光谱数据采集
- 7个环境参数监测
- 3个生理参数计算
- HTTP API接口
- UART实时输出
- 循环缓冲区存储

## 硬件需求

- ESP32开发板
- 12个光谱传感器（或分时复用）
- 温湿度传感器（如DHT22）
- 土壤湿度传感器
- 光照传感器
- CO2传感器（可选）
- pH传感器（可选）

## 引脚连接

| 功能 | GPIO | ADC通道 |
|------|------|---------|
| 红光 (Red) | GPIO34 | ADC2_CH6 |
| 远红光 (Far Red) | GPIO35 | ADC2_CH7 |
| 蓝光 (Blue) | GPIO33 | ADC2_CH5 |
| 白光 (White) | GPIO32 | ADC2_CH4 |
| 通道5 | GPIO4 | ADC2_CH0 |
| 通道6 | GPIO5 | ADC2_CH1 |
| 通道7 | GPIO6 | ADC2_CH2 |
| 通道8 | GPIO7 | ADC2_CH3 |

## 编译和烧录

```bash
# 设置目标芯片
idf.py set-target esp32

# 配置
idf.py menuconfig

# 编译
idf.py build

# 烧录
idf.py -p /dev/ttyUSB0 flash

# 监视串口输出
idf.py -p /dev/ttyUSB0 monitor# esp32c6_plant_emo
