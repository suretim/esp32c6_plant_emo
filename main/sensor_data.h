//sensor_data.h
#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

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
#define AS7341_ENABLE_FDEN          0x40
#define AS7341_ENABLE_WEN           0x08

// SMUX命令
#define AS7341_SMUX_CMD_WRITE       0x00
#define AS7341_SMUX_CMD_READ        0x20
#define AS7341_SMUX_CMD_EXEC        0x10

// 预期芯片ID
#define AS7341_EXPECTED_ID          0x24

// PPFD配置结构体
typedef struct {
    float weight_par[8];      // PAR权重
    float weight_plant[8];     // 植物响应权重
    float calibration_k;       // 校准系数
    float reference_ppfd;      // 参考PPFD值
} ppfd_config_t;

// 数据结构 - AS7341的11通道
typedef struct {
    uint32_t cursor;
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
    float ci;      // 叶绿素指数
    float ppfd_par;      // 简单PAR估算
    float ppfd_plant;    // 植物响应PPFD
} sensor_data_t;
