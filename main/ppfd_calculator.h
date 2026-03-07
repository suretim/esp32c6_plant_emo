// ppfd_calculator.h
#ifndef PPFD_CALCULATOR_H
#define PPFD_CALCULATOR_H

#include <stdint.h>

typedef struct {
    float weight_par[8];      // PAR权重
    float weight_plant[8];     // 植物响应权重
    float calibration_k;       // 校准系数
    float reference_ppfd;      // 参考PPFD值（用于校准）
} ppfd_config_t;

// 初始化PPFD计算器
void ppfd_init(ppfd_config_t *config);

// 计算PPFD
void ppfd_calculate(ppfd_config_t *config );

// 自动校准（需要标准PPFD计）
void ppfd_calibrate(ppfd_config_t *config, uint16_t *channels, float standard_ppfd);

#endif