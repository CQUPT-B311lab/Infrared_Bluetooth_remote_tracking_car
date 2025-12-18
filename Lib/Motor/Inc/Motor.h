#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

#define PLUS_PER_CYC 830 // 定时器每积累820表示外轮旋转一圈

#define R_TIRE 0.0325f // 轮胎半径（米）

#define TICK_TO_V(x) ((x / PLUS_PER_CYC) * (2 * PI * R_TIRE) / 0.01)
#define V_TO_TICK(x) ((x * 0.01) / (2 * PI * R_TIRE) * PLUS_PER_CYC)

int16_t GetSpeed(TIM_HandleTypeDef *tim);

void SetSpeed(TIM_HandleTypeDef *tim, int16_t speed);

void Motor_Init(void);

#endif