#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

#define PLUS_PER_SEC 224 // 定时器每积累224表示外轮旋转一圈

#define R_TIRE 32.5e-3f // 轮胎直径（米）

int16_t GetSpeed(TIM_HandleTypeDef *tim);

void SetSpeed(TIM_HandleTypeDef *tim, int16_t speed);

void Motor_Init(void);

#endif