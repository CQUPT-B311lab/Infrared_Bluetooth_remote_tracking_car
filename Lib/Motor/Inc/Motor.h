#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

int16_t GetSpeed(TIM_HandleTypeDef *tim);

void SetSpeed(TIM_HandleTypeDef *tim, int16_t speed);

void Motor_Init(void);

#endif