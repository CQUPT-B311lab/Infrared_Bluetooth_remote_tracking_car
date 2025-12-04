#ifndef __MOTOR_H__
#define __MOTOR_H__

int16_t GetSpeed(TIM_HandleTypeDef *tim);
void SetSpeed(TIM_HandleTypeDef *tim, int16_t speed);

#endif