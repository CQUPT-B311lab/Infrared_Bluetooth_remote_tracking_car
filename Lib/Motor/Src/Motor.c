
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include <stdint.h>

int16_t GetSpeed(TIM_HandleTypeDef *tim) {
  int16_t speed = (int16_t)__HAL_TIM_GET_COUNTER(tim);
  __HAL_TIM_SET_COUNTER(tim, 0);
  return speed;
}

void SetSpeed(TIM_HandleTypeDef *tim, int16_t speed) {
  if (speed > 0) {
    __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, speed);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  } else if (speed < 0) {
    __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, -speed);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
  } else { // 刹车
    __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  }
}