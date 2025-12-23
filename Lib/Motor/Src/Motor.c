
#include "main.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include <stdint.h>

int16_t GetSpeed(TIM_HandleTypeDef *tim) {
  int16_t speed;
  // 左轮TIM2，右轮TIM3
  if (tim->Instance == TIM2) {
    speed = -(int16_t)__HAL_TIM_GET_COUNTER(tim);
  } else {
    speed = -(int16_t)__HAL_TIM_GET_COUNTER(tim);
  }
  __HAL_TIM_SET_COUNTER(tim, 0);
  return speed;
}

void Motor_Init(void) {
  extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;

  // 启动PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  // 启动编码器
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  // 使能驱动芯片
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
}

void SetSpeed(TIM_HandleTypeDef *tim, int16_t speed) {
  int16_t s = speed;
  if (s > (int16_t)PWM_MAX)
    s = (int16_t)PWM_MAX;
  if (s < -(int16_t)PWM_MAX)
    s = -(int16_t)PWM_MAX;

  uint16_t abs_speed = (s >= 0) ? (uint16_t)s : (uint16_t)(-s);

  if (tim->Instance == TIM1) {
    // 右轮 - 使用 AIN1/AIN2
    __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_1, abs_speed);

    if (s < 0) {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else if (s > 0) {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    } else {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }

  } else if (tim->Instance == TIM4) {
    // 左轮 - 使用 BIN1/BIN2，TIM4 用 CHANNEL_4
    __HAL_TIM_SET_COMPARE(tim, TIM_CHANNEL_4, abs_speed);

    if (s < 0) {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    } else if (s > 0) {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
  }
}