#include "trace_sensor.h"
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>

// ADC通道  寻迹模块通道编号（0：左，8：右，以前进方向为参考）
//   6         6
//   7         8
//   8         3
//   9         1

volatile uint16_t adc_dma_buffer[TRACE_CHANNELS];

// 传感器数据
static SensorData_t sensors[TRACE_CHANNELS];

// 滤波缓冲区
#define FILTER_SIZE 5
static uint16_t filter_buffer[TRACE_CHANNELS][FILTER_SIZE] = {0};
static uint8_t filter_index = 0;

// 阈值
static uint16_t black_threshold = 3800; // 高于此值为黑线
static uint16_t white_threshold = 3700; // 低于此值为白色

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

uint8_t trace_flag = 0;
uint8_t signed_out = 0;

// 初始化函数
void TraceSensor_Init(void) {
  // 初始化传感器数据结构
  for (int i = 0; i < TRACE_CHANNELS; i++) {
    sensors[i].raw_value = 0;
    sensors[i].filtered_value = 0;
    sensors[i].is_black = 0;
  }
}

// 启动传感器（在系统初始化完成后调用）
void TraceSensor_Start(void) {
  // 启动ADC DMA转换
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, TRACE_CHANNELS);
}

// 停止传感器
void TraceSensor_Stop(void) { HAL_ADC_Stop_DMA(&hadc1); }

// 更新传感器数据
void TraceSensor_Update(void) {
  for (int i = 0; i < TRACE_CHANNELS; i++) {

    sensors[i].raw_value = adc_dma_buffer[i];

    // 更新滤波缓冲区
    filter_buffer[i][filter_index] = sensors[i].raw_value;

    // 计算移动平均值
    uint32_t sum = 0;
    for (int j = 0; j < FILTER_SIZE; j++) {
      sum += filter_buffer[i][j];
    }
    sensors[i].filtered_value = sum / FILTER_SIZE;

    // 判断是否在黑线上
    if (sensors[i].filtered_value > black_threshold) {
      sensors[i].is_black = 1;
    } else if (sensors[i].filtered_value < white_threshold) {
      sensors[i].is_black = 0;
    }
    // 在阈值之间的保持之前的状态
  }

  // 更新滤波索引
  filter_index = (filter_index + 1) % FILTER_SIZE;
}

// 获取原始ADC值
uint16_t TraceSensor_GetRawValue(uint8_t channel) {
  if (channel < TRACE_CHANNELS) {
    return sensors[channel].raw_value;
  }
  return 0;
}

// 获取滤波后的值
uint16_t TraceSensor_GetFilteredValue(uint8_t channel) {
  if (channel < TRACE_CHANNELS) {
    return sensors[channel].filtered_value;
  }
  return 0;
}

// 判断是否检测到黑线
uint8_t TraceSensor_IsBlack(uint8_t channel) {
  if (channel < TRACE_CHANNELS) {
    return sensors[channel].is_black;
  }
  return 0;
}

// 获取循迹状态（用于控制决策）
uint8_t TraceSensor_GetLineState(void) {
  uint8_t left = TraceSensor_IsBlack(0);  // CHANNEL_8
  uint8_t right = TraceSensor_IsBlack(1); // CHANNEL_9

  if (left && !right) { // 偏左
    if (!signed_out) {

      trace_flag++;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

      if (trace_flag >= 2) {
        signed_out = 1;
      }
    }
    return 1;
  } else if (!left && right) { // 偏右
    if (!signed_out) {

      trace_flag++;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

      if (trace_flag >= 2) {
        signed_out = 1;
      }
    }
    return 2;
  } else if (left && right) { // 十字路口或停车线
    if (!signed_out) {

      trace_flag++;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

      if (trace_flag >= 2) {
        signed_out = 1;
      }
    }
    return 3;
  } else { // 没线
    trace_flag = 0;
    signed_out = 0;
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
    return 0;
  }
}

// 校准函数（手动将传感器放在白线和黑线上）
void TraceSensor_Calibrate(void) {
  // 这里实现校准逻辑
  // 在实际使用时，通过串口命令触发校准
}