#include "trace_sensor.h"
#include "main.h"

volatile uint16_t adc_dma_buffer[TRACE_CHANNELS];

// 传感器数据
static SensorData_t sensors[TRACE_CHANNELS];

// 滤波缓冲区
#define FILTER_SIZE 5
static uint16_t filter_buffer[TRACE_CHANNELS][FILTER_SIZE] = {0};
static uint8_t filter_index = 0;

// 阈值
static uint16_t black_threshold = 500;  // 低于此值为黑线
static uint16_t white_threshold = 3000; // 高于此值为白色

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

// 初始化函数
void TraceSensor_Init(void) {
  // 初始化传感器数据结构
  for (int i = 0; i < TRACE_CHANNELS; i++) {
    sensors[i].raw_value = 0;
    sensors[i].filtered_value = 0;
    sensors[i].is_black = 0;
    sensors[i].voltage = 0.0f;
  }
}

// 启动传感器（在系统初始化完成后调用）
void TraceSensor_Start(void) {
  // 启动ADC DMA转换
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, TRACE_CHANNELS);
}

// 停止传感器
void TraceSensor_Stop(void) { HAL_ADC_Stop_DMA(&hadc1); }

// 更新传感器数据（应在主循环中定期调用）
void TraceSensor_Update(void) {
  for (int i = 0; i < TRACE_CHANNELS; i++) {
    // 1. 获取原始值（来自DMA缓冲区）
    sensors[i].raw_value = adc_dma_buffer[i];

    // 2. 更新滤波缓冲区
    filter_buffer[i][filter_index] = sensors[i].raw_value;

    // 3. 计算移动平均值（软件滤波）
    uint32_t sum = 0;
    for (int j = 0; j < FILTER_SIZE; j++) {
      sum += filter_buffer[i][j];
    }
    sensors[i].filtered_value = sum / FILTER_SIZE;

    // 4. 计算电压值（假设3.3V参考电压）
    sensors[i].voltage = (sensors[i].filtered_value * 3.3f) / 4095.0f;

    // 5. 判断是否在黑线上（使用迟滞比较防止抖动）
    if (sensors[i].filtered_value < black_threshold) {
      sensors[i].is_black = 1;
    } else if (sensors[i].filtered_value > white_threshold) {
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

  if (left && !right) {
    return 1; // 偏左，需要右转
  } else if (!left && right) {
    return 2; // 偏右，需要左转
  } else if (left && right) {
    return 3; // 十字路口或停车线
  } else {
    return 0; // 直行状态
  }
}

// 校准函数（手动将传感器放在白线和黑线上）
void TraceSensor_Calibrate(void) {
  // 这里实现校准逻辑
  // 在实际使用时，通过串口命令触发校准
}