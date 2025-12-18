#ifndef __TRACE_SENSOR_H
#define __TRACE_SENSOR_H

#include "main.h"

#define TRACE_CHANNELS 2

typedef struct {
  uint16_t raw_value;      // 原始ADC值
  uint16_t filtered_value; // 滤波后的值
  uint8_t is_black;        // 是否检测到黑线
  float voltage;           // 电压值(3.3V参考)
} SensorData_t;

// 函数声明
void TraceSensor_Init(void);
void TraceSensor_Update(void);
void TraceSensor_Start(void);
void TraceSensor_Stop(void);
uint16_t TraceSensor_GetRawValue(uint8_t channel);
uint16_t TraceSensor_GetFilteredValue(uint8_t channel);
uint8_t TraceSensor_IsBlack(uint8_t channel);
uint8_t TraceSensor_GetLineState(void);
void TraceSensor_Calibrate(void);

#endif