#ifndef MPU6500_H
#define MPU6500_H

#include "main.h"

// 定义MPU6500的寄存器地址
#define MPU6500_WHO_AM_I_REG 0x75
#define MPU6500_PWR_MGMT_1_REG 0x6B
#define MPU6500_SMPLRT_DIV_REG 0x19
#define MPU6500_CONFIG_REG 0x1A
#define MPU6500_GYRO_CONFIG_REG 0x1B
#define MPU6500_ACCEL_CONFIG_REG 0x1C
#define MPU6500_ACCEL_XOUT_H_REG 0x3B
#define MPU6500_TEMP_OUT_H_REG 0x41
#define MPU6500_GYRO_XOUT_H_REG 0x43

// 定义MPU6500的设备地址
#define MPU6500_ADDRESS 0x68
// 角度结构体

typedef struct {
  float roll;  // 横滚角 (x轴)
  float pitch; // 俯仰角 (y轴)
  float yaw;   // 偏航角 (z轴)
  float temp;  // 添加温度字段
} MPU6500_Angle;
// SPI操作函数声明
void MPU6500_Init(void);
void MPU6500_Write_Byte(uint8_t reg, uint8_t data);
uint8_t MPU6500_Read_Byte(uint8_t reg);
void MPU6500_Read_Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                       int16_t *gy, int16_t *gz, int16_t *temp);
void MPU6500_Compute_Angles(MPU6500_Angle *angle);
#endif // MPU6500_H