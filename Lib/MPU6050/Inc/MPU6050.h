#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"
#include "math.h"

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75

typedef struct {
  float roll;  // 横滚角：向左为负，向右为正
  float pitch; // 俯仰角
  float yaw;   // 航向角：0~-179°向左，0~180°向右
} MPU6050_Angle;

extern MPU6050_Angle mpu_angle;

uint8_t MPU6050_Init(void);
void MPU6050_Read_Accel(short *accel);
void MPU6050_Read_Gyro(short *gyro);
void MPU6050_Read_All(short *accel, short *gyro);
void MPU6050_Calculate_Angle(void);

#endif
