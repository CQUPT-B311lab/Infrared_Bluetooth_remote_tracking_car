#ifndef __MPU6500_H
#define __MPU6500_H

#include "stdint.h"

// MPU6500寄存器地址定义
// 根据PS-MPU-6500A-01规格书（参考MPU-6500寄存器映射文档）
#define MPU6500_SELF_TEST_X_GYRO 0x00  // 陀螺仪X轴自检
#define MPU6500_SELF_TEST_Y_GYRO 0x01  // 陀螺仪Y轴自检
#define MPU6500_SELF_TEST_Z_GYRO 0x02  // 陀螺仪Z轴自检
#define MPU6500_SELF_TEST_X_ACCEL 0x0D // 加速度计X轴自检
#define MPU6500_SELF_TEST_Y_ACCEL 0x0E // 加速度计Y轴自检
#define MPU6500_SELF_TEST_Z_ACCEL 0x0F // 加速度计Z轴自检

#define MPU6500_XG_OFFSET_H 0x13 // 陀螺仪X轴偏移量高8位
#define MPU6500_XG_OFFSET_L 0x14 // 陀螺仪X轴偏移量低8位
#define MPU6500_YG_OFFSET_H 0x15 // 陀螺仪Y轴偏移量高8位
#define MPU6500_YG_OFFSET_L 0x16 // 陀螺仪Y轴偏移量低8位
#define MPU6500_ZG_OFFSET_H 0x17 // 陀螺仪Z轴偏移量高8位
#define MPU6500_ZG_OFFSET_L 0x18 // 陀螺仪Z轴偏移量低8位

#define MPU6500_SMPLRT_DIV_REG 0x19    // 采样率分频器
#define MPU6500_CONFIG_REG 0x1A        // 配置寄存器（陀螺仪低通滤波器）
#define MPU6500_GYRO_CONFIG_REG 0x1B   // 陀螺仪配置寄存器
#define MPU6500_ACCEL_CONFIG_REG 0x1C  // 加速度计配置寄存器
#define MPU6500_ACCEL_CONFIG2_REG 0x1D // 加速度计配置寄存器2（低通滤波器）
#define MPU6500_LP_ACCEL_ODR 0x1E      // 低功耗加速度计输出数据率
#define MPU6500_WOM_THR 0x1F           // 运动唤醒阈值

// 加速度计数据输出寄存器
#define MPU6500_ACCEL_XOUT_H_REG 0x3B // 加速度计X轴数据高8位
#define MPU6500_ACCEL_XOUT_L_REG 0x3C // 加速度计X轴数据低8位
#define MPU6500_ACCEL_YOUT_H_REG 0x3D // 加速度计Y轴数据高8位
#define MPU6500_ACCEL_YOUT_L_REG 0x3E // 加速度计Y轴数据低8位
#define MPU6500_ACCEL_ZOUT_H_REG 0x3F // 加速度计Z轴数据高8位
#define MPU6500_ACCEL_ZOUT_L_REG 0x40 // 加速度计Z轴数据低8位

// 温度数据输出寄存器
#define MPU6500_TEMP_OUT_H_REG 0x41 // 温度数据高8位
#define MPU6500_TEMP_OUT_L_REG 0x42 // 温度数据低8位

// 陀螺仪数据输出寄存器
#define MPU6500_GYRO_XOUT_H_REG 0x43 // 陀螺仪X轴数据高8位
#define MPU6500_GYRO_XOUT_L_REG 0x44 // 陀螺仪X轴数据低8位
#define MPU6500_GYRO_YOUT_H_REG 0x45 // 陀螺仪Y轴数据高8位
#define MPU6500_GYRO_YOUT_L_REG 0x46 // 陀螺仪Y轴数据低8位
#define MPU6500_GYRO_ZOUT_H_REG 0x47 // 陀螺仪Z轴数据高8位
#define MPU6500_GYRO_ZOUT_L_REG 0x48 // 陀螺仪Z轴数据低8位

// 外部传感器数据寄存器（用于辅助I2C总线）
#define MPU6500_EXT_SENS_DATA_00 0x49 // 外部传感器数据00
#define MPU6500_EXT_SENS_DATA_01 0x4A // 外部传感器数据01
#define MPU6500_EXT_SENS_DATA_02 0x4B // 外部传感器数据02
#define MPU6500_EXT_SENS_DATA_03 0x4C // 外部传感器数据03
#define MPU6500_EXT_SENS_DATA_04 0x4D // 外部传感器数据04
#define MPU6500_EXT_SENS_DATA_05 0x4E // 外部传感器数据05
#define MPU6500_EXT_SENS_DATA_06 0x4F // 外部传感器数据06
#define MPU6500_EXT_SENS_DATA_07 0x50 // 外部传感器数据07
#define MPU6500_EXT_SENS_DATA_08 0x51 // 外部传感器数据08
#define MPU6500_EXT_SENS_DATA_09 0x52 // 外部传感器数据09
#define MPU6500_EXT_SENS_DATA_10 0x53 // 外部传感器数据10
#define MPU6500_EXT_SENS_DATA_11 0x54 // 外部传感器数据11
#define MPU6500_EXT_SENS_DATA_12 0x55 // 外部传感器数据12
#define MPU6500_EXT_SENS_DATA_13 0x56 // 外部传感器数据13
#define MPU6500_EXT_SENS_DATA_14 0x57 // 外部传感器数据14
#define MPU6500_EXT_SENS_DATA_15 0x58 // 外部传感器数据15
#define MPU6500_EXT_SENS_DATA_16 0x59 // 外部传感器数据16
#define MPU6500_EXT_SENS_DATA_17 0x5A // 外部传感器数据17
#define MPU6500_EXT_SENS_DATA_18 0x5B // 外部传感器数据18
#define MPU6500_EXT_SENS_DATA_19 0x5C // 外部传感器数据19
#define MPU6500_EXT_SENS_DATA_20 0x5D // 外部传感器数据20
#define MPU6500_EXT_SENS_DATA_21 0x5E // 外部传感器数据21
#define MPU6500_EXT_SENS_DATA_22 0x5F // 外部传感器数据22
#define MPU6500_EXT_SENS_DATA_23 0x60 // 外部传感器数据23

#define MPU6500_I2C_SLV0_DO 0x63 // I2C从机0数据输出
#define MPU6500_I2C_SLV1_DO 0x64 // I2C从机1数据输出
#define MPU6500_I2C_SLV2_DO 0x65 // I2C从机2数据输出
#define MPU6500_I2C_SLV3_DO 0x66 // I2C从机3数据输出

#define MPU6500_I2C_MST_DELAY_CTRL 0x67 // I2C主机延时控制
#define MPU6500_SIGNAL_PATH_RESET 0x68  // 信号路径复位
#define MPU6500_MOT_DETECT_CTRL 0x69    // 运动检测控制
#define MPU6500_USER_CTRL_REG 0x6A      // 用户控制寄存器
#define MPU6500_PWR_MGMT_1_REG 0x6B     // 电源管理寄存器1
#define MPU6500_PWR_MGMT_2_REG 0x6C     // 电源管理寄存器2

// FIFO相关寄存器
#define MPU6500_FIFO_COUNTH 0x72 // FIFO计数器高8位
#define MPU6500_FIFO_COUNTL 0x73 // FIFO计数器低8位
#define MPU6500_FIFO_R_W 0x74    // FIFO读写寄存器

// WHO AM I寄存器
#define MPU6500_WHO_AM_I_REG 0x75 // 设备ID寄存器（MPU6500应为0x70）

// 中断相关寄存器
#define MPU6500_XA_OFFSET_H 0x77 // 加速度计X轴偏移高8位
#define MPU6500_XA_OFFSET_L 0x78 // 加速度计X轴偏移低8位
#define MPU6500_YA_OFFSET_H 0x7A // 加速度计Y轴偏移高8位
#define MPU6500_YA_OFFSET_L 0x7B // 加速度计Y轴偏移低8位
#define MPU6500_ZA_OFFSET_H 0x7D // 加速度计Z轴偏移高8位
#define MPU6500_ZA_OFFSET_L 0x7E // 加速度计Z轴偏移低8位

// 中断配置寄存器
#define MPU6500_INT_ENABLE_REG 0x38 // 中断使能寄存器
#define MPU6500_INT_PIN_CFG 0x37    // 中断引脚配置
#define MPU6500_INT_STATUS_REG 0x3A // 中断状态寄存器

// 陀螺仪配置位定义
#define GYRO_FS_SEL_250DPS 0x00  // ±250°/s
#define GYRO_FS_SEL_500DPS 0x08  // ±500°/s
#define GYRO_FS_SEL_1000DPS 0x10 // ±1000°/s
#define GYRO_FS_SEL_2000DPS 0x18 // ±2000°/s

// 加速度计配置位定义
#define ACCEL_FS_SEL_2G 0x00  // ±2g
#define ACCEL_FS_SEL_4G 0x08  // ±4g
#define ACCEL_FS_SEL_8G 0x10  // ±8g
#define ACCEL_FS_SEL_16G 0x18 // ±16g

// 数字低通滤波器(DLPF)配置
#define DLPF_BW_250HZ 0x00  // 陀螺仪带宽250Hz，加速度计带宽260Hz
#define DLPF_BW_184HZ 0x01  // 陀螺仪带宽184Hz，加速度计带宽184Hz
#define DLPF_BW_92HZ 0x02   // 陀螺仪带宽92Hz，加速度计带宽92Hz
#define DLPF_BW_41HZ 0x03   // 陀螺仪带宽41Hz，加速度计带宽41Hz
#define DLPF_BW_20HZ 0x04   // 陀螺仪带宽20Hz，加速度计带宽20Hz
#define DLPF_BW_10HZ 0x05   // 陀螺仪带宽10Hz，加速度计带宽10Hz
#define DLPF_BW_5HZ 0x06    // 陀螺仪带宽5Hz，加速度计带宽5Hz
#define DLPF_BW_3600HZ 0x07 // 陀螺仪带宽3600Hz，加速度计带宽460Hz

// 用户控制寄存器位定义
#define USER_CTRL_FIFO_EN 0x40      // 使能FIFO
#define USER_CTRL_I2C_MST_EN 0x20   // 使能I2C主机模式
#define USER_CTRL_I2C_IF_DIS 0x10   // 禁用I2C接口（使用SPI时必需）
#define USER_CTRL_FIFO_RST 0x04     // 复位FIFO
#define USER_CTRL_I2C_MST_RST 0x02  // 复位I2C主机
#define USER_CTRL_SIG_COND_RST 0x01 // 复位信号路径

// 电源管理寄存器1位定义
#define PWR1_DEVICE_RESET 0x80     // 设备复位
#define PWR1_SLEEP 0x40            // 睡眠模式
#define PWR1_CYCLE 0x20            // 循环模式（低功耗加速度计模式）
#define PWR1_GYRO_STANDBY 0x10     // 陀螺仪待机模式
#define PWR1_PD_PTAT 0x08          // 温度传感器断电
#define PWR1_CLKSEL_INTERNAL 0x00  // 内部8MHz振荡器
#define PWR1_CLKSEL_GYRO_X 0x01    // PLL，使用X轴陀螺仪作为参考
#define PWR1_CLKSEL_GYRO_Y 0x02    // PLL，使用Y轴陀螺仪作为参考
#define PWR1_CLKSEL_GYRO_Z 0x03    // PLL，使用Z轴陀螺仪作为参考
#define PWR1_CLKSEL_EXT_32KHZ 0x04 // 外部32.768kHz
#define PWR1_CLKSEL_EXT_19MHZ 0x05 // 外部19.2MHz
#define PWR1_CLKSEL_STOP 0x07      // 停止时钟，保持时序产生电路复位

// 电源管理寄存器2位定义
#define PWR2_DISABLE_XA 0x20 // 禁用X轴加速度计
#define PWR2_DISABLE_YA 0x10 // 禁用Y轴加速度计
#define PWR2_DISABLE_ZA 0x08 // 禁用Z轴加速度计
#define PWR2_DISABLE_XG 0x04 // 禁用X轴陀螺仪
#define PWR2_DISABLE_YG 0x02 // 禁用Y轴陀螺仪
#define PWR2_DISABLE_ZG 0x01 // 禁用Z轴陀螺仪
#define PWR2_DISABLE_ALL_ACCEL                                                 \
  (PWR2_DISABLE_XA | PWR2_DISABLE_YA | PWR2_DISABLE_ZA)
#define PWR2_DISABLE_ALL_GYRO                                                  \
  (PWR2_DISABLE_XG | PWR2_DISABLE_YG | PWR2_DISABLE_ZG)

// 中断使能寄存器位定义
#define INT_ENABLE_DATA_RDY 0x01   // 数据就绪中断
#define INT_ENABLE_DMP 0x02        // DMP中断
#define INT_ENABLE_FIFO_OFLOW 0x10 // FIFO溢出中断
#define INT_ENABLE_WOM 0x40        // 运动唤醒中断

// 设备ID
#define MPU6500_DEVICE_ID 0x70

// 数据结构定义
typedef struct {
  float roll;  // 横滚角 (度)
  float pitch; // 俯仰角 (度)
  float yaw;   // 偏航角 (度)
  float temp;  // 温度 (°C)
} MPU6500_Angle;

// 函数声明
void MPU6500_Init(void);
void MPU6500_Init_With_Calibration(void);
void MPU6500_Write_Byte(uint8_t reg, uint8_t data);
uint8_t MPU6500_Read_Byte(uint8_t reg);
void MPU6500_Read_Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                       int16_t *gy, int16_t *gz, int16_t *temp);
void MPU6500_Calibrate_Gyro(uint16_t samples);
void MPU6500_Compute_Angles(MPU6500_Angle *angle);
int is_device_stationary(void);

#endif /* __MPU6500_H */