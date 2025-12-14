#include "mpu6050.h"
#include "main.h"

MPU6050_Angle mpu_angle = {0};
extern I2C_HandleTypeDef hi2c1;

// 静态变量用于陀螺仪积分
static float gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
static uint32_t last_time = 0;

// DMP配置寄存器
#define DMP_MEM_START_ADDR 0x6E
#define DMP_FIFO_RATE 0x23
#define DMP_CFG_1 0x70
#define DMP_CFG_2 0x71

uint8_t MPU6050_DMP_Simple_Init(void) {
  uint8_t data;

  // 1. 基本初始化
  if (!MPU6050_Init())
    return 0;

  // 2. 重置DMP
  data = 0x80;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);
  HAL_Delay(100);
  data = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);

  // 3. 加载DMP固件（需要预先定义固件数组）
  // 这里省略固件加载过程...

  // 4. 设置DMP输出速率
  data = 100; // 100Hz
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, DMP_FIFO_RATE, 1, &data, 1, 1000);

  // 5. 启用DMP
  data = 0x02; // 启用DMP
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6A, 1, &data, 1, 1000);
  data = 0xC0; // 启用FIFO和DMP
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6A, 1, &data, 1, 1000);

  return 1;
}

uint8_t MPU6050_Init(void) {
  uint8_t check;
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, 1000);

  if (check == 64) {
    // 复位设备
    uint8_t data = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 1000);
    HAL_Delay(100);

    // 唤醒设备，使用内部8MHz晶振
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 1000);

    // 陀螺仪±2000dps (FS=3)
    data = 0x18;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG, 1, &data, 1, 1000);

    // 加速度计±2g (FS=0)，灵敏度16384 LSB/g
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG, 1, &data, 1, 1000);

    // 低通滤波器设置，带宽94Hz
    data = 0x02;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG, 1, &data, 1, 1000);

    // 采样率1kHz
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV, 1, &data, 1, 1000);

    last_time = HAL_GetTick();
    return 1;
  }
  return 0;
}

void MPU6050_Read_All(short *accel, short *gyro) {
  uint8_t data[14];
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, data, 14, 1000);

  // 注意数据顺序：先高字节后低字节
  accel[0] = (short)((data[0] << 8) | data[1]); // Accel X
  accel[1] = (short)((data[2] << 8) | data[3]); // Accel Y
  accel[2] = (short)((data[4] << 8) | data[5]); // Accel Z

  // 温度数据忽略
  // temp = (short)((data[6] << 8) | data[7]);

  gyro[0] = (short)((data[8] << 8) | data[9]);   // Gyro X
  gyro[1] = (short)((data[10] << 8) | data[11]); // Gyro Y
  gyro[2] = (short)((data[12] << 8) | data[13]); // Gyro Z
}

void MPU6050_Calculate_Angle(void) {
  short accel_raw[3], gyro_raw[3];
  MPU6050_Read_All(accel_raw, gyro_raw);

  // 计算实际时间间隔（秒）
  if (last_time == 0) {
    last_time = HAL_GetTick();
  }
  uint32_t current_time = HAL_GetTick();
  float dt = (current_time - last_time) / 1000.0f;
  last_time = current_time;

  // 确保dt在合理范围内
  if (dt <= 0 || dt > 0.1f)
    dt = 0.01f;

  // 将加速度计原始数据转换为g（±2g量程：16384 LSB/g）
  float accel_g[3];
  accel_g[0] = accel_raw[0] / 16384.0f;
  accel_g[1] = accel_raw[1] / 16384.0f;
  accel_g[2] = accel_raw[2] / 16384.0f;

  // 加速度计计算角度（pitch和roll）
  // 注意：这里使用atan2f，返回弧度，乘以57.29578转换为度
  float accel_angle_x = atan2f(accel_g[1], accel_g[2]) * 57.29578f; // roll
  float accel_angle_y =
      atan2f(-accel_g[0],
             sqrtf(accel_g[1] * accel_g[1] + accel_g[2] * accel_g[2])) *
      57.29578f; // pitch

  // 陀螺仪数据转换为°/s（±2000dps量程：16.4 LSB/°/s）
  float gyro_rate_x = gyro_raw[0] / 16.4f;
  float gyro_rate_y = gyro_raw[1] / 16.4f;
  float gyro_rate_z = gyro_raw[2] / 16.4f;

  // 陀螺仪积分（角度变化 = 角速度 × 时间）
  gyro_angle_x += gyro_rate_x * dt;
  gyro_angle_y += gyro_rate_y * dt;
  gyro_angle_z += gyro_rate_z * dt;

  // 互补滤波器融合加速度计和陀螺仪
  float alpha = 0.96f; // 可以根据需要调整

  mpu_angle.roll = alpha * gyro_angle_x + (1 - alpha) * accel_angle_x;
  mpu_angle.pitch = alpha * gyro_angle_y + (1 - alpha) * accel_angle_y;

  // 注意：没有磁力计，yaw无法校准，会持续漂移
  mpu_angle.yaw = gyro_angle_z;

  // 可选：限制yaw在-180°到180°之间
  if (mpu_angle.yaw > 180.0f)
    mpu_angle.yaw -= 360.0f;
  if (mpu_angle.yaw < -180.0f)
    mpu_angle.yaw += 360.0f;

  // mpu_angle.yaw += 5.11;
}