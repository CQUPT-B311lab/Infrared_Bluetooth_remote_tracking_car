#include "mpu6500.h"
#include "cmsis_gcc.h"
#include "main.h"
#include "math.h"
#include "stdlib.h"
#include "stm32f1xx_hal_gpio.h"

extern SPI_HandleTypeDef hspi1;

// 定义常量
#define RAD_TO_DEG 57.295779513f    // 弧度转角度
#define MPU6500_SAMPLE_TIME 0.01f   // 假设采样时间为10ms (100Hz)
#define GYRO_SCALE_FACTOR 131.0f    // 对于±250dps量程 (LSB/°/s)
#define ACCEL_SCALE_FACTOR 16384.0f // 对于±2g量程 (LSB/g)
#define COMP_FILTER_ALPHA 0.98f     // 互补滤波器系数

// 用于保存上一次的角度和陀螺仪偏移量的全局变量
static MPU6500_Angle last_angle = {0.0f, 0.0f, 0.0f};
static int16_t gyro_offset[3] = {0, 0, 0};
static uint8_t is_calibrated = 0;
static uint32_t last_calibration_time = 0;

#define STATIONARY_ACCEL_THRESHOLD 200.0f // 加速度变化阈值 (LSB)
#define STATIONARY_GYRO_THRESHOLD 50.0f   // 陀螺仪变化阈值 (LSB)
#define STATIONARY_WINDOW_SIZE 10         // 检测窗口大小

// 检测设备是否静止
int is_device_stationary(void) {
  static int16_t last_ax = 0, last_ay = 0, last_az = 0;
  static int16_t last_gx = 0, last_gy = 0, last_gz = 0;
  static uint8_t stationary_count = 0;

  int16_t ax, ay, az, gx, gy, gz, temp;
  MPU6500_Read_Data(&ax, &ay, &az, &gx, &gy, &gz, &temp);

  // 计算变化量
  int16_t accel_diff =
      abs(ax - last_ax) + abs(ay - last_ay) + abs(az - last_az);
  int16_t gyro_diff = abs(gx - last_gx) + abs(gy - last_gy) + abs(gz - last_gz);

  // 更新上次值
  last_ax = ax;
  last_ay = ay;
  last_az = az;
  last_gx = gx;
  last_gy = gy;
  last_gz = gz;

  // 判断是否静止
  if (accel_diff < STATIONARY_ACCEL_THRESHOLD &&
      gyro_diff < STATIONARY_GYRO_THRESHOLD) {
    stationary_count++;
  } else {
    stationary_count = 0;
  }

  return stationary_count >= STATIONARY_WINDOW_SIZE;
}

// 初始化MPU6500
void MPU6500_Init(void) {
  HAL_Delay(100); // 上电稳定延时

  // 复位设备
  MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, 0x80);
  HAL_Delay(100);

  // 检查设备ID
  uint8_t id = MPU6500_Read_Byte(MPU6500_WHO_AM_I_REG);
  if (id != MPU6500_DEVICE_ID) {
    // 设备ID错误
    // Error_Handler();
    // while (1)
    //   ; // 或者返回错误码
  }

  // 唤醒设备，选择时钟源
  MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, PWR1_CLKSEL_GYRO_Z);

  // 禁用I2C接口（使用SPI时必需）
  MPU6500_Write_Byte(MPU6500_USER_CTRL_REG, USER_CTRL_I2C_IF_DIS);

  // 配置数字低通滤波器
  MPU6500_Write_Byte(MPU6500_CONFIG_REG, DLPF_BW_184HZ);

  // 配置陀螺仪
  MPU6500_Write_Byte(MPU6500_GYRO_CONFIG_REG, GYRO_FS_SEL_250DPS);

  // 配置加速度计
  MPU6500_Write_Byte(MPU6500_ACCEL_CONFIG_REG, ACCEL_FS_SEL_2G);

  // 配置加速度计低通滤波器
  MPU6500_Write_Byte(MPU6500_ACCEL_CONFIG2_REG, 0x01); // 184Hz带宽

  // 设置采样率（1kHz / (1+0) = 1kHz）
  MPU6500_Write_Byte(MPU6500_SMPLRT_DIV_REG, 0x00);

  // 禁用所有中断（可选）
  MPU6500_Write_Byte(MPU6500_INT_ENABLE_REG, 0x00);

  HAL_Delay(50); // 给设备一些时间完成配置
}

// 写入MPU6500寄存器
void MPU6500_Write_Byte(uint8_t reg, uint8_t data) {
  uint8_t tx_buffer[2];
  tx_buffer[0] = reg & 0x7F; // 写操作，最高位清零
  tx_buffer[1] = data;

  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);

  // 添加微小延时以确保操作完成
  HAL_Delay(1);
}

// 读取MPU6500寄存器
uint8_t MPU6500_Read_Byte(uint8_t reg) {
  uint8_t tx_data = reg | 0x80; // 读取操作，最高位置1
  uint8_t rx_data = 0;

  // 使用TransmitReceive确保单次传输完成
  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);

  return rx_data;
}

// 读取MPU6500数据
void MPU6500_Read_Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                       int16_t *gy, int16_t *gz, int16_t *temp) {
  uint8_t tx_buffer[15] = {0};
  uint8_t rx_buffer[15] = {0};

  // 设置读命令
  tx_buffer[0] = MPU6500_ACCEL_XOUT_H_REG | 0x80;

  // 后续字节发送0xFF（产生时钟信号）
  for (int i = 1; i < 15; i++) {
    tx_buffer[i] = 0xFF;
  }

  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 15, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(SPI_NSS_GPIO_Port, SPI_NSS_Pin, GPIO_PIN_SET);

  // 解析数据（跳过第一个字节，因为是发送命令时收到的无效数据）
  *ax = ((int16_t)rx_buffer[1] << 8) | rx_buffer[2];
  *ay = ((int16_t)rx_buffer[3] << 8) | rx_buffer[4];
  *az = ((int16_t)rx_buffer[5] << 8) | rx_buffer[6];
  *temp = ((int16_t)rx_buffer[7] << 8) | rx_buffer[8];
  *gx = ((int16_t)rx_buffer[9] << 8) | rx_buffer[10];
  *gy = ((int16_t)rx_buffer[11] << 8) | rx_buffer[12];
  *gz = ((int16_t)rx_buffer[13] << 8) | rx_buffer[14];
}

// 陀螺仪零偏校准函数
void MPU6500_Calibrate_Gyro(uint16_t samples) {
  int32_t gyro_sum[3] = {0, 0, 0};
  int16_t ax, ay, az, gx, gy, gz, temp;

  // 采集多次数据以计算平均偏移
  for (uint16_t i = 0; i < samples; i++) {
    MPU6500_Read_Data(&ax, &ay, &az, &gx, &gy, &gz, &temp);
    gyro_sum[0] += gx;
    gyro_sum[1] += gy;
    gyro_sum[2] += gz;
    HAL_Delay(10); // 10ms延时，保持稳定
  }

  // 计算平均偏移
  gyro_offset[0] = gyro_sum[0] / samples;
  gyro_offset[1] = gyro_sum[1] / samples;
  gyro_offset[2] = gyro_sum[2] / samples;

  is_calibrated = 1;
}

// 互补滤波计算角度函数
void MPU6500_Compute_Angles(MPU6500_Angle *angle) {
  int16_t ax, ay, az, gx, gy, gz, temp_raw;
  float accel_roll, accel_pitch;
  float gyro_roll_rate, gyro_pitch_rate, gyro_yaw_rate;
  static float yaw_drift_compensation = 0.0f; // 偏航角漂移补偿
  static uint8_t stationary_counter = 0;      // 静止状态计数器
  uint32_t current_time = HAL_GetTick();

  // 读取原始数据
  MPU6500_Read_Data(&ax, &ay, &az, &gx, &gy, &gz, &temp_raw);

  // 温度转换为摄氏度 - 根据规格书第12页
  angle->temp = (float)temp_raw / 333.87f + 21.0f;

  // 应用陀螺仪零偏校准
  if (is_calibrated) {
    gx -= gyro_offset[0];
    gy -= gyro_offset[1];
    gz -= gyro_offset[2];
  }

  // 从加速度计计算角度
  accel_roll = atan2f((float)ay, (float)az) * RAD_TO_DEG;
  accel_pitch =
      atan2f(-(float)ax, sqrtf((float)(ay * ay + az * az))) * RAD_TO_DEG;

  // 将陀螺仪值转换为角速度(°/s)
  gyro_roll_rate = (float)gx / GYRO_SCALE_FACTOR;
  gyro_pitch_rate = (float)gy / GYRO_SCALE_FACTOR;
  gyro_yaw_rate = (float)gz / GYRO_SCALE_FACTOR;

  // 增强的死区滤波
  const float GYRO_DEADBAND = 1.5f; // 增大死区阈值到1.5度每秒

  if (fabs(gyro_roll_rate) < GYRO_DEADBAND)
    gyro_roll_rate = 0.0f;
  if (fabs(gyro_pitch_rate) < GYRO_DEADBAND)
    gyro_pitch_rate = 0.0f;
  if (fabs(gyro_yaw_rate) < GYRO_DEADBAND) {
    gyro_yaw_rate = 0.0f;

    // 检测静止状态
    if (is_device_stationary()) {
      stationary_counter++;

      // 如果连续检测到10次静止状态
      if (stationary_counter >= 10) {
        // 重置yaw偏移量
        if (current_time - last_calibration_time > 10000) { // 至少10秒一次校准
          MPU6500_Calibrate_Gyro(50);
          last_calibration_time = current_time;
        }

        // 更强的漂移补偿
        if (fabs(last_angle.yaw) > 0.1f) {
          yaw_drift_compensation = last_angle.yaw * 0.01f;
        }
      }
    } else {
      stationary_counter = 0; // 重置静止计数器
    }
  } else {
    stationary_counter = 0; // 如果有明显运动，重置静止计数器
  }

  // 互补滤波器
  if (last_angle.roll == 0 && last_angle.pitch == 0) {
    angle->roll = accel_roll;
    angle->pitch = accel_pitch;
    angle->yaw = 0.0f;
  } else {
    angle->roll = COMP_FILTER_ALPHA *
                      (last_angle.roll + gyro_roll_rate * MPU6500_SAMPLE_TIME) +
                  (1.0f - COMP_FILTER_ALPHA) * accel_roll;
    angle->pitch = COMP_FILTER_ALPHA * (last_angle.pitch +
                                        gyro_pitch_rate * MPU6500_SAMPLE_TIME) +
                   (1.0f - COMP_FILTER_ALPHA) * accel_pitch;

    // 偏航角加入漂移补偿
    if (stationary_counter < 5) {
      angle->yaw = last_angle.yaw + gyro_yaw_rate * MPU6500_SAMPLE_TIME;
    } else {
      // 静止状态下应用漂移补偿
      angle->yaw = last_angle.yaw - yaw_drift_compensation;
    }
  }

  // 保存本次计算的角度用于下次计算
  last_angle = *angle;
}

// 初始化带校准的函数
void MPU6500_Init_With_Calibration(void) {
  // 先初始化MPU6500
  MPU6500_Init();

  // 等待传感器稳定
  HAL_Delay(1000);

  // 校准陀螺仪
  MPU6500_Calibrate_Gyro(100);
}