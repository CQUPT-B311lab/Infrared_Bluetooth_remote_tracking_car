#include "mpu6500.h"
#include "cmsis_gcc.h"
#include "main.h"
#include "math.h"

extern SPI_HandleTypeDef hspi1;

// 定义CS引脚操作宏
#define MPU6500_CS_LOW() __NOP()
#define MPU6500_CS_HIGH() __NOP()
// #define MPU6500_CS_LOW() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET)
// #define MPU6500_CS_HIGH() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET)

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

// 静止检测历史数据存储
static int16_t gyro_history[STATIONARY_WINDOW_SIZE][3];
static int16_t accel_history[STATIONARY_WINDOW_SIZE][3];
static uint8_t history_index = 0;
static uint8_t history_filled = 0;

// 检测设备是否静止
int is_device_stationary(void) {
  int16_t ax, ay, az, gx, gy, gz, temp;
  float avg_accel_x = 0, avg_accel_y = 0, avg_accel_z = 0;
  float avg_gyro_x = 0, avg_gyro_y = 0, avg_gyro_z = 0;
  float accel_variance = 0, gyro_variance = 0;

  // 读取当前数据
  MPU6500_Read_Data(&ax, &ay, &az, &gx, &gy, &gz, &temp);

  // 保存到历史数据
  gyro_history[history_index][0] = gx;
  gyro_history[history_index][1] = gy;
  gyro_history[history_index][2] = gz;
  accel_history[history_index][0] = ax;
  accel_history[history_index][1] = ay;
  accel_history[history_index][2] = az;

  // 更新索引
  history_index = (history_index + 1) % STATIONARY_WINDOW_SIZE;
  if (!history_filled && history_index == 0) {
    history_filled = 1;
  }

  // 如果历史数据不足，无法判断
  if (!history_filled) {
    return 0;
  }

  // 计算平均值
  for (int i = 0; i < STATIONARY_WINDOW_SIZE; i++) {
    avg_accel_x += accel_history[i][0];
    avg_accel_y += accel_history[i][1];
    avg_accel_z += accel_history[i][2];
    avg_gyro_x += gyro_history[i][0];
    avg_gyro_y += gyro_history[i][1];
    avg_gyro_z += gyro_history[i][2];
  }

  avg_accel_x /= STATIONARY_WINDOW_SIZE;
  avg_accel_y /= STATIONARY_WINDOW_SIZE;
  avg_accel_z /= STATIONARY_WINDOW_SIZE;
  avg_gyro_x /= STATIONARY_WINDOW_SIZE;
  avg_gyro_y /= STATIONARY_WINDOW_SIZE;
  avg_gyro_z /= STATIONARY_WINDOW_SIZE;

  // 计算方差
  for (int i = 0; i < STATIONARY_WINDOW_SIZE; i++) {
    accel_variance += powf(accel_history[i][0] - avg_accel_x, 2) +
                      powf(accel_history[i][1] - avg_accel_y, 2) +
                      powf(accel_history[i][2] - avg_accel_z, 2);

    gyro_variance += powf(gyro_history[i][0] - avg_gyro_x, 2) +
                     powf(gyro_history[i][1] - avg_gyro_y, 2) +
                     powf(gyro_history[i][2] - avg_gyro_z, 2);
  }

  accel_variance /= (STATIONARY_WINDOW_SIZE * 3);
  gyro_variance /= (STATIONARY_WINDOW_SIZE * 3);

  // 方差小于阈值表示设备静止
  return (accel_variance < STATIONARY_ACCEL_THRESHOLD &&
          gyro_variance < STATIONARY_GYRO_THRESHOLD);
}

// 初始化MPU6500
void MPU6500_Init(void) {
  HAL_Delay(100); // 上电稳定延时
  // 复位设备
  MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, 0x80);
  HAL_Delay(100); // 等待复位完成
  // 唤醒设备
  MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, 0x00);
  // 检查设备ID - 可选但推荐
  uint8_t id = MPU6500_Read_Byte(MPU6500_WHO_AM_I_REG);
  if (id != 0x70) { // MPU6500的ID是0x70
                    // 设备ID错误，可以在这里处理错误
                    // 例如: Error_Handler();
  }
  // 配置电源管理，选择最佳时钟源
  MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, 0x01); // 使用PLL作为时钟源
  // 设置采样率分频器
  MPU6500_Write_Byte(MPU6500_SMPLRT_DIV_REG, 0x07); // 1000/(1+7) = 125Hz
  // 配置数字低通滤波器
  MPU6500_Write_Byte(MPU6500_CONFIG_REG, 0x06); // 5Hz带宽
  // 配置陀螺仪量程为±250dps
  MPU6500_Write_Byte(MPU6500_GYRO_CONFIG_REG, 0x00);
  // 配置加速度计量程为±2g
  MPU6500_Write_Byte(MPU6500_ACCEL_CONFIG_REG, 0x00);
  // 配置中 - 可选
  // MPU6500_Write_Byte(MPU6500_INT_ENABLE_REG, 0x00); // 禁用所有中断
  HAL_Delay(50); // 给设备一些时间完成配置
}
// 写入MPU6500寄存器
void MPU6500_Write_Byte(uint8_t reg, uint8_t data) {
  uint8_t tx_buffer[2];
  tx_buffer[0] = reg & 0x7F; // 写操作，最高位清零
  tx_buffer[1] = data;

  MPU6500_CS_LOW();
  HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY);
  MPU6500_CS_HIGH();

  // 添加微小延时以确保操作完成
  HAL_Delay(1);
}

// 读取MPU6500寄存器
uint8_t MPU6500_Read_Byte(uint8_t reg) {
  uint8_t tx_data = reg | 0x80; // 读取操作，最高位置1
  uint8_t rx_data = 0;

  MPU6500_CS_LOW();

  // 发送寄存器地址
  HAL_SPI_Transmit(&hspi1, &tx_data, 1, HAL_MAX_DELAY);

  // 接收数据
  HAL_SPI_Receive(&hspi1, &rx_data, 1, HAL_MAX_DELAY);

  MPU6500_CS_HIGH();

  return rx_data;
}

// 读取MPU6500数据 - 彻底重写
void MPU6500_Read_Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                       int16_t *gy, int16_t *gz, int16_t *temp) {
  uint8_t tx_data[15]; // 1个地址字节 + 14个虚拟字节
  uint8_t rx_data[15]; // 同上

  // 清空发送缓冲区
  for (int i = 0; i < 15; i++) {
    tx_data[i] = 0xFF;
    rx_data[i] = 0;
  }

  // 设置读取操作和起始寄存器地址
  tx_data[0] = MPU6500_ACCEL_XOUT_H_REG | 0x80;

  MPU6500_CS_LOW();

  // 使用HAL_SPI_TransmitReceive进行SPI全双工传输
  HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 15, HAL_MAX_DELAY);

  MPU6500_CS_HIGH();

  // 有效数据从rx_data[1]开始，因为第一个字节是发送地址时收到的无效数据
  *ax = ((int16_t)rx_data[1] << 8) | rx_data[2];
  *ay = ((int16_t)rx_data[3] << 8) | rx_data[4];
  *az = ((int16_t)rx_data[5] << 8) | rx_data[6];
  // 温度数据在rx_data[7]和rx_data[8]，这里跳过
  *temp = ((int16_t)rx_data[7] << 8) | rx_data[8];
  *gx = ((int16_t)rx_data[9] << 8) | rx_data[10];
  *gy = ((int16_t)rx_data[11] << 8) | rx_data[12];
  *gz = ((int16_t)rx_data[13] << 8) | rx_data[14];
}
// 陀螺仪零偏校准函数
void MPU6500_Calibrate_Gyro(uint16_t samples) {
  // return;
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
  // 温度转换为摄氏度 - MPU6500温度换算公式
  // 温度 = (TEMP_OUT / 340) + 36.53
  angle->temp = (float)temp_raw / 340.0f + 36.53f;
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

  // 增强的死区滤波 - 更严格的阈值
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

        // 更强的漂移补偿 - 逐渐将偏航角拉回固定值
        if (fabs(last_angle.yaw) > 0.1f) {
          // 每次补偿1%的当前yaw值，使其逐渐归零或保持不变
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

    // 偏航角加入漂移补偿 - 只有在非静止状态下才更新偏航角
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