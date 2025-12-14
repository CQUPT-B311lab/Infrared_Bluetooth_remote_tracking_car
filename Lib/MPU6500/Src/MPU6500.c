// #include "MPU6500.h"
// #include "COM.h"
// #include "main.h"
// #include "math.h"
// #include "string.h"

// extern SPI_HandleTypeDef hspi1;

// // CS引脚控制宏
// #define MPU6500_CS_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
// #define MPU6500_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

// // 常量定义
// #define RAD_TO_DEG 57.295779513f        // 弧度转角度
// #define INV_GYRO_SCALE 0.007633587786f  // 1/131.0f
// #define INV_ACCEL_SCALE 0.000061035156f // 1/16384.0f
// #define COMP_FILTER_ALPHA 0.98f
// #define INV_COMP_FILTER 0.02f // 1.0f - COMP_FILTER_ALPHA
// #define SAMPLE_TIME 0.01f
// #define PI 3.14159265359f
// #define PI_2 1.57079632679f // π/2

// // 角度计算优化：预计算常用值
// #define DEG_TO_RAD 0.01745329251f // π/180

// // 全局变量
// static MPU6500_Data last_data = {0};
// static int16_t gyro_offset[3] = {0, 0, 0};
// static uint8_t is_calibrated = 0;

// // 静止检测优化：使用指数加权移动平均（EWMA）
// #define EWMA_ALPHA 0.1f
// #define EWMA_BETA 0.9f // 1 - ALPHA
// static float ewma_accel[3] = {0, 0, 0};
// static float ewma_gyro[3] = {0, 0, 0};
// static float ewma_accel_var[3] = {0, 0, 0};
// static float ewma_gyro_var[3] = {0, 0, 0};
// static uint32_t stationary_counter = 0;

// // ==================== 优化的SPI读取 ====================

// void MPU6500_Read_Raw_Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t
// *gx,
//                            int16_t *gy, int16_t *gz, int16_t *temp) {
//   uint8_t tx_buffer[15];
//   uint8_t rx_buffer[15];
//   // 使用memset快速填充
//   memset(tx_buffer, 0xFF, sizeof(tx_buffer));
//   tx_buffer[0] = MPU6500_ACCEL_XOUT_H_REG | 0x80;
//   MPU6500_CS_LOW();
//   HAL_SPI_TransmitReceive(&hspi1, tx_buffer, rx_buffer, 15, HAL_MAX_DELAY);
//   MPU6500_CS_HIGH();
//   // 使用位运算快速解析（避免中间变量）
//   *ax = (rx_buffer[1] << 8) | rx_buffer[2];
//   *ay = (rx_buffer[3] << 8) | rx_buffer[4];
//   *az = (rx_buffer[5] << 8) | rx_buffer[6];
//   *temp = (rx_buffer[7] << 8) | rx_buffer[8];
//   *gx = (rx_buffer[9] << 8) | rx_buffer[10];
//   *gy = (rx_buffer[11] << 8) | rx_buffer[12];
//   *gz = (rx_buffer[13] << 8) | rx_buffer[14];
// }
// // ==================== 优化的校准函数 ====================
// void MPU6500_Calibrate_Gyro(uint16_t samples) {
//   int32_t gyro_sum[3] = {0, 0, 0};
//   int16_t gx, gy, gz, ax, ay, az, temp;
//   // 只读取陀螺仪数据
//   for (uint16_t i = 0; i < samples; i++) {
//     MPU6500_Read_Raw_Data(&ax, &ay, &az, &gx, &gy, &gz, &temp);
//     gyro_sum[0] += gx;
//     gyro_sum[1] += gy;
//     gyro_sum[2] += gz;
//     // 进度显示（可选）
//     // printf("校准进度: %d/%d\r", i+1, samples);
//     HAL_Delay(5); // 较短延时
//   }
//   // 正确计算平均值
//   gyro_offset[0] = (int16_t)(gyro_sum[0] / samples);
//   gyro_offset[1] = (int16_t)(gyro_sum[1] / samples);
//   gyro_offset[2] = (int16_t)(gyro_sum[2] / samples);
//   is_calibrated = 1;
//   // 输出校准结果（可选）
//   // printf("\r\n校准完成！零偏值：\r\n");
//   // printf("X轴: %d LSB (%.2f°/s)\r\n", gyro_offset[0],
//   // (float)gyro_offset[0]*INV_GYRO_SCALE); printf("Y轴: %d LSB
//   (%.2f°/s)\r\n",
//   // gyro_offset[1], (float)gyro_offset[1]*INV_GYRO_SCALE); printf("Z轴: %d
//   LSB
//   // (%.2f°/s)\r\n", gyro_offset[2], (float)gyro_offset[2]*INV_GYRO_SCALE);
// }

// // ==================== 快速反正切近似 ====================
// static float fast_atan2(float y, float x) {
//   float abs_y = y < 0 ? -y : y; // fabsf的快速替代
//   float abs_x = x < 0 ? -x : x;
//   float a, r, angle;
//   // 避免除零
//   if (abs_y < 1e-10f && abs_x < 1e-10f) {
//     return 0.0f;
//   }
//   // 使用多项式近似
//   if (abs_x > abs_y) {
//     a = abs_y / abs_x;
//     // 3阶多项式近似：atan(a) ≈ a - 0.28*a³/(1+0.28*a²)
//     float a2 = a * a;
//     r = a - 0.28f * a * a2 / (1.0f + 0.28f * a2);
//   } else {
//     a = abs_x / abs_y;
//     float a2 = a * a;
//     r = PI_2 - (a - 0.28f * a * a2 / (1.0f + 0.28f * a2));
//   }
//   // 确定象限
//   if (x < 0) {
//     angle = PI - r;
//   } else {
//     angle = r;
//   }
//   if (y < 0) {
//     angle = -angle;
//   }
//   return angle * RAD_TO_DEG;
// }
// // ==================== 快速平方根近似 ====================
// // 快速平方根（精度约1%，速度比sqrtf快2-3倍）
// static float fast_sqrt(float x) {
//   if (x <= 0.0f)
//     return 0.0f;
//   // 使用Quake3的快速平方根算法（单次牛顿迭代）
//   union {
//     float f;
//     uint32_t i;
//   } u;
//   u.f = x;
//   u.i = 0x5f3759df - (u.i >> 1);             // 魔法常数
//   u.f = u.f * (1.5f - 0.5f * x * u.f * u.f); // 一次牛顿迭代
//   return 1.0f / u.f;
// }
// // ==================== 优化后的主数据处理 ====================
// void MPU6500_Get_All_Data(MPU6500_Data *data) {
//   int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, temp_raw;
//   // 1. 读取原始数据
//   MPU6500_Read_Raw_Data(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw,
//                         &temp_raw);
//   // 2. 温度转换（使用预计算的系数）
//   data->temp =
//       ((float)temp_raw * 0.002994f) + 21.0f; // 近似：1/333.87 ≈ 0.002994
//   // 3. 加速度计数据转换（使用乘法避免除法）
//   data->x_acc = (float)ax_raw * INV_ACCEL_SCALE;
//   data->y_acc = (float)ay_raw * INV_ACCEL_SCALE;
//   data->z_acc = (float)az_raw * INV_ACCEL_SCALE;
//   // 4. 陀螺仪校准和转换
//   float gyro_x_dps, gyro_y_dps, gyro_z_dps;
//   if (is_calibrated) {
//     gyro_x_dps = (float)(gx_raw - gyro_offset[0]) * INV_GYRO_SCALE;
//     gyro_y_dps = (float)(gy_raw - gyro_offset[1]) * INV_GYRO_SCALE;
//     gyro_z_dps = (float)(gz_raw - gyro_offset[2]) * INV_GYRO_SCALE;
//   } else {
//     gyro_x_dps = (float)gx_raw * INV_GYRO_SCALE;
//     gyro_y_dps = (float)gy_raw * INV_GYRO_SCALE;
//     gyro_z_dps = (float)gz_raw * INV_GYRO_SCALE;
//   }
//   // 5. 死区滤波（避免使用fabsf）
//   const float GYRO_DEADBAND = 0.5f;
//   const float GYRO_DEADBAND_NEG = -GYRO_DEADBAND;
//   if (gyro_x_dps < GYRO_DEADBAND && gyro_x_dps > GYRO_DEADBAND_NEG)
//     gyro_x_dps = 0.0f;
//   if (gyro_y_dps < GYRO_DEADBAND && gyro_y_dps > GYRO_DEADBAND_NEG)
//     gyro_y_dps = 0.0f;
//   if (gyro_z_dps < GYRO_DEADBAND && gyro_z_dps > GYRO_DEADBAND_NEG)
//     gyro_z_dps = 0.0f;
//   // 6. 从加速度计计算角度（使用快速近似函数）
//   // 注意坐标系：x_acc=左，y_acc=后，z_acc=上
//   float y_acc = data->y_acc;
//   float z_acc = data->z_acc;
//   // 计算Roll（绕X轴）：atan2(y, z)
//   // 使用快速近似避免atan2f和sqrtf
//   float accel_roll, accel_pitch;
//   if (z_acc != 0.0f) {
//     // 当z轴不为0时，使用简化计算
//     float tan_roll = y_acc / z_acc;
//     // 小角度近似：atan(x) ≈ x - x³/3 (当|x| < 1时)
//     if (tan_roll > 1.0f)
//       tan_roll = 1.0f;
//     if (tan_roll < -1.0f)
//       tan_roll = -1.0f;
//     accel_roll = tan_roll * RAD_TO_DEG;
//     // 对于大角度，使用完整计算
//     if (tan_roll > 0.5f || tan_roll < -0.5f) {
//       accel_roll = fast_atan2(y_acc, z_acc);
//     }
//   } else {
//     accel_roll = (y_acc > 0) ? 90.0f : -90.0f;
//   }
//   // 计算Pitch（绕Y轴）：atan2(-x, sqrt(y²+z²))
//   float xz_sq = y_acc * y_acc + z_acc * z_acc;
//   if (xz_sq > 0.0f) {
//     float tan_pitch = -data->x_acc / fast_sqrt(xz_sq);
//     if (tan_pitch > 1.0f)
//       tan_pitch = 1.0f;
//     if (tan_pitch < -1.0f)
//       tan_pitch = -1.0f;
//     accel_pitch = tan_pitch * RAD_TO_DEG;
//     if (tan_pitch > 0.5f || tan_pitch < -0.5f) {
//       accel_pitch = fast_atan2(-data->x_acc, fast_sqrt(xz_sq));
//     }
//   } else {
//     accel_pitch = (data->x_acc > 0) ? -90.0f : 90.0f;
//   }
//   // 7. 互补滤波器（使用预计算的系数）
//   if (last_data.roll == 0.0f && last_data.pitch == 0.0f) {
//     data->roll = accel_roll;
//     data->pitch = accel_pitch;
//     data->yaw = 0.0f;
//   } else {
//     // 预计算陀螺仪积分
//     float gyro_roll_integral = last_data.roll + gyro_x_dps * SAMPLE_TIME;
//     float gyro_pitch_integral = last_data.pitch + gyro_y_dps * SAMPLE_TIME;
//     // 互补滤波
//     data->roll =
//         COMP_FILTER_ALPHA * gyro_roll_integral + INV_COMP_FILTER *
//         accel_roll;
//     data->pitch =
//         COMP_FILTER_ALPHA * gyro_pitch_integral + INV_COMP_FILTER *
//         accel_pitch;
//     data->yaw = last_data.yaw + gyro_z_dps * SAMPLE_TIME;
//     // 角度范围限制（使用if-else比fmodf快）
//     if (data->roll > 180.0f) {
//       data->roll -= 360.0f;
//     } else if (data->roll < -180.0f) {
//       data->roll += 360.0f;
//     }
//     if (data->pitch > 180.0f) {
//       data->pitch -= 360.0f;
//     } else if (data->pitch < -180.0f) {
//       data->pitch += 360.0f;
//     }
//     if (data->yaw > 180.0f) {
//       data->yaw -= 360.0f;
//     } else if (data->yaw < -180.0f) {
//       data->yaw += 360.0f;
//     }
//   }
//   // 8. 更新EWMA用于静止检测
//   float current_accel[3] = {data->x_acc, data->y_acc, data->z_acc};
//   float current_gyro[3] = {gyro_x_dps, gyro_y_dps, gyro_z_dps};
//   for (int i = 0; i < 3; i++) {
//     // 更新EWMA均值
//     float accel_diff = current_accel[i] - ewma_accel[i];
//     ewma_accel[i] += EWMA_ALPHA * accel_diff;
//     // 更新EWMA方差
//     ewma_accel_var[i] =
//         EWMA_BETA * (ewma_accel_var[i] + EWMA_ALPHA * accel_diff *
//         accel_diff);
//     float gyro_diff = current_gyro[i] - ewma_gyro[i];
//     ewma_gyro[i] += EWMA_ALPHA * gyro_diff;
//     ewma_gyro_var[i] =
//         EWMA_BETA * (ewma_gyro_var[i] + EWMA_ALPHA * gyro_diff * gyro_diff);
//   }
//   // 9. 保存本次数据
//   last_data = *data;
// }
// // ==================== 优化后的静止检测 ====================
// int MPU6500_Is_Device_Stationary(void) {
//   // 使用EWMA方差判断
//   float total_accel_var =
//       ewma_accel_var[0] + ewma_accel_var[1] + ewma_accel_var[2];
//   float total_gyro_var = ewma_gyro_var[0] + ewma_gyro_var[1] +
//   ewma_gyro_var[2]; const float ACCEL_VAR_THRESH = 0.001f; // 0.001 g² const
//   float GYRO_VAR_THRESH = 0.1f;    // 0.1 (°/s)² if (total_accel_var <
//   ACCEL_VAR_THRESH && total_gyro_var < GYRO_VAR_THRESH) {
//     stationary_counter++;
//     if (stationary_counter > 100)
//       stationary_counter = 100;
//   } else {
//     stationary_counter = 0;
//   }
//   return stationary_counter > 20; // 连续20次静止
// }
// // ==================== 其他函数（保持原样） ====================
// void MPU6500_Init(void) {
//   HAL_Delay(100);
//   // 复位设备
//   MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, 0x80);
//   HAL_Delay(100);
//   // 唤醒设备，选择时钟源
//   MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, 0x01);
//   // 禁用I2C接口（使用SPI时必需）
//   MPU6500_Write_Byte(MPU6500_USER_CTRL_REG, 0x10);
//   // 配置数字低通滤波器
//   MPU6500_Write_Byte(MPU6500_CONFIG_REG, 0x01);
//   // 配置陀螺仪
//   MPU6500_Write_Byte(MPU6500_GYRO_CONFIG_REG, 0x00);
//   // 配置加速度计
//   MPU6500_Write_Byte(MPU6500_ACCEL_CONFIG_REG, 0x00);
//   // 配置加速度计低通滤波器
//   MPU6500_Write_Byte(MPU6500_ACCEL_CONFIG2_REG, 0x01);
//   // 设置采样率
//   MPU6500_Write_Byte(MPU6500_SMPLRT_DIV_REG, 0x00);
//   HAL_Delay(50);
// }
// void MPU6500_Init_With_Calibration(void) {
//   MPU6500_Init();
//   HAL_Delay(1000);
//   MPU6500_Calibrate_Gyro(200);
// }
// void MPU6500_Write_Byte(uint8_t reg, uint8_t data) {
//   uint8_t tx_buffer[2];
//   tx_buffer[0] = reg & 0x7F;
//   tx_buffer[1] = data;
//   MPU6500_CS_LOW();
//   HAL_SPI_Transmit(&hspi1, tx_buffer, 2, HAL_MAX_DELAY);
//   MPU6500_CS_HIGH();
// }
// uint8_t MPU6500_Read_Byte(uint8_t reg) {
//   uint8_t tx_data = reg | 0x80;
//   uint8_t rx_data = 0;
//   MPU6500_CS_LOW();
//   HAL_SPI_TransmitReceive(&hspi1, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
//   MPU6500_CS_HIGH();
//   return rx_data;
// }
// void MPU6500_Compute_Angles(MPU6500_Data *data) { MPU6500_Get_All_Data(data);
// }
// // ==================== 性能测试函数 ====================
// // 测试计算性能
// uint32_t MPU6500_Test_Performance(uint32_t iterations) {
//   MPU6500_Data data;
//   uint32_t start_time = HAL_GetTick();
//   for (uint32_t i = 0; i < iterations; i++) {
//     MPU6500_Get_All_Data(&data);
//   }
//   uint32_t end_time = HAL_GetTick();
//   return end_time - start_time;
// }

#include "MPU6500.h"
#include "COM.h"
#include "main.h"
#include "math.h"
#include "string.h"

// 替换SPI句柄为I2C句柄
extern I2C_HandleTypeDef hi2c1; // 假设使用I2C1

// 删除SPI相关的CS引脚控制宏

// 常量定义（保持不变）
#define RAD_TO_DEG 57.295779513f
#define INV_GYRO_SCALE 0.007633587786f
#define INV_ACCEL_SCALE 0.000061035156f
#define COMP_FILTER_ALPHA 0.98f
#define INV_COMP_FILTER 0.02f
#define SAMPLE_TIME 0.01f
#define PI 3.14159265359f
#define PI_2 1.57079632679f
#define DEG_TO_RAD 0.01745329251f

// 全局变量（保持不变）
static MPU6500_Data last_data = {0};
static int16_t gyro_offset[3] = {0, 0, 0};
static uint8_t is_calibrated = 0;

// 静止检测优化
#define EWMA_ALPHA 0.1f
#define EWMA_BETA 0.9f
static float ewma_accel[3] = {0, 0, 0};
static float ewma_gyro[3] = {0, 0, 0};
static float ewma_accel_var[3] = {0, 0, 0};
static float ewma_gyro_var[3] = {0, 0, 0};
static uint32_t stationary_counter = 0;

// ==================== I2C读写函数 ====================

// I2C写一个字节
void MPU6500_Write_Byte(uint8_t reg, uint8_t data) {
  uint8_t buffer[2] = {reg, data};
  HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR << 1, buffer, 2, HAL_MAX_DELAY);
}

// I2C读一个字节
uint8_t MPU6500_Read_Byte(uint8_t reg) {
  uint8_t data = 0;
  HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, MPU6500_ADDR << 1, &data, 1, HAL_MAX_DELAY);
  return data;
}

// I2C连续读取多个字节（优化版本）
void MPU6500_Read_Bytes(uint8_t reg, uint8_t *data, uint8_t length) {
  HAL_I2C_Master_Transmit(&hi2c1, MPU6500_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c1, MPU6500_ADDR << 1, data, length,
                         HAL_MAX_DELAY);
}

// ==================== 优化的读取原始数据函数（I2C版本） ====================
void MPU6500_Read_Raw_Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                           int16_t *gy, int16_t *gz, int16_t *temp) {
  uint8_t buffer[14]; // 7个数据，每个16位，共14字节

  // 从加速度计X轴高字节开始连续读取14个字节
  MPU6500_Read_Bytes(MPU6500_ACCEL_XOUT_H_REG, buffer, 14);

  // 解析数据（与SPI版本相同）
  *ax = (buffer[0] << 8) | buffer[1];
  *ay = (buffer[2] << 8) | buffer[3];
  *az = (buffer[4] << 8) | buffer[5];
  *temp = (buffer[6] << 8) | buffer[7];
  *gx = (buffer[8] << 8) | buffer[9];
  *gy = (buffer[10] << 8) | buffer[11];
  *gz = (buffer[12] << 8) | buffer[13];
}

// ==================== 初始化函数（I2C版本） ====================
void MPU6500_Init(void) {
  HAL_Delay(100);

  // 复位设备
  MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, PWR1_DEVICE_RESET);
  HAL_Delay(100);

  // 唤醒设备，选择内部时钟源
  MPU6500_Write_Byte(MPU6500_PWR_MGMT_1_REG, PWR1_CLKSEL_INTERNAL);

  // I2C模式下不需要禁用I2C接口（与SPI不同）
  // 设置用户控制寄存器：使能FIFO，复位FIFO
  MPU6500_Write_Byte(MPU6500_USER_CTRL_REG,
                     USER_CTRL_FIFO_EN | USER_CTRL_FIFO_RST);

  // 配置数字低通滤波器（陀螺仪和加速度计都使用92Hz带宽）
  MPU6500_Write_Byte(MPU6500_CONFIG_REG, DLPF_BW_92HZ);

  // 配置陀螺仪量程 ±250°/s
  MPU6500_Write_Byte(MPU6500_GYRO_CONFIG_REG, GYRO_FS_SEL_250DPS);

  // 配置加速度计量程 ±2g
  MPU6500_Write_Byte(MPU6500_ACCEL_CONFIG_REG, ACCEL_FS_SEL_2G);

  // 配置加速度计低通滤波器
  MPU6500_Write_Byte(MPU6500_ACCEL_CONFIG2_REG, DLPF_BW_92HZ);

  // 设置采样率分频器（0 = 1kHz）
  MPU6500_Write_Byte(MPU6500_SMPLRT_DIV_REG, 0);

  // 禁用所有轴（准备校准）
  MPU6500_Write_Byte(MPU6500_PWR_MGMT_2_REG,
                     PWR2_DISABLE_XA | PWR2_DISABLE_YA | PWR2_DISABLE_ZA |
                         PWR2_DISABLE_XG | PWR2_DISABLE_YG | PWR2_DISABLE_ZG);

  HAL_Delay(50);

  // 重新启用所有传感器
  MPU6500_Write_Byte(MPU6500_PWR_MGMT_2_REG, 0x00);

  HAL_Delay(50);
}

void MPU6500_Init_With_Calibration(void) {
  MPU6500_Init();
  HAL_Delay(1000);
  MPU6500_Calibrate_Gyro(200);
}

// ==================== 校准函数（保持不变） ====================
void MPU6500_Calibrate_Gyro(uint16_t samples) {
  int32_t gyro_sum[3] = {0, 0, 0};
  int16_t gx, gy, gz, ax, ay, az, temp;

  for (uint16_t i = 0; i < samples; i++) {
    MPU6500_Read_Raw_Data(&ax, &ay, &az, &gx, &gy, &gz, &temp);
    gyro_sum[0] += gx;
    gyro_sum[1] += gy;
    gyro_sum[2] += gz;
    HAL_Delay(5);
  }

  gyro_offset[0] = (int16_t)(gyro_sum[0] / samples);
  gyro_offset[1] = (int16_t)(gyro_sum[1] / samples);
  gyro_offset[2] = (int16_t)(gyro_sum[2] / samples);
  is_calibrated = 1;
}

// ==================== 快速反正切近似（保持不变） ====================
static float fast_atan2(float y, float x) {
  float abs_y = y < 0 ? -y : y;
  float abs_x = x < 0 ? -x : x;
  float a, r, angle;

  if (abs_y < 1e-10f && abs_x < 1e-10f) {
    return 0.0f;
  }

  if (abs_x > abs_y) {
    a = abs_y / abs_x;
    float a2 = a * a;
    r = a - 0.28f * a * a2 / (1.0f + 0.28f * a2);
  } else {
    a = abs_x / abs_y;
    float a2 = a * a;
    r = PI_2 - (a - 0.28f * a * a2 / (1.0f + 0.28f * a2));
  }

  if (x < 0) {
    angle = PI - r;
  } else {
    angle = r;
  }

  if (y < 0) {
    angle = -angle;
  }

  return angle * RAD_TO_DEG;
}

// ==================== 快速平方根近似（保持不变） ====================
static float fast_sqrt(float x) {
  if (x <= 0.0f)
    return 0.0f;

  union {
    float f;
    uint32_t i;
  } u;

  u.f = x;
  u.i = 0x5f3759df - (u.i >> 1);
  u.f = u.f * (1.5f - 0.5f * x * u.f * u.f);
  return 1.0f / u.f;
}

// ==================== 主数据处理函数（保持不变） ====================
void MPU6500_Get_All_Data(MPU6500_Data *data) {
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, temp_raw;

  // 1. 读取原始数据
  MPU6500_Read_Raw_Data(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw,
                        &temp_raw);

  // 2. 温度转换
  // data->temp = ((float)temp_raw * 0.002994f) + 21.0f;

  // 3. 加速度计数据转换
  data->x_acc = (float)ax_raw * INV_ACCEL_SCALE;
  data->y_acc = (float)ay_raw * INV_ACCEL_SCALE;
  data->z_acc = (float)az_raw * INV_ACCEL_SCALE;

  // 4. 陀螺仪校准和转换
  float gyro_x_dps, gyro_y_dps, gyro_z_dps;
  if (is_calibrated) {
    gyro_x_dps = (float)(gx_raw - gyro_offset[0]) * INV_GYRO_SCALE;
    gyro_y_dps = (float)(gy_raw - gyro_offset[1]) * INV_GYRO_SCALE;
    gyro_z_dps = (float)(gz_raw - gyro_offset[2]) * INV_GYRO_SCALE;
  } else {
    gyro_x_dps = (float)gx_raw * INV_GYRO_SCALE;
    gyro_y_dps = (float)gy_raw * INV_GYRO_SCALE;
    gyro_z_dps = (float)gz_raw * INV_GYRO_SCALE;
  }

  // 5. 死区滤波
  const float GYRO_DEADBAND = 0.5f;
  const float GYRO_DEADBAND_NEG = -GYRO_DEADBAND;
  if (gyro_x_dps < GYRO_DEADBAND && gyro_x_dps > GYRO_DEADBAND_NEG)
    gyro_x_dps = 0.0f;
  if (gyro_y_dps < GYRO_DEADBAND && gyro_y_dps > GYRO_DEADBAND_NEG)
    gyro_y_dps = 0.0f;
  if (gyro_z_dps < GYRO_DEADBAND && gyro_z_dps > GYRO_DEADBAND_NEG)
    gyro_z_dps = 0.0f;

  // 6. 从加速度计计算角度
  float y_acc = data->y_acc;
  float z_acc = data->z_acc;
  float accel_roll, accel_pitch;

  if (z_acc != 0.0f) {
    float tan_roll = y_acc / z_acc;
    if (tan_roll > 1.0f)
      tan_roll = 1.0f;
    if (tan_roll < -1.0f)
      tan_roll = -1.0f;
    accel_roll = tan_roll * RAD_TO_DEG;
    if (tan_roll > 0.5f || tan_roll < -0.5f) {
      accel_roll = fast_atan2(y_acc, z_acc);
    }
  } else {
    accel_roll = (y_acc > 0) ? 90.0f : -90.0f;
  }

  float xz_sq = y_acc * y_acc + z_acc * z_acc;
  if (xz_sq > 0.0f) {
    float tan_pitch = -data->x_acc / fast_sqrt(xz_sq);
    if (tan_pitch > 1.0f)
      tan_pitch = 1.0f;
    if (tan_pitch < -1.0f)
      tan_pitch = -1.0f;
    accel_pitch = tan_pitch * RAD_TO_DEG;
    if (tan_pitch > 0.5f || tan_pitch < -0.5f) {
      accel_pitch = fast_atan2(-data->x_acc, fast_sqrt(xz_sq));
    }
  } else {
    accel_pitch = (data->x_acc > 0) ? -90.0f : 90.0f;
  }

  // 7. 互补滤波器
  if (last_data.roll == 0.0f && last_data.pitch == 0.0f) {
    data->roll = accel_roll;
    data->pitch = accel_pitch;
    data->yaw = 0.0f;
  } else {
    float gyro_roll_integral = last_data.roll + gyro_x_dps * SAMPLE_TIME;
    float gyro_pitch_integral = last_data.pitch + gyro_y_dps * SAMPLE_TIME;

    data->roll =
        COMP_FILTER_ALPHA * gyro_roll_integral + INV_COMP_FILTER * accel_roll;
    data->pitch =
        COMP_FILTER_ALPHA * gyro_pitch_integral + INV_COMP_FILTER * accel_pitch;
    data->yaw = last_data.yaw + gyro_z_dps * SAMPLE_TIME;

    // 角度范围限制
    if (data->roll > 180.0f) {
      data->roll -= 360.0f;
    } else if (data->roll < -180.0f) {
      data->roll += 360.0f;
    }
    if (data->pitch > 180.0f) {
      data->pitch -= 360.0f;
    } else if (data->pitch < -180.0f) {
      data->pitch += 360.0f;
    }
    if (data->yaw > 180.0f) {
      data->yaw -= 360.0f;
    } else if (data->yaw < -180.0f) {
      data->yaw += 360.0f;
    }
  }

  // 8. 更新EWMA用于静止检测
  float current_accel[3] = {data->x_acc, data->y_acc, data->z_acc};
  float current_gyro[3] = {gyro_x_dps, gyro_y_dps, gyro_z_dps};
  for (int i = 0; i < 3; i++) {
    float accel_diff = current_accel[i] - ewma_accel[i];
    ewma_accel[i] += EWMA_ALPHA * accel_diff;
    ewma_accel_var[i] =
        EWMA_BETA * (ewma_accel_var[i] + EWMA_ALPHA * accel_diff * accel_diff);

    float gyro_diff = current_gyro[i] - ewma_gyro[i];
    ewma_gyro[i] += EWMA_ALPHA * gyro_diff;
    ewma_gyro_var[i] =
        EWMA_BETA * (ewma_gyro_var[i] + EWMA_ALPHA * gyro_diff * gyro_diff);
  }

  // 9. 保存本次数据
  last_data = *data;
}

// ==================== 静止检测函数（保持不变） ====================
int MPU6500_Is_Device_Stationary(void) {
  float total_accel_var =
      ewma_accel_var[0] + ewma_accel_var[1] + ewma_accel_var[2];
  float total_gyro_var = ewma_gyro_var[0] + ewma_gyro_var[1] + ewma_gyro_var[2];
  const float ACCEL_VAR_THRESH = 0.001f;
  const float GYRO_VAR_THRESH = 0.1f;

  if (total_accel_var < ACCEL_VAR_THRESH && total_gyro_var < GYRO_VAR_THRESH) {
    stationary_counter++;
    if (stationary_counter > 100)
      stationary_counter = 100;
  } else {
    stationary_counter = 0;
  }

  return stationary_counter > 20;
}

// ==================== 辅助函数 ====================
void MPU6500_Compute_Angles(MPU6500_Data *data) { MPU6500_Get_All_Data(data); }

// 设备ID检查
uint8_t MPU6500_Check_ID(void) {
  uint8_t id = MPU6500_Read_Byte(MPU6500_WHO_AM_I_REG);
  return (id == MPU6500_DEVICE_ID);
}