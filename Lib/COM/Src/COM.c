#include "COM.h"
#include "Motor.h"
#include "PID.h"
#include "main.h"
#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f1xx_hal_uart.h"
#include "string.h"

extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern volatile uint8_t print_flag;
extern volatile uint8_t stop_flag;

extern PID_t PID;

extern volatile int16_t M_speed_L;
extern volatile int16_t M_speed_R;

extern volatile float target_L;
extern volatile float target_R;

extern PID_t pid_L;
extern PID_t pid_R;

// 通信消息格式化（也便于vofa解析）：#消息类型|长度:内容\n
//                                   ^数字   ^数字

/**
 * @brief 向上位机发送格式化消息
 * @param msg_id 消息类型枚举
 * @param info 消息内容字符串
 * @retval 1:成功 0:失败
 * @note 格式: #消息ID|长度:内容\n
 */
uint8_t msg(UART_MSG_ID_t msg_id, const char *info) {
  if (info == NULL) {
    return 0;
  }

  char tx_buffer[128];
  size_t info_len = strlen(info);

  // 格式化消息: #ID|LEN:INFO\n
  int total_len = snprintf(tx_buffer, sizeof(tx_buffer), "#%d|%d:%s\n",
                           (int)msg_id, (int)info_len, info);

  // 检查缓冲区溢出
  if (total_len < 0 || total_len >= (int)sizeof(tx_buffer)) {
    // 发送错误消息（手动构造避免递归）
    const char *overflow_msg = "#2|18:MSG_BUFFER_OVERFLOW\n";
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)overflow_msg,
                          strlen(overflow_msg));
    return 0;
  }

  // 发送消息
  HAL_StatusTypeDef status =
      HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buffer, total_len);
  return (status == HAL_OK) ? 1 : 0;
}

/**
 * @brief 向上位机发送带格式的格式化消息
 * @param msg_id 消息类型枚举
 * @param ··· 参数列表
 * @param format 指定要显示的格式化字符串
 * @retval 1:成功 0:失败
 */
uint8_t msgf(UART_MSG_ID_t msg_id, char *format, ...) {
  char String[128];      // 定义字符数组
  va_list arg;           // 定义可变参数列表数据类型的变量arg
  va_start(arg, format); // 从format开始，接收参数列表到arg变量
  vsprintf(String, format,
           arg); // 使用vsprintf打印格式化字符串和参数列表到字符数组中
  va_end(arg);   // 结束变量arg
  return msg(msg_id, String);
}

/**
 * @brief 发送速度数据的快速函数
 * @param speed_L 左轮速度
 * @param speed_R 右轮速度
 * @param target 目标速度
 */
void msg_speed(float speed_L, float speed_R, float target) {
  char buf[48];
  snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f", speed_L, speed_R, target);
  msg(MSG_SPEED, buf);
}

/**
 * @brief 发送陀螺仪数据的快速函数
 * @param yaw 偏航角
 * @param pitch 俯仰角
 * @param roll 翻滚角
 */
void msg_gyroscope(float yaw, float pitch, float roll) {
  char buf[48] = {'\0'};
  char buf1[8] = {'\0'};
  char buf2[8] = {'\0'};
  char buf3[8] = {'\0'};
  snprintf(buf, sizeof(buf), "%s,%s,%s", float_to_string(yaw, buf1, 2),
           float_to_string(pitch, buf2, 2), float_to_string(roll, buf3, 2));
  msg(MSG_GYROSCOPE, buf);
}

/**
 * @brief 发送加速度仪数据的快速函数
 * @param x_acc x加速度
 * @param y_acc y加速度
 * @param z_acc z加速度
 */
void msg_acceleration(float x_acc, float y_acc, float z_acc) {
  char buf[48] = {'\0'};
  char buf1[8] = {'\0'};
  char buf2[8] = {'\0'};
  char buf3[8] = {'\0'};
  snprintf(buf, sizeof(buf), "%s,%s,%s", float_to_string(x_acc, buf1, 2),
           float_to_string(y_acc, buf2, 2), float_to_string(z_acc, buf3, 2));
  msg(MSG_ACCELERATION, buf);
}

/**
 * @brief 去除字符串末尾的换行符
 */
static void trim_newline(char *str) {
  size_t len = strlen(str);
  while (len > 0 && (str[len - 1] == '\n' || str[len - 1] == '\r')) {
    str[--len] = '\0';
  }
}

/**
 * @brief 解析上位机指令
 * @param cmd 指令字符串
 * @retval 1:解析成功 0:解析失败
 * @note 格式: #指令ID:参数\n 或 #指令ID\n
 */
uint8_t cmd_parser(const char *cmd) {
  if (cmd == NULL || cmd[0] != '#') {
    return 0;
  }

  // 复制到本地缓冲区处理
  char cmd_buf[64];
  strncpy(cmd_buf, cmd, sizeof(cmd_buf) - 1);
  cmd_buf[sizeof(cmd_buf) - 1] = '\0';
  trim_newline(cmd_buf);

  // 解析指令ID和参数
  int cmd_id = -1;
  char param[32] = {0};

  // 查找冒号
  char *colon_ptr = strchr(cmd_buf, ':');

  if (colon_ptr == NULL) {
    // 无参数指令: #ID
    if (sscanf(cmd_buf, "#%d", &cmd_id) != 1) {
      msg(MSG_ERROR, "Invalid cmd format");
      return 0;
    }
  } else {
    // 有参数指令: #ID:PARAM
    *colon_ptr = '\0'; // 分割字符串
    if (sscanf(cmd_buf, "#%d", &cmd_id) != 1) {
      msg(MSG_ERROR, "Invalid cmd ID");
      return 0;
    }
    strncpy(param, colon_ptr + 1, sizeof(param) - 1);
  }

  // 根据指令ID处理
  char response[64];

  switch ((UART_CMD_ID_t)cmd_id) {

  case CMD_OTA:
    msg(MSG_OTA, "Entering OTA mode");
    // TODO: 跳转到Bootloader
    return 1;

  case CMD_SET_SPEED: {
    if (strlen(param) == 0) {
      msg(MSG_ERROR, "GO_STRAIGHT needs speed");
      return 0;
    }
    float speed = atof(param);

    target_L = speed;
    target_R = speed;

    return 1;
  }
  case CMD_GO_STRAIGHT: {
    if (strlen(param) == 0) {
      msg(MSG_ERROR, "GO_STRAIGHT needs speed");
      return 0;
    }
    float speed = atof(param);

    // 设置PID目标速度
    target_L = speed;
    target_R = speed;
    stop_flag = 0;

    // 两轮同向同速（直行）
    // 注意：实际控制由PID在定时器中断中完成

    snprintf(response, sizeof(response), "Straight:%.2f", speed);
    msg(MSG_LOG, response);
    return 1;
  }

  case CMD_TURN_LEFT: {
    float angle = strlen(param) > 0 ? atof(param) : 45.0f;
    stop_flag = 0;

    // 差速转向：左轮慢/反转，右轮快
    // 这里直接设置PWM，不经过PID
    SetSpeed(&htim1, -50); // 左轮
    SetSpeed(&htim4, 150); // 右轮

    snprintf(response, sizeof(response), "TurnL:%.1f", angle);
    msg(MSG_LOG, response);
    return 1;
  }

  case CMD_TURN_RIGHT: {
    float angle = strlen(param) > 0 ? atof(param) : 45.0f;
    stop_flag = 0;

    // 差速转向：左轮快，右轮慢/反转
    SetSpeed(&htim1, 150); // 左轮
    SetSpeed(&htim4, -50); // 右轮

    snprintf(response, sizeof(response), "TurnR:%.1f", angle);
    msg(MSG_LOG, response);
    return 1;
  }

  case CMD_STOP:
    stop_flag = 1;
    msg(MSG_LOG, "STOPPED");
    return 1;

  case CMD_START:
    stop_flag = 0;
    msg(MSG_LOG, "STARTED");
    return 1;

  case CMD_SET_PID: {
    char target;
    float p, i, d;
    if (sscanf(param, "%c,%f,%f,%f", &target, &p, &i, &d) == 3) {
      if (target == 'R') {
        pid_R.Kp = p;
        pid_R.Ki = i;
        pid_R.Kd = d;

        pid_R.I = 0;
        pid_R.Err = 0;
        pid_R.Err1 = 0;
        pid_R.Out = 0;
      } else if (target == 'L') {
        pid_L.Kp = p;
        pid_L.Ki = i;
        pid_L.Kd = d;

        pid_L.I = 0;
        pid_L.Err = 0;
        pid_L.Err1 = 0;
        pid_L.Out = 0;
      }

      snprintf(response, sizeof(response), "PID:%c:%.3f,%.3f,%.3f", target, p,
               i, d);
      msg(MSG_LOG, response);
      return 1;
    } else {
      msg(MSG_ERROR, "PID format:P,I,D");
      return 0;
    }
  }

  case CMD_GET_SPEED: {
    float speed_L = (float)M_speed_L / ENCODER_PPR * SPEED_SCALE;
    float speed_R = (float)M_speed_R / ENCODER_PPR * SPEED_SCALE;
    float target = PID.Exp / ENCODER_PPR * SPEED_SCALE;

    msg_speed(speed_L, speed_R, target);
    return 1;
  }

  case CMD_SET_TARGET_SPEED: {
    if (strlen(param) == 0) {
      msg(MSG_ERROR, "Need target speed");
      return 0;
    }
    float target = atof(param);
    // 将实际速度转换为脉冲数
    PID.Exp = target / SPEED_SCALE * ENCODER_PPR;

    snprintf(response, sizeof(response), "Target:%.2f", target);
    msg(MSG_LOG, response);
    return 1;
  }
  case CMD_PING: {
    msg(MSG_LOG, "Ack Ping");
    return 1;
  }

  default: {
    snprintf(response, sizeof(response), "Unknown CMD:%d", cmd_id);
    msg(MSG_ERROR, response);
    return 0;
  }
  }
}