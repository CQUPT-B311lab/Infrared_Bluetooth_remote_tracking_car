#include "COM.h"
#include "MPU6500.h"
#include "Motor.h"
#include "PID.h"
#include "main.h"
#include "stdarg.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f1xx_hal_uart.h"
#include "string.h"
#include <stdint.h>
#include <sys/_intsup.h>

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern volatile uint8_t print_flag;
extern volatile uint8_t stop_flag;
extern volatile int16_t M_speed_L;
extern volatile int16_t M_speed_R;

extern volatile float target_L; // 单位：脉冲/10ms
extern volatile float target_R; // 单位：脉冲/10ms
extern volatile float target_Y;

extern PID_t pid_L;
extern PID_t pid_R;
extern PID_t pid_yaw;
extern PID_t pid_trace;

extern uint8_t joystick_mode;

extern uint8_t turn_l_flag;
extern uint8_t turn_r_flag;

extern volatile int16_t target_d;
extern uint8_t distance_mode;

extern uint8_t trace_mode;

extern uint16_t black_threshold;

/* ========================= 工具函数 ========================= */

#ifndef COM_TX_BUF_SIZE
#define COM_TX_BUF_SIZE 160
#endif

static void trim_newline(char *str) {
  size_t len = strlen(str);
  while (len > 0 && (str[len - 1] == '\n' || str[len - 1] == '\r')) {
    str[--len] = '\0';
  }
}

/* 发送忙判断：DMA在发送时避免覆盖/乱序。此处用UART状态做轻量保护 */
static uint8_t uart2_tx_ready(void) {
  return (huart2.gState == HAL_UART_STATE_READY) ? 1 : 0;
}
/* ========================= 消息发送 ========================= */
/**
 * @brief 向上位机发送格式化消息
 * @param msg_id 消息类型枚举
 * @param info 消息内容字符串
 * @retval 1:成功 0:失败
 * @note 格式: #消息ID|长度:内容\n
 */
uint8_t msg(UART_MSG_ID_t msg_id, const char *info) {
  if (info == NULL)
    return 0;

  if (!uart2_tx_ready()) {
    // 忙则丢弃（也可改为阻塞发送/排队）
    return 0;
  }

  static char tx_buffer[COM_TX_BUF_SIZE];
  size_t info_len = strlen(info);

  int total_len = snprintf(tx_buffer, sizeof(tx_buffer), "#%d|%d:%s\n",
                           (int)msg_id, (int)info_len, info);
  if (total_len < 0 || total_len >= (int)sizeof(tx_buffer)) {
    const char *overflow_msg = "#2|18:MSG_BUFFER_OVERFLOW\n";
    if (uart2_tx_ready()) {
      HAL_UART_Transmit_DMA(&huart2, (uint8_t *)overflow_msg,
                            (uint16_t)strlen(overflow_msg));
    }
    return 0;
  }

  return (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)tx_buffer,
                                (uint16_t)total_len) == HAL_OK)
             ? 1
             : 0;
}

/**
 * @brief 向上位机发送带格式的格式化消息
 * @param msg_id 消息类型枚举
 * @param format 指定要显示的格式化字符串
 * @param ... 参数
 * @retval 1:成功 0:失败
 */
uint8_t msgf(UART_MSG_ID_t msg_id, char *format, ...) {
  if (format == NULL)
    return 0;

  char String[128];
  va_list arg;
  va_start(arg, format);
  vsnprintf(String, sizeof(String), format, arg);
  va_end(arg);

  return msg(msg_id, String);
}

/**
 * @brief 发送速度数据的快速函数
 * @param speed_L 左轮速度
 * @param speed_R 右轮速度
 * @param target 目标速度（同单位）
 */
void msg_speed(float speed_L, float speed_R, float target) {
  char buf[64];
  snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f", speed_L, speed_R, target);
  msg(MSG_SPEED, buf);
}

/**
 * @brief 发送陀螺仪数据的快速函数
 */
void msg_gyroscope(float yaw, float pitch, float roll) {
  char buf[64] = {'\0'};
  char buf1[16] = {'\0'};
  char buf2[16] = {'\0'};
  char buf3[16] = {'\0'};

  snprintf(buf, sizeof(buf), "%s,%s,%s", float_to_string(yaw, buf1, 2),
           float_to_string(pitch, buf2, 2), float_to_string(roll, buf3, 2));
  msg(MSG_GYROSCOPE, buf);
}

/**
 * @brief 发送加速度仪数据的快速函数
 */
void msg_acceleration(float x_acc, float y_acc, float z_acc) {
  char buf[64] = {'\0'};
  char buf1[16] = {'\0'};
  char buf2[16] = {'\0'};
  char buf3[16] = {'\0'};

  snprintf(buf, sizeof(buf), "%s,%s,%s", float_to_string(x_acc, buf1, 2),
           float_to_string(y_acc, buf2, 2), float_to_string(z_acc, buf3, 2));
  msg(MSG_ACCELERATION, buf);
}

/**
 * @brief 周期性串口流输出
 * @note 单位：脉冲/10ms
 */
void COM_StreamTick(void) {
  // 例：输出测量与目标（脉冲/10ms）
  msgf(MSG_LOG, "measL:%d,measR:%d,tL:%.1f,tR:%.1f", (int)M_speed_L,
       (int)M_speed_R, target_L, target_R);
}

/* ========================= 指令解析 ========================= */
/**
 * @brief 解析上位机指令
 * @param cmd 指令字符串
 * @retval 1:解析成功 0:解析失败
 * @note 格式:
 *   - 无参数: #ID
 *   - 有参数: #ID:PARAM
 *
 * 建议参数格式：
 *   CMD_SET_SPEED / CMD_GO_STRAIGHT:  #<id>:<v>            (v=脉冲/10ms)
 *   设左右轮不同目标：                #<id>:L,R             (L,R=脉冲/10ms)
 *   CMD_SET_PID:                      #<id>:L,kp,ki,kd 或 R,kp,ki,kd
 */
uint8_t cmd_parser(const char *cmd) {
  if (joystick_mode) {

    return 1;
  } else {

    if (cmd == NULL || cmd[0] != '#')
      return 0;

    char cmd_buf[64];
    strncpy(cmd_buf, cmd, sizeof(cmd_buf) - 1);
    cmd_buf[sizeof(cmd_buf) - 1] = '\0';
    trim_newline(cmd_buf);

    int cmd_id = -1;
    char param[40] = {0};

    char *colon_ptr = strchr(cmd_buf, ':');
    if (colon_ptr == NULL) {
      if (sscanf(cmd_buf, "#%d", &cmd_id) != 1) {
        msg(MSG_ERROR, "Invalid cmd format");
        return 0;
      }
    } else {
      *colon_ptr = '\0';
      if (sscanf(cmd_buf, "#%d", &cmd_id) != 1) {
        msg(MSG_ERROR, "Invalid cmd ID");
        return 0;
      }
      strncpy(param, colon_ptr + 1, sizeof(param) - 1);
      trim_newline(param);
    }

    char response[96];

    switch ((UART_CMD_ID_t)cmd_id) {
    case CMD_OTA:
      msg(MSG_OTA, "Entering OTA mode");
      // TODO: 跳转到Bootloader
      return 1;

    case CMD_SET_SPEED: {
      // 支持两种格式：
      //  1) #ID:30        -> L=30,R=30
      //  2) #ID:30,28     -> L=30,R=28
      if (strlen(param) == 0) {
        msg(MSG_ERROR, "SET_SPEED needs param");
        return 0;
      }

      float l = 0, r = 0;
      int n = sscanf(param, "%f,%f\n", &l, &r);
      l = V_TO_TICK(l);
      r = V_TO_TICK(r);
      if (n == 1) {
        target_L = l;
        target_R = l;
        pid_yaw.Outmax = l < 5 ? 10 : l * 2;
        pid_yaw.Outmin = l < 5 ? -10 : -l * 2;
      } else if (n == 2) {
        target_L = l;
        target_R = r;
        float minspeed = l > r ? r : l;
        pid_yaw.Outmax = minspeed < 5 ? 10 : 2 * minspeed;
        pid_yaw.Outmin = minspeed < 5 ? -10 : -2 * minspeed;
      } else {
        msg(MSG_ERROR, "SET_SPEED format:v or vL,vR");
        return 0;
      }

      snprintf(response, sizeof(response), "SetSpeed L=%.1f R=%.1f(pulse/10ms)",
               target_L, target_R);
      msg(MSG_LOG, response);
      return 1;
    }

    case CMD_GO_STRAIGHT: {
      if (strlen(param) == 0) {
        msg(MSG_ERROR, "GO_STRAIGHT needs speed");
        return 0;
      }
      float speed = atof(param);
      target_L = speed;
      target_R = speed;
      stop_flag = 0;
      snprintf(response, sizeof(response), "Straight:%.1f(pulse/10ms)", speed);
      msg(MSG_LOG, response);
      return 1;
    }

    case CMD_TURN_LEFT: {
      turn_l_flag = 1;
      turn_r_flag = 0;
      msg(MSG_LOG, "Ack");
      return 1;
    }

    case CMD_TURN_RIGHT: {
      turn_l_flag = 0;
      turn_r_flag = 1;
      msg(MSG_LOG, "Ack");
      return 1;
    }

    case CMD_STOP_TURN: {
      turn_l_flag = 0;
      turn_r_flag = 0;
      return 0;
    }

    case CMD_SET_DISTANCE: {
      if (strlen(param) == 0) {
        msg(MSG_ERROR, "Need target speed");
        return 0;
      }
      float distance = atof(param);

      target_d = (int16_t)((distance / (2 * PI * R_TIRE)) * PLUS_PER_CYC);
      distance_mode = 1;

      return 0;
    }

    case CMD_STOP:
      stop_flag = 1;
      target_L = 0;
      target_R = 0;
      msg(MSG_LOG, "STOPPED");
      return 1;

    case CMD_START:
      stop_flag = 0;
      msg(MSG_LOG, "STARTED");
      return 1;

    case CMD_SET_PID: {
      // 期望格式：L,kp,ki,kd 或 R,kp,ki,kd
      char side = 0;
      int p = 0, i = 0, d = 0;

      int n = sscanf(param, "%c,%d,%d,%d", &side, &p, &i, &d);
      if (n != 4) {
        msg(MSG_ERROR, "PID format:L,kp,ki,kd");
        return 0;
      }

      PID_t *pid = NULL;
      if (side == 'L' || side == 'l')
        pid = &pid_L;
      if (side == 'R' || side == 'r')
        pid = &pid_R;
      if (side == 'Y' || side == 'y')
        pid = &pid_yaw;
      if (side == 'T' || side == 't')
        pid = &pid_trace;

      if (pid == NULL) {
        msg(MSG_ERROR, "PID side must be L/R/Y");
        return 0;
      }

      pid->Kp = p;
      pid->Ki = i;
      pid->Kd = d;

      // 清状态，避免参数切换引发冲击
      pid->I = 0;
      pid->Err = 0;
      pid->Err1 = 0;
      pid->Out = 0;

      snprintf(response, sizeof(response), "PID %c=%d,%d,%d       ", side, p, i,
               d);
      msg(MSG_LOG, response);
      return 1;
    }

    case CMD_GET_PID: {
      if (strlen(param) == 0) {
        msg(MSG_ERROR, "SET_SPEED needs param");
        return 0;
      }
      char target;
      sscanf(param, "%c", &target);
      char float_buf1[8];
      char float_buf2[8];
      char float_buf3[8];
      PID_t *pid = NULL;
      if (target == 'L') {
        pid = &pid_L;
      }
      if (target == 'R') {
        pid = &pid_R;
      }
      if (target == 'Y') {
        pid = &pid_yaw;
      }

      msgf(MSG_LOG, "P:%s, I:%s, D:%s", float_to_string(pid->Kp, float_buf1, 2),
           float_to_string(pid->Ki, float_buf2, 2),
           float_to_string(pid->Kd, float_buf3, 2));
      return 1;
    }

    case CMD_GET_SPEED: {
      msgf(MSG_SPEED, "%d,%d,%.1f,%.1f", (int)M_speed_L, (int)M_speed_R,
           target_L, target_R);
      // 格式：measL,measR,targetL,targetR
      return 1;
    }

    case CMD_SET_TARGET_SPEED: {
      if (strlen(param) == 0) {
        msg(MSG_ERROR, "Need target speed");
        return 0;
      }
      float t = atof(param);
      stop_flag = 1;
      target_L = V_TO_TICK(t);
      target_R = target_L;
      pid_yaw.Outmax = target_L < 5 ? 10 : target_L * 2;
      pid_yaw.Outmin = target_L < 5 ? -10 : -target_L * 2;
      snprintf(response, sizeof(response), "Target:%.1f(pulse/10ms)", t);
      msg(MSG_LOG, response);
      return 1;
    }

    case CMD_PING:
      msg(MSG_LOG, "Ack Ping");
      return 1;

    case CMD_RESET_PID: {
      __disable_irq();
      pid_L.I = pid_L.Err = pid_L.Err1 = pid_L.Out = 0;
      pid_R.I = pid_R.Err = pid_R.Err1 = pid_R.Out = 0;
      __enable_irq();
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      HAL_Delay(200);
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      msg(MSG_LOG, "PID reset");
      return 1;
    }

    case CMD_SET_YAW: {
      if (strlen(param) == 0) {
        msg(MSG_ERROR, "Need target speed");
        return 0;
      }

      float y = atof(param);
      target_Y = y;
      stop_flag = 1;

      return 1;
    }

    case CMD_SET_THRESHOLD: {
      if (strlen(param) == 0) {
        msg(MSG_ERROR, "Need target speed");
        return 0;
      }

      uint16_t thr = atol(param);
      black_threshold = thr;
      return 1;
    }

    case CMD_TRACE_MODE: {
      if (trace_mode == 1) {
        trace_mode = 0;
      } else {
        trace_mode = 1;
      }
      return 1;
    }

    default:
      snprintf(response, sizeof(response), "Unknown CMD:%d", cmd_id);
      msg(MSG_ERROR, response);
      return 0;
    }
  }
}