#ifndef __COM_H__
#define __COM_H__

#include "stdint.h"

// 向上位机发送的消息类型
typedef enum {
  MSG_LOG = 0,
  MSG_DEBUG = 1,
  MSG_ERROR = 2,
  MSG_OTA = 3,
  MSG_SPEED = 4,        // 小车2个轮子实际速度与目标速度
  MSG_GYROSCOPE = 5,    // 陀螺仪数据
  MSG_ACCELERATION = 6, // 加速度仪数据
  MSG_OTA_RETRY = 7,
  MSG_OTA_ACK = 8,
} UART_MSG_ID_t;

// 上位机指令
typedef enum {
  // 基本指令
  CMD_OTA = 0,
  CMD_SET_SPEED = 1,
  CMD_GO_STRAIGHT = 2,
  CMD_TURN_LEFT = 3,
  CMD_TURN_RIGHT = 4,
  // 扩展指令
  CMD_STOP = 10,
  CMD_START = 11,
  CMD_SET_PID = 12,
  CMD_GET_SPEED = 13,
  CMD_SET_TARGET_SPEED = 14,
  CMD_GET_PID = 15,   // 获取当前PID参数
  CMD_RESET_PID = 16, // 重置PID状态
  // 测试指令
  CMD_PING = 20,
} UART_CMD_ID_t;

uint8_t msg(UART_MSG_ID_t msg_id, const char *info);

uint8_t msgf(UART_MSG_ID_t msg_id, char *format, ...);

void msg_speed(float speed_L, float speed_R, float target);

void msg_gyroscope(float yaw, float pitch, float roll);

void msg_acceleration(float x_acc, float y_acc, float z_acc);

uint8_t cmd_parser(const char *cmd);

// 可选连续上报函数
void COM_StreamTick(void);

#endif