#ifndef __OTA_H__
#define __OTA_H__

#include "main.h"
#include <stdint.h>

// 包头
#define OTA_HEAD_0 0xAA
#define OTA_HEAD_1 0x55

// 命令类型
typedef enum {
  OTA_CMD_START = 0x01,  // 开始OTA，携带固件总大小和CRC
  OTA_CMD_DATA = 0x02,   // 数据包
  OTA_CMD_END = 0x03,    // 传输结束
  OTA_CMD_VERIFY = 0x04, // 验证固件
  OTA_CMD_REBOOT = 0x05, // 重启
  OTA_CMD_ABORT = 0x06,  // 中止OTA
} OTA_Cmd_t;

// 响应类型
typedef enum {
  OTA_RSP_OK = 0x00,          // 成功
  OTA_RSP_ERROR = 0x01,       // 通用错误
  OTA_RSP_CRC_ERROR = 0x02,   // CRC校验失败
  OTA_RSP_SEQ_ERROR = 0x03,   // 序号错误（丢包）
  OTA_RSP_FLASH_ERROR = 0x04, // Flash写入错误
  OTA_RSP_SIZE_ERROR = 0x05,  // 大小错误
  OTA_RSP_VERIFY_FAIL = 0x06, // 固件验证失败
  OTA_RSP_RESEND = 0x10,      // 请求重发
} OTA_Rsp_t;

// OTA状态机
typedef enum {
  OTA_STATE_IDLE = 0,
  OTA_STATE_READY = 1,     // 收到START，准备接收
  OTA_STATE_RECEIVING = 2, // 正在接收数据
  OTA_STATE_VERIFY = 3,    // 验证中
  OTA_STATE_COMPLETE = 4,  // 完成
  OTA_STATE_ERROR = 5,
} OTA_State_t;

// OTA控制结构体
typedef struct {
  OTA_State_t state;
  uint32_t fw_size;       // 固件总大小
  uint32_t fw_crc;        // 固件预期CRC
  uint32_t received_size; // 已接收大小
  uint16_t expected_seq;  // 期望的包序号
  uint16_t total_packets; // 总包数
  uint32_t last_activity; // 最后活动时间（超时检测）
  uint8_t retry_count;    // 重试计数
} OTA_Control_t;

// 配置参数
#define OTA_PACKET_DATA_SIZE 128      // 每包数据大小
#define OTA_PACKET_MAX_SIZE 140       // 包最大大小 (2+1+2+2+128+4+1)
#define OTA_TIMEOUT_MS 5000           // 超时时间
#define OTA_MAX_RETRY 3               // 最大重试次数
#define OTA_APP_START_ADDR 0x08002000 // APP起始地址
#define OTA_APP_MAX_SIZE (56 * 1024)  // APP最大56KB

// 函数声明
void OTA_Init(void);
void OTA_Process(uint8_t *data, uint16_t len);
void OTA_TimeoutCheck(void);
uint8_t OTA_IsActive(void);

#endif