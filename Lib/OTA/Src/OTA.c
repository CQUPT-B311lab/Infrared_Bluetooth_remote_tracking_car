// OTA.c
#include "OTA.h"
#include "COM.h"
#include "stdio.h"
#include <string.h>


extern CRC_HandleTypeDef hcrc;
extern UART_HandleTypeDef huart2;

static OTA_Control_t ota_ctrl = {0};
static uint8_t ota_tx_buf[16]; // 响应包缓冲区

// Flash操作相关
static HAL_StatusTypeDef OTA_FlashErase(uint32_t start_addr, uint32_t size);
static HAL_StatusTypeDef OTA_FlashWrite(uint32_t addr, uint8_t *data,
                                        uint32_t len);
static uint32_t OTA_CalculateCRC(uint32_t addr, uint32_t size);

/**
 * @brief 发送OTA响应包
 */
static void OTA_SendResponse(OTA_Cmd_t cmd, OTA_Rsp_t rsp, uint16_t seq) {
  ota_tx_buf[0] = OTA_HEAD_0;
  ota_tx_buf[1] = OTA_HEAD_1;
  ota_tx_buf[2] = cmd;
  ota_tx_buf[3] = (seq >> 8) & 0xFF;
  ota_tx_buf[4] = seq & 0xFF;
  ota_tx_buf[5] = 0x00; // data_len high
  ota_tx_buf[6] = 0x01; // data_len low = 1
  ota_tx_buf[7] = rsp;  // response code

  // 计算CRC（使用硬件CRC）
  uint32_t crc =
      HAL_CRC_Calculate(&hcrc, (uint32_t *)ota_tx_buf, 2); // 8 bytes / 4
  ota_tx_buf[8] = (crc >> 24) & 0xFF;
  ota_tx_buf[9] = (crc >> 16) & 0xFF;
  ota_tx_buf[10] = (crc >> 8) & 0xFF;
  ota_tx_buf[11] = crc & 0xFF;

  HAL_UART_Transmit(&huart2, ota_tx_buf, 12, 100);
}

/**
 * @brief 发送重发请求
 */
static void OTA_RequestResend(uint16_t seq) {
  if (ota_ctrl.retry_count >= OTA_MAX_RETRY) {
    ota_ctrl.state = OTA_STATE_ERROR;
    msg(MSG_ERROR, "OTA max retry exceeded");
    return;
  }
  ota_ctrl.retry_count++;
  OTA_SendResponse(OTA_CMD_DATA, OTA_RSP_RESEND, seq);
}

/**
 * @brief 验证包CRC
 */
static uint8_t OTA_VerifyPacketCRC(uint8_t *data, uint16_t len) {
  if (len < 6)
    return 0; // 最小包长度

  // 提取包中的CRC（最后4字节）
  uint32_t received_crc =
      ((uint32_t)data[len - 4] << 24) | ((uint32_t)data[len - 3] << 16) |
      ((uint32_t)data[len - 2] << 8) | ((uint32_t)data[len - 1]);

  // 计算数据部分的CRC
  __HAL_CRC_DR_RESET(&hcrc);
  uint32_t calc_crc =
      HAL_CRC_Calculate(&hcrc, (uint32_t *)data, (len - 4 + 3) / 4);

  return (received_crc == calc_crc) ? 1 : 0;
}

/**
 * @brief 初始化OTA
 */
void OTA_Init(void) {
  memset(&ota_ctrl, 0, sizeof(ota_ctrl));
  ota_ctrl.state = OTA_STATE_IDLE;
}

/**
 * @brief 处理OTA_START命令
 * @param data 包含固件大小(4B)和固件CRC(4B)
 */
static void OTA_HandleStart(uint8_t *data, uint16_t data_len) {
  if (data_len != 8) {
    OTA_SendResponse(OTA_CMD_START, OTA_RSP_SIZE_ERROR, 0);
    return;
  }

  // 解析固件大小和CRC
  ota_ctrl.fw_size = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[2] << 8) | ((uint32_t)data[3]);
  ota_ctrl.fw_crc = ((uint32_t)data[4] << 24) | ((uint32_t)data[5] << 16) |
                    ((uint32_t)data[6] << 8) | ((uint32_t)data[7]);

  // 检查固件大小
  if (ota_ctrl.fw_size > OTA_APP_MAX_SIZE || ota_ctrl.fw_size == 0) {
    OTA_SendResponse(OTA_CMD_START, OTA_RSP_SIZE_ERROR, 0);
    return;
  }

  // 计算总包数
  ota_ctrl.total_packets =
      (ota_ctrl.fw_size + OTA_PACKET_DATA_SIZE - 1) / OTA_PACKET_DATA_SIZE;
  ota_ctrl.expected_seq = 0;
  ota_ctrl.received_size = 0;
  ota_ctrl.retry_count = 0;
  ota_ctrl.last_activity = HAL_GetTick();

  // 擦除Flash
  msg(MSG_OTA, "Erasing flash...");
  if (OTA_FlashErase(OTA_APP_START_ADDR, ota_ctrl.fw_size) != HAL_OK) {
    OTA_SendResponse(OTA_CMD_START, OTA_RSP_FLASH_ERROR, 0);
    ota_ctrl.state = OTA_STATE_ERROR;
    return;
  }

  ota_ctrl.state = OTA_STATE_READY;
  OTA_SendResponse(OTA_CMD_START, OTA_RSP_OK, 0);
  msg(MSG_OTA, "Ready to receive");
}

/**
 * @brief 处理数据包
 */
static void OTA_HandleData(uint16_t seq, uint8_t *data, uint16_t data_len) {
  if (ota_ctrl.state != OTA_STATE_READY &&
      ota_ctrl.state != OTA_STATE_RECEIVING) {
    OTA_SendResponse(OTA_CMD_DATA, OTA_RSP_ERROR, seq);
    return;
  }

  ota_ctrl.state = OTA_STATE_RECEIVING;
  ota_ctrl.last_activity = HAL_GetTick();

  // 检查序号
  if (seq != ota_ctrl.expected_seq) {
    if (seq < ota_ctrl.expected_seq) {
      // 重复包，可能是ACK丢失，直接确认
      OTA_SendResponse(OTA_CMD_DATA, OTA_RSP_OK, seq);
    } else {
      // 丢包，请求重发期望的包
      OTA_RequestResend(ota_ctrl.expected_seq);
    }
    return;
  }

  // 写入Flash
  uint32_t write_addr = OTA_APP_START_ADDR + (seq * OTA_PACKET_DATA_SIZE);
  if (OTA_FlashWrite(write_addr, data, data_len) != HAL_OK) {
    OTA_SendResponse(OTA_CMD_DATA, OTA_RSP_FLASH_ERROR, seq);
    ota_ctrl.state = OTA_STATE_ERROR;
    return;
  }

  // 更新状态
  ota_ctrl.received_size += data_len;
  ota_ctrl.expected_seq++;
  ota_ctrl.retry_count = 0;

  // 发送确认
  OTA_SendResponse(OTA_CMD_DATA, OTA_RSP_OK, seq);

  // 进度提示（每10%）
  if ((ota_ctrl.expected_seq * 10 / ota_ctrl.total_packets) >
      ((ota_ctrl.expected_seq - 1) * 10 / ota_ctrl.total_packets)) {
    char progress[32];
    snprintf(progress, sizeof(progress), "Progress: %d%%",
             (int)(ota_ctrl.received_size * 100 / ota_ctrl.fw_size));
    msg(MSG_OTA, progress);
  }
}

/**
 * @brief 处理传输结束
 */
static void OTA_HandleEnd(void) {
  if (ota_ctrl.state != OTA_STATE_RECEIVING) {
    OTA_SendResponse(OTA_CMD_END, OTA_RSP_ERROR, 0);
    return;
  }

  if (ota_ctrl.received_size != ota_ctrl.fw_size) {
    OTA_SendResponse(OTA_CMD_END, OTA_RSP_SIZE_ERROR, 0);
    ota_ctrl.state = OTA_STATE_ERROR;
    return;
  }

  ota_ctrl.state = OTA_STATE_VERIFY;
  OTA_SendResponse(OTA_CMD_END, OTA_RSP_OK, 0);
  msg(MSG_OTA, "Transfer complete");
}

/**
 * @brief 验证固件
 */
static void OTA_HandleVerify(void) {
  if (ota_ctrl.state != OTA_STATE_VERIFY) {
    OTA_SendResponse(OTA_CMD_VERIFY, OTA_RSP_ERROR, 0);
    return;
  }

  msg(MSG_OTA, "Verifying firmware...");

  // 计算已写入固件的CRC
  uint32_t calc_crc = OTA_CalculateCRC(OTA_APP_START_ADDR, ota_ctrl.fw_size);

  if (calc_crc == ota_ctrl.fw_crc) {
    ota_ctrl.state = OTA_STATE_COMPLETE;
    OTA_SendResponse(OTA_CMD_VERIFY, OTA_RSP_OK, 0);
    msg(MSG_OTA, "Verify OK!");
  } else {
    ota_ctrl.state = OTA_STATE_ERROR;
    OTA_SendResponse(OTA_CMD_VERIFY, OTA_RSP_VERIFY_FAIL, 0);
    msg(MSG_ERROR, "Verify FAILED!");
  }
}

/**
 * @brief 重启到新固件
 */
static void OTA_HandleReboot(void) {
  if (ota_ctrl.state != OTA_STATE_COMPLETE) {
    OTA_SendResponse(OTA_CMD_REBOOT, OTA_RSP_ERROR, 0);
    return;
  }

  OTA_SendResponse(OTA_CMD_REBOOT, OTA_RSP_OK, 0);
  msg(MSG_OTA, "Rebooting...");

  HAL_Delay(100); // 等待消息发送完成

  // 系统复位
  NVIC_SystemReset();
}

/**
 * @brief 中止OTA
 */
static void OTA_HandleAbort(void) {
  ota_ctrl.state = OTA_STATE_IDLE;
  memset(&ota_ctrl, 0, sizeof(ota_ctrl));
  OTA_SendResponse(OTA_CMD_ABORT, OTA_RSP_OK, 0);
  msg(MSG_OTA, "OTA Aborted");
}

/**
 * @brief 主处理函数 - 解析并处理OTA包
 */
void OTA_Process(uint8_t *data, uint16_t len) {
  // 检查包头
  if (len < 9 || data[0] != OTA_HEAD_0 || data[1] != OTA_HEAD_1) {
    return; // 不是OTA包
  }

  // 验证CRC
  if (!OTA_VerifyPacketCRC(data, len)) {
    // CRC错误，请求重发当前期望的包
    OTA_SendResponse((OTA_Cmd_t)data[2], OTA_RSP_CRC_ERROR,
                     ota_ctrl.expected_seq);
    return;
  }

  // 解析包
  OTA_Cmd_t cmd = (OTA_Cmd_t)data[2];
  uint16_t seq = ((uint16_t)data[3] << 8) | data[4];
  uint16_t data_len = ((uint16_t)data[5] << 8) | data[6];
  uint8_t *payload = &data[7];

  // 根据命令处理
  switch (cmd) {
  case OTA_CMD_START:
    OTA_HandleStart(payload, data_len);
    break;
  case OTA_CMD_DATA:
    OTA_HandleData(seq, payload, data_len);
    break;
  case OTA_CMD_END:
    OTA_HandleEnd();
    break;
  case OTA_CMD_VERIFY:
    OTA_HandleVerify();
    break;
  case OTA_CMD_REBOOT:
    OTA_HandleReboot();
    break;
  case OTA_CMD_ABORT:
    OTA_HandleAbort();
    break;
  default:
    OTA_SendResponse(cmd, OTA_RSP_ERROR, seq);
    break;
  }
}

/**
 * @brief 超时检测 - 在主循环中调用
 */
void OTA_TimeoutCheck(void) {
  if (ota_ctrl.state == OTA_STATE_RECEIVING ||
      ota_ctrl.state == OTA_STATE_READY) {
    if (HAL_GetTick() - ota_ctrl.last_activity > OTA_TIMEOUT_MS) {
      msg(MSG_ERROR, "OTA timeout");
      OTA_HandleAbort();
    }
  }
}

/**
 * @brief 检查OTA是否激活
 */
uint8_t OTA_IsActive(void) { return (ota_ctrl.state != OTA_STATE_IDLE); }

// ==================== Flash操作 ====================

/**
 * @brief 擦除Flash
 */
static HAL_StatusTypeDef OTA_FlashErase(uint32_t start_addr, uint32_t size) {
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase_init;
  uint32_t page_error;

  // 计算需要擦除的页数（STM32F103C8每页1KB）
  uint32_t num_pages = (size + 1023) / 1024;

  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = start_addr;
  erase_init.NbPages = num_pages;

  HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_init, &page_error);

  HAL_FLASH_Lock();
  return status;
}

/**
 * @brief 写入Flash（按半字写入）
 */
static HAL_StatusTypeDef OTA_FlashWrite(uint32_t addr, uint8_t *data,
                                        uint32_t len) {
  HAL_FLASH_Unlock();

  HAL_StatusTypeDef status = HAL_OK;

  // STM32F103需要按半字（16位）写入
  for (uint32_t i = 0; i < len; i += 2) {
    uint16_t half_word;
    if (i + 1 < len) {
      half_word = data[i] | (data[i + 1] << 8);
    } else {
      half_word = data[i] | 0xFF00; // 最后一个字节，填充0xFF
    }

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, half_word);
    if (status != HAL_OK) {
      break;
    }
  }

  HAL_FLASH_Lock();
  return status;
}

/**
 * @brief 计算Flash区域CRC（使用硬件CRC）
 */
static uint32_t OTA_CalculateCRC(uint32_t addr, uint32_t size) {
  __HAL_CRC_DR_RESET(&hcrc);

  // 按4字节对齐计算
  uint32_t word_count = (size + 3) / 4;
  return HAL_CRC_Calculate(&hcrc, (uint32_t *)addr, word_count);
}