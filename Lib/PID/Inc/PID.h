#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

typedef struct {
  float Kp;
  float Ki;
  float Kd;

  float Exp;  // 期望
  float Mea;  // 测量
  float Err;  // 当前误差
  float Err1; // 上次误差

  float I;    // 积分项
  float Imax; // 积分限幅

  float Out;    // 输出
  float Outmax; // 输出上限
  float Outmin; // 输出下限
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_max,
              float out_min, float i_max);

/**
 * @brief 位置式PID更新（固定周期调用即可）
 * @param pid PID对象
 * @return pid->Out
 */
float PID_Update(PID_t *pid);

#endif