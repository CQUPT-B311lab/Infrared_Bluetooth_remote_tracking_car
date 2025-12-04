#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

extern uint32_t PID_time;

typedef struct {
  float Kp;
  float Ki;
  float Kd;
  float Err; // 误差
  float Err1;
  float Err2;
  float Exp; // 期望
  float Mea; // 测量
  float Out; // 输出

  float I;      // 积分
  float Imax;   // 积分限幅
  float Outmax; // 输出限幅
  float Outmin; // 输出限幅

  float Out_delta; // 增量式输出
} PID_t;

void PID_Init(PID_t *PID, float p, float i, float d, float out_max,
              float out_min, float i_max);
void PID_Updata(PID_t *PID);

#endif