#include "PID.h"

static float clampf(float x, float minv, float maxv) {
  if (x > maxv)
    return maxv;
  if (x < minv)
    return minv;
  return x;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_max,
              float out_min, float i_max) {
  pid->Kp = kp;
  pid->Ki = ki;
  pid->Kd = kd;

  pid->Outmax = out_max;
  pid->Outmin = out_min;
  pid->Imax = i_max;

  pid->Exp = 0.0f;
  pid->Mea = 0.0f;
  pid->Err = 0.0f;
  pid->Err1 = 0.0f;

  pid->I = 0.0f;
  pid->Out = 0.0f;
}

float PID_Update(PID_t *pid) {
  pid->Err = pid->Exp - pid->Mea;

  // 积分（固定周期调用，Ki 直接按“每周期”整定即可）
  pid->I += pid->Ki * pid->Err;
  pid->I = clampf(pid->I, -pid->Imax, pid->Imax);

  // 微分
  float d = pid->Kd * (pid->Err - pid->Err1);
  pid->Err1 = pid->Err;

  // 输出
  pid->Out = pid->Kp * pid->Err + pid->I + d;
  pid->Out = clampf(pid->Out, pid->Outmin, pid->Outmax);

  return pid->Out;
}