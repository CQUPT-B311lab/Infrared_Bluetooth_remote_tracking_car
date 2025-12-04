#include "PID.h"

uint32_t PID_time = 0;

void PID_Init(PID_t *PID, float p, float i, float d, float out_max,
              float out_min, float i_max) {
  PID->Kp = p;
  PID->Ki = i;
  PID->Kd = d;
  PID->Outmax = out_max;
  PID->Outmin = out_min;
  PID->Imax = i_max;
  PID->Err = 0.0;
  PID->Err1 = 0.0;
  PID->Err2 = 0.0;
  PID->Exp = 0.0;
  PID->I = 0.0;
  PID->Mea = 0.0;
  PID->Out = 0.0;
  PID_time = 0;
}

void PID_Updata(PID_t *PID) {
  PID_time++;
  PID->Err = PID->Exp - PID->Mea;

  PID->Out_delta = PID->Kp * (PID->Err - PID->Err1) + PID->Ki * PID->Err +
                   PID->Kd * (PID->Err - 2 * PID->Err1 + PID->Err2);

  PID->Err2 = PID->Err1;
  PID->Err1 = PID->Err;

  PID->Out += PID->Out_delta;
  if (PID->Out > PID->Outmax) {
    PID->Out = PID->Outmax;
  }
  if (PID->Out < PID->Outmin) {
    PID->Out = PID->Outmin;
  }
}
