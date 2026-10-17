/**
  * @file    pid.h
  * @brief   PID 调节器 + 一阶低通滤波器
  */
#ifndef PID_H
#define PID_H

#include "foc_types.h"

/* ---- PID??? ---- */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float lim, float lim_integ);
void PID_Reset(PID_Controller_t *pid);
float PID_Update(PID_Controller_t *pid, float err, float dt_s);

/* ---- ????? ---- */
void LPF_Init(LowPassFilter_t *lpf, float tau_s);
float LPF_Update(LowPassFilter_t *lpf, float x, float dt_s);

#endif

