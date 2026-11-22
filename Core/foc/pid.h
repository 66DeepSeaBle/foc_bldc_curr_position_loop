/**
  * @file    pid.h
  * @brief   PID 调节器 + 一阶低通滤波器
  */
#ifndef PID_H
#define PID_H

#include "foc_types.h"

/* ---- PID 初始化 ----
   lim       : 输出对称限幅
   lim_integ : 积分对称限幅(通常 < lim); <=0 表示与 lim 相同
   内部会调 PID_Reset() 清积分与 prev_err。 */
void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float lim, float lim_integ);
void PID_Reset(PID_Controller_t *pid);                       /* 清积分+上一拍误差(切环/复位时用) */
float PID_Update(PID_Controller_t *pid, float err, float dt_s);  /* 返回限幅后的输出 */

/* ---- 一阶低通滤波器(tau_s 单位秒; tau<=0 表示直通) ---- */
void LPF_Init(LowPassFilter_t *lpf, float tau_s);
float LPF_Update(LowPassFilter_t *lpf, float x, float dt_s);

#endif

