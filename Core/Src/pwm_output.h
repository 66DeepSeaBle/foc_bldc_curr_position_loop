/**
  * @file    pwm_output.h
  * @brief   SVPWM 调制 + 占空比输出 + 死区补偿
  */
#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include "foc_types.h"

/* ---- ??? ---- */
void PWM_Output_Init(SVPWM_t *pwm, float pwm_freq, float deadtime_ns);

/* ---- SVPWM: ??Valpha,Vbeta,??????? ---- */
void PWM_Output_SVPWM(SVPWM_t *pwm, float valpha, float vbeta,
                      float iu, float iv, float iw);

/* ---- ????? (??MotorSetDuty3) ---- */
void PWM_Output_Apply(const SVPWM_t *pwm);

/* ---- ????? ---- */
float PWM_Output_GetDu(const SVPWM_t *pwm);
float PWM_Output_GetDv(const SVPWM_t *pwm);
float PWM_Output_GetDw(const SVPWM_t *pwm);

/* ---- ???? (???????) ---- */
void PWM_Output_Disable(void);

#endif

