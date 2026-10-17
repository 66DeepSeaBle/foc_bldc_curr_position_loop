/**
  * @file    current_loop.h
  * @brief   ??? (D/Q?PI??) ??
  */
#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#include "foc_types.h"

/* ---- ??? ---- */
void CurrentLoop_Init(CurrentLoop_t *loop, float kp, float ki,
                      float vlim, float id_ref, float iq_ref);

/* ---- ?? (?PWM????) ---- */
void CurrentLoop_Update(CurrentLoop_t *loop,
                        float id_fb, float iq_fb,
                        float dt_s);

/* ---- ???? ---- */
void CurrentLoop_SetIdRef(CurrentLoop_t *loop, float id_ref);
void CurrentLoop_SetIqRef(CurrentLoop_t *loop, float iq_ref);
/* 2026-10-11k: 直接预置 q 轴积分(duty)。
   用途: 堵转破黏脉冲 —— 稳态电压就是 I×R/Vbus, 预置它能把电流建立时间
   从 ~250ms(ki=0.3 太慢)降到 ~1ms, 否则 100ms 的脉冲根本出不了力。 */
void CurrentLoop_SetQInteg(CurrentLoop_t *loop, float integ_duty);

/* ---- ???? ---- */
float CurrentLoop_GetVd(const CurrentLoop_t *loop);
float CurrentLoop_GetVq(const CurrentLoop_t *loop);

/* ---- ???? ---- */
void CurrentLoop_ResetInteg(CurrentLoop_t *loop);

#endif

