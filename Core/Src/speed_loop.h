/**
  * @file    speed_loop.h
  * @brief   ???PI?? ??
  */
#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H

#include "foc_types.h"

/* ---- ??? ---- */
void SpeedLoop_Init(SpeedLoop_t *loop, float kp, float ki,
                    float iq_max_normal, float iq_max_zone,
                    float settle_err_erpm, uint32_t settle_ms,
                    float ramp_rate_erpm_s);

/* ---- ??: ??IQ?? (A) ---- */
float SpeedLoop_Update(SpeedLoop_t *loop,
                       float erpm_now,
                       float ref_erpm,
                       float dt_s,
                       uint32_t now_tick);

/* ---- ?????? (eRPM) ---- */
void SpeedLoop_SetRef(SpeedLoop_t *loop, float ref_erpm);

/* ---- ???? (eRPM/s) ---- */
void SpeedLoop_SetRampRate(SpeedLoop_t *loop, float ramp_rate_erpm_s);

/* ---- ???? ---- */
void SpeedLoop_ResetInteg(SpeedLoop_t *loop);

/* ---- ????IQ?? ---- */
float SpeedLoop_GetIqCap(const SpeedLoop_t *loop);

/* ---- ????: ????IQ?? (MotionGuard) ---- */
void SpeedLoop_OverrideIqCap(SpeedLoop_t *loop, float cap);

#endif

