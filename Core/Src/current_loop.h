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

/* ---- ???? ---- */
float CurrentLoop_GetVd(const CurrentLoop_t *loop);
float CurrentLoop_GetVq(const CurrentLoop_t *loop);

/* ---- ???? ---- */
void CurrentLoop_ResetInteg(CurrentLoop_t *loop);

#endif

