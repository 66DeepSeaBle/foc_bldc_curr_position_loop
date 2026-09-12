/**
  * @file    angle_observer.h
  * @brief   ???????:?????????????
  */
#ifndef ANGLE_OBSERVER_H
#define ANGLE_OBSERVER_H

#include "foc_types.h"

/* ---- ??? ---- */
void AngleObserver_Init(Angle_Observer_t *obs, float hall_offset_deg,
                        uint8_t hall_dir_rev);

/* ---- ??:?????????? ---- */
void AngleObserver_OnHallEdge(Angle_Observer_t *obs, Hall_State_t *hall,
                              uint32_t now_tick, float pwm_freq,
                              float min_dt_ticks, float omega_clamp_erpm,
                              float alpha_max);

/* ---- ??????:???PWM???? ---- */
float AngleObserver_GetTheta(Angle_Observer_t *obs, const Hall_State_t *hall,
                             uint32_t now_tick, float pwm_freq,
                             uint32_t timeout_ticks, float creep_hz,
                             uint8_t creep_suppress, float *erpm_out);

/* ---- ?????? (eRPM) ---- */
float AngleObserver_GetOmegaErpm(const Angle_Observer_t *obs, float pwm_freq);

/* ---- ???????? (eRPM) ---- */
float AngleObserver_GetOmegaFiltErpm(const Angle_Observer_t *obs, float pwm_freq);

/* ---- ??机械角度 (deg, 多圈累加) ---- */
float AngleObserver_GetMechDeg(const Angle_Observer_t *obs, float pole_pairs);

/* ---- ???? ---- */
void AngleObserver_SetOffset(Angle_Observer_t *obs, float offset_deg);

#endif

