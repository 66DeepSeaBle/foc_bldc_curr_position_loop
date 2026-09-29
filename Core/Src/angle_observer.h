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

/* ---- 用对齐结果自动校正霍尔表零点 ----
   对齐阶段 theta 被强制到已知电角度(0°), 此刻转子 d 轴就停在该角度上; 读当时的
   霍尔状态 h0, 由 GetSectorCenter(h0)+offset ≡ 0 反解 offset 即可.
   一次性消除"霍尔扇区表零点/接线"与 FOC 角度约定之间的固定偏差, 不需要手调
   hall_offset_deg. 实测偏差 ≈204°(只剩 sin(204°)≈-0.4 的转矩且符号反),
   表现为走得动但一冲一停、静摩擦大处彻底卡死、空载也不转. */
void AngleObserver_CalibrateOffsetAtZero(Angle_Observer_t *obs, uint8_t hall_state);

/* ---- ???? ---- */
void AngleObserver_SetOffset(Angle_Observer_t *obs, float offset_deg);

#endif

