/**
  * @file    angle_observer.h
  * @brief   角度观测器接口: 由霍尔扇区估算电角度/机械角/转速
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

/* ---- 20kHz 轮询式边沿检测(2026-10-11f, 推荐入口) ----
   由 ADC 中断每拍调用, 传当前读到的霍尔状态 h_new; 内部做"相邻性+最小间隔"
   校验, 非法(噪声/抖动)直接丢弃。返回 1 = 本次接受了一次边沿。 */
uint8_t AngleObserver_PollHall(Angle_Observer_t *obs, Hall_State_t *hall,
                               uint8_t h_new, uint32_t now_tick, float pwm_freq,
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

/* ---- 把当前位置定为 0(ALIGN/STARTUP 交接时用) ----
   2026-10-11c: 位置已改为绝对值算法, 清零必须搬参考点而不能直接写
   elec_acc_deg(那会在下一拍被覆盖)。 */
void AngleObserver_ZeroPos(Angle_Observer_t *obs);

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

