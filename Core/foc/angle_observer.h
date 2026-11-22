/**
  * @file    angle_observer.h
  * @brief   角度观测器接口: 由霍尔扇区估算电角度/机械角/转速
  */
#ifndef ANGLE_OBSERVER_H
#define ANGLE_OBSERVER_H

#include "foc_types.h"

/* ---- 初始化 ----
   hall_offset_deg : 霍尔零点偏移(deg); 一般给 0, 由 ALIGN 结束后的
                     AngleObserver_CalibrateOffsetAtZero() 自动标定。
   hall_dir_rev    : 1 = 霍尔扇区顺序反转(对应 board_config.h 的 CFG_HALL_DIR_REV)。 */
void AngleObserver_Init(Angle_Observer_t *obs, float hall_offset_deg,
                        uint8_t hall_dir_rev);

/* ---- 处理一条边沿(低层入口, 不做相邻性校验) ---- */
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

/* ---- 取电角度(rad), erpm_out 可选输出转速 ----
   每个 PWM 周期调一次。返回值 = 扇区中心角 + 扇区内进角(adv) + 零点偏移。
   无新边沿时靠模型外推推进 adv, 并按 POS_MODEL_TAU_S 衰减(停转后不会自己爬)。
   creep_hz / creep_suppress 用来在极低速时限制外推速度。 */
float AngleObserver_GetTheta(Angle_Observer_t *obs, const Hall_State_t *hall,
                             uint32_t now_tick, float pwm_freq,
                             uint32_t timeout_ticks, float creep_hz,
                             uint8_t creep_suppress, float *erpm_out);

/* ---- 读转速(eRPM), 未滤波 ---- */
float AngleObserver_GetOmegaErpm(const Angle_Observer_t *obs, float pwm_freq);

/* ---- 读转速(eRPM), 已滤波(带宽给控制环用) ---- */
float AngleObserver_GetOmegaFiltErpm(const Angle_Observer_t *obs, float pwm_freq);

/* ---- 真实机械角度 (deg, 多圈) ----
   = 相位展开后的连续电角度 / 极对数。停转时增量恒为 0 ⇒ 不会漂移。 */
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

/* ---- 直接设霍尔零点偏移(deg) ----
   平时由 AngleObserver_CalibrateOffsetAtZero() 自动标定, 这个只在手动调试时用。 */
void AngleObserver_SetOffset(Angle_Observer_t *obs, float offset_deg);

#endif

