/**
  * @file    current_loop.h
  * @brief   电流环(d/q 双 PI, 输出占空比量)
  */
#ifndef CURRENT_LOOP_H
#define CURRENT_LOOP_H

#include "foc_types.h"

/* ---- 初始化 ----
   kp/ki 的单位是 duty/A 与 duty/(A·s) —— vd/vq 直接就是占空比量(见 SVPWM),
   不是伏特。vlim 同时用作输出限幅和积分限幅。 */
void CurrentLoop_Init(CurrentLoop_t *loop, float kp, float ki,
                      float vlim, float id_ref, float iq_ref);

/* ---- 跑一拍(d/q 双 PI) ----
   每个 PWM 周期调一次; dt_s = 1/pwm_freq。
   id_fb/iq_fb 是 Park 变换给出的实测值; 结果存在 loop->vd/vq。
   ⚠ 本函数**不**负责 torque_dir 方向修正 —— 那是指令侧的事(见 foc_control.c)。 */
void CurrentLoop_Update(CurrentLoop_t *loop,
                        float id_fb, float iq_fb,
                        float dt_s);

/* ---- 设电流指令(A) ---- */
void CurrentLoop_SetIdRef(CurrentLoop_t *loop, float id_ref);
void CurrentLoop_SetIqRef(CurrentLoop_t *loop, float iq_ref);
/* 2026-10-11k: 直接预置 q 轴积分(duty)。
   用途: 堵转破黏脉冲 —— 稳态电压就是 I×R/Vbus, 预置它能把电流建立时间
   从 ~250ms(ki=0.3 太慢)降到 ~1ms, 否则 100ms 的脉冲根本出不了力。 */
void CurrentLoop_SetQInteg(CurrentLoop_t *loop, float integ_duty);

/* ---- 读本拍输出(只给日志/诊断用) ---- */
float CurrentLoop_GetVd(const CurrentLoop_t *loop);
float CurrentLoop_GetVq(const CurrentLoop_t *loop);

/* ---- 清两个轴的积分与 prev_err ----
   切环 / 重新使能 PWM / 退出到位静音时必须调, 否则残留的积分会凭空施加一个电压。 */
void CurrentLoop_ResetInteg(CurrentLoop_t *loop);

#endif

