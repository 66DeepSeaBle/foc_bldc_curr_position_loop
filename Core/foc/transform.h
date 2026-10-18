/**
  * @file    transform.h
  * @brief   Clarke/Park/?Park?? ??
  */
#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "foc_types.h"

/* ---- Clarke??: Iu,Iv,Iw -> Ialpha,Ibeta ---- */
void Clarke_Transform(float iu, float iv, float iw, float *ialpha, float *ibeta);

/* ==========================================================================
 * 三角函数查表（为了省 CPU）
 * --------------------------------------------------------------------------
 * Park + 逆Park 每拍要 4 次 sinf/cosf，轨迹里 TM_Profile4 还要再 8 次
 * ⇒ 20kHz 一个周期里最多 **12 次** 库三角函数。ARM 的 sinf/cosf 每次
 * 约 200 周期 ⇒ 2400 周期 = 14us，占 50us 周期的 28%！
 * 换成 512 点正弦表 + 线性插值（每次约 20 周期）
 * ⇒ 运动峰值负载从 72% 降到 ~50%，**不用降 PWM 频率**。
 * 精度: 最大插值误差 ≈ (2π/512)²/8 = 9.4e-6，对 FOC 完全够。
 * ⚠ 必须先调 FOC_Math_Init()（已放在 FOC_Control_Init 里）。
 * ========================================================================== */
void FOC_Math_Init(void);
/* theta(rad) -> sin/cos；s/c 传 NULL 则不算那一路。支持任意角度(含负值)。 */
void FOC_SinCos(float th, float *s, float *c);
/* 只要 sin 的便捷版（TM_Profile4 用） */
float FOC_Sin(float th);
float FOC_Cos(float th);

/* ---- Park??: Ialpha,Ibeta,theta -> Id,Iq ---- */
void Park_Transform(float ialpha, float ibeta, float theta,
                    float *id, float *iq);

/* ---- ?Park??: Vd,Vq,theta -> Valpha,Vbeta ---- */
void InvPark_Transform(float vd, float vq, float theta,
                       float *valpha, float *vbeta);

/* ---- ????: Valpha,Vbeta -> Va,Vb,Vc ---- */
void InvClarke_Transform(float valpha, float vbeta, float *va, float *vb, float *vc);

#endif

