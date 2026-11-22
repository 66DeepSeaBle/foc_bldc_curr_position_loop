/**
  * @file    pwm_output.h
  * @brief   SVPWM 调制 + 占空比输出 + 死区补偿
  */
#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include "foc_types.h"

/* ---- 初始化: 清占空比, 并由死区时间 + PWM 频率算出每相的死区补偿量 ----
   deadtime_ns = 0 ⇒ 关闭死区补偿(见 foc_types.h 的 FOC_Config_t 注释)。
   补偿量 = 2 × deadtime × f_pwm, 单位是占空比量。
   实测: 500ns @ 20kHz ⇒ 0.020(= 2% 占空比)。 */
void PWM_Output_Init(SVPWM_t *pwm, float pwm_freq, float deadtime_ns);

/* ---- 由 α/β 电压算出三相占空比(零序注入 + 死区补偿 + 钳位) ----
   iu/iv/iw : 三相实测电流(A), **只**用来判死区补偿的方向(内部先做一阶低通)。
   ⚠ 本实现是"min-max 零序注入"式 SVPWM(线性区内与七段式扇区算法等价),
      所以代码里没有 N/T1/T2/扇区判断这些中间量 —— 不要以为它算错了。
   输出: pwm->du/dv/dw ∈ [0, 0.95]。 */
void PWM_Output_SVPWM(SVPWM_t *pwm, float valpha, float vbeta,
                      float iu, float iv, float iw);

/* ---- 把 du/dv/dw 真正下发到定时器(BSP_PWM_SetDuty3 → TIM1->CCR1..3) ----
   每拍只在最后调一次; 关断期间不要调(会被 FOC_UpdateCurrent 提前 return 跳过)。 */
void PWM_Output_Apply(const SVPWM_t *pwm);

/* ---- 读回本拍算出的占空比(只给日志/诊断用, 无副作用) ---- */
float PWM_Output_GetDu(const SVPWM_t *pwm);
float PWM_Output_GetDv(const SVPWM_t *pwm);
float PWM_Output_GetDw(const SVPWM_t *pwm);

/* ---- 关断三相输出(BSP_PWM_Off → 清 CCER 的六路使能位, 上下管全关) ----
   与"刹车"不同: 刹车是三相下管同时导通(短路制动), 这里是真正浮空。 */
void PWM_Output_Disable(void);

#endif

