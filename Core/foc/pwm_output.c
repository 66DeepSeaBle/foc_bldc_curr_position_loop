/**
  * @file    pwm_output.c
  * @brief   SVPWM 调制实现(七段式, 含死区补偿)
  */
#include "pwm_output.h"
#include "transform.h"
#include "bsp.h"          /* 执行器输出: 占空比下发/关断都走 BSP(换芯片不用改这里) */
#include <math.h>

static float dt_comp_duty = 0.0f;
static float iu_filt = 0.0f, iv_filt = 0.0f, iw_filt = 0.0f;
static const float DT_COMP_HYST = 0.30f;
static const float DT_COMP_IFILT = 0.3f;

void PWM_Output_Init(SVPWM_t *pwm, float pwm_freq, float deadtime_ns)
{
    pwm->du = 0.0f;
    pwm->dv = 0.0f;
    pwm->dw = 0.0f;
    pwm->valpha = 0.0f;
    pwm->vbeta = 0.0f;
    dt_comp_duty = 2.0f * deadtime_ns * 1e-9f * pwm_freq;
}

/* ---- 死区补偿增量(方向由滤波后的相电流决定) ----
   死区会让实际输出电压比指令少一个 "sign(i) × 死区占空比"。这里按电流方向补回来:
     |i| > DT_COMP_HYST(0.30A) ⇒ 满补偿 ±dt_comp_duty
     |i| < DT_COMP_HYST        ⇒ 按 i/HYST 线性过渡
   用滞环+线性过渡而不是直接判符号, 是为了避免电流过零点时补偿量来回翻转
   (那会变成 bang-bang, 在零电流附近制造额外电流纹波)。 */
static float DT_CompDelta(float i_filt)
{
    if (i_filt >  DT_COMP_HYST) return  dt_comp_duty;
    if (i_filt < -DT_COMP_HYST) return -dt_comp_duty;
    return dt_comp_duty * (i_filt / DT_COMP_HYST);
}

void PWM_Output_SVPWM(SVPWM_t *pwm, float valpha, float vbeta,
                      float iu, float iv, float iw)
{
    float va, vb, vc;
    float vmax, vmin, vcom;

    pwm->valpha = valpha;
    pwm->vbeta = vbeta;

    InvClarke_Transform(valpha, vbeta, &va, &vb, &vc);

    /* 零序注入: 三相同时减去 (vmax+vmin)/2, 把三路占空比整体居中。
       这一步把线性调制区从 Vdc/2 扩到 Vdc/√3(即等效 SVPWM),
       同时保证 du/dv/dw 不会三路一起顶到 0 或 1(那样就没有采样窗口了)。 */
    vmax = va;
    vmin = va;
    if (vb > vmax) vmax = vb;
    if (vc > vmax) vmax = vc;
    if (vb < vmin) vmin = vb;
    if (vc < vmin) vmin = vc;
    vcom = 0.5f * (vmax + vmin);

    pwm->du = 0.5f + (va - vcom);
    pwm->dv = 0.5f + (vb - vcom);
    pwm->dw = 0.5f + (vc - vcom);

    /* ---- 死区补偿: 先把电流一阶低通(系数 0.3), 再按符号/幅值给补偿量 ---- */
    iu_filt += DT_COMP_IFILT * (iu - iu_filt);
    iv_filt += DT_COMP_IFILT * (iv - iv_filt);
    iw_filt += DT_COMP_IFILT * (iw - iw_filt);

    pwm->du += DT_CompDelta(iu_filt);
    pwm->dv += DT_CompDelta(iv_filt);
    pwm->dw += DT_CompDelta(iw_filt);

    /* 钳位到 [0, 0.95]。上限必须与 board_config.h 的 CFG_PWM_MAX_DUTY 一致:
       占空比一旦超过 ADC 触发点 CCR4(=98.85%×ARR), 采样时刻上管还在导通,
       下管分流电阻上没电流 ⇒ 读到的相电流直接作废。 */
    if (pwm->du < 0.0f) pwm->du = 0.0f;
    if (pwm->du > 0.95f) pwm->du = 0.95f;
    if (pwm->dv < 0.0f) pwm->dv = 0.0f;
    if (pwm->dv > 0.95f) pwm->dv = 0.95f;
    if (pwm->dw < 0.0f) pwm->dw = 0.0f;
    if (pwm->dw > 0.95f) pwm->dw = 0.95f;
}

void PWM_Output_Apply(const SVPWM_t *pwm)
{
    BSP_PWM_SetDuty3(pwm->du, pwm->dv, pwm->dw);
}

float PWM_Output_GetDu(const SVPWM_t *pwm) { return pwm->du; }
float PWM_Output_GetDv(const SVPWM_t *pwm) { return pwm->dv; }
float PWM_Output_GetDw(const SVPWM_t *pwm) { return pwm->dw; }

void PWM_Output_Disable(void)
{
    BSP_PWM_Off();
}

