/**
  * @file    pwm_output.c
  * @brief   SVPWM??
  */
#include "pwm_output.h"
#include "transform.h"
#include <math.h>

/* ---- ??????? (?main.c??????) ---- */
extern void MotorSetDuty3(float du, float dv, float dw);
extern void MotorAllOff(void);

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

/* ---- ?????? ---- */
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

    /* ?????? (??????) */
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

    /* ---- ???? ---- */
    iu_filt += DT_COMP_IFILT * (iu - iu_filt);
    iv_filt += DT_COMP_IFILT * (iv - iv_filt);
    iw_filt += DT_COMP_IFILT * (iw - iw_filt);

    pwm->du += DT_CompDelta(iu_filt);
    pwm->dv += DT_CompDelta(iv_filt);
    pwm->dw += DT_CompDelta(iw_filt);

    /* ?? */
    if (pwm->du < 0.0f) pwm->du = 0.0f;
    if (pwm->du > 0.95f) pwm->du = 0.95f;
    if (pwm->dv < 0.0f) pwm->dv = 0.0f;
    if (pwm->dv > 0.95f) pwm->dv = 0.95f;
    if (pwm->dw < 0.0f) pwm->dw = 0.0f;
    if (pwm->dw > 0.95f) pwm->dw = 0.95f;
}

void PWM_Output_Apply(const SVPWM_t *pwm)
{
    MotorSetDuty3(pwm->du, pwm->dv, pwm->dw);
}

float PWM_Output_GetDu(const SVPWM_t *pwm) { return pwm->du; }
float PWM_Output_GetDv(const SVPWM_t *pwm) { return pwm->dv; }
float PWM_Output_GetDw(const SVPWM_t *pwm) { return pwm->dw; }

void PWM_Output_Disable(void)
{
    MotorAllOff();
}

