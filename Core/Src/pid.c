/**
  * @file    pid.c
  * @brief   PID??? + ????? ??
  */
#include "pid.h"
#include <math.h>

void PID_Init(PID_Controller_t *pid, float kp, float ki, float kd,
              float lim, float lim_integ)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->lim = lim;
    pid->lim_integ = (lim_integ > 0.0f) ? lim_integ : lim;
    PID_Reset(pid);
}

void PID_Reset(PID_Controller_t *pid)
{
    pid->integ = 0.0f;
    pid->prev_err = 0.0f;
}

float PID_Update(PID_Controller_t *pid, float err, float dt_s)
{
    float out;
    float d_err;

    /* ???? (???) */
    pid->integ += pid->ki * err * dt_s;
    if (pid->integ >  pid->lim_integ) pid->integ =  pid->lim_integ;
    if (pid->integ < -pid->lim_integ) pid->integ = -pid->lim_integ;

    /* ?? (?dt??) */
    if (dt_s > 1e-6f) {
        d_err = (err - pid->prev_err) / dt_s;
    } else {
        d_err = 0.0f;
    }

    out = pid->kp * err + pid->integ + pid->kd * d_err;

    /* ???? */
    if (out >  pid->lim) out =  pid->lim;
    if (out < -pid->lim) out = -pid->lim;

    pid->prev_err = err;
    return out;
}

/* ---- ????? ---- */
void LPF_Init(LowPassFilter_t *lpf, float tau_s)
{
    lpf->y_prev = 0.0f;
    lpf->tau_s = tau_s;
}

float LPF_Update(LowPassFilter_t *lpf, float x, float dt_s)
{
    if (lpf->tau_s <= 0.0f || dt_s <= 0.0f) {
        lpf->y_prev = x;
        return x;
    }
    float alpha = dt_s / (lpf->tau_s + dt_s);
    lpf->y_prev += alpha * (x - lpf->y_prev);
    return lpf->y_prev;
}

