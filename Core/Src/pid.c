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
    float out_unsat;

    /* ?? (?dt??) */
    if (dt_s > 1e-6f) {
        d_err = (err - pid->prev_err) / dt_s;
    } else {
        d_err = 0.0f;
    }

    /* ???? (???) + 条件积分(抗饱和)
       输出已饱和且误差仍指向饱和方向时停止积分. 否则积分会一路冲到限幅并
       锁死: 实测 ALIGN 期间 id_ref=3A 而 id 冲到 7.2A(vd/vq 双双顶到 ±0.45),
       相位电流超过 6A 过流门限 → 直接进 FAULT. */
    out_unsat = pid->kp * err + pid->integ + pid->kd * d_err;
    if (!((out_unsat >  pid->lim && err > 0.0f) ||
          (out_unsat < -pid->lim && err < 0.0f))) {
        pid->integ += pid->ki * err * dt_s;
        if (pid->integ >  pid->lim_integ) pid->integ =  pid->lim_integ;
        if (pid->integ < -pid->lim_integ) pid->integ = -pid->lim_integ;
    }

    /* 接近限幅时给积分一个慢泄放, 防止"锁死"在限幅上.
       条件积分只能防"误差仍指向饱和方向"时的继续积分; 但实测会出现——
       积分被初始瞬态打到限幅后误差回到 0(例如电流环 iq 指令 0、iq 反馈也
       已≈0), 于是它既不增长也不泄放, 永久停在限幅上: 实测 q 轴锁在 -0.444,
       一直施加一个大电压, 电流慢慢爬到 6.2A 才撞过流保护(6.75s 后跳闸).
       这里只在输出接近限幅(>90%)时泄放(τ≈0.2s), 正常调节不受影响. */
    if (out_unsat >  0.9f * pid->lim || out_unsat < -0.9f * pid->lim) {
        pid->integ -= pid->integ * (dt_s / 0.2f);
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

