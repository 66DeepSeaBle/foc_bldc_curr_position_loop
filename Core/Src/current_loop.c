/**
  * @file    current_loop.c
  * @brief   ?????
  */
#include "current_loop.h"
#include "pid.h"

void CurrentLoop_Init(CurrentLoop_t *loop, float kp, float ki,
                      float vlim, float id_ref, float iq_ref)
{
    loop->id_ref = id_ref;
    loop->iq_ref = iq_ref;
    loop->vlim = vlim;
    loop->vd = 0.0f;
    loop->vq = 0.0f;
    PID_Init(&loop->pid_d, kp, ki, 0.0f, vlim, vlim);
    PID_Init(&loop->pid_q, kp, ki, 0.0f, vlim, vlim);
}

void CurrentLoop_Update(CurrentLoop_t *loop,
                        float id_fb, float iq_fb,
                        float dt_s)
{
    float err_d = loop->id_ref - id_fb;
    loop->vd = PID_Update(&loop->pid_d, err_d, dt_s);

    float err_q = loop->iq_ref - iq_fb;
    loop->vq = PID_Update(&loop->pid_q, err_q, dt_s);

    loop->id = id_fb;
    loop->iq = iq_fb;
}

void CurrentLoop_SetIdRef(CurrentLoop_t *loop, float id_ref)
{
    loop->id_ref = id_ref;
}

void CurrentLoop_SetIqRef(CurrentLoop_t *loop, float iq_ref)
{
    loop->iq_ref = iq_ref;
}

void CurrentLoop_SetQInteg(CurrentLoop_t *loop, float integ_duty)
{
    if (integ_duty >  loop->vlim) integ_duty =  loop->vlim;
    if (integ_duty < -loop->vlim) integ_duty = -loop->vlim;
    loop->pid_q.integ = integ_duty;
}

float CurrentLoop_GetVd(const CurrentLoop_t *loop)
{
    return loop->vd;
}

float CurrentLoop_GetVq(const CurrentLoop_t *loop)
{
    return loop->vq;
}

void CurrentLoop_ResetInteg(CurrentLoop_t *loop)
{
    PID_Reset(&loop->pid_d);
    PID_Reset(&loop->pid_q);
}

