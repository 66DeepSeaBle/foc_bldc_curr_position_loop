/**
  * @file    speed_loop.c
  * @brief   ?????
  */
#include "speed_loop.h"
#include <math.h>

void SpeedLoop_Init(SpeedLoop_t *loop, float kp, float ki,
                    float iq_max_normal, float iq_max_zone,
                    float settle_err_erpm, uint32_t settle_ms,
                    float ramp_rate_erpm_s)
{
    loop->kp = kp;
    loop->ki = ki;
    loop->iq_max_normal = iq_max_normal;
    loop->iq_max_zone = iq_max_zone;
    loop->settle_err_erpm = settle_err_erpm;
    loop->settle_ms = settle_ms;
    loop->ramp_rate = ramp_rate_erpm_s;
    loop->ref_erpm = 0.0f;
    loop->ref_target = 0.0f;
    loop->erpm_now = 0.0f;
    loop->integ = 0.0f;
    loop->iq_cap = iq_max_zone;
    loop->settled = 0;
    loop->settled_tick = 0;
    loop->tick = 0;
}

float SpeedLoop_Update(SpeedLoop_t *loop,
                       float erpm_now,
                       float ref_erpm,
                       float dt_s,
                       uint32_t now_tick)
{
    float err, iq_target;

    loop->tick = now_tick;

    /* ?????? */
    if (loop->ramp_rate > 0.0f) {
        float step = loop->ramp_rate * dt_s;
        if (loop->ref_erpm < ref_erpm) {
            loop->ref_erpm += step;
            if (loop->ref_erpm > ref_erpm) loop->ref_erpm = ref_erpm;
        } else if (loop->ref_erpm > ref_erpm) {
            loop->ref_erpm -= step;
            if (loop->ref_erpm < ref_erpm) loop->ref_erpm = ref_erpm;
        }
    } else {
        loop->ref_erpm = ref_erpm;
    }

    loop->erpm_now = erpm_now;

    /* ---- ???? ---- */
    err = loop->ref_erpm - erpm_now;
    if (fabsf(err) < loop->settle_err_erpm) {
        if (loop->settled_tick == 0) loop->settled_tick = now_tick;
    } else {
        loop->settled_tick = 0;
    }

    uint32_t settle_ticks = (uint32_t)((float)loop->settle_ms * 20000.0f / 1000.0f);
    if (loop->settled_tick != 0 && (now_tick - loop->settled_tick) > settle_ticks) {
        loop->settled = 1;
        loop->iq_cap = loop->iq_max_normal;
    } else {
        loop->settled = 0;
        loop->iq_cap = loop->iq_max_zone;
    }

    /* ---- PI + 条件积分(抗饱和) ----
       输出已饱和且误差仍指向饱和方向时不再积分. 否则积分会一路冲到限幅,
       等真实运动一出现又要很久才退回来 —— 实测表现为"顶住不动 → 猛窜一下
       → 再顶住"的走停循环(约 1.4s 一个周期). */
    {
        float iq_unsat = loop->kp * err + loop->integ;
        if (!((iq_unsat >  loop->iq_cap && err > 0.0f) ||
              (iq_unsat < -loop->iq_cap && err < 0.0f))) {
            loop->integ += loop->ki * err * dt_s;
            if (loop->integ >  loop->iq_cap) loop->integ =  loop->iq_cap;
            if (loop->integ < -loop->iq_cap) loop->integ = -loop->iq_cap;
        }
    }

    iq_target = loop->kp * err + loop->integ;
    if (iq_target >  loop->iq_cap) iq_target =  loop->iq_cap;
    if (iq_target < -loop->iq_cap) iq_target = -loop->iq_cap;

    return iq_target;
}

void SpeedLoop_SetRef(SpeedLoop_t *loop, float ref_erpm)
{
    loop->ref_target = ref_erpm;
}

void SpeedLoop_SetRampRate(SpeedLoop_t *loop, float ramp_rate_erpm_s)
{
    loop->ramp_rate = ramp_rate_erpm_s;
}

void SpeedLoop_ResetInteg(SpeedLoop_t *loop)
{
    loop->integ = 0.0f;
}

float SpeedLoop_GetIqCap(const SpeedLoop_t *loop)
{
    return loop->iq_cap;
}

void SpeedLoop_OverrideIqCap(SpeedLoop_t *loop, float cap)
{
    (void)loop;
    /* ??????? loop->iq_cap ???????? */
}

