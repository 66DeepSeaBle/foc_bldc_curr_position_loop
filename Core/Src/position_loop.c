/**
  * @file    position_loop.c
  * @brief   ?????
  */
#include "position_loop.h"
#include <math.h>

void PositionLoop_Init(PositionLoop_t *loop, float kp,
                       float speed_max_erpm, float decel_erpm_s,
                       float arrive_tol_deg, uint32_t arrive_ms,
                       float travel_limit_deg, uint8_t pole_pairs)
{
    loop->kp = kp;
    loop->speed_max_erpm = speed_max_erpm;
    loop->decel_erpm_s = decel_erpm_s;
    loop->arrive_tol_deg = arrive_tol_deg;
    loop->arrive_ms = arrive_ms;
    loop->travel_limit_deg = travel_limit_deg;
    loop->pole_pairs = pole_pairs;
    loop->target_deg = 0.0f;
    loop->pos_now_deg = 0.0f;
    loop->pos_err_deg = 0.0f;
    loop->arrived = 0;
    loop->arrived_tick = 0;
}

float PositionLoop_Update(PositionLoop_t *loop,
                          float pos_now_deg,
                          float target_deg,
                          float dt_s,
                          uint32_t now_tick)
{
    float pos_err = target_deg - pos_now_deg;
    float speed_target;
    uint32_t arrive_ticks;

    (void)dt_s;

    loop->pos_now_deg = pos_now_deg;
    loop->target_deg = target_deg;
    loop->pos_err_deg = pos_err;

    /* ---- ???? (??) ---- */
    arrive_ticks = (uint32_t)((float)loop->arrive_ms * 20000.0f / 1000.0f);
    if (fabsf(pos_err) < loop->arrive_tol_deg) {
        if (loop->arrived_tick == 0) {
            loop->arrived_tick = now_tick;
        } else if (!loop->arrived && (now_tick - loop->arrived_tick) > arrive_ticks) {
            loop->arrived = 1;
        }
    } else {
        loop->arrived_tick = 0;
        loop->arrived = 0;
    }

    if (loop->arrived) {
        return 0.0f;
    }

    /* ---- ???????? ---- */
    float dist_deg = fabsf(pos_err);
    /* v_max = sqrt(2 * decel * dist_deg * (6/pole_pairs)) ???eRPM */
    float speed_limit = sqrtf(2.0f * loop->decel_erpm_s * dist_deg * 6.0f / (float)loop->pole_pairs);
    if (speed_limit > loop->speed_max_erpm) {
        speed_limit = loop->speed_max_erpm;
    }
    if (speed_limit < 10.0f) speed_limit = 10.0f;  // ????,??????

    /* ---- P??? ---- */
    speed_target = loop->kp * pos_err;
    if (speed_target >  speed_limit) speed_target =  speed_limit;
    if (speed_target < -speed_limit) speed_target = -speed_limit;

    /* ---- ????? ---- */
    if (loop->travel_limit_deg > 0.0f && pos_now_deg >= loop->travel_limit_deg) {
        if (speed_target > 0.0f) speed_target = 0.0f;
    }

    return speed_target;
}

void PositionLoop_SetTarget(PositionLoop_t *loop, float target_deg)
{
    loop->target_deg = target_deg;
    loop->arrived = 0;
    loop->arrived_tick = 0;
}

uint8_t PositionLoop_IsArrived(const PositionLoop_t *loop)
{
    return loop->arrived;
}

float PositionLoop_GetErrDeg(const PositionLoop_t *loop)
{
    return loop->pos_err_deg;
}

void PositionLoop_ResetArrived(PositionLoop_t *loop)
{
    loop->arrived = 0;
    loop->arrived_tick = 0;
}

