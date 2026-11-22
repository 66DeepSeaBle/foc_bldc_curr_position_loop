/**
  * @file    position_loop.c
  * @brief   位置环实现
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

    /* ---- 到位判定: 误差进容差后要连续保持 arrive_ms, 防止边界抖动误判 ---- */
    arrive_ticks = (uint32_t)((float)loop->arrive_ms * (float)FOC_CTRL_HZ / 1000.0f);
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

    /* ---- 限速: 按减速度反算"不超调的最大速度" ---- */
    float dist_deg = fabsf(pos_err);
    /* v_max = sqrt(2 * decel * dist_deg) 换成 eRPM:
       机械 deg/s = eRPM × 1.5(4 对极) ⇒ eRPM = deg/s / 1.5
       这里用 6/pole_pairs 表达同一个换算, 改极对数不用改公式 */
    float speed_limit = sqrtf(2.0f * loop->decel_erpm_s * dist_deg * 6.0f / (float)loop->pole_pairs);
    if (speed_limit > loop->speed_max_erpm) {
        speed_limit = loop->speed_max_erpm;
    }
    if (speed_limit < 10.0f) speed_limit = 10.0f;  /* 下限兜底: 距离很近时限速会趋近 0, 环路就完全给不出速度 */

    /* ---- P 控制 + 限速 ---- */
    speed_target = loop->kp * pos_err;
    if (speed_target >  speed_limit) speed_target =  speed_limit;
    if (speed_target < -speed_limit) speed_target = -speed_limit;

    /* ---- 行程软限位: 只挡正向越界, 反向不限 ---- */
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

