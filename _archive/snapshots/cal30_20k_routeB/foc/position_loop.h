/**
  * @file    position_loop.h
  * @brief   位置环: P 控制 + 限速 + 到位判断
  */
#ifndef POSITION_LOOP_H
#define POSITION_LOOP_H

#include "foc_types.h"

/* ---- ??? ---- */
void PositionLoop_Init(PositionLoop_t *loop, float kp,
                       float speed_max_erpm, float decel_erpm_s,
                       float arrive_tol_deg, uint32_t arrive_ms,
                       float travel_limit_deg, uint8_t pole_pairs);

/* ---- ??: ?????? (eRPM) ---- */
float PositionLoop_Update(PositionLoop_t *loop,
                          float pos_now_deg,
                          float target_deg,
                          float dt_s,
                          uint32_t now_tick);

/* ---- ?????? (????) ---- */
void PositionLoop_SetTarget(PositionLoop_t *loop, float target_deg);

/* ---- ?????? ---- */
uint8_t PositionLoop_IsArrived(const PositionLoop_t *loop);

/* ---- ???????? ---- */
float PositionLoop_GetErrDeg(const PositionLoop_t *loop);

/* ---- ?????? ---- */
void PositionLoop_ResetArrived(PositionLoop_t *loop);

#endif

