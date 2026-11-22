/**
  * @file    position_loop.h
  * @brief   位置环: P 控制 + 限速 + 到位判断
  */
#ifndef POSITION_LOOP_H
#define POSITION_LOOP_H

#include "foc_types.h"

/* ---- 初始化 ----
   kp                : 位置比例增益, 单位 eRPM/deg
   speed_max_erpm    : 限速(eRPM)
   decel_erpm_s      : 用于反算限速的减速度(eRPM/s)
   arrive_tol_deg    : 到位容差(deg)  arrive_ms : 容差内持续多久算到位
   travel_limit_deg  : 正向行程软限位(<=0 表示不限)
   pole_pairs        : 极对数, 用在 sqrt() 里的 eRPM⇄°/s 换算 */
void PositionLoop_Init(PositionLoop_t *loop, float kp,
                       float speed_max_erpm, float decel_erpm_s,
                       float arrive_tol_deg, uint32_t arrive_ms,
                       float travel_limit_deg, uint8_t pole_pairs);

/* ---- 跑一拍: 返回速度目标(eRPM) ----
   dt_s 当前未使用(本环是纯 P + 限速, 不含积分/微分)。 */
float PositionLoop_Update(PositionLoop_t *loop,
                          float pos_now_deg,
                          float target_deg,
                          float dt_s,
                          uint32_t now_tick);

/* ---- 设目标位置(机械 deg, 多圈); 同时清掉 arrived 状态 ---- */
void PositionLoop_SetTarget(PositionLoop_t *loop, float target_deg);

/* ---- 查到位标志 ---- */
uint8_t PositionLoop_IsArrived(const PositionLoop_t *loop);

/* ---- 读上一次计算的位置误差(deg) ---- */
float PositionLoop_GetErrDeg(const PositionLoop_t *loop);

/* ---- 手动清到位标志(重新开始一次移动时用) ---- */
void PositionLoop_ResetArrived(PositionLoop_t *loop);

#endif

