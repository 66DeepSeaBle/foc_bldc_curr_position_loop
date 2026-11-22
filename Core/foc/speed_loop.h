/**
  * @file    speed_loop.h
  * @brief   速度环(PI + 指令斜坡 + Iq 限幅切换)
  */
#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H

#include "foc_types.h"

/* ---- 初始化 ----
   iq_max_zone   : 运动中的 Iq 限幅(A)
   iq_max_normal : 稳定后的 Iq 限幅(A), 通常更小(降噪/降热)
   settle_err_erpm / settle_ms : "已稳定"的判据
   ramp_rate_erpm_s : 指令斜坡速率, <=0 表示不爬坡(直接跟随) */
void SpeedLoop_Init(SpeedLoop_t *loop, float kp, float ki,
                    float iq_max_normal, float iq_max_zone,
                    float settle_err_erpm, uint32_t settle_ms,
                    float ramp_rate_erpm_s);

/* ---- 跑一拍: 返回 Iq 指令(A) ----
   内部依次做: 指令斜坡 → 稳定判定(切 iq_cap) → PI + 条件积分(抗饱和) → 限幅。 */
float SpeedLoop_Update(SpeedLoop_t *loop,
                       float erpm_now,
                       float ref_erpm,
                       float dt_s,
                       uint32_t now_tick);

/* ---- 设定速度指令(eRPM) ----
   ⚠ 注意: 本函数只写 loop->ref_target, 而 SpeedLoop_Update() 用的是入参 ref_erpm ——
      实际调用方(foc_control.c)直接传值, 并不读 ref_target。
      保留此函数只为上层接口完整。 */
void SpeedLoop_SetRef(SpeedLoop_t *loop, float ref_erpm);

/* ---- 改指令斜坡速率(eRPM/s) ---- */
void SpeedLoop_SetRampRate(SpeedLoop_t *loop, float ramp_rate_erpm_s);

/* ---- 清积分 ---- */
void SpeedLoop_ResetInteg(SpeedLoop_t *loop);

/* ---- 读当前生效的 Iq 限幅(A) ---- */
float SpeedLoop_GetIqCap(const SpeedLoop_t *loop);

/* ---- ⚠ 空实现占位(当前无任何作用, 调用它不会改变 iq_cap) ----
   历史遗留接口。真要临时改限幅, 请直接写 loop->iq_cap 或改 iq_max_zone。 */
void SpeedLoop_OverrideIqCap(SpeedLoop_t *loop, float cap);

#endif

