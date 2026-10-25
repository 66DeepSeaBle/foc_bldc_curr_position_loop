/**
  * @file    foc_app.h
  * @brief   ★ 2026-10-24 应用层接口(位置环「速度档 / 力矩档」+ 移动命令)
  *
  * ============================================================================
  * 这个文件是给**上层应用**用的门面, 上层的 MCU/PLC/上位机只需要这三件事:
  *
  *   ① 设/读「力矩档」—— 位置环允许输出的最大 q 轴电流(A)
  *   ② 设/读「速度档」—— 定时轨迹的巡航速度(机械 °/s)
  *   ③ 发一条「走到某角度」的命令(可指定速度、时长)
  *
  * 典型用法:
  *     FOC_AppTuning_t cfg = { .torque_a = 3.0f, .speed_deg_s = 240.0f };
  *     FOC_App_SetTuning(&cfg);                // 设一次, 之后一直生效
  *     FOC_App_MoveTo(180.0f, 0.0f, 0.0f);     // 按当前速度档走到 180°
  *     FOC_App_MoveTo(0.0f, 600.0f, 0.0f);     // 这一动用 600°/s
  *     FOC_App_MoveTo(90.0f, 0.0f, 1.5f);      // 这一动限时 1.5s(时长优先)
  *
  * ============================================================================
  * 硬件链路(实测值, 见 FOC_App_GetHw())
  * ----------------------------------------------------------------------------
  *   采样电阻 R_shunt = 10 mΩ, 内部运放 PGA 增益 = ×8
  *   ⇒ 灵敏度 = 0.01 × 8 = **0.08 V/A**
  *   ⇒ 12bit/3.3V 下 **1 LSB ≈ 10.1 mA**
  *   零电流输出 ≈ 2621 counts ≈ 2.11 V
  *   ⇒ 正方向余量 3.3 − 2.11 = 1.19 V ⇒ **+14.8 A**; 负方向 ⇒ −26 A
  *   ⇒ 6 A 只用到正向余量的约 40%
  *
  *   为什么不能把增益提到 ×16 来"充分利用":
  *     零电流偏置被 ×16 放大后 = 2.11 V × 2 = **4.2 V > 3.3 V** ⇒ 运放/ADC 直接
  *     饱和, 读数恒为满量程。所以 **×8 已经是该偏置下的最大可用增益**。
  *     想再提高分辨率只能改硬件(把 0.2V 级偏置降到 0.1V 级), 固件做不到。
  *
  *   力矩档上限为什么定 5.0A 而不是 6.0A:
  *     过流保护 CFG_OVER_CURRENT_A = 6.0A 是按**相电流峰值**判的, 而 q 轴
  *     电流指令最终就对应相电流幅值 ⇒ 若把力矩档也设到 6A, 一进入稳态就
  *     立刻触发过流跳闸。所以固件把上限压在 5.0A(留 1A 余量),
  *     见 FOC_APP_TORQUE_MAX_A。
  * ============================================================================
  */

#ifndef FOC_APP_H
#define FOC_APP_H

#include <stdint.h>
#include "foc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 力矩档(A): 位置环允许输出的最大 q 轴电流 ---------------------------- */
#define FOC_APP_TORQUE_DEF_A     3.00f   /* 2026-10-25 A2e: 2.50 → 3.00
                                            （默认值，上电即生效；`[`/`]` 仍可±0.1A）
                                            理由：2.50 时破黏电流不够，长行程最后一段
                                            反复“卡 500ms → 滑一格”。3.00 让
                                            trim/kick 上限同步抬高 20%（r=cap/2.5）。 */
#define FOC_APP_TORQUE_MIN_A     0.30f
#define FOC_APP_TORQUE_MAX_A     5.00f   /* 过流 6A 留 1A 余量 */

/* 速度档(机械 °/s): 定时轨迹的巡航速度上限 -------------------------------- */
/* 速度档与「时长」的关系: 4 段轨迹的巡航速度 vc ≈ 1.8 × 行程 / 总时长
   ⇒ 给定速度与行程时, 固件自动反算时长 T = 1.8 × 行程 / 速度,
   再交由原有的定时轨迹去跑(不需要改轨迹算法)。
   ⚠ 速度档同时是 `SetTimedTarget` 里的最短用时门限(Tmin_v), 所以它也会
     影响**数字键 0~9 / z** 的手感 —— 默认 800°/s 与改动前完全一致。 */
#define FOC_APP_SPEED_DEF_DEG_S  800.0f  /* 默认 = 原 POS_TM_VMAX_MECH_DEG_S。
                                            ★ 必须保持 800！数字键的时长是
                                            `Tmin_v = 1.8×D/vmax`, 改成别的值会把
                                            用户调好的手感直接改掉(试过 648 ⇒ 360°
                                            从 0.85s 变成 1.00s)。 */
#define FOC_APP_SPEED_MIN_DEG_S  10.0f
#define FOC_APP_SPEED_MAX_DEG_S  1500.0f /* 实际还受加速度上限限制, 见说明 */

/* 速度档与「时长」的关系: 4 段轨迹的巡航速度 vc ≈ 1.8 × 行程 / 总时长
   ⇒ 给定速度与行程时, 固件自动反算时长 T = 1.8 × 行程 / 速度,
   再交由原有的定时轨迹去跑(不需要改轨迹算法)。 */

typedef struct {
    float torque_a;       /* 力矩档(A), <=0 表示"不改这个" */
    float speed_deg_s;    /* 速度档(°/s), <=0 表示"不改这个" */
} FOC_AppTuning_t;

typedef struct {
    float shunt_ohm;      /* 采样电阻(Ω) */
    float amp_gain;       /* 运放增益(倍) */
    float v_per_a;        /* = shunt × gain, 每安培对应的运放输出电压 */
    float a_per_lsb;      /* ADC 1 LSB 对应多少安培 */
    float v_zero;         /* 零电流时运放输出电压(V) */
    float range_pos_a;    /* 正方向可测上限(A) */
    float range_neg_a;    /* 负方向可测上限(A) */
} FOC_AppHw_t;

/* ------------------------------------------------------------------------ */
void    FOC_App_Init(FOC_Control_t *foc);       /* 开机调用一次, 之后其它接口才有效 */

/* 力矩档 / 速度档: 一次设两个(任一字段 <=0 = 不改) */
uint8_t FOC_App_SetTuning(const FOC_AppTuning_t *cfg);
void    FOC_App_GetTuning(FOC_AppTuning_t *out);

/* 单参数版本 */
void    FOC_App_SetTorque(float amp);
float   FOC_App_GetTorque(void);
void    FOC_App_SetSpeed(float deg_s);
float   FOC_App_GetSpeed(void);

/* 力矩档派生出来的各项实际上限(调试用: 确认档位真的生效)
   力矩档 X 安时: drive=0.68X trim=0.80X damp=0.32X kick=0.80X */
float   FOC_App_GetIqLimitDrive(void);
float   FOC_App_GetIqLimitTrim(void);
float   FOC_App_GetIqLimitDamp(void);
float   FOC_App_GetIqLimitKick(void);

/* 取走上一段定时轨迹的客观指标(主循环调, 读即清)。返回 1 = 有新数据。
   max_tv   : 本次最大跟踪误差(参考−实际)    单位 0.001°
   max_iq   : 本次最大 |Iqref|              单位 0.001A
   band     : 当时的力矩档                  单位 0.001A
   drive    : 当时派生的驱动侧上限(=0.68×band) 单位 0.001A
   ★ 判据: 若 max_iq ≈ drive 且 max_tv 很大 ⇒ 驱动侧**权限不够**;
           若 max_iq 明显小于 drive    ⇒ 不是权限问题, 是没撞上深齿槽。 */
uint8_t FOC_App_TakeMoveStats(int32_t *max_tv_mdeg, int32_t *max_iq_ma,
                              int32_t *band_ma, int32_t *drive_ma);

/* 移动命令。
     target_deg : 目标机械角度(多圈, 可负)
     speed_deg_s: 本次巡航速度(°/s); <=0 ⇒ 用当前速度档
     dur_s      : 本次总时长(s);     >0 ⇒ **优先用时长**(此时忽略 speed)
   返回 1 = 已接受并开始执行; 0 = 参数非法/未初始化(目标无效或已在目标上) */
uint8_t FOC_App_MoveTo(float target_deg, float speed_deg_s, float dur_s);

/* 反算: 走 dist_deg 需要多少秒(按当前/给定速度), 用于上位机预估 */
float   FOC_App_EstimateTime(float dist_deg, float speed_deg_s);

/* 硬件链路实测参数(需要传入零电流 ADC 码值, 例如 [ADCR] 里的 offU) */
void    FOC_App_GetHw(uint16_t zero_counts, FOC_AppHw_t *out);

/* ---- 只读状态查询(给上层轮询用) ------------------------------------------ */
/* 当前机械角度(多圈、可负)。上电/重新标定后原点会重置, 上层需自己幂等 */
float   FOC_App_GetPos(void);
/* 1 = 已在目标容差内且停住(arrive_ms 消抖已过); 0 = 还在走/已超差 */
uint8_t FOC_App_Arrived(void);
/* 1 = 处于 FAULT 状态(过流/跑飞), 需重新标定(按 'r' 或上层重发标定流程) */
uint8_t FOC_App_IsFault(void);

#ifdef __cplusplus
}
#endif

#endif /* FOC_APP_H */
