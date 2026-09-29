/**
  * @file    foc_control.h
  * @brief   FOC 控制接口和状态数据。
  */
#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include "foc_types.h"
#include "angle_observer.h"
#include "current_loop.h"
#include "speed_loop.h"
#include "position_loop.h"
#include "pwm_output.h"

/* ---- FOC 控制器状态 ---- */
typedef struct {
    FOC_State_t state;
    FOC_Flags_t flags;
    FOC_Config_t config;

    Angle_Observer_t angle;
    Hall_State_t hall;
    ThreePhase_Current_t current;
    CurrentLoop_t current_loop;
    SpeedLoop_t speed_loop;
    PositionLoop_t position_loop;
    SVPWM_t pwm;

    float pos_target_deg_cmd;
    float speed_ref_erpm_cmd;
    float iq_ref_cmd;
    int8_t torque_dir;           /* +1/-1: 抵消换相方向, 使 +Iq 指令→+机械角方向 */

    uint32_t tick;
    uint32_t fault_code;

    /* 对齐和启动阶段的状态数据。 */
    uint8_t align_done;
    uint8_t startup_done;
    uint32_t align_tick;
    uint32_t startup_tick;
    float startup_omega;

    /* 开环力矩探针: 静止时给恒定 iq 脉冲, 测 "+Iq→哪个机械方向",
       用于一次确定 torque_dir 反馈符号(不再靠反复试方向). */
    float    probe_iq_A;
    uint32_t probe_total;
    uint32_t probe_remain;

    /* ---- 开环速度测试(S 键) ---- 全部绕过电流环, 只用电流"幅值"做反馈,
       因此对 dq 符号/相序免疫(前三版都栽在"电流环符号不对"上).
       段0: 静止, 电压从 0 慢速线性升到 0.06 duty(每拍只动 ~2e-6, 无阶跃),
            电流到 2.5A 就冻结电压, 再稳压 0.3s 求平均 → R_eq = v/|I|。
       段1: 开环 V/f 旋转, 幅值型慢速调节器把电流幅值拉到 ~1A。 */
    uint8_t  ol_active;
    uint8_t  ol_pwm_only;        /* 1 = 只做"三相同 50% 占空比"硬件烟测, 不旋转 */
    uint8_t  ol_manual;          /* 1 = 手动开环(o 键): 电压/频率由按键直接给 */
    float    ol_man_v;           /* 手动模式电压幅值 (占空比量, 0~0.45) */
    float    ol_man_hz;          /* 手动模式电频率 (Hz, 0~30) */
    uint32_t ol_last_warn;       /* 手动模式软限流警告节流 */
    uint8_t  ol_phase;           /* 0=放电, 1=升压, 2=稳压平均, 3=旋转 */
    int8_t   ol_dir;             /* 旋转方向 (+1/-1), 运行中可用 f 翻转 */
    uint32_t ol_tick;            /* 总 tick */
    uint32_t ol_phase_tick;      /* 当前段内 tick */
    uint32_t ol_total;           /* 总时长 (tick) */
    uint32_t ol_settle;          /* 段0 放电(三相短接)时长 (tick) */
    uint32_t ol_dc_ramp;         /* 段0 升压时长 (tick) */
    uint32_t ol_dc_hold;         /* 段0 稳压平均时长 (tick) */
    uint32_t ol_ramp;            /* 段1 升速时长 (tick) */
    float    ol_theta;           /* 开环电角度 (rad) */
    float    ol_omega;           /* 当前每拍电角度增量 (rad/tick) */
    float    ol_omega_tgt;       /* 目标每拍增量 (rad/tick) */
    float    ol_v_amp;           /* 当前施加电压幅值 (占空比量) */
    float    ol_i_ref;           /* 段1 目标电流幅值 (A) */
    float    ol_i_meas;          /* 当前电流幅值 (A) */
    float    ol_i_peak;          /* 期间最大 |相电流| (A) */
    float    ol_vb_min;          /* 期间最小母线电压 (V) */
    float    ol_vb_max;          /* 期间最大母线电压 (V) */
    float    ol_dc_sum_u;        /* 段0 三相电流累加, 用于求平均 */
    float    ol_dc_sum_v;
    float    ol_dc_sum_w;
    uint32_t ol_dc_cnt;
    float    ol_dc_amp;          /* 段0 平均电流幅值 (A) */
    float    ol_dc_vv;           /* 段0 施加电压 (V) */
    float    ol_dc_r;            /* 段0 算出的等效相电阻 (Ω) */
    uint32_t ol_hall_edges;      /* 测试期间霍尔跳变次数: 客观证明转子有没有转 */
    uint8_t  ol_hall_prev;       /* 上一次霍尔扇区 */
    int8_t   ol_saved_torque;    /* 测试前的 torque_dir, 结束时还原 */
} FOC_Control_t;

/* ---- 初始化 ---- */
void FOC_Control_Init(FOC_Control_t *foc, const FOC_Config_t *config);

/* ---- 主控制更新，通常由 PWM/ADC 控制循环调用 ---- */
void FOC_Control_Run(FOC_Control_t *foc, float dt_s);

/* ---- 状态控制 ---- */
void FOC_Control_SetState(FOC_Control_t *foc, FOC_State_t state);

/* ---- 控制命令输入 ---- */
void FOC_Control_SetPosTarget(FOC_Control_t *foc, float target_deg);
void FOC_Control_SetSpeedRef(FOC_Control_t *foc, float ref_erpm);
void FOC_Control_SetIqRef(FOC_Control_t *foc, float iq_ref);
void FOC_Control_SetTorqueProbe(FOC_Control_t *foc, float iq_A, uint32_t dur_ms);

/* ---- 开环速度测试: 用于单独验证功率级/电机/供电(不依赖霍尔) ---- */
void FOC_Control_StartOpenLoopTest(FOC_Control_t *foc, float hz,
                                   float current_A, uint32_t dur_ms);
void FOC_Control_StopOpenLoopTest(FOC_Control_t *foc, const char *reason);
void FOC_Control_FlipOpenLoopDir(FOC_Control_t *foc);
/* 硬件烟测: 三相都固定在 50% 占空比(v=0, 端子短接), 不做任何旋转。
   用途: 断开电机引出线后跑, 若 |I| 仍很大 → 是采样/功率级问题, 不是电机。 */
void FOC_Control_StartPwmSmokeTest(FOC_Control_t *foc);

/* ---- 手动开环 (o/u/i/h/l/f/t 键) ----
   手上只有万用表时的主手段:
     1) 按 o 进入, 保持 f=0Hz(静止), 按 u 逐步升压, 同时用万用表量任意两相
        引出线之间的电压, 和日志里的 Vll 对比 → 直接判定"命令电压有没有真的
        加到电机上";
     2) 按 h 逐步给频率(3~5Hz), 电机应该开始转。
   全程纯电压开环, 不用电流环、不用霍尔、不用观测器; 只用电流幅值做软限流。 */
void FOC_Control_StartManualTest(FOC_Control_t *foc);
void FOC_Control_ManualAdjust(FOC_Control_t *foc, float dv, float dhz);

/* ---- 故障处理 ---- */
void FOC_Control_SetFault(FOC_Control_t *foc, uint32_t code);

/* ---- 状态查询 ---- */
FOC_State_t FOC_Control_GetState(const FOC_Control_t *foc);
uint8_t FOC_Control_IsFault(const FOC_Control_t *foc);

#endif

