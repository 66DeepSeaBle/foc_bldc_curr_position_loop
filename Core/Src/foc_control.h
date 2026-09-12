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

/* ---- 故障处理 ---- */
void FOC_Control_SetFault(FOC_Control_t *foc, uint32_t code);

/* ---- 状态查询 ---- */
FOC_State_t FOC_Control_GetState(const FOC_Control_t *foc);
uint8_t FOC_Control_IsFault(const FOC_Control_t *foc);

#endif

