/**
  ******************************************************************************
  * @file    foc_control.h
  * @brief   FOC 控制核心的对外接口 + 全局上下文类型(FOC_Control_t)。
  ******************************************************************************
  *
  *  FOC_Control_t 是整个控制器的"状态快照": 状态机、配置、测量值、三个环、
  *  开环自检用的临时量。app.c 直接读它的字段做日志; 控制律只在 foc_control.c
  *  内部使用。
  *
  ******************************************************************************
  */
#ifndef FOC_CONTROL_H
#define FOC_CONTROL_H

#include "foc_types.h"
#include "angle_observer.h"   /* 上下文里嵌了 Angle_Observer_t, 也把观测器接口带给上层 */
#include "current_loop.h"
#include "speed_loop.h"
#include "position_loop.h"
#include "pwm_output.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ★ 2026-10-11ak "默认目标"的定时轨迹时长: 数字键/`z` 按目标一键到位时,
   ≤360° 的移动统一按这个时长走 4 段轨迹(加速 2/9 + 匀速 3/9 + 减速 2/9 +
   逼近 2/9), 而不是靠"速度环 + 长距爬行"的旧律(旧律在齿槽上是一跳一跳的)。
   ★★ 2026-10-17 1.0 → 1.5s(用户要求"更平滑"): 实测 270°/360° 的 1s 轨迹里
   `tv`(轨迹位置误差) 会摆 ±20~38° —— 读数滞后 + 中速段测速不可信导致跟踪不上,
   进而环路饱和、速度忽快忽慢。拉到 1.5s 后需要的加速度/巡航速度都降到 ~2/3
   (360° 的巡航从 624 → 408°/s), 跟踪误差和读数摆动明显变小。
   代价: 长行程慢 0.5s; 想要快就用命令 `t<角度> d<秒>` 单独指定。 */
#define FOC_POS_DEFAULT_TM_S   1.5f

/* ==========================================================================
 * 全局上下文
 * ========================================================================== */
typedef struct {
    /* ---- 状态机 ---- */
    FOC_State_t  state;
    FOC_Flags_t  flags;
    uint32_t     fault_code;        /* 故障码(见 FOC_Control_SetFault 的注释) */
    uint32_t     tick;              /* 20kHz 控制拍计数 */

    /* ---- 配置与测量 ---- */
    FOC_Config_t          config;
    ThreePhase_Current_t  current;  /* 三相电流 + 母线电压 */
    Hall_State_t          hall;     /* 霍尔扇区/边沿时刻/方向 */
    Angle_Observer_t      angle;    /* 角度观测器状态 */

    /* ---- 三个控制环 + 输出 ---- */
    CurrentLoop_t   current_loop;
    SpeedLoop_t     speed_loop;
    PositionLoop_t  position_loop;
    SVPWM_t         pwm;

    /* ---- 力矩方向与位置目标 ---- */
    int8_t   torque_dir;            /* ±1, 力矩方向 */
    float    pos_target_deg_cmd;    /* 位置目标(机械°, 多圈) */
    uint8_t  pos_target_user_set;   /* 1 = 用户设过目标(STARTUP 交接时不要覆盖) */
    float    speed_ref_erpm_cmd;    /* 速度目标(eRPM) */
    float    iq_ref_cmd;            /* q 轴电流指令(A) */

    /* ---- 对齐/起步 ---- */
    uint8_t  align_done;            /* 1 = 对齐完成 */
    uint8_t  startup_done;          /* 1 = I-F 起步完成 */
    uint32_t align_tick;
    uint32_t startup_tick;
    float    startup_omega;         /* I-F 拖转角速度(rad/tick) */

    /* ---- 开环自检 / 手动开环 / PWM 烟测 ---- */
    uint8_t  ol_active;             /* 1 = 开环测试进行中 */
    uint8_t  ol_manual;             /* 1 = 手动开环(u/i/n/m/h/l) */
    uint8_t  ol_pwm_only;           /* 1 = 只做 PWM 烟测(三相 50%) */
    uint8_t  ol_phase;              /* 测试段号 */
    int8_t   ol_dir;                /* 旋转方向 */
    uint8_t  ol_settle;             /* 1 = 当前处于"稳压/静置"子段 */
    uint8_t  ol_dc_ramp;            /* 1 = 段0 正在升压 */
    uint8_t  ol_dc_hold;            /* 1 = 段0 已冻结电压(等稳定) */
    uint8_t  ol_ramp;               /* 1 = 段1 电压斜坡中 */
    uint32_t ol_tick;               /* 测试已进行 tick */
    uint32_t ol_phase_tick;         /* 当前子段已进行 tick */
    uint32_t ol_total;              /* 本次测试总 tick */
    uint32_t ol_last_warn;          /* 上次告警 tick(限流日志节流) */
    float    ol_theta;              /* 开环电角度(rad) */
    float    ol_omega;              /* 当前电角速度(rad/tick) */
    float    ol_omega_tgt;          /* 目标电角速度(rad/tick) */
    float    ol_v_amp;              /* 开环电压幅值(占空比) */
    float    ol_man_v;              /* 手动模式电压(占空比) */
    float    ol_man_hz;             /* 手动模式电频率(Hz) */
    float    ol_i_ref;              /* 电流幅值指令(A) */
    float    ol_i_meas;             /* 电流幅值测量(A) */
    float    ol_i_peak;             /* 电流幅值峰值(A) */
    float    ol_vb_min;             /* 测试期间母线最低值(V) */
    float    ol_vb_max;             /* 测试期间母线最高值(V) */
    float    ol_dc_sum_u;           /* 段0 三相累加(算 R_eq) */
    float    ol_dc_sum_v;
    float    ol_dc_sum_w;
    uint32_t ol_dc_cnt;             /* 段0 累加样本数 */
    float    ol_dc_amp;             /* 段0 冻结时的电压幅值(占空比) */
    float    ol_dc_vv;              /* 段0 冻结时的相电压(V) */
    float    ol_dc_r;               /* 段0 算出的等效电阻(Ω) */
    uint32_t ol_hall_edges;         /* 测试期间霍尔边沿数(证明转子真的转了) */
    uint8_t  ol_hall_prev;          /* 上一次霍尔扇区(数边沿用) */
    int8_t   ol_saved_torque;       /* 测试前的 torque_dir(结束后还原) */

    /* ---- 力矩探针(j/k 键): 测"+Iq 对应哪个机械方向" ---- */
    float    probe_iq_A;            /* 探针电流(A) */
    uint32_t probe_total;           /* 探针总 tick */
    uint32_t probe_remain;          /* 探针剩余 tick */
} FOC_Control_t;

/* ==========================================================================
 * 接口
 * ========================================================================== */

/* 初始化(装配配置、清状态), 必须在任何 FOC_Control_* 之前调用 */
void  FOC_Control_Init(FOC_Control_t *foc, const FOC_Config_t *config);

/* 20kHz 控制环: 每个 PWM 周期调一次(由 app.c 的 ADC 中断钩子调用) */
void  FOC_Control_Run(FOC_Control_t *foc, float dt_s);

/* 状态机 */
void        FOC_Control_SetState(FOC_Control_t *foc, FOC_State_t state);
FOC_State_t FOC_Control_GetState(const FOC_Control_t *foc);
void        FOC_Control_SetFault(FOC_Control_t *foc, uint32_t code);
uint8_t     FOC_Control_IsFault(const FOC_Control_t *foc);

/* 目标与指令 */
void  FOC_Control_SetPosTarget(FOC_Control_t *foc, float target_deg);
void  FOC_Control_SetTimedTarget(FOC_Control_t *foc, float target_deg, float dur_s);
void  FOC_Control_SetSpeedRef(FOC_Control_t *foc, float ref_erpm);
void  FOC_Control_SetIqRef(FOC_Control_t *foc, float iq_ref);
void  FOC_Control_SetTorqueProbe(FOC_Control_t *foc, float iq_A, uint32_t dur_ms);

/* 开环/手动自检 */
void  FOC_Control_StartOpenLoopTest(FOC_Control_t *foc, float hz, float amp,
                                    uint32_t dur_ms);
void  FOC_Control_StopOpenLoopTest(FOC_Control_t *foc, const char *reason);
void  FOC_Control_FlipOpenLoopDir(FOC_Control_t *foc);
void  FOC_Control_StartPwmSmokeTest(FOC_Control_t *foc);
void  FOC_Control_StartManualTest(FOC_Control_t *foc);
void  FOC_Control_ManualAdjust(FOC_Control_t *foc, float dv, float dhz);

/* ---- 只给日志/诊断用的读接口 ---- */
uint8_t  FOC_Control_GetTimedPhase(void);   /* 0=无轨迹, 1..9=轨迹段号 */
void     FOC_Control_GetTrajDiag(float *verr, float *iq_ff, float *iq_trim,
                                 float *vobs);
float    FOC_Control_GetPosIntegA(void);    /* 位置积分项(A) */
float    FOC_Control_GetPosErpm(void);      /* 位置环用的速度(eRPM) */
float    FOC_Control_GetVelPosErpm(void);   /* 位置差分测速(eRPM, 未融合) */
float    FOC_Control_GetVelWinErpm(void);   /* 扇区窗口速度(eRPM) */
uint8_t  FOC_Control_GetPosMove(void);      /* 1 = 判定为"在动" */
uint16_t FOC_Control_GetPosJumpCnt(void);   /* 读数跳变计数 */
uint8_t  FOC_Control_GetPosHoldFlag(void);  /* 1 = 到位死区(撤力) */
uint8_t  FOC_Control_GetPosGiveFlag(void);  /* 1 = 爬行认输 */
uint8_t  FOC_Control_GetPosBlockFlag(void); /* 1 = 机构推不动 */
uint8_t  FOC_Control_GetPosSettleFlag(void);/* 1 = 近目标松手静默中 */
uint16_t FOC_Control_GetPosHopCnt(void);    /* 本轮连续推不动次数 */

#ifdef __cplusplus
}
#endif

#endif /* FOC_CONTROL_H */
