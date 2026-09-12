/**
  * @file    foc_control.c
  * @brief   FOC???????
  */
#include "foc_control.h"
#include "transform.h"
#include "pid.h"
#include "log.h"
#include <math.h>

/* ---- ?????? ---- */
static void FOC_DoIdle(FOC_Control_t *foc);
static void FOC_DoAlign(FOC_Control_t *foc, float dt_s);
static void FOC_DoStartup(FOC_Control_t *foc, float dt_s);
static void FOC_DoCurrentLoop(FOC_Control_t *foc, float dt_s);
static void FOC_DoSpeedLoop(FOC_Control_t *foc, float dt_s);
static void FOC_DoPositionLoop(FOC_Control_t *foc, float dt_s);
static void FOC_DoBrake(FOC_Control_t *foc, float dt_s);
static void FOC_DoFault(FOC_Control_t *foc);
static void FOC_UpdateAngle(FOC_Control_t *foc, float *erpm_out);
static void FOC_UpdateCurrent(FOC_Control_t *foc);

/* 三相输出是否已使能(正在驱动). 输出未使能(三相浮空/停止)时 ADC 采到的
   电流无意义(浮空基线≠0A基准), 不能拿去做过流判断, 否则会误报过流. */
static uint8_t s_pwm_on = 0;

/* 转速反馈低通状态: 霍尔在扇区边界抖动时, 瞬时 omega=扇区宽/dt 会大幅正负跳变
   (每拍 ±数百 eRPM 的假转速). 速度环若直接用会被噪声驱动 → 电流在 ±限幅间打满
   → 电机剧烈振动(bang-bang 极限环). 用 ~100ms 时间常数滤波后再喂速度环.
   20kHz 下 alpha = dt/(tau+dt) ≈ 50us/100ms ≈ 0.0005. */
static float s_erpm_filt = 0.0f;

/* ---- ??? ---- */
void FOC_Control_Init(FOC_Control_t *foc, const FOC_Config_t *config)
{
    foc->config = *config;

    foc->flags.enabled = 0;
    foc->flags.aligned = 0;
    foc->flags.hall_synced = 0;
    foc->flags.fault = 0;
    foc->flags.overcurrent = 0;
    foc->flags.calibrating = 0;
    s_pwm_on = 0;
    s_erpm_filt = 0.0f;
    /* 力矩方向: 实测只有 dir_rev=1 + torque_dir=-1 一组稳定(不振动、能 hold),
       其余组合要么剧烈振动要么跑飞 → 默认 -1. */
    foc->torque_dir = -1;

    foc->state = FOC_STATE_IDLE;
    foc->tick = 0;
    foc->fault_code = 0;

    foc->pos_target_deg_cmd = 0.0f;
    foc->speed_ref_erpm_cmd = 0.0f;
    foc->iq_ref_cmd = 0.0f;

    foc->align_done = 0;
    foc->startup_done = 0;
    foc->align_tick = 0;
    foc->startup_tick = 0;
    foc->startup_omega = 0.0f;

    /* ??????? */
    foc->hall.u = 0;
    foc->hall.v = 0;
    foc->hall.w = 0;
    foc->hall.state = 0;
    foc->hall.prev_state = 0;
    foc->hall.edge_tick = 0;
    foc->hall.prev_dt = 0;
    foc->hall.dir = 1;

    /* ???????? */
    AngleObserver_Init(&foc->angle, config->hall_offset_deg,
                       config->hall_dir_rev);

    /* ??????
       kp/ki 单位 = duty/A 与 duty/(A·s)(vd/vq 直接就是占空比量, 见 SVPWM).
       原值 kp=0.02 / ki=0.0005 时积分等于没开: integ += 0.0005*err*50us,
       1A 误差下 7 秒才积 0.0035, 而满量程是 0.45(需 600s) → 电流环实际只有
       纯比例作用, 稳态静差极大. 实测: 命令 1.5A 只到 0.56A(由 I=kp*Vbus/R
       的静差公式反推也是 0.56A), 力矩不够 → 电机完全不动. */
    CurrentLoop_Init(&foc->current_loop, 0.060f, 0.500f, 0.45f, 0.0f, 0.0f);

    /* ??????
       ki 原来 0.20 太大: 霍尔边沿任何时候都可能给出几百~上千 eRPM 的瞬时值,
       ki*err 能到几十~几百/s, 积分在 0.1s 内就从 +1.5 打到 -1.5 = 满力矩反拉,
       于是"冲一下→被打回→再冲"的快速左右摆. 位置环转速尺度提上来后(见下),
       转速误差本身有几百 eRPM, kp 项足够起启动/阻尼作用, 积分只需慢慢消静差,
       故 ki 降到 0.05. */
    SpeedLoop_Init(&foc->speed_loop, 0.008f, 0.05f,
                   1.5f, 1.5f, 10.0f, 200U, 2000.0f);

    /* ??????
       kp=0.3 eRPM/deg 太小: 45° 误差只给 13.5 eRPM(≈2.5°/s), 而霍尔瞬时速度
       是几百~上千 eRPM, 差 100 倍以上 → 速度环永远处于 bang-bang.
       提到 4.0: 45° → 180 eRPM(≈42°/s, 约 1.1s 到位), 与实际速度同量级;
       decel 150→2000 让 √(2*a*dist) 限速曲线不再一上来就把指令压死. */
    PositionLoop_Init(&foc->position_loop, 4.0f,
                      2000.0f, 2000.0f,
                      5.0f, 500U,
                      500.0f, config->pole_pairs);

    /* ???PWM */
    PWM_Output_Init(&foc->pwm, config->pwm_freq, 500.0f);

    /* ???? */
    foc->current.u = 0.0f;
    foc->current.v = 0.0f;
    foc->current.w = 0.0f;
    foc->current.vbus = 0.0f;

    /* 开环力矩探针默认关闭 */
    foc->probe_iq_A = 0.0f;
    foc->probe_total = 0;
    foc->probe_remain = 0;
}

/* ---- ???? ---- */
void FOC_Control_Run(FOC_Control_t *foc, float dt_s)
{
    /* 过流检查: 只在三相输出真正使能驱动时才判断; IDLE/制动/故障或刚切到
       ALIGN 而 PWM 尚未使能的那一拍, 测到的电流是三相浮空的无效值, 跳过. */
    if (foc->state != FOC_STATE_IDLE &&
        foc->state != FOC_STATE_BRAKE &&
        foc->state != FOC_STATE_FAULT &&
        s_pwm_on &&
        (fabsf(foc->current.u) > foc->config.overcurrent_amp ||
         fabsf(foc->current.v) > foc->config.overcurrent_amp)) {
        FOC_Control_SetFault(foc, 0x0001);
        return;
    }

    foc->tick++;

    switch (foc->state) {
        case FOC_STATE_IDLE:
            FOC_DoIdle(foc);
            break;
        case FOC_STATE_ALIGN:
            FOC_DoAlign(foc, dt_s);
            break;
        case FOC_STATE_STARTUP:
            FOC_DoStartup(foc, dt_s);
            break;
        case FOC_STATE_CURRENT_LOOP:
            FOC_DoCurrentLoop(foc, dt_s);
            break;
        case FOC_STATE_SPEED_LOOP:
            FOC_DoSpeedLoop(foc, dt_s);
            break;
        case FOC_STATE_POSITION_LOOP:
            FOC_DoPositionLoop(foc, dt_s);
            break;
        case FOC_STATE_BRAKE:
            FOC_DoBrake(foc, dt_s);
            break;
        case FOC_STATE_FAULT:
            FOC_DoFault(foc);
            break;
        default:
            FOC_DoIdle(foc);
            break;
    }
}

/* ---- ??????? ---- */

static void FOC_DoIdle(FOC_Control_t *foc)
{
    (void)foc;
    foc->flags.enabled = 0;
    PWM_Output_Disable();
    s_pwm_on = 0;
}

static void FOC_DoAlign(FOC_Control_t *foc, float dt_s)
{
    /* ????:??theta=0,?Id?? */
    const float align_current = foc->config.align_current;
    const uint32_t align_ms = 1500;
    uint32_t elapsed = foc->tick - foc->align_tick;
    float theta_align;

    (void)dt_s;

    if (foc->align_tick == 0) {
        foc->align_tick = foc->tick;
        LOGD("[ALIGN] Starting alignment, Id=%.1fA\n", align_current);
        foc->current_loop.id_ref = align_current;
        foc->current_loop.iq_ref = 0.0f;
    }

    elapsed = foc->tick - foc->align_tick;
    if (elapsed < 600) {
        theta_align = 1.57079633f;  // 90� ???
    } else {
        theta_align = 0.0f;          // 0� ??
    }

    foc->angle.theta = theta_align;

    /* ????? */
    FOC_UpdateCurrent(foc);

    if (elapsed > align_ms * 20U) {
        foc->align_done = 1;
        foc->flags.aligned = 1;
        /* ????????,????(???) */
        LOGD("[ALIGN] Align done, offset=%.1fdeg\n", foc->config.hall_offset_deg);
        foc->state = FOC_STATE_STARTUP;
        foc->startup_tick = foc->tick;
        foc->current_loop.id_ref = 0.0f;
        foc->current_loop.iq_ref = foc->config.align_current * 0.5f;
    }
}

static void FOC_DoStartup(FOC_Control_t *foc, float dt_s)
{
    /* I-F??:??????,???? */
    const float target_hz = foc->config.startup_hz;
    const uint32_t startup_ms = 2500;
    const uint32_t max_ms = 6000;
    uint32_t elapsed = foc->tick - foc->startup_tick;
    float omega_target;
    float erpm;

    if (foc->startup_tick == 0) {
        foc->startup_tick = foc->tick;
        LOGD("[STARTUP] I-F startup, target %.1fHz\n", target_hz);
    }

    elapsed = foc->tick - foc->startup_tick;

    /* ??????? */
    if (elapsed < startup_ms * 20U) {
        omega_target = 2.0f * 3.14159265f * target_hz / foc->config.pwm_freq *
                       (float)elapsed / (float)(startup_ms * 20U);
    } else {
        omega_target = 2.0f * 3.14159265f * target_hz / foc->config.pwm_freq;
    }

    foc->startup_omega = omega_target;
    foc->angle.theta += omega_target;
    while (foc->angle.theta >= 6.28318531f) foc->angle.theta -= 6.28318531f;
    while (foc->angle.theta < 0.0f) foc->angle.theta += 6.28318531f;

    /* ????? */
    foc->current_loop.iq_ref = foc->config.align_current * 0.5f;
    FOC_UpdateCurrent(foc);

    /* ???????? */
    erpm = AngleObserver_GetOmegaFiltErpm(&foc->angle, foc->config.pwm_freq);
    /* 同步到就尽早交接(>0.3s 且已有转速), 并把开环拖转累积的巨大机械行程清零,
       否则目标0/45/90 要追几千度 → 永远顶/振/跑飞, 无法调闭环 */
    if (foc->angle.synced && elapsed > 6000U && fabsf(erpm) > 20.0f) {
        float pos;
        foc->startup_done = 1;
        foc->flags.hall_synced = 1;
        foc->state = FOC_STATE_POSITION_LOOP;
        foc->current_loop.id_ref = 0.0f;
        foc->current_loop.iq_ref = 0.0f;
        CurrentLoop_ResetInteg(&foc->current_loop);
        SpeedLoop_ResetInteg(&foc->speed_loop);
        /* 以交接点为机械零点: 先让观测器按当前扇区产出一拍 theta(把相位
           展开器对齐到扇区角), 再清零连续电角度. 否则 I-F 期间被开环直接
           改写的 theta 会在交接后第一次展开时被当成一次大跳变. */
        (void)AngleObserver_GetTheta(&foc->angle, &foc->hall, foc->tick,
                                     foc->config.pwm_freq, 20000U, 1.0f, 1, NULL);
        foc->angle.elec_acc_deg = 0.0f;
        pos = AngleObserver_GetMechDeg(&foc->angle, foc->config.pole_pairs);
        foc->pos_target_deg_cmd = pos;
        LOGD("[STARTUP] Synced erpm=%.0f hold@%.1fdeg\n", erpm, pos);
        return;
    }

    if (elapsed > max_ms * 20U) {
        LOGW("[STARTUP] Timeout, hall not synced\n");
        foc->state = FOC_STATE_FAULT;
        foc->fault_code = 0x0002;
    }
}

static void FOC_DoCurrentLoop(FOC_Control_t *foc, float dt_s)
{
    FOC_UpdateAngle(foc, NULL);
    FOC_UpdateCurrent(foc);
}

static void FOC_DoSpeedLoop(FOC_Control_t *foc, float dt_s)
{
    float erpm_now;
    float iq_target;

    FOC_UpdateAngle(foc, &erpm_now);

    /* ????? */
    s_erpm_filt += 0.0005f * (erpm_now - s_erpm_filt);
    iq_target = SpeedLoop_Update(&foc->speed_loop,
                                 s_erpm_filt,
                                 foc->speed_ref_erpm_cmd,
                                 dt_s,
                                 foc->tick);

    /* ????? */
    foc->current_loop.id_ref = 0.0f;
    foc->current_loop.iq_ref = iq_target;

    /* ???? */
    FOC_UpdateCurrent(foc);
}

static void FOC_DoPositionLoop(FOC_Control_t *foc, float dt_s)
{
    float erpm_now;
    float iq_target;
    float pos_now_deg;
    float speed_target;
    int8_t saved_torque;

    /* ??????? */
    FOC_UpdateAngle(foc, &erpm_now);

    /* ---- 开环力矩探针 ----
       绕过位置/转速环, 直接施加恒定 iq 脉冲(期间强制 torque_dir=+1,
       使实际施加电流的符号就是 probe_iq_A), 观察机械角往哪边走.
       一次即可测定 "+Iq→+mech 还是 -mech", 从而确定反馈符号. */
    if (foc->probe_remain > 0U) {
        foc->probe_remain--;
        foc->current_loop.id_ref = 0.0f;
        if (foc->probe_remain == 0U || fabsf(erpm_now) > 2500.0f) {
            /* 脉冲结束(或转速过高安全截断) */
            foc->current_loop.iq_ref = 0.0f;
            LOGD("[PROBE] stop erpm=%.0f mech=%.1fdeg\n",
                 erpm_now,
                 AngleObserver_GetMechDeg(&foc->angle, foc->config.pole_pairs));
            FOC_UpdateCurrent(foc);
            return;
        }
        saved_torque = foc->torque_dir;
        foc->torque_dir = 1;                      /* 探针符号==probe_iq_A */
        foc->current_loop.iq_ref = foc->probe_iq_A;
        FOC_UpdateCurrent(foc);
        foc->torque_dir = saved_torque;
        return;
    }

    /* 真实机械角度(多圈累加), 取代原先的 "转速×0.1" 伪位置 */
    pos_now_deg = AngleObserver_GetMechDeg(&foc->angle, foc->config.pole_pairs);

    /* ????? */
    speed_target = PositionLoop_Update(&foc->position_loop,
                                       pos_now_deg,
                                       foc->pos_target_deg_cmd,
                                       dt_s,
                                       foc->tick);

    s_erpm_filt += 0.0005f * (erpm_now - s_erpm_filt);

    /* ---- 到位后速度环积分平滑泄放 ----
       起步/加速阶段速度环积分会被限幅顶到 ~1.5A, 而 err≈0 时它几乎不下降
       (iq = kp*err + integ 一直输出满电流). 电机被大扭矩顶着不动, 此时霍尔
       若在扇区边界来回跳, theta 每拍跳 ~60° → 转矩矢量跟着翻 → 剧烈振动.
       这里在到位容差内给积分一个 ~0.15s 的泄放, 让扭矩平滑回落到 kp*err≈0.
       注意: 只泄放积分、闭环保持; 不能像硬切 0 力矩那样处理——那会让转子
       失去阻尼, 靠惯性滑出容差带再被拉回, 变成"动一下停一下"的蠕动. */
    if (fabsf(foc->position_loop.pos_err_deg) < foc->position_loop.arrive_tol_deg) {
        foc->speed_loop.integ *= (1.0f - dt_s / 0.15f);
    }

    iq_target = SpeedLoop_Update(&foc->speed_loop,
                                 s_erpm_filt,
                                 speed_target,
                                 dt_s,
                                 foc->tick);

    /* ????? */
    foc->current_loop.id_ref = 0.0f;
    foc->current_loop.iq_ref = iq_target;

    /* ???? */
    FOC_UpdateCurrent(foc);
}

static void FOC_DoBrake(FOC_Control_t *foc, float dt_s)
{
    (void)foc;
    (void)dt_s;
    PWM_Output_Disable();
    s_pwm_on = 0;
}

static void FOC_DoFault(FOC_Control_t *foc)
{
    foc->flags.fault = 1;
    PWM_Output_Disable();
    s_pwm_on = 0;
}

/* ---- ???? ---- */
static void FOC_UpdateAngle(FOC_Control_t *foc, float *erpm_out)
{
    float erpm;
    /* creep_suppress=1: 闭环(位置/速度/电流环)下停转时不再按1Hz开环蠕动.
       蠕动会让 theta 与转子真实角度脱开, 换相扭矩对不上 → 电机只振不转.
       停住时保持最后有效扇区角度, 转子一动霍尔边沿自会更新. */
    foc->angle.theta = AngleObserver_GetTheta(&foc->angle,
                                              &foc->hall,
                                              foc->tick,
                                              foc->config.pwm_freq,
                                              20000U,
                                              1.0f,
                                              1,
                                              &erpm);
    if (erpm_out != NULL) *erpm_out = erpm;
}

/* ---- ????? + SVPWM?? ---- */
static void FOC_UpdateCurrent(FOC_Control_t *foc)
{
    float ialpha, ibeta, id, iq;
    float valpha, vbeta;

    /* Clarke?? */
    Clarke_Transform(foc->current.u, foc->current.v, foc->current.w,
                     &ialpha, &ibeta);

    /* Park?? */
    Park_Transform(ialpha, ibeta, foc->angle.theta, &id, &iq);

    /* 方向修正放在指令侧(torque_dir×iq_ref), 反馈保持正常负反馈,
       电流环才稳定. (不能取负反馈, 会变正反馈→q电流跑飞过流) */
    {
        float saved = foc->current_loop.iq_ref;
        foc->current_loop.iq_ref = saved * (float)foc->torque_dir;
        CurrentLoop_Update(&foc->current_loop, id, iq, 1.0f / foc->config.pwm_freq);
        foc->current_loop.iq_ref = saved;
    }

    /* ?Park?? */
    InvPark_Transform(CurrentLoop_GetVd(&foc->current_loop),
                      CurrentLoop_GetVq(&foc->current_loop),
                      foc->angle.theta,
                      &valpha, &vbeta);

    /* SVPWM?? */
    PWM_Output_SVPWM(&foc->pwm, valpha, vbeta,
                     foc->current.u, foc->current.v, foc->current.w);
    PWM_Output_Apply(&foc->pwm);
    s_pwm_on = 1;   /* 三相输出已使能, 之后的采样才有效, 可做过流判断 */
}

/* ---- ???? ---- */
void FOC_Control_SetState(FOC_Control_t *foc, FOC_State_t state)
{
    if (foc->flags.fault) return;
    foc->state = state;
    if (state == FOC_STATE_IDLE) {
        PWM_Output_Disable();
        s_pwm_on = 0;
    }
}

void FOC_Control_SetPosTarget(FOC_Control_t *foc, float target_deg)
{
    foc->pos_target_deg_cmd = target_deg;
    if (foc->state == FOC_STATE_POSITION_LOOP) {
        PositionLoop_ResetArrived(&foc->position_loop);
    }
}

void FOC_Control_SetSpeedRef(FOC_Control_t *foc, float ref_erpm)
{
    foc->speed_ref_erpm_cmd = ref_erpm;
}

void FOC_Control_SetIqRef(FOC_Control_t *foc, float iq_ref)
{
    foc->iq_ref_cmd = iq_ref;
}

void FOC_Control_SetTorqueProbe(FOC_Control_t *foc, float iq_A, uint32_t dur_ms)
{
    if (foc->state != FOC_STATE_POSITION_LOOP) {
        LOGD("[PROBE] ignored: state not POSITION_LOOP (%d)\n", (int)foc->state);
        return;
    }
    uint32_t n = dur_ms * 20U;   /* 20kHz */
    if (n == 0U) n = 1U;
    foc->probe_iq_A  = iq_A;
    foc->probe_total = n;
    foc->probe_remain = n;
    LOGD("[PROBE] START iq=%+.2fA dur=%ums torque_dir=%d mech=%.1fdeg\n",
         iq_A, dur_ms, (int)foc->torque_dir,
         AngleObserver_GetMechDeg(&foc->angle, foc->config.pole_pairs));
}

void FOC_Control_SetFault(FOC_Control_t *foc, uint32_t code)
{
    foc->fault_code = code;
    foc->flags.fault = 1;
    foc->state = FOC_STATE_FAULT;
    PWM_Output_Disable();
    s_pwm_on = 0;
}

FOC_State_t FOC_Control_GetState(const FOC_Control_t *foc)
{
    return foc->state;
}

uint8_t FOC_Control_IsFault(const FOC_Control_t *foc)
{
    return foc->flags.fault;
}

