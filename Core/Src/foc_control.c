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
static void FOC_DoOpenLoop(FOC_Control_t *foc, float dt_s);
static void FOC_DoManual(FOC_Control_t *foc, float i_amp);
static void FOC_UpdateAngle(FOC_Control_t *foc, float *erpm_out);
static void FOC_UpdateCurrent(FOC_Control_t *foc);

/* 三相输出是否已使能(正在驱动). 输出未使能(三相浮空/停止)时 ADC 采到的
   电流无意义(浮空基线≠0A基准), 不能拿去做过流判断, 否则会误报过流. */
static uint8_t s_pwm_on = 0;

/* 过流去抖 + 开通消隐.
   电流采样在开关瞬间有尖峰(静止时噪声已有 ±0.5A, PWM 刚开通的瞬态更大),
   单次采样超门限就跳闸会误保护 —— 实测 ALIGN 一开始就立刻 FAULT, 而当时
   vd 只有 -0.070(环路并未饱和), 即"先出现大电流读数、环路才去拉", 属采样
   尖峰/瞬态而非环路失控. 对策: (a) 使能输出后先消隐 ~100ms;
   (b) 必须连续 4 次(200us)都超门限才判过流. */
static uint8_t  s_oc_cnt = 0;
static uint32_t s_pwm_on_tick = 0;
static const uint8_t  OC_DEBOUNCE = 4U;
static const uint32_t OC_BLANK_TICKS = 2000U;   /* 20kHz 下 = 100ms */

/* 开环速度测试参数. 全部只用"电流幅值"做判断, 对 dq 符号/相序免疫. */
/* 段0 升压上限 (占空比量).
   为什么放到 0.35 这么高: 实测相电阻 0.52Ω, 若"占空比×母线"真的加到电机上,
   只要 0.067 duty(1.3V) 就能推到 2.5A。如果升到 0.35 duty 才凑到冻结电流,
   说明**实际加到电机上的电压远小于命令值**(高边没真正开通/驱动坏), 而不是
   电流读数偏小 —— 这一条直接把两种可能分开。 */
static const float    OL_DC_VMAX     = 0.35f;
static const float    OL_DC_I_FREEZE = 2.0f;     /* 电流到这就冻结电压 */
static const uint32_t OL_SETTLE_MS   = 500U;     /* 段0 先三相短接放电 */
static const uint32_t OL_DC_RAMP_MS  = 2500U;    /* 段0 升压时间 */
static const uint32_t OL_DC_HOLD_MS  = 1500U;    /* 段0 稳压平均时间(留出量万用表的时间) */
static const uint32_t OL_ACC_RAMP_MS = 4000U;    /* 段1 升速时间 */
static const float    OL_TRIP_A      = 4.0f;     /* 硬限流, 超过立即停 */
static const float    OL_I_REG_KI    = 2.5e-5f;  /* duty/(A·tick) ≈ 0.5 duty/(A·s) */
static const float    OL_V_AMP_MIN   = 0.010f;
static const float    OL_V_AMP_MAX   = 0.300f;
/* 手动开环模式的软限流: 超了就把电压打八折(不中止, 免得打断手动操作) */
static const float    OL_MAN_ILIM    = 3.0f;
/* 手动模式的硬保护: 持续大电流 = 功率级有硬伤。
   实测 W 下管击穿时, 即使 v=0(三相都是 50% 占空比)也会持续 12A —— 电流经
   U/V 上管 → 电机 → 击穿的 W 下管 → 采样电阻回流, 几十秒就能把 MOS 烧到
   70~80°C。所以必须能真的关 PWM, 不能只软限流(软限流降的是施加电压, 而
   v=0 时根本没有电压可降)。按任意手动键(o/u/i/h/l)可重新进入。 */
static const float    OL_MAN_TRIP_A  = 6.0f;
static const uint32_t OL_MAN_TRIP_MS = 300U;

/* 转速反馈低通状态: 霍尔在扇区边界抖动时, 瞬时 omega=扇区宽/dt 会大幅正负跳变
   (每拍 ±数百 eRPM 的假转速). 速度环若直接用会被噪声驱动 → 电流在 ±限幅间打满
   → 电机剧烈振动(bang-bang 极限环). 用 ~100ms 时间常数滤波后再喂速度环.
   20kHz 下 alpha = dt/(tau+dt) ≈ 50us/100ms ≈ 0.0005. */
static float s_erpm_filt = 0.0f;

/* 手动模式的持续大电流计数(见 OL_MAN_TRIP_*) */
static uint32_t s_man_oc_cnt = 0;

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
    /* 力矩方向(2026-09-30 修正为 +1):
       手动开环 V/f 走 InvPark(0, +v, theta), **不经过 torque_dir**, 实测能把
       电机平稳拖转(hall 按 1->3->2->6->4->5 顺序走、89s 内 2201 个边沿);
       而 STARTUP/闭环走 iq_ref*torque_dir: 取 -1 时等于把矢量放到 theta-90°,
       拖动方向与 theta 斜坡相反 → 转子原地抖动、拖不动 → 6s 超时 FAULT。
       旧的 "-1" 结论是在 W 相半桥损坏(下管击穿/缺相)期间测出来的, 不可信。
       现场若发现方向相反或跑飞, 按 'y' 现场翻转即可。 */
    foc->torque_dir = 1;

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
       按实测硬件(母线 24V, 相电阻 0.3Ω)标定: 被控对象直流增益
       K = Vbus/R ≈ 80 A/duty, 故环路增益 = kp*K 必须 ≲1 才稳定.
         kp=0.06 -> 环路增益 ≈4.8 → 严重过冲(实测 id 指令 3A 冲到 7.2A, 相位
         电流越过 6A 过流门限 → 进 FAULT);
         kp=0.02 -> 环路增益 ≈1.6, 仍偏高且 ki=0.0005 时积分几乎不工作(静差大).
       取 kp=0.010(环路增益 ≈0.8, 稳定且不过冲), ki=0.30(1A 误差下 0.3 duty/s,
       约 0.2s 补足静差; 积分零点 ki/kp=30rad/s 远低于电流环带宽, 不会振). */
    CurrentLoop_Init(&foc->current_loop, 0.010f, 0.300f, 0.45f, 0.0f, 0.0f);

    /* ??????
       ki 原来 0.20 太大: 霍尔边沿任何时候都可能给出几百~上千 eRPM 的瞬时值,
       ki*err 能到几十~几百/s, 积分在 0.1s 内就从 +1.5 打到 -1.5 = 满力矩反拉,
       于是"冲一下→被打回→再冲"的快速左右摆. 位置环转速尺度提上来后(见下),
       转速误差本身有几百 eRPM, kp 项足够起启动/阻尼作用, 积分只需慢慢消静差,
       故 ki 降到 0.05.
       限流 1.5A→分区: 运动/爬行时(未稳定)用 iq_max_zone=3.5A —— 实测满 1.5A
       时转子仍"一走一停"(每步 1 个霍尔扇区 2.31°)、且能卡在静摩擦大的位置
       完全不动、超调后也刹不住, 说明力矩裕量不足; 过流保护在 6A, align 已用
       2A, 3.5A 有裕量(注意堵转会发热, 调试用可以, 长期要回落).
       稳定后(转速误差连续 200ms 小于 10eRPM)回落到 iq_max_normal=1.5A 保持. */
    SpeedLoop_Init(&foc->speed_loop, 0.008f, 0.05f,
                   1.5f, 3.5f, 10.0f, 200U, 2000.0f);

    /* ??????
       kp=0.3 eRPM/deg 太小: 45° 误差只给 13.5 eRPM(≈2.5°/s), 而霍尔瞬时速度
       是几百~上千 eRPM, 差 100 倍以上 → 速度环永远处于 bang-bang.
       提到 4.0: 45° → 180 eRPM(≈42°/s, 约 1.1s 到位), 与实际速度同量级;
       decel 150→2000 让 √(2*a*dist) 限速曲线不再一上来就把指令压死. */
    PositionLoop_Init(&foc->position_loop, 4.0f,
                      2000.0f, 2000.0f,
                      3.0f, 500U,
                      500.0f, config->pole_pairs);

    /* ???PWM */
    PWM_Output_Init(&foc->pwm, config->pwm_freq, config->deadtime_ns);

    /* ???? */
    foc->current.u = 0.0f;
    foc->current.v = 0.0f;
    foc->current.w = 0.0f;
    foc->current.vbus = 0.0f;

    /* 开环力矩探针默认关闭 */
    foc->probe_iq_A = 0.0f;
    foc->probe_total = 0;
    foc->probe_remain = 0;

    /* 开环速度测试默认关闭 */
    foc->ol_active = 0;
    foc->ol_pwm_only = 0;
    foc->ol_manual = 0;
    foc->ol_man_v = 0.0f;
    foc->ol_man_hz = 0.0f;
    foc->ol_last_warn = 0;
    foc->ol_phase = 0;
    foc->ol_dir = 1;
    foc->ol_tick = 0;
    foc->ol_phase_tick = 0;
    foc->ol_total = 0;
    foc->ol_settle = 1;
    foc->ol_dc_ramp = 1;
    foc->ol_dc_hold = 1;
    foc->ol_ramp = 1;
    foc->ol_theta = 0.0f;
    foc->ol_omega = 0.0f;
    foc->ol_omega_tgt = 0.0f;
    foc->ol_v_amp = 0.0f;
    foc->ol_i_ref = 0.0f;
    foc->ol_i_meas = 0.0f;
    foc->ol_i_peak = 0.0f;
    foc->ol_vb_min = 0.0f;
    foc->ol_vb_max = 0.0f;
    foc->ol_dc_sum_u = 0.0f;
    foc->ol_dc_sum_v = 0.0f;
    foc->ol_dc_sum_w = 0.0f;
    foc->ol_dc_cnt = 0;
    foc->ol_dc_amp = 0.0f;
    foc->ol_dc_vv = 0.0f;
    foc->ol_dc_r = 0.0f;
    foc->ol_hall_edges = 0;
    foc->ol_hall_prev = 0;
    foc->ol_saved_torque = 1;
}

/* ---- ???? ---- */
void FOC_Control_Run(FOC_Control_t *foc, float dt_s)
{
    /* 过流检查: 只在三相输出真正使能驱动时才判断; IDLE/制动/故障或刚切到
       ALIGN 而 PWM 尚未使能的那一拍, 测到的电流是三相浮空的无效值, 跳过.
       再加开通消隐(100ms)与连续 4 次去抖, 避免开关尖峰误触发保护.
       手动开环(ol_manual)时**完全不跳闸**: 实测"三相同占空比(v=0,端子短接)"
       都会读到 12A 假电流, 一跳闸就关 PWM → 用户万用表只能量到 0V, 根本没法
       诊断. 手动模式只靠幅值软限流(超 3A 自动降压), 由用户自己看着办. */
    if (foc->state != FOC_STATE_IDLE &&
        foc->state != FOC_STATE_BRAKE &&
        foc->state != FOC_STATE_FAULT &&
        !foc->ol_manual &&
        s_pwm_on &&
        (foc->tick - s_pwm_on_tick) > OC_BLANK_TICKS) {
        if (fabsf(foc->current.u) > foc->config.overcurrent_amp ||
            fabsf(foc->current.v) > foc->config.overcurrent_amp) {
            s_oc_cnt++;
            if (s_oc_cnt >= OC_DEBOUNCE) {
                FOC_Control_SetFault(foc, 0x0001);
                return;
            }
        } else {
            s_oc_cnt = 0;
        }
    } else {
        s_oc_cnt = 0;
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
        case FOC_STATE_OPENLOOP:
            FOC_DoOpenLoop(foc, dt_s);
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
        /* 对齐结束: 此刻 theta 被强制为 0°, 转子 d 轴就停在该电角度上, 而霍尔
           状态此刻是可读的 → 用它自动校正霍尔扇区表零点.
           实测不校正时: 对齐时 hall=2, 而表里 h2 镜像后是 203.85° → 电流矢量
           与真实轴差 ~204°, 只剩 sin(204°)≈-0.40 的转矩且符号相反, 表现为
           "走得动但一冲一停、静摩擦大处彻底卡死、空载也不转"; 之前的
           torque_dir=-1 只是在补符号, 补不了这个相位差. */
        AngleObserver_CalibrateOffsetAtZero(&foc->angle, foc->hall.state);
        LOGD("[ALIGN] Align done, hall=%d -> offset calibrated %.1fdeg\n",
             (int)foc->hall.state,
             foc->angle.offset * 57.29577951f);
        foc->state = FOC_STATE_STARTUP;
        foc->startup_tick = foc->tick;
        foc->current_loop.id_ref = 0.0f;
        foc->current_loop.iq_ref = foc->config.startup_current;
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

    /* I-F 拖转电流: 用独立的 startup_current(不再用 align_current*0.5=1.0A).
       实测 1.0A 时转子只被拖到 ~13eRPM 就滑脱、6s 超时进 FAULT(42BLH90 齿槽
       转矩较大, 1A 在临界上). */
    foc->current_loop.iq_ref = foc->config.startup_current;
    FOC_UpdateCurrent(foc);

    /* ???????? */
    erpm = AngleObserver_GetOmegaFiltErpm(&foc->angle, foc->config.pwm_freq);
    /* 同步到就尽早交接(>0.3s 且已有转速), 并把开环拖转累积的巨大机械行程清零,
       否则目标0/45/90 要追几千度 → 永远顶/振/跑飞, 无法调闭环.
       判据放宽到 8eRPM: 霍尔表零点已在 ALIGN 阶段自动校正, 交接后闭环不依赖
       "转多快", 只要霍尔有效即可(原来卡 >20, 实测拖转只有 ~13 就判失败). */
    if (foc->angle.synced && elapsed > 6000U && fabsf(erpm) > 8.0f) {
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
        /* 多边沿测速历史一起清掉: I-F 期间 elec_acc_deg 没在更新, 旧历史
           算出来的差值是无意义的 */
        for (int i = 0; i < 4; i++) {
            foc->angle.edge_hist_tick[i] = 0U;
            foc->angle.edge_hist_acc[i]  = 0.0f;
        }
        foc->angle.edge_hist_idx = 0U;
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

/* =====================================================================
 * 手动开环 (o 键) —— 手上只有万用表时的主手段
 * ---------------------------------------------------------------------
 * 自动测试的问题: 一旦哪里不对就中止, 而且一步给一大截, 用户插不上手。
 * 手动模式把控制权全交给按键:
 *   o = 进入/重进,  u = 电压 +0.01,  i = 电压 -0.01,
 *   h = 电频率 +1Hz, l = -1Hz,       f = 反向,  t = 停止
 * 推荐流程:
 *   step1 保持 f=0Hz(静止), 用 u 每步 +0.01 duty 慢慢升压, 同时拿万用表量
 *         任意两相引出线之间的电压, 和日志 `[MAN] ... Vll=..V` 对比:
 *         * 两者接近 → 电压真的加到电机上了, 问题在电流采样标定;
 *         * 万用表几乎不动而 Vll 一直在涨 → 电压根本没加上(高边/自举/驱动),
 *           这同时也直接解释了"电机不转"。
 *   step2 把频率用 h 升到 3~5Hz(pp=4 → 45~75rpm), 电机应该开始转。
 * 全程纯电压开环: 不用电流环、不用霍尔、不用观测器, 只用电流幅值做软限流
 * (超 3A 就把电压打八折并向用户回报, 不中止)。
 * ===================================================================== */
static void FOC_DoManual(FOC_Control_t *foc, float i_amp)
{
    float valpha, vbeta;
    float i_abs;

    /* 角度按设定频率自行累加 */
    foc->ol_omega = 6.28318531f * foc->ol_man_hz / foc->config.pwm_freq;
    foc->ol_theta += (float)foc->ol_dir * foc->ol_omega;
    while (foc->ol_theta >= 6.28318531f) foc->ol_theta -= 6.28318531f;
    while (foc->ol_theta <  0.0f)       foc->ol_theta += 6.28318531f;
    foc->angle.theta = foc->ol_theta;

    /* 软限流 */
    if (i_amp > OL_MAN_ILIM) {
        foc->ol_man_v *= 0.8f;
        if (foc->tick - foc->ol_last_warn > 4000U) {   /* 200ms 最多一条 */
            foc->ol_last_warn = foc->tick;
            LOGW("[MAN] |I|=%.2fA > %.1fA -> v down to %.4f\n",
                 i_amp, OL_MAN_ILIM, foc->ol_man_v);
        }
    }

    /* 硬保护: 持续大电流 → 关 PWM。即使 v=0 也会触发(下管击穿时的回流路径)。 */
    if (i_amp > OL_MAN_TRIP_A) {
        s_man_oc_cnt++;
        if (s_man_oc_cnt > (OL_MAN_TRIP_MS * 20U)) {
            s_man_oc_cnt = 0U;
            foc->ol_man_v = 0.0f;
            LOGW("[MAN] !! %0.2fA for %ums -> PWM OFF. HW FAULT? check W-phase FET/shunt. press u to re-arm\n",
                 i_amp, (unsigned)OL_MAN_TRIP_MS);
            FOC_Control_SetFault(foc, 0x0001);
            return;
        }
    } else {
        s_man_oc_cnt = 0U;
    }
    foc->ol_v_amp = foc->ol_man_v;

    /* 矢量固定在 q 轴(vd=0, vq=v) → 施加角度 = theta+90° */
    InvPark_Transform(0.0f, foc->ol_v_amp, foc->ol_theta, &valpha, &vbeta);
    PWM_Output_SVPWM(&foc->pwm, valpha, vbeta,
                     foc->current.u, foc->current.v, foc->current.w);
    PWM_Output_Apply(&foc->pwm);
    if (!s_pwm_on) s_pwm_on_tick = foc->tick;
    s_pwm_on = 1;

    /* 统计 */
    if (foc->current.vbus > 0.1f) {
        if (foc->ol_vb_min <= 0.0f || foc->current.vbus < foc->ol_vb_min) {
            foc->ol_vb_min = foc->current.vbus;
        }
        if (foc->current.vbus > foc->ol_vb_max) {
            foc->ol_vb_max = foc->current.vbus;
        }
    }
    i_abs = fabsf(foc->current.u);
    if (fabsf(foc->current.v) > i_abs) i_abs = fabsf(foc->current.v);
    if (fabsf(foc->current.w) > i_abs) i_abs = fabsf(foc->current.w);
    if (i_abs > foc->ol_i_peak) foc->ol_i_peak = i_abs;

    /* 霍尔跳变计数: 客观证明转子有没有转 */
    if (foc->hall.state != foc->ol_hall_prev &&
        foc->hall.state >= 1U && foc->hall.state <= 6U) {
        foc->ol_hall_edges++;
        foc->ol_hall_prev = foc->hall.state;
    }
}

void FOC_Control_StartManualTest(FOC_Control_t *foc)
{
    foc->ol_active     = 1;
    foc->ol_manual     = 1;
    foc->ol_pwm_only   = 0;
    foc->ol_phase      = 0;
    foc->ol_phase_tick = 0;
    foc->ol_tick       = 0;
    foc->ol_dir        = 1;
    foc->ol_man_v      = 0.0f;      /* 电压从 0 开始, 由用户一步步加 */
    foc->ol_man_hz     = 0.0f;      /* 频率从 0 开始(静止) */
    foc->ol_theta      = 0.0f;
    foc->ol_omega      = 0.0f;
    foc->ol_v_amp      = 0.0f;
    foc->ol_total      = 0xFFFFFFFFU;   /* 不自动停, 靠 t 键 */
    foc->ol_i_peak     = 0.0f;
    foc->ol_vb_min     = 0.0f;
    foc->ol_vb_max     = 0.0f;
    foc->ol_hall_edges = 0U;
    foc->ol_hall_prev  = foc->hall.state;
    foc->ol_last_warn  = 0U;
    s_man_oc_cnt       = 0U;

    foc->flags.fault   = 0;
    foc->fault_code    = 0;
    foc->flags.enabled = 1;
    foc->state         = FOC_STATE_OPENLOOP;

    CurrentLoop_ResetInteg(&foc->current_loop);
    s_oc_cnt = 0;

    LOGD("[MAN] START manual open-loop. v=0 f=0. vb=%.1fV pp=%d\r\n",
         foc->current.vbus, (int)foc->config.pole_pairs);
    LOGD("[MAN] step1: f=0Hz, press 'u' step by step; put DMM across 2 motor wires,\r\n");
    LOGD("[MAN]        compare with Vll in the [MAN] log (if DMM stays ~0 while Vll grows,\r\n");
    LOGD("[MAN]        the voltage is NOT reaching the motor -> high-side / bootstrap problem)\r\n");
    LOGD("[MAN] step2: press 'h' to raise freq to 3~5Hz -> motor should spin\r\n");
}

void FOC_Control_ManualAdjust(FOC_Control_t *foc, float dv, float dhz)
{
    if (!foc->ol_active || !foc->ol_manual) {
        LOGD("[MAN] not in manual mode, press 'o' first\r\n");
        return;
    }

    foc->ol_man_v += dv;
    if (foc->ol_man_v < 0.0f)  foc->ol_man_v = 0.0f;
    if (foc->ol_man_v > 0.45f) foc->ol_man_v = 0.45f;

    /* 重设状态: 上一版被过流踢出 FAULT 后, 用户以为还在手动模式, 其实 PWM
       早被关掉 → DMM 量到 0V 却不明所以. 现在每次调节都重新进入 OPENLOOP. */
    foc->flags.fault = 0;
    foc->fault_code  = 0;
    foc->state       = FOC_STATE_OPENLOOP;
    s_oc_cnt = 0;

    foc->ol_man_hz += dhz;
    if (foc->ol_man_hz < 0.0f)  foc->ol_man_hz = 0.0f;
    if (foc->ol_man_hz > 30.0f) foc->ol_man_hz = 30.0f;

    /* 回到静止时把相位复位, 免得再起转时突然大跳 */
    if (foc->ol_man_hz <= 0.0f) foc->ol_theta = 0.0f;

    LOGD("[MAN] v=%.4f (Vll=%.2fV)  f=%.1fHz (%.0frpm@pp=%d)\r\n",
         foc->ol_man_v,
         1.7320508f * foc->ol_man_v * foc->current.vbus,
         foc->ol_man_hz,
         foc->ol_man_hz * 60.0f / (float)foc->config.pole_pairs,
         (int)foc->config.pole_pairs);
}

/* =====================================================================
 * 开环速度测试 (S 键) —— 单独验证"功率级 + 电机 + 供电"
 * ---------------------------------------------------------------------
 * 为什么完全不用电流环: 前几版的教训 —— ALIGN 里 `Iq=+1.9A 而 vq=-0.40 顶在
 * 限幅`, 也就是"电压矢量与电流矢量几乎反向", 说明电流环的符号/相序链路上
 * 有问题; 一旦掺进电流环, 就分不清"电机不转"是功率级问题还是环路符号问题.
 * 所以本测试**只做纯电压开环**, 反馈只用电流"幅值"
 *     |I| = sqrt(2/3*(iu^2+iv^2+iw^2))  (与相序/符号完全无关)
 * 并用它做限流和慢速调节, 对 dq 符号免疫.
 *
 * 段0 (0 ~ 3.3s): 静止. 电压从 0 **线性缓慢**升到 0.06 duty(每拍只动 ~2e-6,
 *     没有任何阶跃 —— 上一版一跳变就冒出 6.75A 假尖峰, 就是阶跃引起的).
 *     电流到 2.5A 或升到顶就冻结电压, 再稳压 0.3s 求平均 →
 *         R_eq = v / |I|
 *     这就是最近几轮唯一悬而未决的量(闭环日志折算 ≈3Ω, 与实测 0.6Ω线间矛盾).
 *
 * 段1 (3.3 ~ 10s): 开环 V/f 旋转. theta 由内部斜坡 0→15Hz(电)产生,
 *     施加电压矢量固定在 q 轴, 幅值由"幅值型慢速调节器"拉到 ol_i_ref(1A)。
 *     转子被旋转电压矢量拖着走 —— 只要功率级和电机正常就会转。
 *
 * 不依赖霍尔、不依赖观测器、不依赖电流环符号; 10s 到点自动停, 或按 T 停,
 * 运行中按 F 可翻转旋转方向。
 * ===================================================================== */
static void FOC_DoOpenLoop(FOC_Control_t *foc, float dt_s)
{
    float ia, ib, i_amp;
    float valpha, vbeta;
    float i_abs;

    (void)dt_s;

    if (!foc->ol_active) {
        foc->state = FOC_STATE_IDLE;
        return;
    }

    foc->ol_tick++;

    /* 电流幅值(对相序/符号免疫) */
    ia = foc->current.u;
    ib = (foc->current.u + 2.0f * foc->current.v) * 0.57735027f;
    i_amp = sqrtf(ia * ia + ib * ib);
    foc->ol_i_meas = i_amp;

    /* 只为日志填 id/iq(不做 PID) */
    Park_Transform(ia, ib, foc->ol_theta,
                   &foc->current_loop.id, &foc->current_loop.iq);

    /* 手动开环: 电压/频率全由按键给, 直接在这里做完返回 */
    if (foc->ol_manual) {
        FOC_DoManual(foc, i_amp);
        return;
    }

    switch (foc->ol_phase) {
    case 0U:    /* ---- 段0a: 三相短接放电 0.1s, 把 ALIGN 残留电流放掉 ---- */
        foc->ol_phase_tick++;
        foc->ol_v_amp = 0.0f;      /* v=0 → du=dv=dw=0.5, 三相端子短接, 电流按 L/R 衰减 */
        if (!foc->ol_pwm_only && foc->ol_phase_tick >= foc->ol_settle) {
            foc->ol_phase      = 1U;
            foc->ol_phase_tick = 0U;
        }
        break;

    case 1U:    /* ---- 段0b: 慢速线性升压 ---- */
        foc->ol_phase_tick++;
        foc->ol_v_amp = OL_DC_VMAX * (float)foc->ol_phase_tick /
                        (float)foc->ol_dc_ramp;
        if (foc->ol_phase_tick >= foc->ol_dc_ramp || i_amp >= OL_DC_I_FREEZE) {
            foc->ol_phase      = 2U;
            foc->ol_phase_tick = 0U;
            foc->ol_dc_sum_u   = 0.0f;
            foc->ol_dc_sum_v   = 0.0f;
            foc->ol_dc_sum_w   = 0.0f;
            foc->ol_dc_cnt     = 0U;
            LOGD("[OL] DC freeze: duty=%.4f |I|=%.2fA\r\n",
                 foc->ol_v_amp, i_amp);
        }
        break;

    case 2U:    /* ---- 段0c: 电压冻结, 平均 0.3s 算 R ---- */
        foc->ol_phase_tick++;
        foc->ol_dc_sum_u += foc->current.u;
        foc->ol_dc_sum_v += foc->current.v;
        foc->ol_dc_sum_w += foc->current.w;
        foc->ol_dc_cnt++;
        if (foc->ol_phase_tick >= foc->ol_dc_hold) {
            float iu_avg = 0.0f, iv_avg = 0.0f, iw_avg = 0.0f;
            float v_volts;

            if (foc->ol_dc_cnt > 0U) {
                iu_avg = foc->ol_dc_sum_u / (float)foc->ol_dc_cnt;
                iv_avg = foc->ol_dc_sum_v / (float)foc->ol_dc_cnt;
                iw_avg = foc->ol_dc_sum_w / (float)foc->ol_dc_cnt;
            }
            ia = iu_avg;
            ib = (iu_avg + 2.0f * iv_avg) * 0.57735027f;
            foc->ol_dc_amp = sqrtf(ia * ia + ib * ib);
            v_volts        = foc->ol_v_amp * foc->current.vbus;
            foc->ol_dc_vv  = v_volts;
            foc->ol_dc_r   = (foc->ol_dc_amp > 0.02f)
                             ? (v_volts / foc->ol_dc_amp) : 0.0f;
            /* θ=0 时 InvPark(0, v) 给出 dv-dw = +√3*v, 所以 V-W 两相之间
               万用表(读 PWM 平均值)应读到 √3 * ol_v_amp * vbus。 */
            LOGD("[OL] DC test: duty=%.4f vb=%.1fV -> Vph_amp=%.2fV | DMM across V-W should read %.2fV\r\n",
                 foc->ol_v_amp, foc->current.vbus, v_volts,
                 1.7320508f * v_volts);
            LOGD("[OL] DC test: |I|avg=%.2fA (iu=%.2f iv=%.2f iw=%.2f) => R_eq=%.2fohm  [real R=0.52ohm]\r\n",
                 foc->ol_dc_amp, iu_avg, iv_avg, iw_avg, foc->ol_dc_r);
            if (foc->ol_dc_r > 1.5f) {
                LOGW("[OL] => R_eq >> 0.52ohm: the commanded voltage is NOT reaching the motor\r\n");
                LOGW("[OL] => check HIGH-SIDE gate drive / bootstrap (scope HO vs phase) - not a current-sense issue\r\n");
            } else if (foc->ol_dc_r < 0.25f) {
                LOGW("[OL] => R_eq << 0.52ohm: current reading is too SMALL (sensing gain/shunt wrong)\r\n");
            }
            foc->ol_phase      = 3U;
            foc->ol_phase_tick = 0U;
        }
        break;

    default:    /* ---- 段1: 开环 V/f 旋转 + 幅值型慢速调流 ---- */
        foc->ol_phase_tick++;

        /* 只用电流幅值做反馈: 与 dq 符号/相序无关, 不会跑飞。
           OL_I_REG_KI=2.5e-5 duty/(A·tick) ≈ 0.5 duty/(A·s)。 */
        foc->ol_v_amp += OL_I_REG_KI * (foc->ol_i_ref - i_amp);
        if (foc->ol_v_amp < OL_V_AMP_MIN) foc->ol_v_amp = OL_V_AMP_MIN;
        if (foc->ol_v_amp > OL_V_AMP_MAX) foc->ol_v_amp = OL_V_AMP_MAX;

        if (foc->ol_phase_tick < foc->ol_ramp) {
            foc->ol_omega = foc->ol_omega_tgt *
                            (float)foc->ol_phase_tick / (float)foc->ol_ramp;
        } else {
            foc->ol_omega = foc->ol_omega_tgt;
        }
        foc->ol_theta += (float)foc->ol_dir * foc->ol_omega;
        while (foc->ol_theta >= 6.28318531f) foc->ol_theta -= 6.28318531f;
        while (foc->ol_theta <  0.0f)       foc->ol_theta += 6.28318531f;
        break;
    }

    /* ---------- 施加电压: 纯电压开环, 完全不走电流环 ----------
       矢量固定在 q 轴(vd=0, vq=ol_v_amp), 于是施加角度 = theta+90°;
       段0 里 theta 恒为 0 → 矢量停在 90°, 转子对齐; 段1 theta 一转就拖动转子. */
    foc->angle.theta = foc->ol_theta;
    InvPark_Transform(0.0f, foc->ol_v_amp, foc->ol_theta, &valpha, &vbeta);
    PWM_Output_SVPWM(&foc->pwm, valpha, vbeta,
                     foc->current.u, foc->current.v, foc->current.w);
    PWM_Output_Apply(&foc->pwm);
    if (!s_pwm_on) s_pwm_on_tick = foc->tick;
    s_pwm_on = 1;

    /* ---------- 统计 + 安全 ---------- */
    if (foc->current.vbus > 0.1f) {
        if (foc->ol_vb_min <= 0.0f || foc->current.vbus < foc->ol_vb_min) {
            foc->ol_vb_min = foc->current.vbus;
        }
        if (foc->current.vbus > foc->ol_vb_max) {
            foc->ol_vb_max = foc->current.vbus;
        }
    }
    i_abs = fabsf(foc->current.u);
    if (fabsf(foc->current.v) > i_abs) i_abs = fabsf(foc->current.v);
    if (fabsf(foc->current.w) > i_abs) i_abs = fabsf(foc->current.w);
    if (i_abs > foc->ol_i_peak) foc->ol_i_peak = i_abs;

    /* 霍尔跳变计数: 用来客观判断转子有没有真的动。电流保持在 1A 时, 光看电流
       分不清是"拖着转子同步转"还是"转子没跟上、电流在绕组里原地打转"。 */
    if (foc->hall.state != foc->ol_hall_prev &&
        foc->hall.state >= 1U && foc->hall.state <= 6U) {
        foc->ol_hall_edges++;
        foc->ol_hall_prev = foc->hall.state;
    }

    /* 硬限流: 目标才 1A, 单相超限说明桥臂直通/短路 → 立即停.
       放电段(ph=0)不判: 那时残留的是上一个状态(ALIGN)的电流, 本来就可能在
       衰减中, 交给全局 6A 过流保护即可. */
    if (foc->ol_phase >= 1U && i_abs > OL_TRIP_A) {
        LOGW("[OL] ABORT over-current %.2fA (>%.1fA)\n", i_abs, OL_TRIP_A);
        FOC_Control_StopOpenLoopTest(foc, "overcurrent");
        return;
    }

    if (foc->ol_tick >= foc->ol_total) {
        FOC_Control_StopOpenLoopTest(foc, "timeout");
    }
}

void FOC_Control_StartOpenLoopTest(FOC_Control_t *foc, float hz,
                                   float current_A, uint32_t dur_ms)
{
    float    tpm = foc->config.pwm_freq / 1000.0f;   /* tick 每毫秒 */
    uint32_t n   = (uint32_t)((float)dur_ms * tpm);

    foc->ol_dc_ramp = (uint32_t)((float)OL_DC_RAMP_MS * tpm);
    foc->ol_settle  = (uint32_t)((float)OL_SETTLE_MS * tpm);
    foc->ol_dc_hold = (uint32_t)((float)OL_DC_HOLD_MS * tpm);
    foc->ol_ramp    = (uint32_t)((float)OL_ACC_RAMP_MS * tpm);
    if (foc->ol_settle == 0U)  foc->ol_settle = 1U;
    if (foc->ol_dc_ramp == 0U) foc->ol_dc_ramp = 1U;
    if (foc->ol_dc_hold == 0U) foc->ol_dc_hold = 1U;
    if (foc->ol_ramp == 0U)    foc->ol_ramp = 1U;
    /* 至少要留出段0 + 2s 的旋转时间 */
    if (n < foc->ol_settle + foc->ol_dc_ramp + foc->ol_dc_hold + 40000U) {
        n = foc->ol_settle + foc->ol_dc_ramp + foc->ol_dc_hold + 40000U;
    }

    foc->ol_active     = 1;
    foc->ol_pwm_only   = 0U;
    foc->ol_phase      = 0U;
    foc->ol_dir        = 1;
    foc->ol_tick       = 0U;
    foc->ol_phase_tick = 0U;
    foc->ol_total      = n;
    foc->ol_theta      = 0.0f;
    foc->ol_omega      = 0.0f;
    foc->ol_omega_tgt  = 2.0f * 3.14159265f * hz / foc->config.pwm_freq;
    foc->ol_v_amp      = 0.0f;
    foc->ol_i_ref      = current_A;
    foc->ol_i_meas     = 0.0f;
    foc->ol_i_peak     = 0.0f;
    foc->ol_vb_min     = 0.0f;
    foc->ol_vb_max     = 0.0f;
    foc->ol_dc_sum_u   = 0.0f;
    foc->ol_dc_sum_v   = 0.0f;
    foc->ol_dc_sum_w   = 0.0f;
    foc->ol_dc_cnt     = 0U;
    foc->ol_dc_amp     = 0.0f;
    foc->ol_dc_vv      = 0.0f;
    foc->ol_dc_r       = 0.0f;
    foc->ol_hall_edges = 0U;
    foc->ol_hall_prev  = foc->hall.state;

    /* 测试全程走纯电压开环, torque_dir 不参与; 仍记录并还原以免影响后续闭环 */
    foc->ol_saved_torque = foc->torque_dir;

    foc->flags.fault   = 0;
    foc->fault_code    = 0;
    foc->flags.enabled = 1;
    foc->state         = FOC_STATE_OPENLOOP;

    CurrentLoop_ResetInteg(&foc->current_loop);
    SpeedLoop_ResetInteg(&foc->speed_loop);
    s_erpm_filt = 0.0f;
    s_oc_cnt = 0;

    LOGD("[OL] START f=%.1fHz(elec) %.1frpm@pp=%d I=%.2fA dur=%ums (dc-ramp=%ums hold=%ums)\r\n",
         hz, hz * 60.0f / (float)foc->config.pole_pairs,
         (int)foc->config.pole_pairs, current_A, dur_ms,
         (uint32_t)((float)foc->ol_dc_ramp / tpm),
         (uint32_t)((float)foc->ol_dc_hold / tpm));
}

void FOC_Control_StopOpenLoopTest(FOC_Control_t *foc, const char *reason)
{
    if (!foc->ol_active) return;

    foc->ol_active = 0;
    foc->ol_manual = 0;
    foc->ol_pwm_only = 0;
    foc->current_loop.id_ref = 0.0f;
    foc->current_loop.iq_ref = 0.0f;
    foc->torque_dir = foc->ol_saved_torque;
    PWM_Output_Disable();
    s_pwm_on = 0;
    foc->state = FOC_STATE_IDLE;

    /* 一条总结: 转了多少电角度 / 峰值电流 / 母线电压范围 / 段0 测得的电阻 /
       霍尔跳变次数. hall_edges>0 就是转子真的转过(每电周期 6 次跳变). */
    LOGD("[OL] STOP (%s) t=%.2fs theta=%.1fdeg hall_edges=%u peak|i|=%.2fA vb=[%.1f, %.1f]V R_eq=%.2fohm\r\n",
         reason,
         (float)foc->ol_tick / foc->config.pwm_freq,
         foc->ol_theta * 57.29577951f,
         foc->ol_hall_edges,
         foc->ol_i_peak, foc->ol_vb_min, foc->ol_vb_max, foc->ol_dc_r);
}

void FOC_Control_FlipOpenLoopDir(FOC_Control_t *foc)
{
    foc->ol_dir = (int8_t)(-foc->ol_dir);
    LOGD("[OL] dir -> %+d\r\n", (int)foc->ol_dir);
}

/* ---- 硬件烟测: 三相均为 50% 占空比(v=0), 不旋转, 直到按 T ----
   为什么单独做这个: 日志里在"三相同占空比(绕组短接)"状态下读到 8.75A 且还在
   涨到 17.9A —— 短接的绕组电流只能衰减, 不可能上升。所以要么是开关期间的采样
   被污染, 要么是功率级有旁路/直通。把电机三根引出线拆掉再跑这个测试:
     * |I| ≈ 0  → 采样链路 OK, 那就真的是功率级在实际导通, 查驱动/桥臂;
     * |I| 仍很大 → 与电机无关, 是采样链路(运放/ADC触发点/地)被开关噪声污染。
   注意此时 du=dv=dw=0.5 严格相等(已关掉死区补偿), 不存在人为电压差。 */
void FOC_Control_StartPwmSmokeTest(FOC_Control_t *foc)
{
    FOC_Control_StartOpenLoopTest(foc, 15.0f, 0.0f, 10000U);
    foc->ol_pwm_only = 1U;
    foc->ol_phase    = 0U;
    foc->ol_phase_tick = 0U;
    foc->ol_total    = 0xFFFFFFFFU;   /* 不自动停, 靠 T 键 */
    LOGD("[OL] PWM smoke test: du=dv=dw=0.5 (v=0) until 't'. MOTOR WIRES DISCONNECTED?\r\n");
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
    if (!s_pwm_on) s_pwm_on_tick = foc->tick;   /* 记录开通时刻, 用于过流消隐 */
    s_pwm_on = 1;   /* 三相输出已使能, 之后的采样才有效, 可做过流判断 */
}

/* ---- ???? ---- */
void FOC_Control_SetState(FOC_Control_t *foc, FOC_State_t state)
{
    if (foc->flags.fault) return;
    /* 开环测试(手动/自动 V-f/烟测)进行中要切到别的状态 → 先干净地停掉测试
       (恢复 torque_dir). 注意: **不能在这里 return**, 否则状态停在 IDLE,
       调用方(如按 'r' 要进 ALIGN)看起来就像"按了没反应", 得按两次才生效。 */
    if (foc->ol_active && state != FOC_STATE_OPENLOOP) {
        FOC_Control_StopOpenLoopTest(foc, "state change");
    }
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
    s_oc_cnt = 0;
    /* 故障码 + 跳闸瞬间三相电流 + 母线电压:
       0x0001=过流, 0x0002=启动超时(霍尔没同步).
       母线电压是关键: 若 vb 远低于 24V, 说明电源被限流/供电塌陷, 此时
       "0.45 占空比"根本没有 11V 输出, 电流环必然顶到限幅也拉不到目标值. */
    LOGD("[FAULT] code=0x%04X iu=%.2f iv=%.2f iw=%.2f state=%d vb=%.1fV tick=%u\n",
         code, foc->current.u, foc->current.v, foc->current.w,
         (int)foc->state, foc->current.vbus, foc->tick);
}

FOC_State_t FOC_Control_GetState(const FOC_Control_t *foc)
{
    return foc->state;
}

uint8_t FOC_Control_IsFault(const FOC_Control_t *foc)
{
    return foc->flags.fault;
}

