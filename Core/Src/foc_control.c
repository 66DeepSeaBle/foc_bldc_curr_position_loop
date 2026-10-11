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
    foc->pos_target_user_set = 0U;
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
    SpeedLoop_Init(&foc->speed_loop, 0.004f, 0.02f,
                   0.5f, 1.2f, 10.0f, 200U, 1000.0f);

    /* ??????
       kp=0.3 eRPM/deg 太小: 45° 误差只给 13.5 eRPM(≈2.5°/s), 而霍尔瞬时速度
       是几百~上千 eRPM, 差 100 倍以上 → 速度环永远处于 bang-bang.
       提到 4.0: 45° → 180 eRPM(≈42°/s, 约 1.1s 到位), 与实际速度同量级;
       decel 150→2000 让 √(2*a*dist) 限速曲线不再一上来就把指令压死. */
    /* 2026-09-30 大降增益(实测硬件修好、方向改对后出现的"剧烈来回摆"):
       现象: 目标 0°、位置在 -90°~+105° 之间以 ~5Hz 大幅摆动, Iqref 在
       ±3.5A 之间翻, 机构明显在两端硬限位上撞。
       原因: ①位置反馈只有 6 步/电周期(霍尔扇区 ≈15° 机械阶梯)且带滞后,
             ②速度反馈来自多边沿窗口 + 100ms 低通, 相位滞后很大,
             ③位置环 kp=4.0(eRPM/deg) + 速度环 iq_cap=3.5A ⇒ 环路增益远
               高于反馈能支撑的带宽 ⇒ 饱和式 limit cycle(一冲一撞)。
       修改: kp 4.0→1.0, 限速 2000→300eRPM(≈75rpm 机械, 温和),
             decel 2000→400, 到位容差 3°→6°(与 15° 阶梯匹配);
             速度环 iq_cap 3.5→1.2A / 1.5→0.5A, kp 0.008→0.004,
             ki 0.05→0.02, 斜坡 2000→1000 eRPM/s。
       目标是"先不撞、不震", 稳了之后再逐级往上加。 */
    PositionLoop_Init(&foc->position_loop, 1.0f,
                      300.0f, 400.0f,
                      6.0f, 500U,
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
        /* 2026-09-30 保护: 只有霍尔状态有效(1~6)时才用它校正零点。
           hall=0/7 是非法状态(常见原因: 霍尔没供电 / 接插件松 → MCU 内部上拉
           把三相都读成高)。用它校正会把 offset 写成垃圾(实测 hall=0 时 offset
           被校成 0.0°), 而 offset 错了之后转矩矢量方向全错 —— 现象就是
           "给了 1.2A 电流, 电机一动不动、Pos 冻结、fb=0"。
           此时保留上一次的 offset 并报警。 */
        if (foc->hall.state >= 1U && foc->hall.state <= 6U) {
            AngleObserver_CalibrateOffsetAtZero(&foc->angle, foc->hall.state);
            LOGD("[ALIGN] Align done, hall=%d -> offset calibrated %.1fdeg\r\n",
                 (int)foc->hall.state,
                 foc->angle.offset * 57.29577951f);
        } else {
            /* 2026-10-01 ★★★ 必须直接停机, 不能继续往下跑!
               没有正确的 offset, theta 就是错的 ⇒ 电流矢量方向错 ⇒ 位置环
               变成正反馈, 电机会被满力矩抽飞。实测(日志): ALIGN 读不到霍尔
               (hall=0) → 未标定 → 进位置环后 Pos 一路跑到 -39616° (110 圈)、
               vsp 撞死 ±3000 限幅、相电流到 5.4A。
               根因: hall.state 只在 EXTI 里更新, 停机无边沿 ⇒ 一直是初值 0。
               已在 main.c 的 ADC 中断里每拍轮询霍尔引脚, 现在能读到真值;
               这里再加一道保险: 读不到就直接 FAULT。 */
            LOGW("[ALIGN] hall=%d INVALID(0/7) -> FAULT! check hall power/wiring (MCU pull-up reads all-high when unpowered)\r\n",
                 (int)foc->hall.state);
            FOC_Control_SetFault(foc, 0x0003);
            foc->flags.enabled = 0;
            return;
        }
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
    if (foc->angle.synced &&
        foc->hall.state >= 1U && foc->hall.state <= 6U &&
        elapsed > 6000U && fabsf(erpm) > 8.0f) {
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
        /* 2026-10-11c: 位置现在是绝对值算法, 清零要搬参考点(写 elec_acc_deg
           会在下一拍被覆盖) */
        AngleObserver_ZeroPos(&foc->angle);
        /* 多边沿测速历史一起清掉: I-F 期间 elec_acc_deg 没在更新, 旧历史
           算出来的差值是无意义的 */
        for (int i = 0; i < 4; i++) {
            foc->angle.edge_hist_tick[i] = 0U;
            foc->angle.edge_hist_acc[i]  = 0.0f;
        }
        foc->angle.edge_hist_idx = 0U;
        /* ★ 10-11k: 把模型速度也清零。I-F 拖转刚结束的"速度"是开环给的,
           交接到闭环后转子靠摩擦减速(≈290°/s²), 而模型会带着这个旧速度
           继续指数衰减地"爬"—— 实测爬了 9.4°机械(0.6 个扇区) ⇒ 位置读数
           过量、位置环去反推一个其实已经停住的转子(持续振动 + 顶久了塴出去)。 */
        foc->angle.omega       = 0.0f;
        foc->angle.omega_filt  = 0.0f;
        foc->angle.alpha_filt  = 0.0f;
        pos = AngleObserver_GetMechDeg(&foc->angle, foc->config.pole_pairs);
        /* 2026-10-01: 以前这里无条件把目标清成"原地保持 0°", 于是 ALIGN/
           STARTUP(~0.6s)期间按的数字键会被静默丢掉 —— 实测"按 r 后马上按 1
           电机没反应"就是这个。现在只要用户在这轮里设过目标就保留它。 */
        if (!foc->pos_target_user_set) {
            foc->pos_target_deg_cmd = pos;
        }
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

/* 位置环增益 (2026-10-01 v3)
   kp = A/deg, ki = A/(deg·s) 积分, kd = A/eRPM 速度阻尼
   2026-10-01 改为"强阻尼+低增益+速度门控积分": 该机构静摩擦大(起动~1.15A)、
   一动又很轻快, 实测出现 **±1个霍尔扇区(±15°机械)、~3Hz 的极限环**
   (日志: Tgt=45 时 Pos 在 29.20/59.51 之间来回, err 永远 ±15°, arr 永远 0,
    Iqref 在 ±1.8A 饱和来回; Tgt=0 时同样 ±16°)。
   能量来源三个: ① kp 太大 ⇒ 相当于继电器, 一冲就过; ② 阻尼用的速度滤波
   0.1s 太慢(在 3Hz 上有 ~100° 相位滞后, 不但不阻尼反而助推); ③ 积分的
   90° 相位滞后推着 stick-slip 循环。
   v3: kp 0.060→0.040; kd 0.0030→0.0060(并在 12ms 快滤波上算);
   积分只在"转子不动"(|v|<40eRPM)时才增长, 一运动就冻住/缓慢泄放。 */
#define POS_KP_A_PER_DEG     0.040f
/* 位置积分 KI(A/(deg·s)): 只当"破静摩擦的小助手", 绝不能主导驱动。
   ⚠ 2026-10-11 实测教训: 原值 0.25 配上 90° 误差 = **22.5 A/s**, 而门控
   (|v|<40eRPM) 在起步时天然成立 ⇒ 60ms 就把积分顶到上限 1.4A ⇒ 起步等于
   "速度环 + 1.4A"≈ 2.5A 满力矩 ⇒ 100ms 内把转子加速到 ~1300°/s(日志:
   `Pos` 从 -0.01 一跳到 **134.47**, `vsp=590`), 远超规划曲线(当时只允许
   172eRPM) ⇒ 2.5A 也刹不住 ⇒ 冲过目标 44° → 反向再来 → **±37° 极限环**
   (Tgt=90 在 59.5°~134.5° 之间永不停)。同时把门控收紧成"真堵转"
   (POS_INTEG_STALL_TICKS 内没有霍尔边沿) ⇒ 运动中永不积分、不会绕积。 */
#define POS_KI_A_PER_DEG_S   0.25f
/* (历史) 阻尼增益 kd 与"近区加大阻尼/速度死区"三个版本(2026-10-09c/e/g)
   已全部被下面的三段式速度规划(2026-10-09i)取代 —— kd 形式最终败在:
   ① 冲过目标是因为"误差还是正的 ⇒ P 仍在往前推", 而阻尼上限就那么大,
      两者相消(实测卡住/过冲时 Iqref 只有 ±0.6~1.5A);
   ② 停转时观测器残留的"假速度"经 kd 把 P 项抵掉, 还得再加死区去补。
   现在只用 POS_DAMP_MAX_A 作刹车项限幅。 */
/* ---- 三段式速度规划限速 (2026-10-09i, 取代旧的"近区阻尼+死区") ----
   用户要的"启动加速 / 匀速 / 接近目标减速"在这里实现成**只限速、不跟踪**:
     阶段1 启动: 由 P+积分驱动, 力矩上限 2.5A —— 起步力矩不被削弱(齿槽需
                ~1.5A 才能拉出, 不能掐)。
     阶段2 匀速: |v| 被限在 POS_VCRUISE_ERPM(300eRPM ≈ 75 机械rpm ≈ 450°/s)。
     阶段3 减速: 进目标 POS_DECEL_ZONE_DEG(45°) 内, 允许速度按 γv=K×√d 收缩
                (15° 处 100eRPM/150°/s, 4° 处 52eRPM, 1° 处 26eRPM) —— 这
                就是 v²=2·a·d 的运动学减速曲线, 到目标正好降到 0。
   超过允许速度才刹车, 没超速时阻尼完全为 0 ⇒ 对速度反馈的噪声/边界抖动
   免疫(真做成速度环会把 ±50~100eRPM 噪声放大成电流抖动, 就是 2026-09-30
   那次嗡振的成因); 而停转时的"假速度"也不再抵消 P 项。 */
/* ★★ 2026-10-11b 关键修正: 速度曲线改成**单一 √ 曲线 + 巡航上限**, 去掉
   "45° 处换段"。10-11a 实测 Tgt=180 又摆不停(90↔207 来回 7 次破黏), 数据算得:
   在 45° 换段瞬间 v_lim 从 300 一步跳到 174eRPM, 而 v_cmd 只能按 DEC 斜坡下坡
   (~105ms), 这 105ms 里转子仍以 ~450°/s 飞 ⇒ 误差只剩 30° 时实测速度还有 ~300eRPM
   (允许值的 2 倍) ⇒ 2.5A 刹车从 528°/s 需要 28° 才停, 而只剩 15° ⇒ **必然冲进
   下一个齿槽 195.47°**; 而那儿环路只有 -1.3A(P+I 全上也只有这么多)拉不出来
   (起动要 >1.15A, 齿槽更深) ⇒ 只能靠 200ms/2.5A 破黏脉冲, 而那个脉冲太狠
   (200ms 把转子甩回 90°, 105° 行程) ⇒ 冲 195 → 踢回 90 → 再冲 = 死循环。
   所以: ①曲线连续 —— √ 从 d=(V/K)²=100° 起接管, 换段处 200eRPM 两侧相等、无跳变;
   ②巡航 300→200eRPM(450→300°/s), 动能降到 1/2; ③KV 0.010→0.015,
   同样 100eRPM 误差给 1.5A ⇒ 跟得住曲线; ④破黏脉冲 200ms→60ms 且"一见霍尔
   边沿立刻收手" ⇒ 它只负责"破"黏, 不再负责"推"。 */
#define POS_VCRUISE_ERPM     200.0f  /* 巡航速度(eRPM) ≈ 50 机械rpm ≈ 300°/s */
#define POS_ACC_ERPM_S       1200.0f /* 加速段加速度上限(eRPM/s) */
#define POS_DEC_ERPM_S       2000.0f /* 减速段减速度上限(eRPM/s) */
#define POS_JERK_ERPM_S2    12000.0f /* 加加速度上限(eRPM/s²): 加速度用 100ms 升满 */
#define POS_DECEL_ZONE_DEG  100.0f   /* =(V_CRUISE/K)²: 超过它才被巡航限住 */
#define POS_DECEL_K_ERPM     20.0f   /* v_lim=K×√d: 100°→200, 45°→134, 15°→77eRPM */
#define POS_KV_A_PER_ERPM    0.015f  /* 速度环增益(A/eRPM): 100eRPM 误差 → 1.5A */
/* 驱动电流限幅(2026-10-11g): 与"刹车"分开。
   ⚠ 本机构非常轻 + 阻滞大: 要动必须 >1.15A(过静摩擦), 而 1.15A 在它身上
   就是 ~10000°/s² —— 实测 1.86A 下 34ms 就飞到 674°/s。若驱动不单独限幅,
   起步就是 2.5A 满推 ⇒ 两三个扇区就冲过目标 15~80°。
   这里把**驱动**(电流方向与运动同向)限在 1.6A(刚好过静摩擦); **刹车**
   (电流与运动反向)仍放开到 POS_DAMP_MAX_A(2.5A), 保证刹得住。
   卡在齿槽里需要更大扭矩时, 由"堵转才增长的位置积分"(最多 0.8A)补上,
   总上限仍是 POS_IQ_CAP_A(2.5A) —— 也就是"平时温柔、卡住时凶"。 */
#define POS_IQ_RUN_A         1.6f    /* 驱动(加速)方向电流上限 */
#define POS_ARRIVE_V_ERPM    40.0f   /* 到位还需 |v|<它: 还在飞就继续用环刹车 */
/* ★ 2026-10-11j "温柔脱困区": |err| 小于它时, 一旦"基本没动"就不再给
   速度环驱动电流, 只让堵转积分慢慢长到破黏阈值(≈1.2~1.6A)。
   理由(本机物理): 1.15A 是静摩擦阈值, 而 1.86A 就能把转子推到 674°/s —— 
   脱困扭矩离阈值越近, 加速度越小(≈1000~2000°/s² 而不是 8000+), 走完一个
   齿槽(15.5°)就慢得多, 环路才有机会在"下一个齿槽"把它刹住。远目标(|err|
   大)时照旧全速驱动——那里走得快才重要。 */
#define POS_GENTLE_ZONE_DEG  20.0f
#define POS_INTEG_MAX_A      1.0f    /* ★10-11k 1.6→1.0: 积分只做"静态小误差/齿槽内偏移"的
                                        慢补偿。"脱困"完全交给下面的破黏脉冲 —— 因为实测
                                        1.6A 持续顶着只能把转子顶死在齿槽壁上(发出持续振动), 
                                        顶久了还会突然蹦出去(日志: 顶 400ms 后猛地飞 32°)。 */
/* 总限幅与阻尼限幅必须同时抬到 2.5A(2026-10-09f): 实测冲过目标的机理是
   —— 转子被 1.5A 拉出齿槽后开始移动, 而此时 **误差还是正的**(还没过目标),
   所以 P 项在不断“向前推”(如 0.04×29.5=1.18A), 而阻尼只能反向拉到
   修改前的 1.4A ⇒ 两者几乎相消(日志: 1.53A → 冲出去 52°), 转子几乎靠惯性
   滑过目标; 只有过了目标 P 才翻号、刹车才真正生效 ⇒ 必然冲一个大扇区。
   把两者都抬到 2.5A 后, 高速时阻尼 = kd(0.012)×v 可达 2.5A > P, 净输出
   变成刹车, 过目标前就能减速。
   安全: 2.5A 与破黏脉冲/align(2A) 同量级, 远低于过流 6A; 电流环 vlim=0.45
   占空比 ⇒ 10.8V/0.52Ω 能出 20A, 完全推得动(实测破黏时的 2.5A 已证实)。 */
#define POS_IQ_CAP_A         2.5f    /* 总电流上限(ALIGN 用 2A 都没事) */
#define POS_DAMP_TAU_S       0.012f  /* 阻尼用速度低通时间常数(原 0.1s 太慢) */
#define POS_DAMP_MAX_A       2.5f    /* 阻尼项限幅(曾 0.7/1.4, 见上面注释) */
#define POS_INTEG_GATE_ERPM  40.0f   /* |v| 超过它就算"在动", 积分冻结 */
#define POS_INTEG_STALL_TICKS (150U * 20U)   /* 另有此条: 150ms 无霍尔边沿才算堵转 */
/* ⚠ 2026-10-11b 再加一条: "想动"持续 POS_INTEG_ARM_TICKS 之后积分才被允许增长。
   光靠"150ms 无边沿"不够 —— 换目标的一瞬间转子本来就静止(停在旧目标上),
   天然满足"150ms 无边沿" ⇒ 积分会从第 150ms 就开始按 0.10×90°=9A/s 往上顶
   ⇒ 又变成"靠积分冲出去"(10-11a 的病)。加了这条之后: 起步阶段规划器虽然
   立刻要求运动(v_tgt≠0), 但积分要等 300ms 才"开闸", 那之前转子早该动起来
   (有霍尔边沿 ⇒ 堵转条件不成立 ⇒ 积分冻结); 只有真的卡在齿槽里不动,
   "想动 300ms + 150ms 无边沿" 才同时成立 ⇒ 积分只做"破齿槽助手"。 */
#define POS_INTEG_ARM_TICKS   (300U * 20U)   /* "想动"多久才允许积分增长 */

/* ---- 堵转破黏脉冲 (2026-10-09) ----
   实测日志: Tgt=90 时转子冲过头到 105.47° 后卡死, 800ms 没有霍尔边沿, 而
   Iqref 只有 -1.32A(= P 0.040x(-15.5)=-0.62A + 积分 -0.70A) —— 该转子位置的
   静摩擦阈值落在 1.3~1.6A 之间, 环路自己给不出更大的力(1.8A 的 cap 根本没用上),
   于是"永远差 15°"; 而再按 0(-1.60A)就动了。
   这里只加一个"确实不动 + 误差仍大"才触发的短脉冲: 把电流上限临时抬到
   2.5A 打 200ms 破黏, 打完立刻回到正常律。**不抬高环路增益**(2026-09-30 那次
   增益太高导致 ±5Hz 硬限位来回撞), 所以正常运动/到位时行为完全不变。
   触发: |err|>容差 且 距上次霍尔边沿 > 400ms(真不动) 且 距上次脉冲 > 800ms。
   安全: 2.5A 与 align(2A) 同量级, 远低于过流 6A / 手动硬保护 6A。 */
#define POS_KICK_A              2.5f
#define POS_KICK_TICKS          (100U * 20U)   /* 脉冲时长上限(20kHz tick): 只负责"破"黏 */
/* ★★ 2026-10-11k: 为什么脉冲变强变长 + 为什么要"预置电流环积分"
   实测(10-11j 日志): 2.2A/60ms 的脉冲几乎没出力 —— 电流环很慢
   (kp=0.010/ki=0.300 duty/(A·s), 1A 误差下积分只涨 0.3duty/s, 而 1.6A 静稳只要
   0.045duty) ⇒ 要 ~250ms 才把电流建立起来, 60ms 里实际电流只到 0.2A 量级, 
   而位置积分却要 400ms 才顶到 1.6A ⇒ 两头都不是"干脆的一下"。
   现在: ① 脉冲启动时**预置 q 轴积分**(= I×R/vbus, 见 POS_FF_DUTY_PER_A) ⇒ 电流
   1ms 内到位(电流环时间常数 L/R≈0.6ms), 脉冲才真的有力;
   ② 时长 100ms, 速度一超 POS_KICK_EXIT_ERPM 立刻收手;
   ③ 脉冲一开始就把位置积分清零; 脉冲结束后进入休息(受 POS_KICK_COOLDOWN_TICKS
   封住) —— **休息期间不再有任何持续推力**。这一点是用户反馈"到位后小幅振动"
   的关键: 原来靠"位置积分顶到 1.6A 一直顶着", 转子被顶在齿槽壁上,
   电流纹波+角度误差就变成持续可感的振动, 顶久了还会蹦出去(9.5A 过流跳闸)。 */
#define POS_FF_DUTY_PER_A       0.028f  /* 堵转时每 1A 需要的占空比(实测 1.6A→vq≈0.045) */
/* ★ 2026-10-11k 误差符号抖动抑制:
   位置读数在扇区内的分辨率极限就是**半个霍尔扇区**(15.47°/2≈7.7°机械) —— 
   目标附近 err 会在 ±8° 里反复翻号(模型进角/边沿重置的噪声), 环路于是
   "推一下→读数翻号→反推一下" = 用户看到的小幅振动。所以近目标且符号
   刚翻过时, 一律不给任何电流, 等符号稳定下来再说。 */
#define POS_HOLD_BAND_DEG       10.0f
#define POS_ERR_HYST_TICKS      (150U * 20U)
/* 脱困刹车只在该刹的时候给: 速度掉到它以下就立刻松手 */
#define POS_ESC_BRAKE_V_ERPM    60.0f
#define POS_ESC_BRAKE_MIN_TICKS (25U * 20U)
/* ★ 2026-10-11h: 破黏脉冲**一见转动就收手**(|v| 超过它就不再强搭)。
   实测(10-11g): 60ms×2.2A 脉冲把转子推到 ~1300°/s, 而 2.5A 刹车需要 34°
   才能停 ⇒ 从 105.47 一口气飞过目标到 44.48(反而差 +45°), 再被推回来 ⇒ 极限环。
   现在只要速度一上来(60eRPM=90°/s)就立即交回速度环, 由它把速度压在规划曲线上。 */
#define POS_KICK_EXIT_ERPM     60.0f
/* ★ 2026-10-11j "脱困刹车": 本机的硬约束是"速度只能按扇区知道" ——
   堵转之后的第一条边沿，只能告诉你"刚刚跨了 15.5°"，而在那之前
   无论如何都拿不到速度(窗口估计里混着堵转那段)。所以第一个 15.5° 是
   无法控制的。既然齿槽间距就是 15.5°，最巧的策略就是:
     **从齿槽脱困→让它走一个齿槽→立刻全刹车** ⇒ 刚好停在下一个齿槽。
   而"卡在目标旁一个齿槽"正是最常见的工况 ⇒ 一次就到位。 */
#define POS_ESCAPE_BRAKE_TICKS (100U * 20U)
#define POS_KICK_STALL_TICKS   (400U * 20U)    /* 判"堵转"的静止时长 */
#define POS_KICK_COOLDOWN_TICKS (800U * 20U)   /* 两次脉冲最小间隔 */

/* 位置环积分项(A) 与 阻尼用滤波速度(eRPM) */
static float s_pos_integ = 0.0f;
static float s_pos_erpm_filt = 0.0f;
/* 位置差分测速用(2026-10-11g): 10ms 窗口, 无滞后 */
static float    s_pos_vdiff_deg  = 0.0f;
static uint32_t s_pos_vdiff_tick = 0U;
/* 脱困刹车状态(2026-10-11j) */
static uint8_t  s_pos_esc_stalled = 0U;
static uint32_t s_pos_brake_end   = 0U;
/* 误差符号抖动抑制(2026-10-11k): 上次误差符号 + 最后一次翻号时刻 */
static int8_t   s_pos_err_sign    = 0;
static uint32_t s_pos_err_flip_tick = 0U;

/* 速度规划器状态(2026-10-09j): 当前命令速度(eRPM) 与命令加速度(eRPM/s) */
static float s_pos_v_cmd = 0.0f;
static float s_pos_a_cmd = 0.0f;

/* 破黏脉冲状态 */
static uint32_t s_pos_kick_end_tick  = 0U;   /* > foc->tick 表示脉冲进行中 */
static uint32_t s_pos_kick_next_tick = 0U;   /* 下一次允许触发的时刻 */
static uint32_t s_pos_kick_edge_tick = 0U;   /* 脉冲开始时的霍尔边沿计数(见下) */
static uint16_t s_pos_kick_cnt       = 0U;   /* 累计脉冲数(日志用) */
/* 位置积分"开闸"计时: 规划器持续要求运动 POS_INTEG_ARM_TICKS 之后才非 0 */
static uint32_t s_pos_arm_tick       = 0U;

/* 位置环堵转报警标志: 一旦报警就保持, 直到误差回到容差内才复位 */
static uint8_t s_pos_blocked_logged = 0U;
/* "本轮超差"起点: 换目标/刚起步时 err 必然大, 但"刚起步"不等于"堵转"。
   原来只看"距上次霍尔边沿", 而停在到位点时本来就没有边沿 ⇒ 一按目标键立刻
   误报(实测: `Set tar[POS] BLOCKED? err=45.5deg, no hall edge for 11897ms`,
   同时把破黏脉冲也误触发了一次)。改为计时"err 连续超差"的时长。 */
static uint32_t s_pos_err_run_tick = 0U;
/* 位置环"飞车"计时: 滤波速度持续超过门限的拍数 */
static uint32_t s_pos_fast_cnt = 0U;

static void FOC_DoPositionLoop(FOC_Control_t *foc, float dt_s)
{
    float erpm_now;
    float iq_target;
    float pos_now_deg;
    int8_t saved_torque;
    uint32_t err_run = 0U;   /* "err 已连续超差"的拍数(见下面的堵转看门狗) */

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
    /* 只保留它维护 pos_err_deg / arrived(供显示与到位判断); 速度指令不再使用 */
    (void)PositionLoop_Update(&foc->position_loop,
                              pos_now_deg,
                              foc->pos_target_deg_cmd,
                              dt_s,
                              foc->tick);

    /* 阻尼用的速度反馈用 **快滤波**(τ=12ms). 原来与速度环共用 s_erpm_filt
       (τ=0.1s), 在 ~3Hz 的位置振荡上有 ~100° 相位滞后 ⇒ 阻尼项变成"助推"。
       ---- 2026-10-11g 再加一路无滞后测速 ----
       `erpm_now` 只有霍尔边沿测速: 边沿之间是保持值, 窗口估计还跨 3 个扇区
       (~45°机械)取平均 ⇒ 加速/减速时严重滞后。实测(10-11f 日志): 转子已在
       扇区内飞过 23.1°(≈674°/s), 而环路只看到 84eRPM(差 8 倍) ⇒ 环路以为
       "还没到规划速度", 继续正推(1.86A) ⇒ 一直推到冲过目标 15~80° 才看到
       高速去反拉 ⇒ ±2.5A 来回摆几转停不下来。
       位置现在是绝对值算法(连续、无歧义) ⇒ 直接用**位置差分**(10ms 窗口)
       算速度: 无滞后。两路取**幅值较大者**: 宁可高估(多刹一点, 安全),
       绝不低估(低估 = 跑飞/冲过头)。 */
    {
        uint32_t dtk = foc->tick - s_pos_vdiff_tick;
        if (dtk >= 200U) {                                   /* 10ms 窗口 */
            float v_pos = (pos_now_deg - s_pos_vdiff_deg)
                          / (float)dtk * 20000.0f / 1.5f;    /* deg/s → eRPM */
            s_pos_vdiff_deg  = pos_now_deg;
            s_pos_vdiff_tick = foc->tick;
            if (fabsf(v_pos) > fabsf(erpm_now)) erpm_now = v_pos;
        }
    }
    s_pos_erpm_filt += (dt_s / POS_DAMP_TAU_S) * (erpm_now - s_pos_erpm_filt);

    /* ---- 跑飞保护 (2026-10-01 ★新增, 安全兵) ----
       位置环必须有"跑飞"兜底: 一旦角度/方向出错(实测: ALIGN 未标定 offset),
       闭环会变成正反馈, 电机一路加速 —— 日志里 Pos 到过 -39616°(110 圈)、
       vsp 撞死 ±3000 限幅、相电流 5.4A、机构被抽得飞快。两道门:
       ① 位置偏离目标超过 720°(2 圈) —— 任何正常运动都不可能;
       ② 滤波速度持续 200ms(4000 拍) 超过 1500eRPM —— 飞车。
       命中就 FAULT + 关 PWM, 按 'r' 清故障重来。 */
    if (fabsf(foc->position_loop.pos_err_deg) > 720.0f) {
        LOGW("[POS] RUNAWAY: err=%.0fdeg > 720 -> FAULT\r\n",
             foc->position_loop.pos_err_deg);
        FOC_Control_SetFault(foc, 0x0010);
        s_pos_fast_cnt = 0U;
        return;
    }
    if (fabsf(s_pos_erpm_filt) > 1500.0f) {
        s_pos_fast_cnt++;
        if (s_pos_fast_cnt > 4000U) {
            LOGW("[POS] RUNAWAY: |vsp|=%.0feRPM -> FAULT\r\n", s_pos_erpm_filt);
            FOC_Control_SetFault(foc, 0x0011);
            s_pos_fast_cnt = 0U;
            return;
        }
    } else {
        s_pos_fast_cnt = 0U;
    }

    /* ---- 堵转看门狗(2026-10-01, 2026-10-09 修误报) ----
       位置误差还大、但霍尔超过 0.8s 没跳边沿 = 转子根本没动(机械卡死/到限位/
       角度估计失效). 只报警不停机, 便于区分"算法不动"和"机构动不了". 报一次.
       ⚠ 原来只看"距上次霍尔边沿 > 0.8s", 而**停在到位点时本来就没有边沿** ⇒
       一按目标键就误报(实测: `[POS] BLOCKED? err=45.5deg, no hall edge for
       11897ms` —— 转子在 44.48° 停着 hold, 并不是堵转). 现在改为要求"err 本轮
       连续超差 > 0.8s", 换目标/刚起步时计时从 0 重新起算。 */
    {
        float err_now = foc->position_loop.pos_err_deg;
        uint32_t since_edge = foc->tick - foc->hall.edge_tick;

        if (fabsf(err_now) > foc->position_loop.arrive_tol_deg) {
            if (s_pos_err_run_tick == 0U) s_pos_err_run_tick = foc->tick;
        } else {
            s_pos_err_run_tick = 0U;
        }
        err_run = (s_pos_err_run_tick == 0U) ? 0U
                                             : (foc->tick - s_pos_err_run_tick);

        if (err_run > 16000U && since_edge > 16000U) {
            if (!s_pos_blocked_logged) {
                s_pos_blocked_logged = 1;
                LOGW("[POS] BLOCKED? err=%.1fdeg, no hall edge for %lums (Iqref=%.2fA)\n",
                     err_now, (unsigned long)(since_edge / 20U),
                     foc->current_loop.iq_ref);
            }
        } else {
            s_pos_blocked_logged = 0U;
        }
    }

    /* ---- 位置环律 v4 (2026-10-01): 到位完全撤力矩; 运动中 P + 限幅阻尼 + 门控积分 ----
       v1(纯 PD kp=0.025) 出不了力; v2(+积分 kp=0.06) 到位后在目标周围 ±1 个霍尔
       扇区、~3Hz 振荡; v3(低增益+强阻尼+速度门控积分) 能把目标走到了
       (`Pos=90.00 err=-0.00 arr=1`、`Pos=44.48 arr=1` 都出现过), 但**到位后
       会嗡嗡振**: 日志显示 err=0.00 而 `vsp=±250eRPM`、`Iqref` 在 ±1.7A 来回。
       原因: err≈0 ⇒ 电流全部来自阻尼项 -kd*v, 而 v 在霍尔边界抖动时是尖刺,
       乘 kd=0.006 就变成 ±1.5A 真电流 —— 把机构掘起(交叉堆掘/爬行),
       偏偏位置反馈对扇区内的掘动又"看不见"(adv 被限幅) ⇒ 自激嗡嗡。
       v4: ① 到位(|err|<容差) 时 **完全撤力矩**(靠静摩擦保持, 不发热, 也不会
       自激); ② 运动中阻尼项限幅 ±0.7A, 不让测速尖刺直接变成大电流。
       v4 的关键还有**不要让总电流饱和**: 饱和后 iq=clamp(p+d+∫) 里阻尼项被
       限幅吃掉 ⇒ 阻尼完全失去权限, 控制律退化成继电器 ⇒ 必然极限环
       (日志: Tgt=0 时 Iqref 恒 ±1.80A 来回、vsp ±700、Pos 在 ±16° 之间荡)。
       所以把 总限幅压到 1.6A、积分限幅压到 0.7A:
         卡住时 p+∫ ≤ 0.66+0.7 = 1.36A > 起动阈值 ~1.15A ⇒ 起得来;
         动起来时 p+d+∫ ≈ 0.66-0.7+0.7 = 0.66A < 阈值 ⇒ 自然减速停住。 */
    {
        float err = foc->position_loop.pos_err_deg;
        float v   = s_pos_erpm_filt;

        /* ---- 脱困刹车计时(2026-10-11j) ----
           堵转(边沿间隔 > 400ms)期间置标志; 一出现新边沿且近目标 ⇒ 启动一段
           强制全刹车(见 POS_ESCAPE_BRAKE_TICKS 注释)。 */
        {
            uint32_t se = foc->tick - foc->hall.edge_tick;
            if (se > POS_KICK_STALL_TICKS) {
                s_pos_esc_stalled = 1U;
            } else if (se < 5U && s_pos_esc_stalled) {
                s_pos_esc_stalled = 0U;
                if (fabsf(err) < POS_GENTLE_ZONE_DEG) {
                    s_pos_brake_end = foc->tick + POS_ESCAPE_BRAKE_TICKS;
                }
            }
            /* ★ 10-11k: 刹车只在该刹的时候给 —— 速度掉下来就立刻松手。
               原来无论速度多少都硬顶 100ms×2.5A ⇒ 转子早已停住还在被反推
               近 100ms(等于"持续反推"), 既振动又可能把它倒推出齿槽。 */
            if (s_pos_brake_end != 0U) {
                uint32_t active = foc->tick - (s_pos_brake_end - POS_ESCAPE_BRAKE_TICKS);
                if (foc->tick >= s_pos_brake_end ||
                    (active > POS_ESC_BRAKE_MIN_TICKS &&
                     fabsf(v) < POS_ESC_BRAKE_V_ERPM)) {
                    s_pos_brake_end = 0U;
                    s_pos_integ     = 0.0f;
                }
            }
        }

        /* ★ 10-11k 误差符号翻号检测(抖动抑制, 见 POS_HOLD_BAND_DEG 注释) */
        {
            int8_t sgn = (err > 0.0f) ? 1 : ((err < 0.0f) ? -1 : 0);
            if (sgn != 0 && sgn != s_pos_err_sign) {
                s_pos_err_sign      = sgn;
                s_pos_err_flip_tick = foc->tick;
            }
        }

        if (fabsf(err) < foc->position_loop.arrive_tol_deg &&
            fabsf(v) < POS_ARRIVE_V_ERPM) {
            /* 到位(且在容差内已经不快了): 撤力矩 + 清零积分 */
            iq_target = 0.0f;
            s_pos_integ = 0.0f;   /* ★10-11k: 到位就是"放手", 积分直接清零(原来只泄放) */
            /* 到位即清脉冲与规划器: 下次移动可从零开始 */
            s_pos_kick_end_tick  = 0U;
            s_pos_kick_next_tick = 0U;
            s_pos_arm_tick       = 0U;
            s_pos_v_cmd = 0.0f;
            s_pos_a_cmd = 0.0f;
        } else {
            /* ⚠ 2026-10-11b: 进门多了"|v|<POS_ARRIVE_V_ERPM"这一条 —— 原来只要
               |err|<6° 就拓力矩, 而进容差时往往还有上百 eRPM 速度 ⇒ 靠惯性滑出去
               一个齿槽(15°)又变超差 ⇒ 在容差带边上反复。现在 "还在飞" 就不撤力矩,
               继续由下面的速度环把它杀回到接近 0 再撒手。 */
            /* ======== 位置 → 速度规划: jerk 受限 S 曲线 (2026-10-09j) ========
               用户要求"启动慢加速 → 匀速 → 接近目标减速"并按段划分。这里用
               加加速度受限的 S 曲线速度规划器, 7 个自然段(加速、减速各自再细分
               就是常说的 9 段):
                 ①加速起头(jerk+) ②恒加速 ③加速收尾(jerk-) ④匀速
                 ⑤减速起头(jerk-) ⑥恒减速 ⑦减速收尾(jerk+)
               ★ 与上一版的关键区别: 上一版只"限速不跟踪" ⇒ 驱动(P+积分 1.5~2.4A)
                 要靠 0.015×超速 去平衡 ⇒ 实际速度会稳在"允许速度 + 驱动/0.015"
                 (比允许速度高一倍!) ⇒ 过冲、目标附近大幅左右摆(实测 Tgt=180
                 在 120°~200° 之间荡)。现在速度是**被命令并被跟踪**的:
                      iq = Kv×(v_cmd − v) + 位置积分
                 速度被真正压在规划曲线上, 接近目标时 v_cmd→0 ⇒ 平滑停住。
               ★ 也不再要位置 P 项: P 与速度环互相顶正是"高出允许速度一倍"的
                 成因; 位置静差交给积分(它本来就是位置积分), 而积分只在"卡住
                 不动"时增长(破齿槽用), 不参与正常运动 ⇒ 不会来回摆。 */
            float iq_drive;
            {
                float dist = fabsf(err);

                /* 巡航限速 与 进近减速曲线 v=√(2ad) */
                float v_lim = (dist < POS_DECEL_ZONE_DEG)
                                  ? POS_DECEL_K_ERPM * sqrtf(dist)
                                  : POS_VCRUISE_ERPM;
                if (v_lim > POS_VCRUISE_ERPM) v_lim = POS_VCRUISE_ERPM;
                float v_tgt = (err > 0.0f) ? v_lim : -v_lim;

                /* 积分"开闸"计时(见 POS_INTEG_ARM_TICKS 注释): 只要规划器还在
                   要求运动就算"想动", 从第一次要求开始计时 */
                if (fabsf(v_tgt) > 1.0f) {
                    if (s_pos_arm_tick == 0U) s_pos_arm_tick = foc->tick;
                } else {
                    s_pos_arm_tick = 0U;
                }

                /* ★★ 2026-10-11e 关键修正: 误差方向一变(转子越过目标), 立刻把命令
                   速度 v_cmd 清 0。这就是"摆不停"的元凶。
                   实测数据(10-11d 日志, Tgt=90): 转子刚冲过目标(err 由正变
                   -15.5°), 环路却还在出 **+1.28A 正推**, 而当时 vsp 只有 66eRPM
                   ⇒ 反推 0.015×(v_cmd−v)=1.28 得 **v_cmd≈+150** —— 即规划器还
                   在从 +200 按 DEC 斜坡往下走, 要 100~150ms 才变负。这 100~150ms
                   里 2.5A 继续往前推 ⇒ 多冲 **+500~750eRPM**(实测峰值 547/729eRPM,
                   而规划只允许 ≤190eRPM) ⇒ 冲过去 60~90° ⇒ 反向再来一次 ⇒ 在
                   0~105° 之间来回摆几秒, 每次都靠耗尽动能才停。
                   原理上: 越过目标时"正确速度"就是 0(或反向的很小值), 没有任何
                   理由保留上一个方向的命令速度 ⇒ 直接清 0, 下一拍从 0 重新向
                   新方向规划。方向突变 = 一步刹车(jerk 大但只是把电流从正拽到
                   负, 不会造成机械冲击/撞限位)。 */
                if ((err > 0.0f && s_pos_v_cmd < 0.0f) ||
                    (err < 0.0f && s_pos_v_cmd > 0.0f)) {
                    s_pos_v_cmd = 0.0f;
                    s_pos_a_cmd = 0.0f;
                }

                /* ①②③⑤⑥⑦: v_cmd 向 v_tgt 平滑逼近(限加速度 + 限 jerk) */
                {
                    float dv     = v_tgt - s_pos_v_cmd;
                    float a_need = dv / dt_s;
                    float a_max  = (dv > 0.0f) ? POS_ACC_ERPM_S : POS_DEC_ERPM_S;

                    if (a_need >  a_max) a_need =  a_max;
                    if (a_need < -a_max) a_need = -a_max;

                    float da     = a_need - s_pos_a_cmd;
                    float da_max = POS_JERK_ERPM_S2 * dt_s;
                    if (da >  da_max) da =  da_max;
                    if (da < -da_max) da = -da_max;

                    s_pos_a_cmd += da;
                    s_pos_v_cmd += s_pos_a_cmd * dt_s;
                    /* 不许冲过目标速度(否则会在目标速度上下反复) */
                    if ((dv > 0.0f && s_pos_v_cmd > v_tgt) ||
                        (dv < 0.0f && s_pos_v_cmd < v_tgt)) {
                        s_pos_v_cmd = v_tgt;
                        s_pos_a_cmd = 0.0f;
                    }
                }

                /* 速度环: 命令速度 vs 实测速度。 */
                iq_drive = POS_KV_A_PER_ERPM * (s_pos_v_cmd - v);
                {
                    float iq_lim = POS_IQ_RUN_A;
                    if ((v > 0.0f && iq_drive < 0.0f) ||
                        (v < 0.0f && iq_drive > 0.0f)) {
                        iq_lim = POS_DAMP_MAX_A;
                    }
                    if (iq_drive >  iq_lim) iq_drive =  iq_lim;
                    if (iq_drive < -iq_lim) iq_drive = -iq_lim;
                }
                /* ★★ 2026-10-11j 温柔脱困: 近目标 + 基本没动 ⇒ 撤掉速度环驱动,
                   只靠积分慢慢顶(见 POS_GENTLE_ZONE_DEG 注释) */
                if (fabsf(err) < POS_GENTLE_ZONE_DEG &&
                    fabsf(v) < POS_INTEG_GATE_ERPM) {
                    iq_drive = 0.0f;
                }
            }

            /* 位置积分: 只在"确实动不了"时才增长。三个条件必须同时成立:
               ① 规划器已持续要求运动 300ms(开闸, 防"换目标就靠积分冲");
               ② 滤波速度接近 0;  ③ 150ms 没有霍尔边沿(真没动)。
               正常运动时①②③至少一个不成立 ⇒ 积分冻结/按 0.1s 泄放。 */
            if (s_pos_arm_tick != 0U &&
                (foc->tick - s_pos_arm_tick) > POS_INTEG_ARM_TICKS &&
                fabsf(v) < POS_INTEG_GATE_ERPM &&
                (foc->tick - foc->hall.edge_tick) > POS_INTEG_STALL_TICKS) {
                float pre = iq_drive + s_pos_integ;
                if (!((pre >  POS_IQ_CAP_A && err > 0.0f) ||
                      (pre < -POS_IQ_CAP_A && err < 0.0f))) {
                    s_pos_integ += POS_KI_A_PER_DEG_S * err * dt_s;
                    if (s_pos_integ >  POS_INTEG_MAX_A) s_pos_integ =  POS_INTEG_MAX_A;
                    if (s_pos_integ < -POS_INTEG_MAX_A) s_pos_integ = -POS_INTEG_MAX_A;
                }
            } else if (fabsf(v) >= POS_INTEG_GATE_ERPM) {
                /* ★★ 2026-10-11i: 转子一旦真动起来, 积分的任务(破齿槽)就完成了,
                   必须**立即清零**。原来只按 τ=0.1s 衰减 ⇒ 脱困后那 1.2A 会:
                   ① 和驱动相加, 使总电流回到 2.5A 继续猛推(实测脱困后 vsp 冲到
                      ±500eRPM, 行程 40~50°);
                   ② 刹车时把反向 2.5A 吃掉一半(-2.5+1.2 = -1.3A) ⇒ 刹车距离从
                      12° 变成 24° ⇒ 飞出去停不下来。
                   清零只是一次"电流下降", 不会造成冲击；下次卡住它会重新长回来。 */
                s_pos_integ = 0.0f;
            } else {
                s_pos_integ *= (1.0f - dt_s / 0.10f);
            }

            iq_target = iq_drive + s_pos_integ;

            /* ---- 堵转破黏脉冲 ----
               ⚠ 原版只把"上限"抬到 2.5A —— 实测那是**空操作**: 卡住时环路自己
               只出 -1.32A(或更糟的 -0.01A), 本来就低于 1.8A 的原上限, 抬上限
               根本不多给力; 日志里 STICTION kick 打了但 Iqref 一点没变。
               现在: 脉冲期间**强制**按误差方向给定 POS_KICK_A(绕过速度环+积分)。
               ★★ 2026-10-11b: 脉冲 200ms→60ms, 并且**一见霍尔边沿立刻收手**
               (see s_pos_kick_edge_tick)。原因(2026-10-11 实测): 200ms/2.5A 对
               这个很轻的转子 = ~5000°/s², 200ms 里跑 100° —— 从 195.47° 卡点
               直接飛回 90°, 而 90° 又是另一个齿槽 ⇒ 环路在 90° 又把转子推到 195°
               ⇒ **90↔207 永动极限环**(日志里连打 7 次破黏)。现在脉冲只负责
               "把齿槽掰开", 一有边沿就松手, 由速度环接管减速/定位。 */
            {
                uint32_t since_edge = foc->tick - foc->hall.edge_tick;
                uint8_t  kicking = 0U;

                if (s_pos_kick_end_tick != 0U && foc->tick < s_pos_kick_end_tick &&
                    foc->hall.edge_tick == s_pos_kick_edge_tick &&
                    fabsf(s_pos_erpm_filt) < POS_KICK_EXIT_ERPM) {
                    kicking = 1U;                      /* 脉冲进行中、还没动、速度还小 */
                } else if (err_run > POS_KICK_STALL_TICKS &&
                           since_edge > POS_KICK_STALL_TICKS &&
                           fabsf(err) > POS_HOLD_BAND_DEG &&
                           (s_pos_kick_next_tick == 0U || foc->tick >= s_pos_kick_next_tick)) {
                    s_pos_kick_end_tick  = foc->tick + POS_KICK_TICKS;
                    s_pos_kick_next_tick = s_pos_kick_end_tick + POS_KICK_COOLDOWN_TICKS;
                    s_pos_kick_edge_tick = foc->hall.edge_tick;
                    s_pos_kick_cnt++;
                    kicking = 1U;
                    /* ★10-11k: 脉冲是"干净的一下" —— 位置积分清零(不叠加),
                       并预置电流环积分让脉冲 1ms 内就达到目标电流 */
                    s_pos_integ = 0.0f;
                    CurrentLoop_SetQInteg(&foc->current_loop,
                                          ((err > 0.0f) ? POS_KICK_A : -POS_KICK_A)
                                          * (float)foc->torque_dir * POS_FF_DUTY_PER_A);
                    LOGW("[POS] STICTION kick #%u: %.1fA for %ums (err=%.1fdeg, stalled %lums)\r\n",
                         (unsigned)s_pos_kick_cnt, POS_KICK_A,
                         (unsigned)(POS_KICK_TICKS / 20U),
                         err, (unsigned long)(since_edge / 20U));
                }

                if (foc->tick < s_pos_brake_end && foc->hall.dir != 0) {
                    /* 刚脱困: 强制全刹车(方向与刚走的方向相反), 停在下一个齿槽 */
                    iq_target = -(float)foc->hall.dir * POS_DAMP_MAX_A;
                } else if (kicking) {
                    iq_target = (err > 0.0f) ? POS_KICK_A : -POS_KICK_A;
                } else {
                    /* ★ 10-11k 抖动抑制: 近目标且误差符号刚翻过 ⇒ 撤掉一切电流
                       (位置读数分辨率只有半个扇区, 此时驱动就是自激振动的源) */
                    if (fabsf(err) < POS_HOLD_BAND_DEG &&
                        s_pos_err_flip_tick != 0U &&
                        (foc->tick - s_pos_err_flip_tick) < POS_ERR_HYST_TICKS) {
                        iq_target   = 0.0f;
                        s_pos_integ = 0.0f;
                    }
                    /* ★★ 2026-10-11i 总输出按状态分档限幅:
                       ① |v|<40(基本没动) ⇒ 允许 2.5A —— 这是掰开齿槽的**唯一**手段;
                       ② 在动 & 电流与运动**同向**(驱动) ⇒ 只给 POS_IQ_RUN_A(1.6A),
                          不让它"脱困后一路加速冲过头";
                       ③ 在动 & 电流反向(刹车) ⇒ 允许 2.5A, 保证刹得住。 */
                    float iq_lim = POS_IQ_CAP_A;
                    if (fabsf(v) >= POS_INTEG_GATE_ERPM &&
                        ((v > 0.0f && iq_target > 0.0f) ||
                         (v < 0.0f && iq_target < 0.0f))) {
                        iq_lim = POS_IQ_RUN_A;
                    }
                    if (iq_target >  iq_lim) iq_target =  iq_lim;
                    if (iq_target < -iq_lim) iq_target = -iq_lim;
                }
            }
        }
    }

    /* ????? */
    /* ★10-11k 兜底: 任何非有限/离谱的值都不许进电流环(实测日志出现过
       Iqref=-637.50 的一瞬间 ⇒ 电流环立刻饱和、相电流冲到 9.5A 过流跳闸) */
    if (!(iq_target >= -POS_IQ_CAP_A && iq_target <= POS_IQ_CAP_A)) {
        iq_target = 0.0f;
    }
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
        float saved  = foc->current_loop.iq_ref;
        float iq_cmd = saved * (float)foc->torque_dir;
        /* ★★ 2026-10-11k 兜底: 任何非有限/离谱的参考值都不许进电流环。
           实测(10-11j 日志)出现过 Iqref=-637.50 的一瞬间 —— 电流环立刻饱和、
           相电流冲到 9.5A、过流跳闸(du 从 0.52 直接跳到 0.797)。位置环自己
           的限幅是 ±2.5A, 所以超过 8A 就说明数值/内存异常, 直接按 0 处理,
           不让它变成真的 9.5A。 */
        if (!(iq_cmd >= -8.0f && iq_cmd <= 8.0f)) iq_cmd = 0.0f;
        foc->current_loop.iq_ref = iq_cmd;
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
    foc->pos_target_user_set = 1U;   /* 即使还在 ALIGN/STARTUP 也要记住 */
    /* 换目标 = 换一次"起步": 清掉上次顶静摩擦积下来的积分, 避免新方向
       一开始就被旧积分顶着(表现为换了目标还先往老方向动一下) */
    s_pos_integ = 0.0f;
    /* 破黏脉冲也一并复位: 换目标后 400ms 就能打第一枪, 不用等冷却 */
    s_pos_kick_end_tick  = 0U;
    s_pos_kick_next_tick = 0U;
    s_pos_kick_cnt       = 0U;
    s_pos_err_run_tick   = 0U;
    s_pos_blocked_logged = 0U;
    s_pos_arm_tick       = 0U;
    s_pos_kick_edge_tick = 0U;
    s_pos_v_cmd          = 0.0f;   /* 规划器复位: 新目标从 0 开始加速 */
    s_pos_a_cmd          = 0.0f;
    s_pos_vdiff_tick     = 0U;     /* 位置差分测速重新起算 */
    s_pos_esc_stalled    = 0U;
    s_pos_brake_end      = 0U;
    s_pos_err_sign       = 0;
    s_pos_err_flip_tick  = 0U;
    if (foc->state == FOC_STATE_POSITION_LOOP) {
        PositionLoop_ResetArrived(&foc->position_loop);
    }
}

/* 位置环积分项(A) — 只给日志看, 判断"是不是靠积分顶着静摩擦" */
float FOC_Control_GetPosIntegA(void)
{
    return s_pos_integ;
}

/* 位置环阻尼用的滤波速度(eRPM) — 只给日志看, 判断振荡/蠕动幅度 */
float FOC_Control_GetPosErpm(void)
{
    return s_pos_erpm_filt;
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

