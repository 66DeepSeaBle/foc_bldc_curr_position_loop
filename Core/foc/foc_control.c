/**
  * @file    foc_control.c
  * @brief   FOC 控制核心: 状态机 + 电流/速度/位置环 + 定时轨迹
  */
#include "foc_control.h"
#include "foc_app.h"
#include "transform.h"
#include "pid.h"
#include "sensor.h"      /* 位置传感器抽象: ALIGN 零点标定走 g_sensor(换编码器不用改这里) */
#include "log.h"
#include "board_config.h"   /* 仅为拿 CMSIS 的 DWT(CYCCNT) 做分段计时(2026-10-24) */
#include <math.h>

/* ---- 各状态的执行体(每拍一个) ---- */
static void FOC_DoIdle(FOC_Control_t *foc);
static void FOC_DoAlign(FOC_Control_t *foc, float dt_s);
static void FOC_DoStartup(FOC_Control_t *foc, float dt_s);
static void FOC_DoCurrentLoop(FOC_Control_t *foc, float dt_s);
static void FOC_DoSpeedLoop(FOC_Control_t *foc, float dt_s);
static void FOC_DoPositionLoop(FOC_Control_t *foc, float dt_s);
static void TM_ExpInit(void);          /* 摩擦前馈 exp 查表初始化(定义在文件后半) */
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
static const uint32_t OC_BLANK_TICKS = (100U * FOC_TICKS_PER_MS);   /* = 100ms */

/* ==========================================================================
 * ★ 2026-10-24 到位静音: 用来判定“静止啸叫”是不是 PWM 载波引起的
 * --------------------------------------------------------------------------
 * 现象(15kHz 日志铁证): 到位后 `arr=1 / Iqref=0.00`, 但 `du=0.500`
 *   ⇒ 三相半桥仍在硬开关 ⇒ 绕组对地 ±12V 方波(相间电压≈零, 没有真电流)
 *   ⇒ 磁致伸缩啸叫, **与电流大小无关**; 20k/15k 只是换了音调。
 * 做法: 位置环已到位 + 轨迹没在跑 + |iq_ref| 很小 + |err| < 1.5°,
 *      连续 ARM_TICKS(150ms) 满足 ⇒ PWM_Output_Disable() 三相全关。
 * 恢复: |iq_ref| 变大 / 位置漂出 1.5° / 状态离开位置环 / 轨迹启动
 *      ⇒ 立刻重新使能(下一次 PWM_Output_Apply 自己会打开 CCER)。
 * 运行时开关: 大写 'A' 切换(FOC_Control_SetPwmIdleOff)。
 * ========================================================================== */
#define PWM_IDLE_OFF_DEG        4.0f    /* 进入门限: 必须 > DONE_DEG(3.0)——静止读数常停在 2~3° */
#define PWM_IDLE_OFF_EXIT_DEG   6.0f    /* 退出门限(滞环): 漂出它才重新使能 */
#define PWM_IDLE_OFF_CUR_A      0.06f   /* |iq_ref| 超它就重新使能 */
#define PWM_IDLE_OFF_ARM_TICKS  (150U * FOC_TICKS_PER_MS)   /* 零电流持续 150ms 才关断 */
static uint8_t  s_pwm_idle_en   = 1U;   /* 1 = 允许到位关 PWM(**默认开启**, 按 'A' 关) */
static uint8_t  s_pwm_idle_off  = 0U;   /* 1 = 当前已关断 */
static uint32_t s_pwm_idle_tick = 0U;

/* ★ 2026-10-24: 关断/使能事件改为"只置标志", 由主循环打印
   实测: 原来在中断里 LOGD(走 vsnprintf + RTT) 会把那一拍的 CUR 从
   8.3us 拉到 17us —— 这是 [PROF] 里 max 虚高的主因之一。 */
static uint8_t  s_pwm_evt      = 0U;    /* 0=无 1=idle-off 2=idle-on */
static int32_t  s_pwm_evt_pos  = 0;
static int32_t  s_pwm_evt_err  = 0;

/* ★ 2026-10-24 位置律耗时统计(DWT 打点, 只给 [PROF] 诊断用, 不参与控制)
   s_law_t0 在分频门里置位, 在外层律结束时相减 ⇒ 得到“外层位置律独占”的周期数。
   目的: 搞清 43 µs 的最坏一拍到底是 轨迹/爬行/电流环 哪一块占的, 再决定搬哪块。 */
static uint32_t s_law_t0      = 0U;
static uint32_t s_law_cyc_sum = 0U;
static uint32_t s_law_cyc_max = 0U;
static uint32_t s_law_cyc_cnt = 0U;

/* ★ 2026-10-24 拆分计时的另两段
     PRE: FOC_DoPositionLoop 入口 → 分频门(每拍都跑)
     CUR: FOC_UpdateCurrent 全程(每拍都跑) */
static uint32_t s_pre_t0      = 0U;
static uint32_t s_pre_cyc_sum = 0U;
static uint32_t s_pre_cyc_max = 0U;
static uint32_t s_pre_cyc_cnt = 0U;
static uint32_t s_cur_cyc_sum = 0U;
static uint32_t s_cur_cyc_max = 0U;
static uint32_t s_cur_cyc_cnt = 0U;

/* ANG: FOC_UpdateAngle(角度观测器)单独计时 —— 用来把 PRE 再拆一刀 */
static uint32_t s_ang_cyc_sum = 0U;
static uint32_t s_ang_cyc_max = 0U;
static uint32_t s_ang_cyc_cnt = 0U;

static float s_cur_dt_s = 0.0f;   /* 电流环的 dt_s = 1/pwm_freq(常量, 缓存) */

static void FOC_ProfAcc(uint32_t cyc, uint32_t *sum, uint32_t *pmax, uint32_t *cnt)
{
    *sum += cyc;
    (*cnt)++;
    if (cyc > *pmax) { *pmax = cyc; }
}

void FOC_Control_SetPwmIdleOff(uint8_t en)
{
    s_pwm_idle_en = en ? 1U : 0U;
    s_pwm_idle_tick = 0U;
}

uint8_t FOC_Control_GetPwmIdleOff(void)
{
    return s_pwm_idle_en;
}

uint8_t FOC_Control_GetPwmIdleOffActive(void)
{
    return s_pwm_idle_off;
}

/* 读并清零拆分计时(主循环调) */
void FOC_Control_TakeSplitProf(FOC_SplitProf_t *out)
{
    uint32_t pre_s, pre_m, pre_c;
    uint32_t ang_s, ang_m, ang_c;
    uint32_t law_s, law_m, law_c;
    uint32_t cur_s, cur_m, cur_c;

    if (out == 0) { return; }

    __disable_irq();
    pre_s = s_pre_cyc_sum;  s_pre_cyc_sum = 0U;
    pre_m = s_pre_cyc_max;  s_pre_cyc_max = 0U;
    pre_c = s_pre_cyc_cnt;  s_pre_cyc_cnt = 0U;
    ang_s = s_ang_cyc_sum;  s_ang_cyc_sum = 0U;
    ang_m = s_ang_cyc_max;  s_ang_cyc_max = 0U;
    ang_c = s_ang_cyc_cnt;  s_ang_cyc_cnt = 0U;
    law_s = s_law_cyc_sum;  s_law_cyc_sum = 0U;
    law_m = s_law_cyc_max;  s_law_cyc_max = 0U;
    law_c = s_law_cyc_cnt;  s_law_cyc_cnt = 0U;
    cur_s = s_cur_cyc_sum;  s_cur_cyc_sum = 0U;
    cur_m = s_cur_cyc_max;  s_cur_cyc_max = 0U;
    cur_c = s_cur_cyc_cnt;  s_cur_cyc_cnt = 0U;
    __enable_irq();

    out->pre_avg = (pre_c != 0U) ? (pre_s / pre_c) : 0U;
    out->pre_max = pre_m;
    out->pre_n   = pre_c;
    out->ang_avg = (ang_c != 0U) ? (ang_s / ang_c) : 0U;
    out->ang_max = ang_m;
    out->ang_n   = ang_c;
    out->law_avg = (law_c != 0U) ? (law_s / law_c) : 0U;
    out->law_max = law_m;
    out->law_n   = law_c;
    out->cur_avg = (cur_c != 0U) ? (cur_s / cur_c) : 0U;
    out->cur_max = cur_m;
    out->cur_n   = cur_c;
}

/* ★ 2026-10-24: 取走待打印的 PWM 关断/使能事件(主循环调, 读即清) */
uint8_t FOC_Control_TakePwmEvent(int32_t *pos_mdeg, int32_t *err_mdeg)
{
    uint8_t ev;

    __disable_irq();
    ev = s_pwm_evt;
    s_pwm_evt = 0U;
    if (pos_mdeg != 0) { *pos_mdeg = s_pwm_evt_pos; }
    if (err_mdeg != 0) { *err_mdeg = s_pwm_evt_err; }
    __enable_irq();
    return ev;
}

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

    /* ★ 初始化正弦查表(Park/逆Park/TM_Profile4 都用它)。
       必须在任何控制中断启动之前调 —— 万一忘了, FOC_SinCos 里有兜底自动初始化,
       但那第一拍会多花 ~0.5ms。 */
    FOC_Math_Init();
    TM_ExpInit();          /* ★ 建议③: 摩擦前馈的 exp 查表 */

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
        LOGD("[ALIGN] Starting alignment, Id=%dmA\n", (int)(align_current * 1000.0f));
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

    if (elapsed > align_ms * FOC_TICKS_PER_MS) {
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
            g_sensor.CalibrateAtZero(foc, foc->hall.state);
            LOGD("[ALIGN] Align done, hall=%d -> offset=%dmdeg\r\n",
                 (int)foc->hall.state,
                 (int)(foc->angle.offset * 57295.77951f));
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
        LOGD("[STARTUP] I-F startup, target=%dmHz\n", (int)(target_hz * 1000.0f));
    }

    elapsed = foc->tick - foc->startup_tick;

    /* ??????? */
    if (elapsed < startup_ms * FOC_TICKS_PER_MS) {
        omega_target = 2.0f * 3.14159265f * target_hz / foc->config.pwm_freq *
                       (float)elapsed / (float)(startup_ms * FOC_TICKS_PER_MS);
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
        elapsed > (300U * FOC_TICKS_PER_MS) && fabsf(erpm) > 8.0f) {
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
                                     foc->config.pwm_freq, FOC_TICKS_PER_S, 1.0f, 1, NULL);
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
        LOGD("[STARTUP] Synced erpm=%d hold@%dmdeg\n",
             (int)erpm, (int)(pos * 1000.0f));
        return;
    }

    if (elapsed > max_ms * FOC_TICKS_PER_MS) {
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
#define POS_VCRUISE_ERPM    100.0f  /* ★10-11m 200→100: 实测本机"齿槽能擒住的极限速度"
                                       就在 ~130eRPM(200°/s)附近 —— 超过它就会冲过
                                       一整个齿槽(15.5°), 于是正反推变成继电器极限环。 */
#define POS_ACC_ERPM_S       1200.0f /* 加速段加速度上限(eRPM/s) */
#define POS_DEC_ERPM_S       2000.0f /* 减速段减速度上限(eRPM/s) */
#define POS_JERK_ERPM_S2    12000.0f /* 加加速度上限(eRPM/s²): 加速度用 100ms 升满 */
#define POS_DECEL_ZONE_DEG  100.0f   /* =(V_CRUISE/K)²: 超过它才被巡航限住 */
#define POS_DECEL_K_ERPM     14.0f   /* v_lim=K×√d: 100°→140(封顶到 VCRUISE=100) */
#define POS_KV_A_PER_ERPM    0.008f  /* ★★10-11n 0.025→0.008: 速度环增益(A/eRPM)。
                                        被控对象是“1A → ~10000°/s²”这种极高增益 +
                                        测速固有 ~30ms 滞后 ⇒ 回路带宽必须 ≲8Hz 才稳。
                                        0.025 对应 ~16Hz(相位滞后 >100°) ⇒ 必摆；
                                        0.008 ≈ 8Hz, 而且它把“驱动=滑动摩擦 1.18A”
                                        对应的稳态速度差放在 ~148eRPM ⇒ 实际爬行
                                        速度只有 50~100eRPM(75~150°/s) —— 正好落在
                                        “齿槽能擒住(≤130eRPM)”的区间里。 */
/* 驱动电流限幅(2026-10-11g): 与"刹车"分开。
   ⚠ 本机构非常轻 + 阻滞大: 要动必须 >1.15A(过静摩擦), 而 1.15A 在它身上
   就是 ~10000°/s² —— 实测 1.86A 下 34ms 就飞到 674°/s。若驱动不单独限幅,
   起步就是 2.5A 满推 ⇒ 两三个扇区就冲过目标 15~80°。
   这里把**驱动**(电流方向与运动同向)限在 1.6A(刚好过静摩擦); **刹车**
   (电流与运动反向)仍放开到 POS_DAMP_MAX_A(2.5A), 保证刹得住。
   卡在齿槽里需要更大扭矩时, 由"堵转才增长的位置积分"(最多 0.8A)补上,
   总上限仍是 POS_IQ_CAP_A(2.5A) —— 也就是"平时温柔、卡住时凶"。 */
#define POS_IQ_RUN_A         1.5f    /* 驱动(加速)方向电流上限 ★10-11m 1.6→1.5 */
#define POS_ARRIVE_V_ERPM    40.0f   /* 到位还需 |v|<它: 还在飞就继续用环刹车 */
/* ★ 2026-10-11j "温柔脱困区": |err| 小于它时, 一旦"基本没动"就不再给
   速度环驱动电流, 只让堵转积分慢慢长到破黏阈值(≈1.2~1.6A)。
   理由(本机物理): 1.15A 是静摩擦阈值, 而 1.86A 就能把转子推到 674°/s —— 
   脱困扭矩离阈值越近, 加速度越小(≈1000~2000°/s² 而不是 8000+), 走完一个
   齿槽(15.5°)就慢得多, 环路才有机会在"下一个齿槽"把它刹住。远目标(|err|
   大)时照旧全速驱动——那里走得快才重要。 */
/* ★ 2026-10-11s 20°→13°: 一个齿槽的静差必须能修。
   实测(10-11r 日志, 三个目标全部中招):
     Tgt=0  → Pos=15.47 err=-15.47 **卡 5 秒不动**, Ipos=-0.80A/Iq=-1.3A 恒顶(发热), arr=0
     Tgt=90 → 卡在 err=12~18° 九秒不收敛, arr=0
     Tgt=45 → 在 29.21/44.48 之间 ~1Hz 一跳一跳
   机理: |err|=15.5° 落在 20° 的温柔区内 ⇒ 前馈被撤掉, 只剩 ≤0.8A 积分
   (< 破黏 1.15A) ⇒ 推不动; 而破黏脉冲门槛抬到 20° 后又打不到它 ⇒ 永远差一个齿槽。
   ⚠ 2026-10-11t 把这一条**收回 20°**了: 10-11s 把温柔区缩到 13° 后, |err|∈[13,20]
   时前馈(1.25A)不再受"转子已停"的约束 ⇒ 在振荡中也被上, 电流直接冲到 ±2.5A.
   现在"一个齿槽的静差"改由**独立、有条件**的"掰一格"负责(见 POS_HOP_MIN_ERR_DEG)。 */
#define POS_GENTLE_ZONE_DEG  20.0f
/* ★★ 2026-10-11n 滑动摩擦前馈: 本机滑动摩擦≈1.18A, 用它把“起步/爬行”的
   基本力补上(详见 FOC_DoPositionLoop 里的长注释)。误差 ≥ POS_FF_RAMP_DEG
   时给满前馈, 小于它线性衰减到 0(免得目标附近来回翻号)。 */
#define POS_FF_FRIC_A        1.25f
/* ★ 2026-10-11r 5°→15°: 前馈的斜坡就是近目标的**等效位置增益**(1.25A/斜坡)。
   5° 时是 0.25A/° —— 而位置读数在扇区边界会以 ±7.7°(半个扇区)成台阶跳,
   一个台阶就是 ±1.9A 的继电器式换向(>破黏 1.15A) ⇒ 必然把转子推过边界,
   然后再推回来 = 自持极限环(实测按 1 后 ±12° 摆 4 秒不停)。
   15° ⇒ 0.083A/°, 一个 7.7° 台阶只有 0.64A < 1.15A ⇒ 转子在齿槽里待得住,
   读数不再跳, 死区 100ms 一满就锁住 ⇒ 静止。真差一个齿槽(15.5°)时
   前馈 1.25A 仍 > 破黏 ⇒ 该修的时候还是修得动。 */
#define POS_FF_RAMP_DEG     15.0f
/* ★★ 2026-10-11p 到位死区(施密特触发器) —— 专治"到位后高频小幅振动":
   实测(10-11o 日志, Tgt=0): 到位后 `Pos` 在 ±5° 以 ~1.2Hz 来回, 而
   `ob/vp/vsp` 却在 ±100~500eRPM 之间刷(尖刺), 环路于是持续给 ±1~2A
   ⇒ 转子被反复掘动 = 用户说的"微小但频率很高的振动"。
   物理原因: 半个霍尔扇区(7.7°)以内的位置本来就测量不准, 而速度只能由
   这个不准的位置微分成 —— 全是尖刺。拿尖刺×Kv 出电流 → 自激。
   对策: 位置误差自己做一个带回差的死区(施密特):
     · `|err| < 4°` 且基本停住 → 进入死区: **完全撤力**(靠齿槽自己保持),
       之后**不再理会任何速度估计**(它们不可信);
     · 直到 `|err| > 12°`(差不多一个齿槽) 才重新介入。
   代价: 目标周围 12° 内不再修 —— 但目标就是齿槽, 读数的分辨率也只有
   半个扇区(7.7°), 在那以内"修"本来就修不准, 只会振。 */
/* ★★ 2026-10-11ah 6.0→7.5: 实测(10-11af 日志, Tgt=360)读数在 360.00↔365.90
   之间跳, err 稳定在 **-5.77 ~ -6.06**, 而进死区要 |err|<6.0 连续 100ms ⇒
   每次都差 0.06° 进不去, 环路于是拿 1.30A 一直顶着(2 秒), 一抖动就把转子
   掘出去。7.5 仍在"半格读数虚偏(5.9)"之上、真实一格静差(11.9)之下。 */
#define POS_HOLD_IN_DEG      7.5f    /* 进入死区: |err| 在这个以内 */
/* ★★ 2026-10-11aa 15.5°→9° —— 这里是本轮找到的**致命死锁**:
   观测器在无边沿时按"最后速度×τ"继续外推, 而外推最多只能走完**一个扇区**
   (15.47°机械)。实测(10-11z 日志) `Pos=195.47 Tgt=180.00 err=-15.47
   Iqref=-0.00 adv=+30.95 dte=2000→7600` —— 读数正好停在扇区末端,
   而 15.47 **小于** 15.5 ⇒ 这个死区永远退不出去 ⇒ 环路永久撤力,
   一个齿槽的静差再也修不回来(日志里 arr=0、一条 CREEP 都没有)。
   退出阈值必须**低于一个扇区**: 取 9°(且 > POS_ACCEPT_BAND_DEG 8°,
   不会在半个扇区内来回抖)。同时观测器的外推时间常数已从 35ms 缩到
   10ms(见 angle_observer.c), 从源头把虚位移压到 0.7°机械以内。 */
/* ★★ 2026-10-11ah 9→13: 死区一旦进入(hl=1)就**不该**被半个扇区的读数
   抖动叫醒 —— 恰恰是它把 Tgt=0 那一份日志救成了完全安静的(hl=1/Iqref=0.00/
   dte 一直涨)。13 < 一格齿槽(15.47) ⇒ 真的丢了一格仍然会醒过来修。 */
#define POS_HOLD_OUT_DEG     13.0f/* 进入死区前 "|err| 连续在容差内" 的时长. 用**时间**而不是瞬时速度判断
   "停下来了没有" —— 因为本机速度反馈在目标附近全是尖刺, 用不了。
   飞掠(300~600°/s)穿过 ±6° 只停 ~40ms, 而真停住会持续几百ms ✓ 区分得开。 */
#define POS_HOLD_SETTLE_TICKS (100U * FOC_TICKS_PER_MS)

/* ★★★ cal25 位置律分频（省 CPU）
 * --------------------------------------------------------------------------
 * 实测(cal23 [PROF]): 20kHz 每拍 24.7us(保持) / 35us(运动), 其中:
 *     st=0(只跑外壳+DoIdle)  = 4.15us
 *     st=5(角度+位置律+电流环) = 24.66us
 * ⇒ **位置律那一大坨(几十个 if/else、观测器、爬行、冻结判定)占 20us**,
 *   而它的物理带宽只有 3Hz —— 根本没道理跑 20kHz。
 *
 * 现在把它降为 20kHz / POS_LAW_DIV = 10kHz（可改成 5kHz 用 4）。
 * **下面这些仍然保持 20kHz**（它们依赖 20kHz 的时间分辨率，不能降）:
 *   · FOC_UpdateAngle()      → 电流环的 Park 角度，必须每拍更新
 *   · 霍尔边沿计数 s_pos_win10_edges / s_pos_dir_run
 *   · 10ms 位置差分测速窗
 *   · s_pos_erpm_filt 低通（τ=12ms，降频会变成 24ms）
 *   · 100ms 扇区计数窗 / 跑飞保护 / 堆转看门狗
 * 跳过的拍只跑 FOC_UpdateCurrent()，iq_ref 沿用上一拍（电流环仍 20kHz）。
 *
 * 所有 tick 计时器（POS_*_TICKS）不受影响 —— foc->tick 还是在
 * FOC_Control_Run 里每拍 +1。需要一键回到原行为: 改成 1U。
 * -------------------------------------------------------------------------- */
#ifndef POS_LAW_DIV
#define POS_LAW_DIV 2U
#endif

/* ★ cal26: 判定"已经滑走一个齿槽"的门限(机械°)。
   到点撤力后转子可能溜回上一个齿槽(实测 16.7°), 这时必须立刻重新介入,
   不能死等 settle(松手静默)和积分开闸超时 —— 那要 0.77s 才爬得回来。
   取 13°(= POS_HOLD_OUT_DEG) ⇒ 覆盖一个齿距(15.47°)的 84%, 正常在目标
   附近的小幅摆动不会触发。 */
#define POS_REENGAGE_DEG     13.0f
/* ★★ 2026-10-11ah 近目标"松手静默"(settle) ------------------------------------
   实测(10-11af 日志, Tgt=180)转子在 **161.0 ↔ 187.3** 之间以 ~5Hz、±13° 永续
   摆动, `Iqref` 在 ±1.30A 之间翻, 12 秒不停; 同一份日志里 Tgt=0 却完全安静
   (`hl=1 arr=1 Iqref=0.00`, dte 一路涨)。差别**只在**有没有进到到位死区。
   机理(三段互相喂能量):
     ① 1.30A 的前馈在 15.5° 内就能把转子加速到 **400~700°/s**(实测 137→187°
        只用 102ms), 而齿槽能擒住的上限只有 ~195°/s ⇒ 必然冲过目标一格;
     ② 冲过之后误差翻号, 环路想刹车 —— 但**滤波速度(τ=12ms)在换向瞬间还带着
        旧方向的数值**(实测 `vsp=+205eRPM` 而 `vp=-462eRPM`, 转子明明在倒着
        走), 于是"刹车"变成"顺着运动方向推", 把转子又撅回来 ⇒ 误差再翻号;
     ③ 这个区间读数分辨率只有 15.47°, 误差本身就在 0↔16.66 之间跳, 环路
        **没有**可靠判据说"已经到位了"。
   对策(本机唯一稳的路子): 近目标**跨过一次霍尔边界就整体松手一段时间**,
   让齿槽自己把转子接住 —— 不推、也不刹(这 250ms 里的速度估计没有意义)。
   触发条件刻意选成"上一条边沿间隔 >100ms": 那说明这是一次**从静止起步的
   离散小步**, 而不是 30ms 一条边沿的高速飞掠 ⇒ 高速进近时照旧由刹车接管。 */
#define POS_SETTLE_TICKS      (150U * FOC_TICKS_PER_MS)   /* 松手静默时长
                                             ★ cal26: 250→150ms。到点回弹后这 250ms
                                             里一直是撤力状态, 是 0.77s 恢复时间的一半。 */
#define POS_SETTLE_SLOW_DT    (100U * FOC_TICKS_PER_MS)   /* 上一条边沿间隔 > 它 ⇒ 视为"离散小步" */
#define POS_SETTLE_ARM_DEG    10.0f          /* 只在 |err|< 它 时触发(近目标) */
/* ★★ 2026-10-11ai 近目标“只允许不足以起步的电流”
   本机实测破黏 1.15~1.6A、滑动摩擦 ≈1.18A。把近目标(|err|<35°)的电流卡在
   **0.85A** 以下 ⇒ 环路**自己不可能把转子从齿槽里拽出来**, 也就不可能
   因为“读数跳变 + 速度符号错”而把机构摇起来; 真要从静止起步只能靠爬行脉冲
   (POS_SOFT_IQ_A=1.30 起步, 限时 80ms, 跨边沿即撤)。 */
#define POS_SOFT_HOLD_A       0.85f
/* ★★ 2026-10-11aj 近目标“真在飞”的**快**判据 + 最后一段刹车
   实测(10-11ai 日志, 按 `8`=360°): 进近时真实速度已到 **600~750°/s**(规划的
   巡航只有 150°/s), 而速度估计在中等速度下低估 3~4 倍 ⇒ 环路还在“驱动”
   ⇒ 以 ~600°/s 冲过 360° 齿槽(它只能捣住 ≤195°/s)⇒ 跑到下一个齿槽
   (375.47) 再被推回来 ⇒ 用户看到“抖动几次然后停止”。
   而“快到慢”的判据不能用速度估计(它是坏的), 也不能只用 dt(边界抖动时
   dt 也很小)。用**连续同向边沿数**:
     · 边界抖动: hall.dir 每一条边沿就翻一次 ⇒ 连续同向数永远是 1;
     · 真在运动: 每条边沿都往同一方向走 ⇒ 600°/s 时 3 条边沿只要 75ms。
   满足“连续同向 ≥3 且距最后边沿 <100ms”就允许刹车(幅值按速度估计、方向用
   hall.dir —— 离散量, 不会被 ±1600eRPM 的假速度带错), 而且只在最后
   POS_SOFT_BRAKE_DEG(~12°) 内生效: 这里刚好是“冲过一格”与“停早了”的分界。 */
#define POS_DIR_RUN_MIN       3U              /* 连续同向边沿数阈值(≈3 个扇区=47°) */
#define POS_DIR_RUN_TICKS     (100U * FOC_TICKS_PER_MS)    /* 且最后边沿在 100ms 内(⇒ >≈155°/s) */
#define POS_SOFT_BRAKE_DEG    12.0f           /* 只有最后 ~12° 内允许满力刹车 */
/* ★★ 2026-10-11ai 位置读数跳变 ⇒ 立即撤力
   `s_pos_jump_cnt` 是已有的“物理上不可能的位移”计数器(10ms 窗口里位移超过
   窗口内边沿数所允许的最大值)。它一跳就说明**位置通道此刻不可信** ——
   此时拿它算出来的误差/速度去给电流, 只会把机构掘起来(实测 20~40Hz 抖动,
   `jp` 2.4s 涨到 137)。所以: 检测到跳变就彻底撤力一段时间, 让机构自己停下
   来(停下后霍尔就不再抖、读数自然恢复), 再重新评估。
   `..._MAX_TICKS` 是安全帽: 连续跳变最多只让环路安静这么久, 之后必须
   重新介入(防止两个齿槽边界永久互刷导致环路永久罢工)。 */
#define POS_FREEZE_TICKS      (150U * FOC_TICKS_PER_MS)
#define POS_FREEZE_MAX_TICKS  (400U * FOC_TICKS_PER_MS)
/* ★★ 2026-10-11q 测速"物理可信度限速": 本机最大加速度实测 ~25000°/s²
   (= 167eRPM/10ms = 0.83eRPM/tick), 所以**每一拍**测速不可能变超过几个 eRPM。
   但位置读数偶尔会整扇区误跳(扇区序号判错/边界抖动), 位置差分就会在 10ms
   窗口里报出 ±1000eRPM 的假尖刺(实测: 转子往后走, `vp` 却报 +1018)。
   这种尖刺一进控制律, 环路就从"刹车"瞬间翻成"满推" ⇒ 转子被推到另一边
   ⇒ 再过目标又翻回来 ⇒ **±20° 持续摆动**(用户按 `1` 后看到的就是它)。
   限速取 3eRPM/tick(允许到 ~90000°/s², 真动力学限幅 0.83 的 3.6 倍),
   真加速完全不受影响, 只挡住物理上不可能的跳变。 */
/* ★★ 2026-10-11r "速度可信度"总修正(替代 10-11q 的每拍限速)
   ------------------------------------------------------------
   10-11q 的"每拍限速 3eRPM"实测**无效**: 10ms 位置差窗口有 200 拍,
   200×3 = 600eRPM ⇒ 一个 ±1000eRPM 的假尖刺照样能爬进滤波器; 更糟的是它
   把**观察器那一路**(边沿测速, 它在 ±150~280eRPM 之间合法地刷)也一起钳住了,
   于是 `vsp` 只剩被污染的差分值, 恒在 -40~-100eRPM(实测 10-11p 日志)。
   而 `vsp`≈-50 乘 Kv=0.008 就是 **0.4A 持续真电流**, 还会让"|v|<40"的
   温柔脱困区/到位撤力两个判据**永远不成立** ⇒ 环路在目标附近永远推来推去
   ⇒ 转子被推过扇区边界 ⇒ 读数跳半个扇区 ⇒ 误差翻号 ⇒ 反向推 = 用户看到的
   "按 1 到目标后来回摆"。
   现在改成"闸门 + 可信度"两条:
     · 位置差闸门: 窗口内跨了几条霍尔边沿, 就限定位置读数最多能变几个扇区;
       超出就是"读数跳变"(扇区序号判错/整圈滑), 这一路直接丢掉不融合。
     · 动/静判定: 50ms 位置差窗口连续 3 个(150ms)都真的挪过 POS_MOVE_DEG,
       才算"真的在动"(s_pos_move); 否则环路把速度当 0 用。
       转子停在扇区边界时读数会以 ±半个扇区来回跳, 单窗口挡不住,
       连续 3 个窗口挡得住 —— 真运动必然满足, 抖动不满足。 */
#define POS_SECTOR_DEG       15.467f  /* 1 个霍尔扇区 = 15.47°机械 */
#define POS_WIN_N0_MAX_DEG   8.0f     /* 10ms 窗口内 0 条边沿时的最大可信位移 */
/* ★★ 2026-10-11t 动/静判定改用"扇区序号净变化"(替代 10-11r 的 50ms 位置差+连续3窗)
   --------------------------------------------------------------------
   10-11r 用"50ms 位置差 + 连续 3 个窗口都动过"才能算"在动", 10-11s 实测它在
   **减速/换向**时必然出现"净位移≈0"的窗口 ⇒ 判成"停了" ⇒ 环路在**高速飞行中撤力**:
     t=57723 Pos=89.66 err=0.34 vsp=-2 mv=0 Iqref=0.00
     t=57758 Pos=68.34 err=21.66 vsp=-164 v50=-208   ← 35ms 后已经在 -400eRPM
   即: 转子刚从齿槽里脱出、正以 300~400eRPM 向目标滑去时, 环路以为它停了、
   直接松手(不刹车) ⇒ 冲过一个多齿槽 ⇒ 再满推 ⇒ **2Hz、±23° 的持续摆动**。
   改用"100ms 内 net 跨了几个扇区"作动/静判据: 它是**整数、单调**, 单点噪声
   改变不了它; 而 100ms 内净跨 <1 个扇区 ⇔ 平均速度 <103eRPM(<155°/s)
   ⇔ 靠摩擦只要 0.45° 就滑完 ⇒ 当"停了"处理绝对安全。
   ⭐ 关键认识: 转子真在动时, 偏大的速度估计只会"多刹一点"(安全方向);
     只有"停着却报大速度"才危险 —— 用净扇区数可以干净地把这两件事分开。 */
#define POS_SECT_WIN_TICKS   (200U * FOC_TICKS_PER_MS)   /* 200ms 扇区观察窗 */
/* ★★ 2026-10-11w 两个档的扇区数阀值(见 POS_SOFT_ZONE_DEG 注释):
   200ms 净跨 1 个扇区 ⇔ 平均 ≥77°/s=52eRPM(在动); 2 个 ⇔ ≥155°/s=103eRPM(真的在飞)。 */
#define POS_MOVE_SECT_MIN    1
#define POS_FLY_SECT_MIN     2
/* ★★ 2026-10-11w 近目标"温柔档" —— 物理依据是**齿槽能擒住的速度上限**
   ------------------------------------------------------------------
   实测(10-11v 日志, Tq=180): 预置电流生效后(实测 Iq 真到了 -2.0~-3.3A),
   转子在目标附近变成 **±15°~25°、5~10Hz 的持续摆动**:
     t=27731 Pos=195.47 err=-15.47 Iqref=-2.50 vsp=556 ob=702   ← 满刹 2.5A
     t=28135 Pos=195.47 err=-15.47 Iqref=-2.50 vsp=486
     t=28337 Pos=188.35 err=-8.35  Iqref=-2.50 vsp=284
   而这两行的真相是: **读数从 186.87 跳到 195.47(一个齿槽), 模型 adv 从 -3.62 走到 +30.95
   (完全无真实位移)**, 但由此算出的假速度 vsp=486eRPM 让环路以为"转子在飞"
   ⇒ 刹掉 2.5A ⇒ 真的把转子扇出去 ⇒ 误差翻号 ⇒ 反向再砸 2.5A = 自持大摆。
   物理约束: **2.5A = 25000°/s² ⇒ 35ms 就能把转子撺过一个齿槽(15.5°)** ——
   而位置分辨率只有 15.5° ⇒ 近目标的环路增益比可用值高 ~50 倍。
   而实测"齿槽能自己擒住的速度"就是 **~130eRPM(195°/s)**, 对应刚好过破黏的
   1.30A(净力矩 1.30-1.18=0.12A ⇒ ~1200°/s² ⇒ 走一个齿槽 ~160ms、末速 195°/s)。
   ⇒ 所以: **|err| < 35° 且"没在飞"时, 电流一律限到 1.30A 且不用速度项。**
     35° 的依据: 规划器曲线 v=14√d 在 d=35° 只允许 83eRPM(124°/s) < 155°/s
     ⇒ "进 35° 时本来就不是在飞" ⇒ 温柔档接手很自然。
     (再大就交给正常律: 那里误差/速度都可信, 且确实需要 2.5A 才能刹住。) */
#define POS_SOFT_ZONE_DEG    35.0f
#define POS_SOFT_IQ_A        1.30f   /* 爬行起始电流 */
/* ★★ 2026-10-11x 近目标爬行(带**回差**)—— 取代 10-11s 的"掰一格"
   ----------------------------------------------------------
   实测(10-11w 日志, Tgt=180 卡在 163.35° **整 7.4 秒**):
     读数 158.4↔168.2 以 **5Hz±5°** 来回振(h 在 2↔6 之间跳、adv 从 +27 到 -33),
     而 `Iqref` 也在 0.00/1.30 之间**切成 50% 占空比的脉冲**(因为驱动门限 20° 被读数振过了)
     ⇒ 平均力矩 0.65A **< 破黏 1.15A** ⇒ 转子永远脱不开; 而那串 1.3A 脉冲
     正是用户听到的"高频振动"。
   根因: 用**瞬时** |err| 开/关近目标驱动 = 把 1.3A 切成 50% 方波。
   对策:
     (a) **回差(施密特)**: |err| 超过 POS_SOFT_ON_DEG 才开(10-11aa 改 12°,
         因为齿槽间距实测 13.81~16.66° 不均匀, 门槛 15° 会漏掉 13.8° 的
         真实一格静差), 要掉到 POS_SOFT_OFF_DEG(8°) 以下才关 ⇒ 读数在
         边界 ±5° 振不影响;
     (b) **方向锁定**: 进入时记下方向, 之后不再逐拍看 err 符号(避免读数翻号导致方向抖);
     (c) **电流自适应上爬**: 从 1.30A 起, 每 200ms "没进展"就 +0.10A, 上限 1.80A ——
         实测本机各齿槽的破黏门槛在 **1.15~1.6A 之间变化**(固定 1.3A 有的齿槽推不动,
         固定 1.6A 又会撅过下一个齿槽)。转子一动就退出/复位。
   退出: |err|<8°(到位) / 读数符号反向(撅过一个齿槽) / 真的在飞(交给满力刹车)。
   连爬 POS_HOP_MAX 次仍进不了 ±6° ⇒ 认输: 撤力、报 GIVE UP、保持静差(按数字键重试)。 */
/* ★★ 2026-10-11ad 12°→9.5°: 实测(10-11ac 两份日志)按 '1'(45°) 后转子停在
   `Pos=33.10 err=11.90 Iqref=0.00` **9 秒不动** —— 而爬行的门槛正好是 12.0°,
   差 0.1° 不触发 ⇒ 白白留下 12° 静差(BLOCKED? 报警也是这个)。
   为什么会有 11.9° 的静差: 修好外推后读数是**偏保守**的(停在"刚跨过的那个
   边界"附近), 所以转子其实停在了**扇区出口那一侧的齿槽**(=目标本身)时,
   读数就会少掉约一个扇区 ⇒ err ≈ -12~-15°。这种"差一格"必须能修,
   而且方向是对的(往前推几度就跨过边界, 读数立刻回到目标附近)。
   门槛取 9.5°: 高于"无声接受带"(POS_ACCEPT_BAND_DEG=8°), 低于实测最小
   的一格静差(11.9°)。 */
#define POS_SOFT_ON_DEG       9.5f
#define POS_SOFT_OFF_DEG      6.0f
#define POS_SOFT_IQ_MAX_A     1.60f
#define POS_SOFT_STEP_A       0.10f
#define POS_SOFT_RAMP_TICKS   (200U * FOC_TICKS_PER_MS)  /* 每 200ms 没进展 +0.10A(按时间上爬, 保留兼容) */
/* ★★ 2026-10-11y 爬行的退出改成"**按位移**"
   ----------------------------------------------------------
   实测(10-11x 日志): 靠 `|err|<8` / `flying` / 变号 退出太慢 ——
     转子离开齿槽后读数要等它走完 15.5° 才从 195.47 变成 180,
     而 `flying` 判据(200ms 净跨≥2 扇区)也要 200ms 才成立
     ⇒ **多推了一整格**(实测 195.47 → 163.79 = 2 个齿槽)。
   改为: **从开始推的那一刻算, 读数在推动方向上相位移动超过 8° 就松手**
     —— 一个齿槽是 15.5°、而模型噪声只有 ±5°(实测), 8° 卡在中间很安全。
   这样松开时转子离目标还有 7.5°, 靠惯性+摩擦再走 0.8~4.2°(取决于电流大小)
     ⇒ 落在目标齿槽里, 由齿槽自己擒住。
   好处: 位移与电流大小**解耦** ⇒ 电流可以放心自适应上爬(1.3→1.8A),
     不管多大都只走一格。 */
#define POS_SOFT_TRAVEL_DEG   8.0f
/* ★★ 2026-10-11ah 600ms→80ms, 并且**与电流成反比**(见爬行块里的 pulse_max):
   实测 1.30A 下 15.5° 只要 ~100ms, 所以"限时 600ms"实际上等于没有限制 ——
   真正在起作用的只有"位移 8°"那一条, 而位移是按**读数**算的(边界附近虚偏
   ±5°)。电流上爬到 2.0A 时, 80ms 的行程会从 4° 涨到 30°(≈两格) ⇒ 就是
   用户看到的"推一下、冲两格、再推"。改成"等冲量"(电流越大脉宽越短)。 */
#define POS_SOFT_PUSH_TICKS   (80U * FOC_TICKS_PER_MS)    /* 单次推动最长 80ms(等冲量: 电流越大越短) */
/* ★★ 2026-10-11y 防抖: 进入爬行前必须有最短间隔, 两次"计一次尝试"也必须隔得够久
   实测(10-11x 日志): 快速摆动中读数反复跨过 15° 门槛又变号
     ⇒ **400ms 里就计了 4 次尝试** ⇒ 误判 GIVE UP ⇒ 之后 5 秒环路完全罢工,
     永久留下 15.5° 静差(`Tgt=180` 停在 195.47 / `Tgt=90` 停在 105.47 /
     `Tgt=45` 停在 29.21 全是这个原因)。
   现在: ① 上一次推动结束后至少等 300ms 才允许再推;
         ② 两次**计数**至少隔 600ms ⇒ 最快也要 2.4s 才认输(真正做到 4 次尝试)。 */
#define POS_SOFT_RETRY_TICKS   (200U * FOC_TICKS_PER_MS)
#define POS_HOP_TRY_MIN_TICKS  (600U * FOC_TICKS_PER_MS)
/* ★★ 2026-10-11aa 认输后"再试一轮"的间隔(见爬行块里的注释)。
   认输**不再是永久的** —— 实测认输后环路永久静默 3.1 秒以上, 只在日志里
   留下一个 15.47° 的静差, 而用户看到的却是"到位置后不再动"。 */
#define POS_SOFT_GIVE_RETRY_TICKS (1500U * FOC_TICKS_PER_MS)
/* ★★ 2026-10-11ab "停稳"才允许推: 距上一条霍尔边沿超过它(≈KICK 的堵转阀值)。
   实测(10-11aa 日志)转子在目标附近以 10~50ms 一条边沿的节律抖动时, 爬行
   每 300~900ms 还是推一次 ⇒ 每一次破黏都把转子抡出 20~40°, 再被推回来 ——
   ”推一下、冲出去、再推“ = 用户听到的"抖动"。只有真停住(无边沿)才推。 */
/* ★★ 2026-10-11ah 400→200ms: 上面那条"停稳才推"的保护现在由**松手静默**
   (POS_SETTLE_TICKS)接了一部分班(每次跨边界后强行撤力 250ms), 所以这里的
   等待可以缩短, 否则一次 16.66° 的爬行要等 3~4 轮 ×400ms 才走完。 */
#define POS_SOFT_QUIET_TICKS   (200U * FOC_TICKS_PER_MS)
/* ★★ 2026-10-11ad "转子确实在动"的判据 —— 距上一条霍尔边沿 < 它。
   为什么必须再加这一条(实测 10-11ac 两份日志): 位置环的 `v` 原来只有
   `s_pos_flying`(需连续 2 个 200ms 窗口净跨≥2 扇区) 和 `s_pos_move` 两条路,
   而 `SetPosTarget` 会把 `s_pos_sect_tick` 归零从新起算 ⇒ **每次换目标后的
   前 200~400ms 里两条都不成立** ⇒ `v=0` ⇒
     ① 速度环失去反馈 ⇒ `iq = 1.25 + 0.008×(v_cmd−0)` 被算成一个"大驱动"
        ⇒ 又因分档限幅判不出方向 ⇒ 给了 2.5A 猛推 ⇒ 实测 167° 的移动里
        转子冲到 **1748°/s(1165eRPM)**, 而规划只有 100eRPM(150°/s) —— 快 11 倍;
     ② 到目标前误差穿过 ±6° 时 `|v|<40` 成立 ⇒ 直接进"到位撤力" ⇒
        转子带着 ~1000eRPM 撞进齿槽 ⇒ 齿槽擒不住 ⇒ 甩过去再弹回来
        = 用户看到的"大角度到位后短暂抖动"(小角度距离短、速度低所以不明显)。
   ⇒ 用"最近有边沿"作第三条判据: 最直接(边沿=转子真的动过)、无滞后。 */
#define POS_EDGE_MOVE_TICKS   (80U * FOC_TICKS_PER_MS)
/* ★★ 2026-10-11ae 定时轨迹(命令 `t<角度> d<秒>` + 回车)
   --------------------------------------------------
   需求: 给定目标角度与"到达目标需要的时间"(秒), 把**时间**分成 9 份:
     第 1 份 = 起步加速, 第 9 份 = 减速, 中间 7 份 = 匀速。
   实现: 梯形速度曲线, 但斜坡内部用**升余弦(S 形)**而不是直线 ——
     加速段 v(t) = v_c·(1−cos(πu))/2, u=t/ta ⇒ 首尾斜率都为 0 = 抖动受限。
     直线斜坡在两端有 ±v_c/ta 的**加速度阶跃**, 而本机 1A≈10000°/s² ⇒
     那个阶跃就等于在齿槽里敲一下(正是前面几轮反复出现的"到位抖动/嗡嗡")。
   由"距离 D、总时间 T"反解巡航速度: 三段行程 = v_c·(ta/2 + tc + td/2)
     = v_c·8T/9 ⇒ **v_c = 9D/(8T)**。
   两个物理上限(超了就自动延长, 并把实际用时打出来):
     ① 巡航 ≤ POS_TM_VMAX_MECH_DEG_S(150°/s = 100eRPM, 齿槽能擒住的上限);
     ② 峰值加速度 = (π/2)·v_c/ta = 15.9·D/T² ≤ POS_TM_ACC_MAX_DEG_S2
        (2000°/s² ≈ 净 0.2A ⇒ 总电流 ≈1.38A, 仍在 POS_IQ_RUN_A(1.5A) 以内)。
   ⇒ 最短用时 T_min = max( sqrt(15.9·D/A_max), 9D/(8·V_max) )。
   位置闭环: 轨迹给出"位置参考 pos_ref", 用 KP×(pos_ref−读数) 修正速度指令
     (前馈 + 比例修正 = 经典轨迹跟踪)。注意读数分辨率只有半个扇区(7.7°),
     所以落点仍有 ±7° 的不确定度, 剩下的由普通近目标逻辑(死区/爬行)收尾。 */
#define POS_TM_PART             9.0f     /* 时间分 9 份 */
/* ★★ 2026-10-11ak 上限提高到 500°/s / 8000°/s²:
   用户要求“默认目标(≤360°)都在 1s 内完成” ⇒ D=360°、T=1s 时需要的
   巡航 = 9D/(8T) = 405°/s、峰值加速度 = 15.9D/T² = 5724°/s²。
   原来的 150°/s / 2000°/s² 会把这个请求自动延长成 2.7s。
   注意 500°/s 仍然**远低于本机能力**(空载 1A≈10000°/s²), 而齿槽只能擒住
   ≤195°/s —— 齿槽这一条不是靠巡航限制保证的, 而是靠**轨迹末速归零**
   (梯形曲线的收尾速度是 0), 所以提高巡航不影响到位安静度。 */
#define POS_TM_VMAX_MECH_DEG_S  800.0f   /* 巡航速度上限(机械°/s) */
#define POS_TM_ACC_MAX_DEG_S2   8000.0f  /* 峰值加速度上限(机械°/s²) */
/* ★★ 2026-10-11an 4 段时间分配 + 慢速逼近段(用户要求: “起步慢慢加速、
   到目标附近减速慢慢接近”)
   ------------------------------------------------------------------
   原来只有 3 段(1/9 加速 + 7/9 匀速 + 1/9 减速), 且减速末端速度=0:
   D=360°/1s 时峰值加速度要到 5724°/s²、减速率 3645°/s², 而且是**到点才
   停** ⇒ 实测(10-11am 日志)轨迹跑完时读数还差 16~22°(齿槽附近读数本身
   就有一个扇区的粒度) ⇒ 手头的力立刻掉到 0, 由“松手静默 + 爬行脉冲”接手
   ⇒ 用户看到的“突然刹车、然后正反转微调”。
   现在改成 4 段, 且**减速只降到 vapp=40°/s**, 再匀速逼近最后一段:
        加速 2/9  →  匀速 3/9  →  减速 2/9(→vapp)  →  逼近 2/9(vapp)
   好处:
     ① 峰值加速度降到 ~1/3(同样的 T、D), 起步平缓;
     ② 最后 2/9 的时间都在 ~40°/s 慢走 ⇒ **到目标前就已经“慢慢接近”**,
        而不是到点才停; 40°/s 远低于齿槽捣住极限 195°/s ⇒ 到位很柔;
     ③ 若期间读数已经跨到目标扇区(err≈0), 心跳会提前结束轨迹交给死区,
        两边都不会抢。
   距离关系: D = vc·(ta/2 + tc + td/2) + vapp·(td/2 + tapp)
     ⇒ 反解 vc(见 SetTimedTarget)。 */
#define POS_TM_TA_FRAC          (2.0f / 9.0f)   /* 加速占比(时间) */
#define POS_TM_TC_FRAC          (3.0f / 9.0f)   /* 匀速占比 */
#define POS_TM_TD_FRAC          (2.0f / 9.0f)   /* 减速占比 */
/* ★★★ cal30: 逼近段速度 40.0 → **28.0** °/s
   依据(cal29 日志, 第 3 次 `8`): 到位前 113ms 里读数从 331.44 跳到 360.66
   ⇒ **~257°/s**, 而齿槽能擒住的上限只有 **~195°/s** ⇒ 冲过齿槽被“弹簧”拉回
   一整个齿格(360.66 → **343.34**, -17.3°), 然后环路拿 1.96A 顶了 ~500ms
   才推回来 —— 就是用户感到的“到位后振动一下”。
   28°/s 只在最后 2/9 时间里起作用(末段少走 2°), 巡航 vc 几乎不变
   (D=360/T=0.81: vc 从 776 → 783), 所以对运动负载率/耗时无影响。 */
#define POS_TM_VAPP_DEG_S       28.0f           /* 逼近段速度(机械°/s) */
/* ★★ 2026-10-11am 轨迹改为“**物理模型前馈 + 小权限位置微调**”，完全不用测速
   ----------------------------------------------------------------------
   实测(10-11al 日志): 把轨迹的速度环增益提到 0.03 之后, **过程大幅抖动** ——
   `D=45°/1s` 的移动里读数 0→15.5→29.2→15.5→37.7→10.5→6.7→32.5→38.4→46.3,
   ±20° 来回, `Iqref` 在 ±2.5A 之间翻(继电器)。
   根因: **本机在 50~400°/s 这个区间的测速全是毛刺** —— 霍尔边沿间隔
   30~150ms, 位置差分又只有 15.47° 的粒度, 实测 `vsp` 在 ±250eRPM 之间来回翻,
   而真实速度只有 34eRPM。任何“速度误差×增益”都只会变成**继电器输出**:
   0.03×250eRPM = 7.5A → 限幅 2.5A ⇒ 电流在 ±2.5A 方波 ⇒ 转子被摇起来。
   对策: 轨迹期间**不用测速**, 电流由**实测物理模型**直接给出:
       iq = 摩擦前馈(按轨迹速度方向, 1.25A)
          + a_轨迹 × 0.0001A/(°/s²)     (实测 1A ≈ 10000°/s²)
          + 位置微调(增益 0.05A/°, 限幅 ±0.8A < 破黏 1.15A)
   因为轨迹曲线是解析的、模型是实测的 ⇒ **电流天生平滑**, 没有反馈噪声;
   而“位置微调”权限故意卡在破黏以下 —— 一个扇区(15.5°)的读数毛刺产生的
   0.8A **根本推不动转子**, 就不可能再自激。
   机理: 这是“前馈伺服”—— 摩擦项抵消摩擦、加速度项出业绩加速度, 剩下的
   位置误差只需要一个**常量偏置**(verr ≈ ±(1.25−fric)/0.05 ≈ ±7°)就能平衡,
   而常量偏置意味着**速度与规划完全一致**(零速度误差), 恰好是我们要的。 */
/* ★★ 2026-10-11ao: 摩擦前馈改成"速度相关(Stribeck)" —— 这是"起步/减速
   不平滑、有振动"的直接根因。
   实测(10-11an 日志解码):
     · `t180 d2`(D=180/T=2s, 规划巡航 138°/s): 起步后 100ms **读数已经
       超前规划 21°**, 300ms 时超前 50°, 峰值实测 860°/s(规划的 6 倍)
       ⇒ 转子一路跑在规划前面; 而位置微调只被允许 ∓0.8A, 最小命令
       = 1.25−0.8 = **+0.45A(仍在往前推)** ⇒ **环路永远饱和** ⇒ 电流在
       0.45~2.05A 之间打摆 = 电流方波 = 用户听到的振动。
     · `8`(D=270/T=1s) 中段 verr 在 −21°…+51° 之间摆(全程饱和), 读数增量
       在 110°/s 与 444°/s 之间跳。
     · `z`(D=134/T=1s) 反而干净 —— 因为它需要的力矩小(命令 ~0.70A)。
   ⇒ 两个结论:
     ① **1.25A 是"静摩擦(破黏)"而不是"滑动摩擦"**。滑动时只需 ~0.70A
        (实测 z 段: 命令 0.70A 就能稳住 ~230°/s)。拿静摩擦的值去维持
        匀速 ⇒ 多出 ~0.5A = 5000°/s² ⇒ 必然一路加速、环路饱和;
     ② 减速段更糟: 前馈仍是 ±1.25A, 位置微调最多拉到 ∓0.8A ⇒ 命令最低
        **+0.45A(还在往前推)** ⇒ "到目标附近减速"根本做不到, 只能等时间
        到点后硬切 ⇒ 就是用户说的"突然刹车"。
   对策: 摩擦前馈按速度成型(Stribeck) —— 静止时给到破黏电流(保证能起步),
   速度一上来就降到滑动摩擦值(保证匀速时不再自加速):
        命令 = 摩擦(v_轨迹) + a_轨迹/10000 + 位置微调 + 速度阻尼
   · 起步: v=0 ⇒ 1.45A(刚好顶住静摩擦), 而 a_轨迹 从 0 平滑长起来
     (S 曲线 a(0)=0) ⇒ **净力矩从 0 平滑增加 ⇒ 起步是真的"慢慢加速"**;
   · 匀速: v 高 ⇒ 摩擦项降到 0.70~0.9A ⇒ 不再自加速, 环路不饱和;
   · 减速: v 降到 vapp(40°/s) ⇒ 摩擦项 ~1.0A, a_轨迹为负 ⇒ 命令自然降到
     0.6~0.7A ⇒ **真的在减速**, 不再需要"硬切"; 到目标前 v→0 时前馈
     按比例缩到 0 ⇒ 末速自然归零。 */
#define POS_TM_FRIC_STATIC_A    1.45f    /* 等效静摩擦(破黏)电流(A) */
#define POS_TM_FRIC_SLIDE_A     0.20f    /* 等效滑动摩擦电流(A)
                                             ★★★ 2026-10-18 (cal17) **0.70→0.20**, 由日志定标:
                                             13:43 版日志里 `Iqref` 的最小值是 **0.09~0.25A**
                                             (依次 0.09/0.11/0.16/0.17/0.19/0.20/0.24/0.25),
                                             **从来没有负过** ⇒ 说明 0.70A 的前馈被 trim 的
                                             −0.45A 抵消掉了, 环路**根本没有刹车能力**:
                                             净力矩最低只能到 ≈0 ⇒ 转子只能靠摩擦自己减速。
                                             后果链(实测 `8`=0.4→360/1.5s):
                                               起步破黏滞后 → trim +0.66A ⇒ iq=1.37A
                                               → 冲到 **704°/s**(规划 407, 1.7 倍)
                                               → tv 变负, trim 拉到 −0.47 ⇒ iq=0.19A
                                               → **净力≈0 ⇒ 只能滑行**
                                               → 滑到齿槽 314.48° 时被擒住 **卡 145ms**
                                               → 参考追上来再推 1.06A ⇒ 又跳一下
                                               = 用户说的"停顿感"(周期 ~250ms)。
                                             给 0.20A(实测维持 407°/s 的真实值)之后:
                                             巡航 `trim≈0`, trim 的 ±2.0A **两侧都可用**
                                             ⇒ iq 最小可到 **−0.9A** ⇒ 净 −1.1A
                                             = 11000°/s², 从 704 拉回 407 只要 26ms。
                                             ⚠ 不要再用 0.70: 那是"名义值", 实际全被
                                              trim 吃掉, 等于把刹车权限锁死。 */
#define POS_TM_FRIC_V0_DEG_S    25.0f    /* Stribeck 过渡速度尺度(机械°/s) */
#define POS_TM_FF_VMIN          1.0f     /* |轨迹速度| 低于它 ⇒ 前馈按比例缩到 0 */
#define POS_TM_FF_SIGN_DEG_S    3.0f     /* 前馈符号的软过渡尺度(机械°/s) */
#define POS_TM_A_PER_DEG_S2     0.0001f   /* 1A ≈ 10000°/s² ⇒ 0.0001 A/(°/s²) */
/* ★★ 2026-10-11ao: 位置微调增益 0.05→0.03, **限幅 0.8→2.0A**。
   限幅原来卡在 0.8A(< 破黏 1.15A)是为了"读数毛刺推不动转子", 但它同时
   也把**刹车权限**一起卡死了(见上注 ② )。注意风险其实由**增益**决定而不是
   限幅: 一个扇区(15.47°)的读数毛刺仍然只产生 0.03×15.47 = 0.46A 的变化;
   要打到 2.0A 需要 66° 的真实误差 —— 那本来就是该刹车的时候。 */
#define POS_TM_P_TRIM_A_PER_DEG 0.030f    /* 位置微调增益(A/机械°) */
#define POS_TM_P_TRIM_MAX_A     2.00f     /* 位置微调限幅(A) —— 必须大于摩擦前馈才能刹车 */
/* ★★ 2026-10-11ao 阻尼(位置环原来"零阻尼"):
   位置误差环是 `a = 10000·Kp·verr` ⇒ ω_n = √(10000×0.03) = 17rad/s
   = **2.8Hz 的无阻尼振荡器** —— 任何扰动(齿槽、摩擦突变、读数台阶)都会
   激起永不衰减的振荡, 这就是"一直在抖"的另一半原因。位置环必须加阻尼,
   但实测本机 50~400°/s 区间的**测速全是毛刺**(±1000eRPM), 直接拿它做
   阻尼只会变成继电器。所以用一个**模型速度观测器**:
     被控对象基本就是"1A → 10000°/s²"的积分器, 所以"净指令电流积分"
     本身就是平滑、无毛刺的速度估计(带宽由我们定), 再用**慢速位置校正**
     消掉摩擦估计误差的长期漂移。
   阻尼增益取 0.002 A/(°/s) ⇒ 环路增益 20rad/s(3.2Hz), 配 30rad/s 的观测器
   带宽 ⇒ 相位滞后约 33°, 阻尼有效而不会自激。 */
#define POS_TM_DAMP_A_PER_DEG_S 0.0020f   /* 阻尼增益(A/(°/s)) */
#define POS_TM_K_DEG_S2_PER_A   10000.0f  /* 实测 1A ≈ 10000°/s² */
#define POS_TM_OBS_WO_RAD_S     30.0f     /* 速度观测器带宽(rad/s) ⇒ 滞后 ~33ms */
#define POS_TM_OBS_VMAX_DEG_S   2000.0f   /* 观测速度限幅 */
/* ★★★ cal28: 观测器模型输入的限幅(A)
   ------------------------------------------------------------------
   α-β 观测器的稳态偏置是可以精确算出来的:  vobs = v_true + (2K/wo)·u
   代入 K=10000, wo=30 ⇒ **每 1A 模型输入就把观测速度抬高 667°/s**。
   u = iq − iq_f = a_ff + trim + damp，而 **trim 在齿槽处能顶到 1.9A**
   ⇒ 偏置可达 1267°/s！

   实测(cal27 t=10869, `8` 加速段中段):
     真速 304°/s、规划 784°/s，但 vob=1019 ⇒ 偏置 ~715°/s
     ⇒ damp = 0.002×(784-1019) = **-0.47A 在刹车**
     而此刻正确的动作是 **+0.96A 往前推**（转子已经掉速了）
     ⇒ 阻尼在帮着齿槽拖住转子 ⇒ verr 涨到 63.5° ⇒ trim 顶到 1.9A
     ⇒ 一旦破黏就以 1190°/s 冲出去(**规划的 1.5 倍**)。

   限幅到 ±0.35A 后:
     · 齿槽处偏置最多 667×0.35 = **233°/s** ⇒ vobs≈537, damp=**+0.49A**(变成往前推) ✔
     · 正常巡航 iq_net ≈ trim+damp ≈ 0.2~0.5A，限幅基本不吃
       ⇒ 保留原来的"用指令电流积分平滑测速"的好处，不会引入霍尔台阶纹波。 */
#define POS_TM_OBS_IQNET_MAX_A  0.35f
/* ★★ 2026-10-11ap 两个限幅 —— 修实测(`8`=360°/1s, 10-11ao 日志)"运行中间一段
   比较大的振动":
   ① **轨迹期间一律跳过"实测速度驱动"的旁路**(超速保险/脱困刹车/破黏脉冲/
      分档限流, 见 FOC_DoPositionLoop 里 2026-10-11ap 的整块跳过):
      实测中段 `Iqref` 在 **±2.50A 之间打摆**、读数在 299↔314↔320 之间来回
      —— 就是"超速保险"(`s_pos_flying && |vsp|>250eRPM ⇒ 满刹车`) 在按
      **假速度**砸电流(当时转子其实还在规划后面 tv=+36°)。轨迹律自己有
      完整限幅, 不需要它们。
   ② **驱动侧限流 1.9A**: 起步时转子被静摩擦搵住不动, 而参考点一直在前进
      ⇒ 位置微调+阻尼不断累积 ⇒ 电流顶到 2.5A 才搵开静摩擦 ⇒ 净力矩
      (2.5−1.45)=1.05A = 10500°/s² ⇒ 实测起步 30ms 就飞到 **~2000°/s**。
      限到 1.9A(比实测最深的破黏 1.6A 还高 0.3A) ⇒ 搵开瞬间净力矩
      ≤0.45A ⇒ 起步平缓得多。刹车侧不受此限(仍到 2.5A)。 */
#define POS_TM_IQ_DRIVE_MAX_A   1.70f     /* 轨迹期间驱动方向电流上限(A)
                                             ★ cal19: 1.90→1.70。实测破黏时电流
                                             顶到 1.90A, 净力矩 (1.90-1.45)=0.45A
                                             一个扇区就能积到 ~373°/s, 实测直接冲到
                                             565~939°/s。1.70A 把"踢出扇区"的净
                                             力矩压到 0.25A ⇒ 起步冲量小三成。 */
#define POS_TM_DAMP_MAX_A       0.80f     /* 阻尼项限幅(A)
                                             ★ cal18: 0.50→0.80。0.50A 只相当于
                                             250°/s 的速度误差; 实测跟踪误差常年
                                             300~660°/s ⇒ 阻尼**一直在饱和**, 等于
                                             没有阻尼。0.80A ≈ 400°/s 的误差余量,
                                             仍然远小于 trim 的 2.0A, 不会盖掉刹车。 */
/* 轨迹状态量(2026-10-11am 模型前馈 / 2026-10-11ao 速度观测器):
   s_tm_iq=本拍电流; s_tm_vprev=上一拍轨迹速度(算加速度项);
   s_tm_vobs/s_tm_posobs=速度观测器状态(只用于阻尼) */
static float   s_tm_iq          = 0.0f;
static float   s_tm_vprev       = 0.0f;
static uint8_t s_tm_vprev_valid = 0U;
static float   s_tm_vobs        = 0.0f;
static float   s_tm_posobs      = 0.0f;
static float   s_tm_dbg_verr    = 0.0f;   /* 日志: 轨迹位置误差(°) */
static float   s_tm_dbg_ff      = 0.0f;   /* 日志: 摩擦前馈(A) */
static float   s_tm_dbg_trim    = 0.0f;   /* 日志: 微调+阻尼(A) */
/* ★ 2026-10-24: 单次轨迹的客观指标(峰值跟踪误差 / 峰值指令电流)。
   用途: 比较“力矩档”是否真的改善了深齿槽滞后 —— 不需要翻长日志,
   每跑完一段轨迹就打一行 [TM] done。 */
static float   s_tm_max_tverr   = 0.0f;   /* max|参考−实际| (°) */
static float   s_tm_max_iq      = 0.0f;   /* max|Iqref| (A) */
static float   s_tm_max_drive   = 0.0f;   /* 本次轨迹用过的**最大**驱动侧上限(A)
                                             2026-10-25: drive 现在是随 |verr| 变的,
                                             只报末尾的瞬时值没意义; 报峰值才能
                                             看出"到底有没有放开到 band"。 */
/* 轨迹结束事件 —— ★ 不能直接在 ISR 里打日志: 实测会让那一拍的 LAW 从
   12.9us 涨到 16.8us(vsnprintf + RTT)。所以只置标志, 由主循环打印。 */
static uint8_t s_tm_done_evt    = 0U;
static int32_t s_tm_done_tv     = 0;
static int32_t s_tm_done_iq     = 0;
static int32_t s_tm_done_band   = 0;
static int32_t s_tm_done_drive  = 0;
/* ★★ 2026-10-11al 60→150: 收尾段靠轨迹修正项把最后几度拉进来(见 POS_TM_DONE_DEG),
   60°/s 对 20° 的滞后要 0.35s 而且已经偏慢; 150°/s 仍低于齿槽擒住极限 195°/s。 */
#define POS_TM_TRIM_MAX_DEG_S   150.0f   /* 跟踪修正项限幅(机械°/s) */
/* ★★ 2026-10-11al 轨迹期间专用的参数(实测 10-11ak 日志按 `8` 仍有拉扯):
   现象: 270°/1s 的轨迹跑完时转子还在 338°(=目标前 21°)且仍以 ~400°/s 在走,
   时间一到轨迹被硬切 + “松手静默”撤力 ⇒ 转子滑到下一齿槽再被拉回来。
   原因两件:
     ① **跟踪增益太小**: 普通档 Kv=0.008A/eRPM。而摩擦前馈 1.25A 已超过滑动
        摩擦(实测起动 1.18A ⇒ 滑动区更小), 多出来的力矩靠 Kv 去平衡 ⇒
        稳态速度误差 =(1.25−fric)/Kv ≈ 30~90eRPM(45~135°/s)! 轨迹于是在
        起步时“伸一下”(实测 45° 的 1s 轨迹在 106ms 内走了 28.9°),
        又在巡航段跟不上, 累计滞后 21°。
     ② **电流权限不够**: 普通档驱动上限 POS_IQ_RUN_A=1.5A < 这个轨迹需要的
        ~1.75A ⇒ 加速段被饱和吃掉。
   对策: 轨迹活跃期间 Kv 提到 0.030(稳态误差降到 1/4)、限幅放到 DAMP_MAX(2.5A);
   并且把轨迹**延伸出“收尾段”**: t≥Tm 后不立即交回, 而是保持参考点在目标上
   (轨迹速度项=0), 只用位置修正项(≤150°/s)把残差慢慢收进去, 直到
   |err|<POS_TM_DONE_DEG 或超过 POS_TM_TAIL_TICKS 安全帽。
   这样“末速归零”才真正实现, 齿槽能直接捣住。 */
#define POS_TM_KV_A_PER_ERPM    0.030f   /* 轨迹期间速度环增益(A/eRPM) */
#define POS_TM_DONE_DEG         3.0f     /* |err| 小于它 ⇒ 轨迹交由普通逻辑收尾 */
#define POS_TM_TAIL_TICKS       (1500U * FOC_TICKS_PER_MS)  /* 收尾段最长再扰 1.5s */
/* ★ cal19 新增两个量 */
#define POS_TM_START_DEG        12.0f    /* 起步破黏窗口: 离起点不足它就给静摩擦 */
#define POS_TM_END_QUIET_TICKS  (120U * FOC_TICKS_PER_MS) /* ★ cal30: 60→120ms 无霍尔边沿才结束
                                             等效“已慢于 **~129°/s**”—— 这是关键:
                                             60ms 只等效 <258°/s, 而实测到位余速
                                             257°/s 恰好卡在它旁边(cal29 t=25863),
                                             于是“quiet”恒为真、根本没起到“确认停住”
                                             的作用 ⇒ 带 257°/s 冲进齿槽 ⇒ 回退 17.3°。
                                             120ms 的边沿间隔 ⇔ 15.47/0.12 = 129°/s,
                                             稳稳低于齿槽擒住极限 195°/s。 */
/* ★ 2026-10-11ab 齿槽能“擒住”的速度上限(实测 ~130eRPM): 位置环拿它当
   "需不需要刹"的门槛 —— 低于它转子会被齿槽自己接住, 高于它一定冲过目标。 */
#define POS_CAPTURE_ERPM      150.0f
/* 位置积分(现在只是静态误差微调, 摩擦已由前馈承担) */
#define POS_INTEG_MAX_A      0.8f
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
#define POS_INTEG_STALL_TICKS (150U * FOC_TICKS_PER_MS)   /* 另有此条: 150ms 无霍尔边沿才算堵转 */
/* ⚠ 2026-10-11b 再加一条: "想动"持续 POS_INTEG_ARM_TICKS 之后积分才被允许增长。
   光靠"150ms 无边沿"不够 —— 换目标的一瞬间转子本来就静止(停在旧目标上),
   天然满足"150ms 无边沿" ⇒ 积分会从第 150ms 就开始按 0.10×90°=9A/s 往上顶
   ⇒ 又变成"靠积分冲出去"(10-11a 的病)。加了这条之后: 起步阶段规划器虽然
   立刻要求运动(v_tgt≠0), 但积分要等 300ms 才"开闸", 那之前转子早该动起来
   (有霍尔边沿 ⇒ 堵转条件不成立 ⇒ 积分冻结); 只有真的卡在齿槽里不动,
   "想动 300ms + 150ms 无边沿" 才同时成立 ⇒ 积分只做"破齿槽助手"。 */
#define POS_INTEG_ARM_TICKS   (300U * FOC_TICKS_PER_MS)   /* "想动"多久才允许积分增长 */

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
#define POS_KICK_A              2.0f
/* ★★ 2026-10-11l 100ms→250ms: 实测(10-11k 日志)100ms×2.8A 的脉冲**完全无效** ——
   泵不出一丝霍尔边沿, 而 10-11j 里 1.6A **持续项** 400ms 却把转子撥开了。
   说明本机构的齿槽"井"很深, 需要的是**冲量(A·s)**而不是峰值:
     1.9A×0.2s ≈ 0.38A·s 能出来, 2.8A×0.1s = 0.28A·s 出不来。
   250ms×2.5A = 0.63A·s ⇒ 有安全余量; 速度一过 POS_KICK_EXIT_ERPM 或
   一见到霍尔边沿就提前松手(下面逻辑里都有), 所以不会失控。 */
#define POS_KICK_TICKS          (250U * FOC_TICKS_PER_MS)
/* ★ 2026-10-11l 认输阀值: 连续多少次脉冲"打完仍无霍尔边沿"就判定机构推不动。
   实测(10-11k 日志) #6...#14 连续 9 次脉冲、每次都实际给出 2.5~2.8A、
   转子一点不动 ⇒ 不是控制问题(要么机构卡死/负载过大, 要么力矩根本
   没加到转子上)。此时再打只会发热+刷屏, 所以停下来并明确报出。 */
#define POS_KICK_MAX_FAIL       4U
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
/* ★★ 2026-10-11v "让电流真的跟上指令" —— 本轮的关键修复
   ------------------------------------------------------------
   实测(10-11u 日志, Tgt=180): 转子在 195.47° 被 1.45A 顶了一个多秒后脉开,
   环路马上按 err 反打到 +2.50A 刹车, **但电流在实际的几十毫秒里根本没建立起来**:
     t=63293 Pos=141.07 err=38.93 Iqref=+2.50 而实测 Iq=-0.09
     t=63328 Pos=134.48 err=45.52 Iqref=+2.50 而实测 Iq=-0.32
     t=63468 Pos=193.01 err=-13.01 Iqref=-2.50 而实测 Iq=+0.15  ← 连符号都反了
   ⇒ 刹车形同虚设 ⇒ 转子从 195 一路冲到 **134(-61°)**, 环路又满推把它推回 195
   ⇒ **几个来回的大摆**(用户: "到达目标值后, 又有一几个来回摆动的动作")。
   根因不是控制律, 而是**电流环的阶跃响应太慢**: 代码里自己写着
   `kp=0.010/ki=0.300 ⇒ 1A 误差下积分只涨 0.3duty/s ⇒ 要 ~250ms 才把电流建立起来`。
   位置环的暂态只有 30~60ms —— 250ms 等于"根本无力"。
   对策(**不改电流环本身的稳定性**): 指令电流发生大阶跃时, 直接把 q 轴积分
   预置到**稳态占空比**(= iq × POS_FF_DUTY_PER_A), 电流环下几个周期(L/R≈0.6ms)
   内就到位。这与 10-11k 给破黏脉冲做的预置是同一手法, 现在推广到每一次阶跃。 */
#define POS_IQ_STEP_PRELOAD_A   0.4f    /* 指令电流跳变超过它 ⇒ 预置积分 */
/* ★ 2026-10-11k 误差符号抖动抑制:
   位置读数在扇区内的分辨率极限就是**半个霍尔扇区**(15.47°/2≈7.7°机械) —— 
   目标附近 err 会在 ±8° 里反复翻号(模型进角/边沿重置的噪声), 环路于是
   "推一下→读数翻号→反推一下" = 用户看到的小幅振动。所以近目标且符号
   刚翻过时, 一律不给任何电流, 等符号稳定下来再说。 */
#define POS_HOLD_BAND_DEG       10.0f
#define POS_ERR_HYST_TICKS      (150U * FOC_TICKS_PER_MS)
/* ★ 2026-10-11r 破黏脉冲的最小误差 10°→20°(整一个齿槽以上)。
   实测(10-11p 日志) 误差 -14.5° 时打的 2.0A×250ms 脉冲, 把转子一口气甩出去
   120°(54.21→-60.79, 正好一个整圈的电角度), 之后环路在 105° 的偏差上
   满推 → 冲过目标 → 再满刹 = ±20° 大摆十几秒停不下来。
   误差只有一个齿槽(15.5°)时不该用脉冲: 前馈(1.25A)+积分(0.8A) 已经
   2.0A > 破黏 1.15A, 慢慢走过去完全走得动, 而且不会飞。 */
#define POS_KICK_MIN_ERR_DEG   28.0f
/* ★★ 2026-10-11s "有意掰一格": 停稳 400ms 以上而误差还 ≥ 13°(一个齿槽)时,
   给一个**刚好过破黏**的固定电流(1.45A, 破黏 1.15A 之上留 26% 余量),
   把转子从这里挤出去。不用脉冲: ① 1.45A 的净力矩只高出滑动摩擦 0.27A,
   越出一个齿槽约 100ms、末速 ~190eRPM, 速度环(Kv×v)一看到速度就把它刹住,
   总行程 ~21° ⇒ 刚好落在下一个齿槽上; ② 脉冲量级的那一下(2.0A×250ms)
   在 10-11p 实测把转子一甩 120°, 再不敢用。
   ③ 必须"确实停了 400ms"才允许: 否则读数在扇区边界上以 ±半个扇区乱跳时
   会跟着推 = 一个齿槽一跳的追逐(比静差更糟)。
   ④ 掰 POS_HOP_MAX 次仍不见好(读数又跳回去) ⇒ 认输、撤力、报一次,
   接受这个静差(比一直顶着发热/一直跳好); 按数字键重新允许。 */
#define POS_HOP_A             1.30f
#define POS_HOP_MAX           6U
/* ★★ 2026-10-11z 配额改成按"**这一推到底有没有让转子挪动**"计
   ----------------------------------------------------------
   实测(10-11y 日志) `t=42867..45847 Pos=195.47 err=-15.47 Iqref=-0.00` ——
   转子差一个齿槽而环路**已认输、罢工 3.1 秒**(jp=6)。原因: 我原来按"进入次数"计量,
   而进入/退出会被读数抖动带着走 ⇒ 几次摆动就把配额烧光 ⇒ 永久静差。
   现在: 用**扇区序号**(整数, 骗不了人) 作为推动结果的仲裁:
     · 推完一瞧 `sect_run` 变了 ⇒ **真挪动了一格** ⇒ 配额清零(下次从最小电流重新开始);
     · `sect_run` 没变 ⇒ 怀疑齿槽太深 ⇒ 失败次数+1, 电流上爬 0.10A;
       连 POS_HOP_MAX(6) 次"一推完全不动" ⇒ 才真认输(并报 GIVE UP)。
   这样"一次成功"会自己重置, "上一次抬过头/读数抖"不会被当成失败。 */
/* ★★ 2026-10-11t 专门给"掰一格"的误差门槛(与温柔区解耦):
   13° 介于半个扇区(7.7°, 读数噪声)与一个扇区(15.5°, 真差一个齿槽)之间。
   10-11s 直接用了 POS_GENTLE_ZONE_DEG 作门槛, 结果一改温柔区就把"掰一格"
   一起改了 —— 现在两者独立。 */
#define POS_HOP_MIN_ERR_DEG   13.0f
/* ★ 2026-10-11u 加上限: "掰一格"只管"差一个齿槽", 绝不能用于换目标的大误差。
   实测(10-11t 日志)按 0(目标 90→0)的一瞬间打了 `NUDGE #1 (err=-90.0deg)` ——
   因为转子在 90° 已静止 2530ms 而我把条件只写成"|err| >= 13°"。后果:
    ① 它会占用"掰不动就认输"的 4 次配额 ⇒ 连着换 4 次目标就可能误报"机构推不动"
      (s_pos_blocked_hard) 而整体停止出力;
    ② `NUDGE` 日志失去含义(本来是用来判"是不是真差一个齿槽"的)。
   上限取 25°(约一个半齿槽): 再大就是正常的规划器工作了。 */
#define POS_HOP_MAX_ERR_DEG   25.0f
/* ★ 2026-10-11k "半个扇区内卡住就静默接受":
   半个霍尔扇区 = 15.47°/2 = 7.7°机械 —— 这是本方案**位置分辨率的极限**, 
   转子到底在这半个扇区的哪一点是物理上无法知道的。所以:
     · 读数在 8° 以内且确实卡住不动 ⇒ 不再推(推也推不准, 只会顶在齿槽壁上
       持续振动/嗡嗡); 就让它停在齿槽里, 由齿槽自己保持;
     · 读数超过 10° 才算"真差一个齿槽", 才允许发破黏脉冲。 */
#define POS_ACCEPT_BAND_DEG      8.0f
/* 脱困刹车只在该刹的时候给(时长由 POS_ESCAPE_BRAKE_TICKS 定) */
#define POS_ESC_BRAKE_V_ERPM    60.0f
#define POS_ESC_BRAKE_MIN_TICKS (25U * FOC_TICKS_PER_MS)
/* ★10-11m 超速保险门限(eRPM): 超过它一律满刹车 */
#define POS_OVERSPEED_ERPM     250.0f
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
#define POS_ESCAPE_BRAKE_TICKS (60U * FOC_TICKS_PER_MS)
#define POS_KICK_STALL_TICKS   (400U * FOC_TICKS_PER_MS)    /* 判"堵转"的静止时长 */
#define POS_KICK_COOLDOWN_TICKS (800U * FOC_TICKS_PER_MS)   /* 两次脉冲最小间隔 */

/* 位置环积分项(A) 与 阻尼用滤波速度(eRPM) */
static float s_pos_integ = 0.0f;
static float s_pos_erpm_filt = 0.0f;
/* 位置差分测速用(2026-10-11g): 10ms 窗口, 无滞后 */
static float    s_pos_vdiff_deg  = 0.0f;
static uint32_t s_pos_vdiff_tick = 0U;
/* 2026-10-11o: 位置差分那一路的原始值(诊断只看, 不参与控制) */
static float    s_pos_vpos_raw   = 0.0f;
/* ---- 2026-10-11r 位置差闸门 + 动/静判定 ---- */
static uint16_t s_pos_win10_edges = 0U;    /* 本 10ms 窗口内的霍尔边沿数 */
static uint32_t s_pos_edge_seen  = 0U;     /* 上一拍看到的 edge_tick(用于计数) */
/* ★ 2026-10-11t 动/静判定: 100ms 内净跨了几个扇区 */
static int32_t  s_pos_sect_prev = 0;
static uint32_t s_pos_sect_tick = 0U;
static uint8_t  s_pos_move      = 0U;     /* 1 = 200ms 内净跨过扇区 ⇒ 真的在动 */
static uint8_t  s_pos_flying    = 0U;     /* 1 = 连续 2 窗净跨≥2 扇区 ⇒ 平均≥155°/s(=真的在飞) */
static uint8_t  s_pos_fly_cnt   = 0U;     /* "在飞"滞后计数(见下) */
static float    s_pos_v_sect    = 0.0f;   /* 由净扇区数算的速度(诊断) */
static uint16_t s_pos_jump_cnt   = 0U;     /* 读数被判为"跳变"丢弃的次数(诊断) */
/* ---- 2026-10-11s "有意掰一格"状态 ---- */
static uint8_t  s_pos_hop_give   = 0U;     /* 1 = 掰不动, 认输(接受静差) */
static uint8_t  s_pos_hop_cnt    = 0U;     /* 连续掰了几次 */
/* 2026-10-11x 近目标"稳定爬行"状态(见 POS_SOFT_ON_DEG 注释) */
static uint8_t  s_pos_soft_on    = 0U;
static int8_t   s_pos_soft_dir   = 0;
static uint32_t s_pos_soft_tick  = 0U;
static float    s_pos_soft_start_deg = 0.0f;  /* 本次推动起点的读数(2026-10-11y 按位移退出) */
static uint32_t s_pos_soft_end_tick  = 0U;    /* 上次推动结束时刻(防抖) */
static int32_t  s_pos_soft_sect0     = 0;     /* 本次推动起点时的扇区序号(2026-10-11z) */
static uint32_t s_pos_soft_etick0    = 0U;    /* 本次推动起点时的霍尔边沿时刻(2026-10-11aa) */
/* ---- 2026-10-11ae 定时轨迹状态(命令 t<deg> d<sec>, 见 POS_TM_PART 注释) ---- */
static uint8_t  s_tm_active = 0U;
static uint8_t  s_tm_phase  = 0U;     /* 1..9: 加速/匀速/减速分段号(0=未激活) */
static uint32_t s_tm_t0     = 0U;
static uint32_t s_tm_Ttick  = 0U;
static float    s_tm_deg0   = 0.0f;   /* 轨迹起点的机械角读数(°) */
static int8_t   s_tm_sgn    = 1;
static float    s_tm_vc     = 0.0f;   /* 巡航速度(机械°/s) */
static float    s_tm_vapp   = 0.0f;   /* 逼近段速度(机械°/s, 2026-10-11an) */
/* 2026-10-11v: 上一次指令电流(用于检测大阶跃 ⇒ 预置电流环积分) */
static float    s_pos_iq_prev    = 0.0f;
/* 脱困刹车状态(2026-10-11j) */
static uint8_t  s_pos_esc_stalled = 0U;
static uint32_t s_pos_brake_end   = 0U;
/* 误差符号抖动抑制(2026-10-11k): 上次误差符号 + 最后一次翻号时刻 */
static int8_t   s_pos_err_sign    = 0;
static uint32_t s_pos_err_flip_tick = 0U;
/* 到位死区状态(2026-10-11p): 1 = 已进入死区、完全撤力 */
static uint8_t  s_pos_hold = 0U;
static uint32_t s_pos_hold_in_tick = 0U;   /* "|err| 连续在容差内"的起点 */
/* 2026-10-11ah 近目标"松手静默"状态(见 POS_SETTLE_TICKS 注释) */
static uint32_t s_pos_settle_tick = 0U;    /* 0 = 不在静默期 */
static uint32_t s_pos_settle_etick = 0U;   /* 已处理过的最新霍尔边沿 */
static uint8_t  s_pos_settling    = 0U;    /* 1 = 本拍静默(强撤力) */
/* 2026-10-11ai 读数跳变 ⇒ 撤力(见 POS_FREEZE_TICKS 注释) */
static uint32_t s_pos_freeze_tick = 0U;    /* 最近一次跳变时刻 */
static uint32_t s_pos_freeze_t0   = 0U;    /* 本轮静默起点 */
static uint16_t s_pos_jump_seen   = 0U;    /* 已处理到的 s_pos_jump_cnt */
static uint8_t  s_pos_frozen      = 0U;    /* 1 = 本拍因读数跳变而撤力 */
/* 2026-10-11aj “真在飞”快判据: 连续同向边沿计数(见 POS_DIR_RUN_MIN)
   ⚠ `Hall_State_t.dir` 是 **uint8_t**(存 -1 时实际是 0xFF) ⇒ 比较前必须
   显式转 int8_t, 否则“同向”永远判不成立(0xFF != -1)。 */
static uint16_t s_pos_dir_run     = 0U;
static int8_t   s_pos_dir_last    = 0;

/* 速度规划器状态(2026-10-09j): 当前命令速度(eRPM) 与命令加速度(eRPM/s) */
static float s_pos_v_cmd = 0.0f;
static float s_pos_a_cmd = 0.0f;

/* 破黏脉冲状态 */
static uint32_t s_pos_kick_end_tick  = 0U;   /* > foc->tick 表示脉冲进行中 */
static uint32_t s_pos_kick_next_tick = 0U;   /* 下一次允许触发的时刻 */
static uint32_t s_pos_kick_edge_tick = 0U;   /* 脉冲开始时的霍尔边沿计数(见下) */
static uint16_t s_pos_kick_cnt       = 0U;   /* 累计脉冲数(日志用) */
/* 2026-10-11l: 连续"打完没动"的脉冲数 / 已判定推不动 */
static uint8_t  s_pos_kick_fail      = 0U;
static uint8_t  s_pos_blocked_hard   = 0U;
/* 位置积分"开闸"计时: 规划器持续要求运动 POS_INTEG_ARM_TICKS 之后才非 0 */
static uint32_t s_pos_arm_tick       = 0U;

/* 位置环堵转报警标志: 一旦报警就保持, 直到误差回到容差内才复位 */
static uint8_t s_pos_blocked_logged = 0U;
/* "本轮超差"起点: 换目标/刚起步时 err 必然大, 但"刚起步"不等于"堵转"。
   原来只看"距上次霍尔边沿", 而停在到位点时本来就没有边沿 ⇒ 一按目标键立刻
   误报(实测: `Set tar[POS] BLOCKED? err=45.5deg, no hall edge for 11897ms`,
   同时把破黏脉冲也误触发了一次)。改为计时"err 连续超差"的时长。 */
static uint32_t s_pos_err_run_tick = 0U;
/* ★★ 2026-10-11ag "机构推不动"不再永久生效 ----------------------------------
   实测(10-11ae 日志) `bh=1` + `err=44.06` 时环路 **25 秒一点力也不出**:
   ① `blocked_hard` 挡住驱动; ② 温和爬行的门槛是 |err| < POS_SOFT_ZONE_DEG(35)
   ⇒ err=44 那个区间**两条路都进不去** ⇒ 永久死锁。现在静默一段时间后自动
   重试(4 次全力脉冲都推不动的状态也可能是暂时的: 机构被异物卡住后又松开/
   手动拨了一下/换相方向错等), 保证环路永远不会永久不出力。 */
#define POS_BLOCK_RETRY_TICKS  (3000U * FOC_TICKS_PER_MS)

/* 位置环"飞车"计时: 滤波速度持续超过门限的拍数 */
static uint32_t s_pos_fast_cnt = 0U;
static uint32_t s_pos_block_tick = 0U;   /* blocked_hard 置位的时刻 */

/* ---- 2026-10-11an 定时轨迹曲线: **4 段**(见 POS_TM_TA_FRAC 注释) ------
   输入: t=已过去的时间(s); ta/tc/td/tapp=加速/匀速/减速/逼近时长(s);
         vc=巡航(°/s); vapp=逼近速度(°/s)
   输出: 返回值=已走过的机械角度(magnitude); *v_out=当前速度(°/s)
   四段全解析式 ⇒ 不积分、不漂。斜坡内部是升余弦(S 形) ⇒ 加速度连续。 */
/* ==========================================================================
 * exp(-v/V0) 查表（建议③）
 * --------------------------------------------------------------------------
 * 轨迹运行时每拍要算 1 次 `expf(-vmag_fric / POS_TM_FRIC_V0_DEG_S)`。
 * ARM 的 expf 约 100~200 周期；换成 64 点表 + 线性插值约 15 周期。
 * 自变量是本机速度(°/s)，超过 250 时 e^(-250/25)=4.5e-5 ⇒ 乘上
 * (STATIC-SLIDE)=1.25A 后只有 6e-5A，完全可以当 0，所以表只做到 250。
 * ========================================================================== */
#define TM_EXP_N      64
#define TM_EXP_VMAX   250.0f
#define TM_EXP_STEP   (TM_EXP_VMAX / (float)TM_EXP_N)
#define TM_EXP_K      ((float)TM_EXP_N / TM_EXP_VMAX)

static float s_tm_exp[TM_EXP_N + 1];

static void TM_ExpInit(void)
{
    int32_t i;
    for (i = 0; i <= TM_EXP_N; i++) {
        s_tm_exp[i] = expf(-(float)i * TM_EXP_STEP / POS_TM_FRIC_V0_DEG_S);
    }
}

/* 返回 exp(-v/V0)，v 取绝对值并鉗到 [0, VMAX] */
static float TM_ExpNeg(float v)
{
    float x, f;
    int32_t i;

    if (!(v > 0.0f)) { return 1.0f; }   /* 兼顾 NaN */
    if (v >= TM_EXP_VMAX) { return 0.0f; }
    x = v * TM_EXP_K;
    i = (int32_t)x;
    if (i >= TM_EXP_N) { return 0.0f; }
    f = x - (float)i;
    return s_tm_exp[i] + f * (s_tm_exp[i + 1] - s_tm_exp[i]);
}

static float TM_Profile4(float t, float ta, float tc, float td, float tapp,
                         float vc, float vapp, float *v_out)
{
    const float PI = 3.14159265f;
    float p1 = vc * ta * 0.5f;
    float p2 = p1 + vc * tc;
    float p3 = p2 + (vc + vapp) * td * 0.5f;
    float p;
    if (t <= 0.0f) {
        *v_out = 0.0f;
        return 0.0f;
    }
    if (t < ta) {                                    /* 加速(S 形): 0 → vc */
        float u = t / ta;
        float su, cu;                              /* ★ 查表代替 sinf/cosf */
        FOC_SinCos(PI * u, &su, &cu);
        *v_out = vc * 0.5f * (1.0f - cu);
        p = vc * ta * (u * 0.5f - su / (2.0f * PI));
    } else if (t < ta + tc) {                        /* 匀速: vc */
        *v_out = vc;
        p = p1 + vc * (t - ta);
    } else if (t < ta + tc + td) {                   /* 减速(S 形): vc → vapp */
        float u  = (t - ta - tc) / td;
        float dv = vc - vapp;
        float su, cu;
        FOC_SinCos(PI * u, &su, &cu);
        *v_out = vapp + dv * 0.5f * (1.0f + cu);
        p = p2 + td * (vapp * u + dv * (u * 0.5f
                                       + su / (2.0f * PI)));
    } else if (t < ta + tc + td + tapp) {            /* 逼近: vapp **线性减到 0**
                                                       ★ cal19: 原来是"恒 vapp",
                                                       时间一到速度从 vapp 直接
                                                       硬切到 0 ⇒ 转子带着
                                                       400+°/s 掠进齿槽再被弹回。
                                                       改成线性收敛 ⇒ 轨迹自己
                                                       在 t=T 时 v=0, 平滑落地。 */
        float u = (t - ta - tc - td) / tapp;
        *v_out = vapp * (1.0f - u);
        p = p3 + vapp * tapp * (u - 0.5f * u * u);
    } else {                                         /* 结束: 停在目标上 */
        *v_out = 0.0f;
        p = p3 + vapp * tapp * 0.5f;
    }
    return p;
}

/* ==========================================================================
 * ★★★ 2026-10-24 运行时可调的「力矩档 / 速度档」(接口见 foc_app.h)
 * --------------------------------------------------------------------------
 * 做法: **保持原宏名不变**, 只把它们的值从编译期常量换成运行期变量
 *       (`#undef` + 重新 `#define`), 下面几千行代码一个字都不用改。
 *
 * 默认 cap = 2.5A(= 原 POS_IQ_CAP_A) ⇒ 比例 r = 1.0 ⇒ **所有数值与改动前完全相同**,
 * 所以这个改动本身是"零行为变化"的。
 *
 * 用户把力矩档调到 X A 时, 下面各项按 r = X/2.5 等比放大, 保持原有标定比例:
 *     POS_IQ_CAP_A / POS_DAMP_MAX_A  (总上限 / 满刹车)
 *     POS_TM_DAMP_MAX_A              (轨迹期阻尼项, 原 0.80 = 0.32·cap)
 *     POS_TM_P_TRIM_MAX_A            (位置微调,     原 2.00 = 0.80·cap)
 *     POS_KICK_A                     (破黏脉冲,     原 2.00 = 0.80·cap)
 * ⚠ 故意**不**缩放(它们是"温柔/防冲过头"的柔性参数, 不是力矩上限):
 *     POS_IQ_RUN_A(1.5) / POS_SOFT_IQ_A(1.30) / POS_SOFT_IQ_MAX_A(1.60) /
 *     POS_INTEG_MAX_A(0.8)
 *
 * ⚠⚠ 2026-10-25 **POS_TM_IQ_DRIVE_MAX_A 已从上面的等比缩放里拿出来** ——
 *   驱动侧上限现在是一个**绝对量**(基准 1.70A, 不随 band 变), 只在
 *   "过了起步窗口 + 确实掉队"时才按掉队量线性放开到 band。
 *   详见 POS_TM_DRIVE_GATE_TA / POS_TM_DRIVE_BOOST_DEG 的注释。
 * ========================================================================== */
/* ★★★ 2026-10-25 第②步: 驱动侧限流与"力矩档"**解耦** -------------------
 * 旧写法: drive = 1.70 × (band/2.50) ⇒ band 放大 ⇒ 起步"踢出扇区"的净力矩同步
 *   变大(净 = drive − 静摩擦 1.45A)。实测:
 *     band 2.70 ⇒ drive 1.84 ⇒ 净 0.39A ⇒ 0→360 的 max|tv| = 25~30°
 *     band 3.50 ⇒ drive 2.38 ⇒ 净 0.93A ⇒ 冲得更狠, tv 反而变差
 *   而 band 缩小时 drive 又不够: band 1.50 ⇒ drive 1.02A ⇒ 深齿槽根本推不动,
 *   实测 0.85s 的行程跑了 **5.2s**，中间 STICTION kick + CREEP 一路升到 1599mA。
 *
 * 新写法: ① 基准恒为 POS_TM_IQ_DRIVE_DEF_A(1.70A 绝对量, 不随 band 变);
 *        ② 过了起步窗口(同摩擦前馈的 ta×1.5)且真的掉队(|verr| 大)时,
 *           按掉队量**线性**放开到 band:
 *              |verr| ≤ 10°   ⇒ 1.70A            (正常跟踪; 起步柔和)
 *              |verr| = 22.5° ⇒ (1.70+band)/2   (band 2.70 ⇒ 2.20A)
 *              |verr| ≥ 35°   ⇒ band            (真卡死 / F=ma 不够)
 *        ③ 后面的限幅只管**驱动方向**(ts·s_tm_iq); 刹车方向仍由
 *           POS_DAMP_MAX_A(= band) 限 ⇒ 放开驱动权限**不会削弱刹车**。
 * 因为放开量是 |verr| 的连续函数(无滞后开关), 不会像"卡住就升/不卡就降"
 * 那样退化成方波(那是 cal18/cal19 反复出错的根因)。
 *
 * ★ 2026-10-25 实测(band 3.30, 门 = ta×1.5 ≈ 285ms):
 *     方向     max\|tv\|   max\|Iqref\|   max drive cap
 *     0→360    25.5°        **1.70A**       2.69A
 *     360→0    54.4/54.6°   **2.29A**       2.96A
 *   ⇒ 两个结论:
 *   ① **0→360 根本不是缺电流**: 环自己只要 1.70A, cap 放到 2.69 也没用上。
 *      它的 25.5° 来自观测器/齿槽, 不是电流权限。
 *   ② **360→0 是真的缺**: 它要 2.29A, band 2.70 时被 drive=1.84 卡住。
 *      但起步窗口把前 285ms 钉在 1.70A ⇒ 误差一路冲到 54°(2.70 档时
 *      固定 1.84A 只有 32~50°)。**这个时间门是负优化, 已置 0。**
 *   verr 是"参考−实际"的**跟踪误差**, 起步时 ≈ 0、随着参考拉开才长大,
 *   所以去掉时间门不会让起步变狠 —— 它天然就只在本环真的掉队后才放开。
 * -------------------------------------------------------------------------- */
#define POS_TM_DRIVE_GATE_TA      0.0f    /* 2026-10-25: 1.5 → 0(时间门取消) */
#define POS_TM_DRIVE_BOOST_DEG   10.0f    /* 超过这个掉队量才开始放开 */
#define POS_TM_DRIVE_BOOST_SPAN  25.0f    /* 再多这么多就放到满(band) */

/* 改动前的原始常量(作为运行期缩放的基准, 不要再改) */
#define POS_IQ_CAP_DEF_A        2.5f
#define POS_TM_IQ_DRIVE_DEF_A   1.70f
#define POS_TM_DAMP_DEF_A       0.80f
#define POS_TM_P_TRIM_DEF_A     2.00f
#define POS_KICK_DEF_A          2.0f
#define POS_VMAX_DEF_DEG_S      800.0f
#define POS_ACC_DEF_DEG_S2      8000.0f

static float s_iq_cap_a      = POS_IQ_CAP_DEF_A;      /* 总电流上限(A) */
static float s_iq_drive_cap  = POS_TM_IQ_DRIVE_DEF_A; /* 轨迹期驱动侧上限(A) */
static float s_tm_damp_cap   = POS_TM_DAMP_DEF_A;     /* 轨迹期阻尼项上限(A) */
static float s_trim_cap      = POS_TM_P_TRIM_DEF_A;   /* 位置微调上限(A) */
static float s_kick_cap      = POS_KICK_DEF_A;        /* 破黏脉冲电流(A) */
static float s_vmax_deg_s    = POS_VMAX_DEF_DEG_S;    /* 巡航速度上限(机械°/s) */
static float s_accmax_deg_s2 = POS_ACC_DEF_DEG_S2;    /* 峰值加速度上限(°/s²) */

#undef  POS_IQ_CAP_A
#define POS_IQ_CAP_A            (s_iq_cap_a)
#undef  POS_DAMP_MAX_A
#define POS_DAMP_MAX_A          (s_iq_cap_a)
#undef  POS_TM_IQ_DRIVE_MAX_A
#define POS_TM_IQ_DRIVE_MAX_A   (s_iq_drive_cap)
#undef  POS_TM_DAMP_MAX_A
#define POS_TM_DAMP_MAX_A       (s_tm_damp_cap)
#undef  POS_TM_P_TRIM_MAX_A
#define POS_TM_P_TRIM_MAX_A     (s_trim_cap)
#undef  POS_KICK_A
#define POS_KICK_A              (s_kick_cap)
#undef  POS_TM_VMAX_MECH_DEG_S
#define POS_TM_VMAX_MECH_DEG_S  (s_vmax_deg_s)
#undef  POS_TM_ACC_MAX_DEG_S2
#define POS_TM_ACC_MAX_DEG_S2   (s_accmax_deg_s2)

static void FOC_App_SetIqCap(float cap)
{
    float r;

    if (!(cap > FOC_APP_TORQUE_MIN_A)) { cap = FOC_APP_TORQUE_MIN_A; }
    if (cap > FOC_APP_TORQUE_MAX_A)    { cap = FOC_APP_TORQUE_MAX_A; }

    r = cap / POS_IQ_CAP_DEF_A;
    s_iq_cap_a     = cap;
    /* ★ 2026-10-25: 驱动侧上限**不随 band 缩放**(绝对量 1.70A)。
       它在轨迹里每拍按 |verr| 重新算, 这里只给个初值。 */
    s_iq_drive_cap = POS_TM_IQ_DRIVE_DEF_A;
    s_tm_damp_cap  = POS_TM_DAMP_DEF_A     * r;
    s_trim_cap     = POS_TM_P_TRIM_DEF_A   * r;
    s_kick_cap     = POS_KICK_DEF_A        * r;
}

static void FOC_App_SetSpeedInt(float deg_s)
{
    if (!(deg_s > FOC_APP_SPEED_MIN_DEG_S)) { deg_s = FOC_APP_SPEED_MIN_DEG_S; }
    if (deg_s > FOC_APP_SPEED_MAX_DEG_S)    { deg_s = FOC_APP_SPEED_MAX_DEG_S; }

    s_vmax_deg_s = deg_s;
    /* 加速度上限跟着速度走: 否则低速档时"加速度/巡航速度"比例失衡,
       短行程会退化成"几乎全程在加速"。
       默认 360°/s → 8000°/s²(与改动前完全一致)。 */
    s_accmax_deg_s2 = deg_s * (POS_ACC_DEF_DEG_S2 / FOC_APP_SPEED_DEF_DEG_S);
    if (s_accmax_deg_s2 < 500.0f)   { s_accmax_deg_s2 = 500.0f; }
    if (s_accmax_deg_s2 > 40000.0f) { s_accmax_deg_s2 = 40000.0f; }
}

static void FOC_DoPositionLoop(FOC_Control_t *foc, float dt_s)
{
    float erpm_now;
    float iq_target;
    float pos_now_deg;
    int8_t saved_torque;
    uint32_t err_run = 0U;   /* "err 已连续超差"的拍数(见下面的堵转看门狗) */

    s_pre_t0 = DWT->CYCCNT;  /* ★ 拆分计时: PRE 段的起点 */
    /* ★ 2026-10-11ag "机构推不动"只当 3s, 之后自动重试(见 POS_BLOCK_RETRY_TICKS) */
    if (s_pos_blocked_hard &&
        (foc->tick - s_pos_block_tick) > POS_BLOCK_RETRY_TICKS) {
        s_pos_blocked_hard = 0U;
        s_pos_kick_fail    = 0U;
        LOGD("[POS] BLOCK retry: allowing pushes again\r\n");
    }
    /* ??????? */
    {
        uint32_t at0 = DWT->CYCCNT;      /* ★ 角度更新单独计时(ANG) */
        FOC_UpdateAngle(foc, &erpm_now);
        FOC_ProfAcc(DWT->CYCCNT - at0,
                    &s_ang_cyc_sum, &s_ang_cyc_max, &s_ang_cyc_cnt);
    }

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
            LOGD("[PROBE] stop erpm=%d mech=%dmdeg\n",
                 (int)erpm_now,
                 (int)(AngleObserver_GetMechDeg(&foc->angle, foc->config.pole_pairs)
                       * 1000.0f));
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

    /* ★★ 2026-10-11r 霍尔边沿计数(给下面的"位置差闸门"用):
       窗口内跨了几条边沿, 就限定了位置读数**物理上**最多能变几个扇区 ——
       这是区分"真位移"和"扇区序号判错"的依据。 */
    if (foc->hall.edge_tick != s_pos_edge_seen) {
        s_pos_edge_seen = foc->hall.edge_tick;
        s_pos_win10_edges++;
        /* ★★ 2026-10-11aj 连续同向边沿计数(见 POS_DIR_RUN_MIN 注释) */
        if ((int8_t)foc->hall.dir == s_pos_dir_last) {
            if (s_pos_dir_run < 250U) s_pos_dir_run++;
        } else {
            s_pos_dir_last = (int8_t)foc->hall.dir;
            s_pos_dir_run  = 1U;
        }
    }

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
        if (dtk >= (10U * FOC_TICKS_PER_MS)) {               /* 10ms 窗口 */
            float dth   = pos_now_deg - s_pos_vdiff_deg;
            float v_pos = dth / (float)dtk * (float)FOC_CTRL_HZ / 1.5f;   /* deg/s → eRPM */
            float maxj  = (s_pos_win10_edges == 0U)
                              ? POS_WIN_N0_MAX_DEG
                              : (((float)s_pos_win10_edges + 0.7f) * POS_SECTOR_DEG);
            uint8_t gated = (fabsf(dth) > maxj) ? 1U : 0U;
            s_pos_vdiff_deg   = pos_now_deg;
            s_pos_vdiff_tick  = foc->tick;
            s_pos_win10_edges = 0U;
            /* ★★ 2026-10-11r 闸门: 这个位置差物理上可能吗?
               窗口内只跨了 n 条边沿 ⇒ 位移最多 (n+1) 个扇区; 一条都没跨 ⇒ 位移
               必然小于一个扇区(实测最快 674°/s ⇒ 10ms 最多 6.7°)。超了就是
               读数跳变(扇区序号判错/边界抖动/整圈滑), 这一路**丢弃不融合**,
               只在日志里保留原值(用户看到的 vp 尖刺仍然是它, 但不再进控制)。 */
            if (gated) {
                s_pos_jump_cnt++;
            }
            /* ★★ 2026-10-11n 致命 bug 修正 —— “取幅值较大者”在**方向翻转**时
               会选到错的那一路:
                 边沿/模型估计 `erpm_now` 在无边沿时按 τ=80ms 指数衰减 ⇒ 转子
                 已反转后它仍然带着**旧方向**的大数值(如 -448eRPM)上百 ms；
                 而位置差分 `v_pos` 方向永远是对的, 但刚起步时数值小。
                 两条一“比幅值”, 就选中了方向错的旧值 ⇒ 环路以为“还在往后飞”
                 ⇒ 一直满推 +2.5A ⇒ 真的推到飞起来 ⇒ 位置读数反向 ⇒ 又满刹
                 ⇒ **±38° 的大摆**(vsp 与位置变化反向, 日志里就是它)。
               修正: 两路**符号矛盾时以位置差分为准**(方向一定对)；只有同号
               时才取幅值大的(宁可高估多刹一点)。 */
            if (!gated) {
                if ((v_pos * erpm_now) < 0.0f) {
                    erpm_now = v_pos;
                } else if (fabsf(v_pos) > fabsf(erpm_now)) {
                    erpm_now = v_pos;
                }
            }
            s_pos_vpos_raw = v_pos;   /* 只给日志看(诊断用, 2026-10-11o) */
        }
    }

    s_pos_erpm_filt += (dt_s / POS_DAMP_TAU_S) * (erpm_now - s_pos_erpm_filt);

    /* ★★ 2026-10-11t 动/静判定(见 POS_SECT_WIN_TICKS 注释): 100ms 内净跨了几个扇区。
       这是**唯一**用来回答"转子到底停了没有"的量 —— 它比位置差可靠得多:
         · 单边沿/模型外推速度在边界翻转(±300eRPM 假值);
         · 位置差分被整扇区跳变污染(±1000eRPM 假值);
         · 位置差的"连续多窗都在动"判据在减速/换向时必然出现净位移≈0 的窗口,
           于是判成"停了" ⇒ 环路在高速飞行中撤力(10-11s 实测的摆动发动机)。
       而净扇区数是整数、单调、单点噪声改不了它。 */
    {
        uint32_t dtk = foc->tick - s_pos_sect_tick;
        if (dtk >= POS_SECT_WIN_TICKS) {
            int32_t dsect = foc->angle.sect_run - s_pos_sect_prev;
            s_pos_sect_prev = foc->angle.sect_run;
            s_pos_sect_tick = foc->tick;
            s_pos_move   = (dsect >= POS_MOVE_SECT_MIN ||
                            dsect <= -POS_MOVE_SECT_MIN) ? 1U : 0U;
            /* ★ 2026-10-11y "在飞"加滞后计数: 实测停住时边界的**非对称抖动**
               也能让 200ms 净跨 ≥2 扇区(见 t=24373: Pos=0.00 err=0.00 却 mv=1
               vs=-103 ⇒ 被判"在飞" ⇒ 环路拿假速度砸 -1.87A ⇒ 把转子撺出去)。
               真飞行必然连续跨窗, 抖动不会 ⇒ 连续 2 个窗(400ms)才确认。 */
            if (dsect >= POS_FLY_SECT_MIN || dsect <= -POS_FLY_SECT_MIN) {
                if (s_pos_fly_cnt < 3U) s_pos_fly_cnt++;
            } else if (s_pos_fly_cnt > 0U) {
                s_pos_fly_cnt--;
            }
            s_pos_flying = (s_pos_fly_cnt >= 2U) ? 1U : 0U;
            s_pos_v_sect = (float)dsect * POS_SECTOR_DEG
                           / ((float)dtk / (float)FOC_CTRL_HZ) / 1.5f;   /* 诊断(eRPM) */
        }
    }

    /* ---- 跑飞保护 (2026-10-01 ★新增, 安全兵) ----
       位置环必须有"跑飞"兜底: 一旦角度/方向出错(实测: ALIGN 未标定 offset),
       闭环会变成正反馈, 电机一路加速 —— 日志里 Pos 到过 -39616°(110 圈)、
       vsp 撞死 ±3000 限幅、相电流 5.4A、机构被抽得飞快。两道门:
       ① 位置偏离目标超过 720°(2 圈) —— 任何正常运动都不可能;
       ② 滤波速度持续 200ms(4000 拍) 超过 1500eRPM —— 飞车。
       命中就 FAULT + 关 PWM, 按 'r' 清故障重来。 */
    if (fabsf(foc->position_loop.pos_err_deg) > 720.0f) {
        LOGW("[POS] RUNAWAY: err=%ddeg > 720 -> FAULT\r\n",
             (int)foc->position_loop.pos_err_deg);
        FOC_Control_SetFault(foc, 0x0010);
        s_pos_fast_cnt = 0U;
        return;
    }
    if (fabsf(s_pos_erpm_filt) > 1500.0f) {
        s_pos_fast_cnt++;
        if (s_pos_fast_cnt > (200U * FOC_TICKS_PER_MS)) {
            LOGW("[POS] RUNAWAY: |vsp|=%deRPM -> FAULT\r\n", (int)s_pos_erpm_filt);
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

        if (err_run > (800U * FOC_TICKS_PER_MS) &&
            since_edge > (800U * FOC_TICKS_PER_MS)) {
            if (!s_pos_blocked_logged) {
                s_pos_blocked_logged = 1;
                LOGW("[POS] BLOCKED? err=%dmdeg, no hall edge for %lums (Iqref=%dmA)\n",
                     (int)(err_now * 1000.0f), (unsigned long)(since_edge / FOC_TICKS_PER_MS),
                     (int)(foc->current_loop.iq_ref * 1000.0f));
            }
        } else {
            s_pos_blocked_logged = 0U;
        }
    }

    /* ★★★ cal25 位置律分频门（见 POS_LAW_DIV 的说明）
       到这里为止，下面这些**已经按 20kHz 跑完了**（不能降）:
         FOC_UpdateAngle / 霍尔边沿计数 / 10ms 测速窗 / s_pos_erpm_filt /
         100ms 扇区窗 / 跑飞保护 / 堵转看门狗
       从这里开始才是"外层位置律"（带宽 3Hz）。分频后跳过的拍只跑电流环，
       foc->current_loop.iq_ref / id_ref 沿用上一拍的值。 */
#if (POS_LAW_DIV > 1U)
    {
        static uint8_t s_law_div = 0U;
        if (++s_law_div < (uint8_t)POS_LAW_DIV) {
            FOC_ProfAcc(DWT->CYCCNT - s_pre_t0,
                        &s_pre_cyc_sum, &s_pre_cyc_max, &s_pre_cyc_cnt);
            FOC_UpdateCurrent(foc);      /* 本拍: 只更新角度 + 电流环(20kHz) */
            return;
        }
        s_law_div = 0U;
        dt_s *= (float)POS_LAW_DIV;      /* 外层律的真实步长 = N 拍 */
        s_law_t0 = DWT->CYCCNT;          /* ★ 位置律开始计时 */
        FOC_ProfAcc(s_law_t0 - s_pre_t0,
                    &s_pre_cyc_sum, &s_pre_cyc_max, &s_pre_cyc_cnt);
    }
#endif

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
        /* ★★ 2026-10-11r: 位置环用**可信速度**。50ms 净位移判定"没在动"时,
           直接把速度当 0 —— 不是丢弃, 而是不去理会它:
             ① 阻尼项 Kv×(v_cmd−v) 不再被 ±100eRPM 的假速度灌成 ±0.8A 真电流
                (这电流正是把转子从齿槽里推出去、读数跳半个扇区、误差翻号、
                 再反向推的那台"发动机", 用户看到的就是"到目标后一直摆");
             ② "|v|<40" 的温柔脱困区/到位撤力/积分门控立刻能成立 ⇒ 近目标
                只剩被 0.8A 限幅的积分(0.8A < 破黏 1.15A) ⇒ 推不动 ⇒ 真的静。 */
        /* ★★ 2026-10-11ab: 上面这条把 v 当 0 的做法近目标“过火”了 ——
           实测(10-11aa 日志)转子从目标冲出去后以 **200~570eRPM** 倒滑
           (ob/vp/vsp 三路一致、`mv=1`, 即真在动), 而 `v` 被当 0 ⇒
             ① `|v|<40` 成立 ⇒ 温柔区 **把驱动清零** ⇒ 什么都不做，转子白滑 20~40°;
             ② 后面分档限幅判不出”驱动/刹车“ ⇒ 驱动给了 2.5A(见文件末尾注释)。
           改为: “200ms 内净跨过扇区(位置上确实在走) **且** 滤波速度超过齿槽能
           擒住的上限(POS_CAPTURE_ERPM=150eRPM=225°/s)” ⇒ 认为真在动。
           低速时不认(转子会被齿槽自己接住, 而且避免 10-11w 那种"把半个
           扇区的抖动乘 Kv 变成方波电流"的病)。 */
        uint8_t moving = ((foc->tick - foc->hall.edge_tick) < POS_EDGE_MOVE_TICKS)
                             ? 1U : 0U;
        float v   = (s_pos_flying || s_pos_move || moving)
                        ? s_pos_erpm_filt : 0.0f;
        /* ★★ 2026-10-11ah 符号校正(近目标极限环的"发动机"之一):
           滤波速度(τ=12ms)**在换向瞬间**还带着旧方向的数值 —— 实测
           `Tgt=180` 时 `vsp=+205eRPM` 而 `vp=-462eRPM`(转子明明在倒着走),
           环路拿它去"刹车"就变成**顺着运动方向推** ⇒ 每换一次向就往振荡里
           灌一次能量 ⇒ ±13° 永动。瞬时估计 `erpm_now`(10ms 位置差分/边沿,
           上面已经和观测器做过同号仲裁)的方向是可靠的 ⇒ 只借它的**符号**,
           幅值仍用滤波值(平滑), 这样既不带相位滞后也不会反向出错。 */
        if (v != 0.0f && (v * erpm_now) < 0.0f) v = -v;
        /* ★★ 2026-10-11aj “真在飞”(快判据, 见 POS_DIR_RUN_MIN) */
        uint8_t flying_fast =
            ((s_pos_dir_run >= POS_DIR_RUN_MIN) &&
             ((foc->tick - foc->hall.edge_tick) < POS_DIR_RUN_TICKS)) ? 1U : 0U;

        /* ★★ 2026-10-11ag 定时轨迹的"心跳"放在这里(与下面走哪一条分支无关) ----
           原来只在"速度规划"那条分支里推进并结束 ⇒ 一旦转子落在到位/死区分支
           (实测 D=0.9° 那种小目标), `s_tm_active` 就永远不会清零,
           `tm=1` 卡了十几秒。现在不管走哪条分支都会推进段号并在超时后结束。
           另外 `s_tm_t0 == 0` 表示"还没真正起步"(命令下完还要 ALIGN/STARTUP),
           第一拍位置环才锚定时间轴 —— 否则轨迹的参考会在 ALIGN 期间自己跑掉。 */
        if (s_tm_active) {
            float t, Tm, ta, tc, td, tapp;
            if (s_tm_t0 == 0U) s_tm_t0 = foc->tick;
            t  = (float)(foc->tick - s_tm_t0) / (float)FOC_CTRL_HZ;
            Tm = (float)s_tm_Ttick / (float)FOC_CTRL_HZ;
            ta   = Tm * POS_TM_TA_FRAC;
            tc   = Tm * POS_TM_TC_FRAC;
            td   = Tm * POS_TM_TD_FRAC;
            tapp = Tm - ta - tc - td;
            if (t < ta) {
                s_tm_phase = 1U;
            } else if (t < (ta + tc)) {
                float f = (tc > 1.0e-6f) ? ((t - ta) / tc) : 1.0f;
                uint8_t ph = (uint8_t)(2.0f + f * 4.0f);   /* 匀速 2..6 */
                s_tm_phase = (ph > 6U) ? 6U : ph;
            } else if (t < (ta + tc + td)) {
                s_tm_phase = 7U;                            /* 减速 7 */
            } else if (t < (ta + tc + td + tapp)) {
                s_tm_phase = 9U;                            /* 逼近 9 */
            } else {
                s_tm_phase = 8U;                            /* 收尾(已到点) */
            }
            /* ★★★ cal19: 结束前必须"转子真的停住了"。
               原来 `t>=Tm` 就直接结束 ⇒ 下一拍跳进"到位/静默撤力"分支把力矩
               清零, 而转子当时还有 ~400°/s: 实测 t=27432 Pos=360.00 →
               t=27457 Pos=341.01(**-19°, 折算 -760°/s**), 然后停在 340.69 达
               230ms, 再被推回 360 —— 用户看到的就是"到了目标又弹一下"。
               现在要求"距上次霍尔边沿 > 120ms"(= 已经慢于 ~129°/s) 才结束;
               等待期间轨迹参考停在目标上(v_eff=0), 由 trim+damp 把它剥住。
               安全帽: 超过 Tm+0.6s 无论如何结束。 */
            if (t >= Tm || fabsf(err) < POS_TM_DONE_DEG) {
                /* ★ cal22: 必须"**既到位又停住**"才结束。
                   cal19 只要 quiet 就结束 ⇒ 转子在 344°(差 17°) 处被齿槽顶住、
                   60ms 没边沿就被判成"安静" ⇒ 轨迹提前收工 ⇒ 又掉回老爬行律。 */
                uint8_t quiet = ((foc->tick - foc->hall.edge_tick) >
                                 POS_TM_END_QUIET_TICKS) ? 1U : 0U;
                uint8_t done  = (quiet && (fabsf(err) < POS_TM_DONE_DEG)) ? 1U : 0U;
                if ((done != 0U) || (t > (Tm + 0.60f))) {
                    s_tm_active = 0U;
                    s_tm_phase  = 0U;
                    /* ★ 2026-10-24: 只置事件, 由主循环打印(见 s_tm_done_evt 注释)。
                       用来看“力矩档”是否真的改善了深齿槽滞后。 */
                    s_tm_done_tv    = (int32_t)(s_tm_max_tverr * 1000.0f);
                    s_tm_done_iq    = (int32_t)(s_tm_max_iq * 1000.0f);
                    s_tm_done_band  = (int32_t)(s_iq_cap_a * 1000.0f);
                    s_tm_done_drive = (int32_t)(s_tm_max_drive * 1000.0f);
                    s_tm_done_evt   = 1U;
                    s_tm_max_tverr = 0.0f;
                    s_tm_max_iq    = 0.0f;
                    s_tm_max_drive = 0.0f;
                    /* 诊断量清零: 不清的话 hold 时一直挂着上次轨迹的 tv/ff/tr */
                    s_tm_dbg_verr = 0.0f;
                    s_tm_dbg_ff   = 0.0f;
                    s_tm_dbg_trim = 0.0f;
                }
            }
        }

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
            /* ★ 10-11m: 去掉了 10-11k 加的"速度小就提前松手" —— 实测那会在
               刚脱困、速度估计还没上来的时候把刹车提前取消掉(日志里
               `t=41848 Pos=41.11 vsp=-185 Iqref=0.00` —— 转子正反向飞着,
               环路却撤了力) ⇒ 直接飞 200°。刹车就固定打满 POS_ESCAPE_BRAKE_TICKS。 */
            if (s_pos_brake_end != 0U && foc->tick >= s_pos_brake_end) {
                s_pos_brake_end = 0U;
                s_pos_integ     = 0.0f;
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

        /* ★★ 2026-10-11p 到位死区(施密特): 先更新状态, 再决定这一拍给不给力 */
        {
            if (fabsf(err) < POS_HOLD_IN_DEG) {
                if (s_pos_hold_in_tick == 0U) s_pos_hold_in_tick = foc->tick;
                if ((foc->tick - s_pos_hold_in_tick) > POS_HOLD_SETTLE_TICKS) {
                    s_pos_hold = 1U;      /* 确实停下来了 ⇒ 进入死区 */
                }
            } else {
                s_pos_hold_in_tick = 0U;
            }
            if (s_pos_hold && fabsf(err) > POS_HOLD_OUT_DEG) {
                s_pos_hold = 0U;          /* 真跑掉一个齿槽以上才重新介入 */
            }
        }

        /* ★★ 2026-10-11ah 近目标"松手静默"(见 POS_SETTLE_TICKS 注释) */
        {
            uint32_t se = foc->tick - foc->hall.edge_tick;
            if (se < 3U && foc->hall.edge_tick != s_pos_settle_etick) {
                s_pos_settle_etick = foc->hall.edge_tick;
                if (!s_tm_active &&
                    fabsf(err) < POS_SETTLE_ARM_DEG &&
                    (foc->hall.prev_dt == 0U ||
                     foc->hall.prev_dt > POS_SETTLE_SLOW_DT)) {
                    s_pos_settle_tick = foc->tick;
                    if (s_pos_soft_on) {
                        /* 这次推动被边沿截断 —— 按"有没有真的动"结算配额 */
                        s_pos_soft_on       = 0U;
                        s_pos_soft_end_tick = foc->tick;
                        s_pos_hop_cnt       = 0U;   /* 有边沿 ⇒ 真的挪过一格 */
                    }
                }
            }
            if (s_pos_settle_tick != 0U &&
                (foc->tick - s_pos_settle_tick) < POS_SETTLE_TICKS) {
                s_pos_settling = 1U;
            } else {
                s_pos_settling = 0U;
                s_pos_settle_tick = 0U;
            }
        }

        /* ★★ 2026-10-11ai 读数跳变 ⇒ 本拍/本节拍彻底撤力(见 POS_FREEZE_TICKS)
           ★ 2026-10-11ak: 定时轨迹期间**不冻** —— 轨迹是速度前馈控制, 位置
           读数的毛刺不影响它跟线; 反而冻结 150~400ms 会让轨迹严重滞后
           (实测把一次 1s 的移动切成一跳一跳的阶梯)。 */
        {
            uint32_t age;
            if (s_pos_jump_cnt != s_pos_jump_seen) {
                s_pos_jump_seen = s_pos_jump_cnt;
                if (!s_tm_active) {
                    if (s_pos_freeze_t0 == 0U) s_pos_freeze_t0 = foc->tick;
                    s_pos_freeze_tick = foc->tick;
                }
            }
            age = (s_pos_freeze_t0 != 0U) ? (foc->tick - s_pos_freeze_t0) : 0U;
            if (s_pos_freeze_t0 != 0U && age < POS_FREEZE_MAX_TICKS &&
                (foc->tick - s_pos_freeze_tick) < POS_FREEZE_TICKS) {
                s_pos_frozen = 1U;
            } else {
                s_pos_frozen = 0U;
                s_pos_freeze_t0 = 0U;
                s_pos_freeze_tick = 0U;
            }
        }

        /* ★★★ cal26: 回弹后**立刻**重新介入。
           实测 `8` 到点撤力 → 溜回一个齿槽(16.7°) → 要 0.77s 才爬回来:
           其中 ~250ms 是 s_pos_settling 的“松手静默”, 另外一部分是积分开闸
           的 300ms。现在: 误差一旦超过一个齿槽(POS_REENGAGE_DEG=13°,
           与 POS_HOLD_OUT_DEG 同量级), 说明转子已经真的溜走了 ⇒ 静默/冻结
           这两种“判停”标志立刻作废, 直接走正常律。
           ⚠ 只清 settling/frozen, **不清 hold** —— hold 自己有自己的施密
           特带(7.5°进 / 13°出), 清掉反而会在目标附近蹭。 */
        if (!s_tm_active && (fabsf(err) > POS_REENGAGE_DEG)) {
            s_pos_settling    = 0U;
            s_pos_settle_tick = 0U;
            s_pos_frozen      = 0U;
            s_pos_freeze_t0   = 0U;
            s_pos_freeze_tick = 0U;
        }

        /* ★★ 2026-10-11ap: 轨迹激活期间这几条全部让位 —— 它们靠读数/测速
           判"停没停", 而本机这两样在运动中都不可信; 实测 `8` 中段就是被
           它们抢走控制权后在 ±2.5A 打摆。 */
        if (!s_tm_active && (s_pos_hold || s_pos_settling || s_pos_frozen)) {
            /* ① 已停在目标齿槽里(hl) 或 ② 刚跨过一条近目标边界(sl, settle):
               一律**完全撤力**(靠齿槽本身保持), 并且不再理会任何速度估计 ——
               这个区间里它们全是尖刺, 一理就自激(含"换向时带旧符号"那一种)。 */
            iq_target = 0.0f;
            s_pos_integ = 0.0f;
            s_pos_v_cmd = 0.0f;
            s_pos_a_cmd = 0.0f;
            s_pos_kick_end_tick  = 0U;
            s_pos_kick_next_tick = 0U;
        } else if (fabsf(err) < foc->position_loop.arrive_tol_deg &&
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
            uint8_t hop = 0U;      /* 2026-10-11s: 本拍是否在"有意掰一格" */
            {
                float dist = fabsf(err);

                /* 巡航限速 与 进近减速曲线 v=√(2ad) */
                float v_lim = (dist < POS_DECEL_ZONE_DEG)
                                  ? POS_DECEL_K_ERPM * sqrtf(dist)
                                  : POS_VCRUISE_ERPM;
                if (v_lim > POS_VCRUISE_ERPM) v_lim = POS_VCRUISE_ERPM;
                float v_tgt = (err > 0.0f) ? v_lim : -v_lim;

                /* ★★ 2026-10-11ae 定时轨迹接管速度规划(命令 `t<角度> d<秒>`):
                   直接用轨迹的"位置参考 + 速度前馈"代替 v=14√d 那套。
                   ⚠ 这里只改 v_tgt(及其后级限流/电流环), 不动后面的近目标
                   逻辑 —— 轨迹走完(t>=T)自动交回普通逻辑收尾。 */
                if (s_tm_active) {
                    float t   = (float)(foc->tick - s_tm_t0) / (float)FOC_CTRL_HZ;
                    float Tm  = (float)s_tm_Ttick / (float)FOC_CTRL_HZ;
                    float ta  = Tm * POS_TM_TA_FRAC;
                    float tc  = Tm * POS_TM_TC_FRAC;
                    float td  = Tm * POS_TM_TD_FRAC;
                    float tapp = Tm - ta - tc - td;
                    float vp  = 0.0f;
                    float p   = TM_Profile4(t, ta, tc, td, tapp,
                                            s_tm_vc, s_tm_vapp, &vp);
                    float ref = s_tm_deg0 + (float)s_tm_sgn * p;
                    float verr = ref - pos_now_deg;      /* 轨迹位置误差(机械°) */
                    /* ★★ 2026-10-11am 模型前馈 + 2026-10-11ao 速度相关摩擦
                       + 阻尼(见 POS_TM_FRIC_* / POS_TM_DAMP_A_PER_DEG_S 注释):
                       电流 = 摩擦(v_轨迹) + a_轨迹×0.0001 + 位置微调 + 阻尼,
                       不用任何"实测速度"做增益 ⇒ 不会有毛刺×增益的继电器翻转。 */
                    float v_eff = (float)s_tm_sgn * vp;      /* 规划的机械°/s */
                    float a_eff = 0.0f;
                    if (s_tm_vprev_valid != 0U) {
                        a_eff = (v_eff - s_tm_vprev) / dt_s;
                        if (a_eff >  POS_TM_ACC_MAX_DEG_S2) a_eff =  POS_TM_ACC_MAX_DEG_S2;
                        if (a_eff < -POS_TM_ACC_MAX_DEG_S2) a_eff = -POS_TM_ACC_MAX_DEG_S2;
                    }
                    s_tm_vprev       = v_eff;
                    s_tm_vprev_valid = 1U;
                    {
                        float vmag = fabsf(v_eff);
                        float iq_f = 0.0f;
                        float trim, damp;
                        /* ★★★ cal18 (2026-10-18) —— Stribeck 的自变量改成
                           **转子实际速度**, 不再用"规划速度。
                           ---- 旧写法的致命点(由日志定量) ----
                           `POS_TM_FRIC_V0_DEG_S` 只有 25°/s, 而规划速度在
                           S 曲线加速段 **100ms 就涨到 84°/s** ⇒ 前馈从
                           1.45A 落到 0.24A; 但此刻转子还**一动没动**
                           (实测 `8`: 前 264ms 只走了 15°, 滞后 25°) ⇒
                           **破黏那 1.2A 全落在 trim 身上** ⇒ trim 堆到
                           +0.44~+1.48A ⇒ 一破黏就以 **730°/s** 冲出去(规划 407)
                           ⇒ 之后全靠 trim 反过来刹车 ⇒ 刹车增益与驱动增益
                           不对称(齿槽吃驱动、不吃刹车) ⇒ **1~3Hz 极限环**
                           = 用户看到的"振动"。
                           ---- 现在的做法 ----
                           vmag_fric = min(|规划速度|, |实测速度|):
                             · 转子不动(vsp≈0) ⇒ 一直给 1.45A 破黏;
                             · 转子起来了 ⇒ 实测上来 ⇒ 前馈自然降到 0.20A;
                             · **取 min 是关键**: v_eff 是上限 ⇒ 测速尖刺只能让
                               前馈**变大**(= 多给一点力), 永远不会因为尖刺
                               把前馈打到 0 —— 保留了原设计"不拿测速
                               做增益"的安全性。
                           副作用: 卡在齿槽里时 vsp≈0 ⇒ 前馈恒为 1.45A,
                           恰好是抬起根据地的力 ⇒ 顺便治了"停顿感"。 */
                        /* ★★★ cal19 (2026-10-18) —— **回退 cal18 的"跟随实测速度"写法**。
                           cal18 用 `min(|v_eff|, 1.5路|vsp|)`, 本意是"转子不动就
                           一直给 1.45A 破黏"。问题: `vsp`/`ob` 会在运动中间**整段
                           塌到 0**(实测 t=27015 ob=0 / 27045 vsp=7 / 27281 ob=0,
                           每次持续 30~100ms) ⇒ 摩擦前馈退化成 **0.20 ↔ 1.45A 的
                           方波** ⇒ 慢→ff=1.45→猛冲(实测 565 / 686 / 939 °/s)
                           →快→ff=0.20→掠速→慢 … 这就是"不平滑"的直接原因。
                           现在改成**一次性的起步破黏**: 只在
                             ① 还在加速段前 1.5·ta 之内, 且
                             ② 还没离开起点一个扇区(|pos-deg0| < 12°)
                           时把摩擦前馈钉在静摩擦 1.45A; 一转起来立刻回到
                           f(规划速度)。于是"踹一脚"只发生在起步, 运动中间
                           前馈是规划速度的平滑函数, 不会再有方波。 */
                        /* ★★★ cal22 (2026-10-18) —— 前馈的"速度来源"分三种场合:
                           ① 正常巡航/加减速: 直接用规划速度 v_eff;
                           ② 起步破黏 (t < 1.5·ta 且还没离开起点一个扇区);
                           ③ **收尾掉队** (t > Tm, 参考已停在目标上, v_eff=0, 但还差 >1°)。
                             旧代码在 ③ 时: v_eff=0 ⇒ 整个前馈块被 `if (vmag>0.01)` 门控
                             跳过 ⇒ iq_f=0 ⇒ 只剩 trim 的 0.030A/° ⇒ 差 17° 才 0.51A
                             **推不动齿槽** ⇒ 卡在原地几百 ms。实测:
                               t=179996 Pos=344.12 err=15.88 tm=9
                               t=180114 Pos=342.57 err=17.43 tm=0 ←轨迹提前结束
                               t=180851 Pos=360.00           ←0.7s 后才靠老爬行律挪进来
                             这一段就是用户感知的"到点前一顿"。
                             现在 ②③ 都强制用静摩擦 1.45A, ③ 用位置误差的方向。 */
                        float   v_ff = v_eff;
                        uint8_t hw   = 0U;
                        if ((t < (ta * 1.5f)) &&
                            (fabsf(pos_now_deg - s_tm_deg0) < POS_TM_START_DEG)) {
                            hw = 1U;                              /* ② 起步破黏 */
                        } else if ((t > Tm) && (fabsf(verr) > 1.0f)) {
                            hw   = 2U;                            /* ③ 收尾掉队 */
                            v_ff = (verr >= 0.0f) ? 50.0f : -50.0f;
                        }
                        vmag = fabsf(v_ff);
                        /* hw ≠ 0 ⇒ expf(0)=1 ⇒ fric = POS_TM_FRIC_STATIC_A (1.45A) */
                        float vmag_fric = (hw != 0U) ? 0.0f : vmag;
                        /* ① 摩擦前馈: 速度相关(Stribeck) + 符号软过渡 */
                        if (vmag > 0.01f) {
                            float fric = POS_TM_FRIC_SLIDE_A
                                       + (POS_TM_FRIC_STATIC_A - POS_TM_FRIC_SLIDE_A)
                                         * TM_ExpNeg(vmag_fric);
                            float s = v_ff / POS_TM_FF_SIGN_DEG_S;
                            if (s >  1.0f) s =  1.0f;
                            if (s < -1.0f) s = -1.0f;
                            if (vmag < POS_TM_FF_VMIN) s *= (vmag / POS_TM_FF_VMIN);
                            iq_f = s * fric;
                        }
                        /* ② 位置微调(弹簧): 有权限驱动也**有权限刹车** */
                        trim = POS_TM_P_TRIM_A_PER_DEG * verr;
                        if (trim >  POS_TM_P_TRIM_MAX_A) trim =  POS_TM_P_TRIM_MAX_A;
                        if (trim < -POS_TM_P_TRIM_MAX_A) trim = -POS_TM_P_TRIM_MAX_A;
                        /* ③ 速度阻尼(用模型观测速度, 不用毛刺测速; 自带限幅) */
                        damp = POS_TM_DAMP_A_PER_DEG_S * (v_eff - s_tm_vobs);
                        if (damp >  POS_TM_DAMP_MAX_A) damp =  POS_TM_DAMP_MAX_A;
                        if (damp < -POS_TM_DAMP_MAX_A) damp = -POS_TM_DAMP_MAX_A;
                        s_tm_iq = iq_f + POS_TM_A_PER_DEG_S2 * a_eff + trim + damp;
                        if (s_tm_iq >  POS_DAMP_MAX_A) s_tm_iq =  POS_DAMP_MAX_A;
                        if (s_tm_iq < -POS_DAMP_MAX_A) s_tm_iq = -POS_DAMP_MAX_A;
                        /* ④ 驱动侧限流: 基准 1.70A 固定 + 掉队时线性放开到 band,
                           见 POS_TM_DRIVE_GATE_TA / POS_TM_DRIVE_BOOST_DEG 注释 */
                        {
                            float cap = POS_TM_IQ_DRIVE_DEF_A;
                            if (t > (ta * POS_TM_DRIVE_GATE_TA)) {
                                float e = fabsf(verr);
                                if (e > POS_TM_DRIVE_BOOST_DEG) {
                                    float k = (e - POS_TM_DRIVE_BOOST_DEG)
                                            / POS_TM_DRIVE_BOOST_SPAN;
                                    if (k > 1.0f) k = 1.0f;
                                    cap += (s_iq_cap_a - POS_TM_IQ_DRIVE_DEF_A) * k;
                                }
                            }
                            if (cap < POS_TM_IQ_DRIVE_DEF_A) cap = POS_TM_IQ_DRIVE_DEF_A;
                            if (cap > s_iq_cap_a)           cap = s_iq_cap_a;
                            s_iq_drive_cap = cap;
                            if (cap > s_tm_max_drive) { s_tm_max_drive = cap; }
                            {   float ts = (float)s_tm_sgn;
                                if (ts * s_tm_iq > cap) {
                                    s_tm_iq = ts * cap;
                                } }
                        }
                        s_tm_dbg_verr = verr;
                        s_tm_dbg_ff   = iq_f;
                        s_tm_dbg_trim = trim + damp;
                        /* ★ 2026-10-24: 单次轨迹的峰值指标 */
                        if (fabsf(verr) > s_tm_max_tverr) { s_tm_max_tverr = fabsf(verr); }
                        if (fabsf(s_tm_iq) > s_tm_max_iq)  { s_tm_max_iq = fabsf(s_tm_iq); }
                        /* ⑤ 速度观测器更新(用刚算出的指令电流作为模型输入)
                           ★★★ cal18: 模型输入必须是"**真正产生加速度的那部分**
                           电流", 也就是 `s_tm_iq - iq_f`。原来减的是
                           `sgn × POS_TM_FRIC_SLIDE_A`(=0.20A 的**滑动**摩擦),
                           而实际减掉的是 `iq_f`(破黏期 1.45A) ⇒ 观测器以为
                           "还有 1.25A 在加速" ⇒ 速度被系统性抬高。
                           α-β 观测器的稳态偏置可以**精确地**算出来:
                             vobs_dot = K·u + wo²·oerr = 0  ⇒ oerr = -K·u/wo²
                             vobs = v_true - 2·wo·oerr = v_true + (2K/wo)·u
                           代入 K=10000, wo=30 ⇒ **vobs = v_true + 667·u** (°/s/A)。
                           日志验证(t=11660): iq=0.92, 真速 192 ⇒
                             预测 vobs = 192 + 667×(0.92-0.20) = 672, 实测 vob=676 ✔
                           于是阻尼项 0.002×(407-676) = -0.54A 恒被 -0.50A 限幅打死
                           ⇒ 环路退化成"纯 P 位置环"(ωn=17rad/s, ζ≈0) ⇒ 极限环。
                           现在 u = iq - iq_f, 稳态就是 a_ff + trim + damp ≈ 0
                           ⇒ 偏置消失, 阻尼真正生效。 */
                        {
                            float wo    = POS_TM_OBS_WO_RAD_S;
                            /* ★ cal28: 模型输入限幅，防止 trim 把观测速度吹大
                               (详见 POS_TM_OBS_IQNET_MAX_A 的注释) */
                            float iq_net = s_tm_iq - iq_f;
                            float oerr;
                            if (iq_net >  POS_TM_OBS_IQNET_MAX_A) iq_net =  POS_TM_OBS_IQNET_MAX_A;
                            if (iq_net < -POS_TM_OBS_IQNET_MAX_A) iq_net = -POS_TM_OBS_IQNET_MAX_A;
                            s_tm_vobs   += POS_TM_K_DEG_S2_PER_A * iq_net * dt_s;
                            s_tm_posobs += s_tm_vobs * dt_s;
                            oerr = pos_now_deg - s_tm_posobs;
                            s_tm_posobs += (2.0f * wo) * oerr * dt_s;
                            s_tm_vobs   += (wo * wo) * oerr * dt_s;
                            if (s_tm_vobs >  POS_TM_OBS_VMAX_DEG_S) s_tm_vobs =  POS_TM_OBS_VMAX_DEG_S;
                            if (s_tm_vobs < -POS_TM_OBS_VMAX_DEG_S) s_tm_vobs = -POS_TM_OBS_VMAX_DEG_S;
                        }
                    }
                    v_tgt = v_eff / 1.5f;    /* 仅供积分开闸/日志参考(不用作电流) */
                    /* ★★ 2026-10-11ap: 轨迹期间位置积分与爬行状态一律清零 ——
                       轨迹律是唯一权威, 不许"破齿槽助手"再叠上来。 */
                    s_pos_integ   = 0.0f;
                    s_pos_soft_on = 0U;
                } else {
                    s_tm_vprev_valid = 0U;
                }

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

                /* ★★ 2026-10-11n 滑动摩擦前馈(关键结构性修正):
                   本机**滑动摩擦 ≈1.18A**(实测破静摩擦 1.15~1.9A, 而 1A ≈ 10000°/s²)。
                   原来环路只有“速度环 ±Kv(v_cmd−v) + 堵转积分”:
                     · 静止时速度环只给 0.008×100 = **0.8A < 摩擦** ⇒ 根本起不动;
                       只能等积分长到 1.3A(要 450ms) 才能起步, 一动起来积分又
                       被清零 ⇒ 驱动立刻降回 0.8A < 摩擦 ⇒ 又停 —— 于是变成
                       “一下一下跳”, 且每次跳都由积分/破黏脉冲的能量决定, 不可控。
                   现在用一个**常值前馈**(误差方向, |err|≥5° 时满 1.25A)把摩擦补上:
                     iq = 1.25·sat(err/5°) + 0.008·(v_cmd−v)
                     · 静止: 1.25A > 1.18A ⇒ 真的能起步(加速度≈700°/s², 很慢);
                     · 加速到 v≈100~150eRPM 时前馈刚好被速度环抵消 ⇒ **自限速**
                       在这个区间 —— 而实测“齿槽能擒住的极限”就是 ~130eRPM;
                     · 误差变小 ⇒ v_cmd=14√d 降下来 ⇒ 速度环给反向力 ⇒ 减速;
                     · 真正的刹车仍可到 2.5A(限幅分档不变)。 */
                if (s_tm_active) {
                    /* ★★ 2026-10-11am 轨迹: 直接用模型前馈给出的电流
                       (2026-10-11ao: 摩擦前馈按速度成型 + 位置微调 + 阻尼,
                       见 POS_TM_FRIC_STATIC_A / POS_TM_DAMP_A_PER_DEG_S),
                       不走速度环 —— 本机这个速度段的测速全是毛刺。 */
                    iq_drive = s_tm_iq;
                } else {
                    float iq_ff = POS_FF_FRIC_A * (err / POS_FF_RAMP_DEG);
                    if (iq_ff >  POS_FF_FRIC_A) iq_ff =  POS_FF_FRIC_A;
                    if (iq_ff < -POS_FF_FRIC_A) iq_ff = -POS_FF_FRIC_A;
                    iq_drive = iq_ff + POS_KV_A_PER_ERPM * (s_pos_v_cmd - v);
                }
                {
                    float iq_lim = s_tm_active ? POS_DAMP_MAX_A : POS_IQ_RUN_A;
                    if (!s_tm_active &&
                        ((v > 0.0f && iq_drive < 0.0f) ||
                         (v < 0.0f && iq_drive > 0.0f))) {
                        iq_lim = POS_DAMP_MAX_A;
                    }
                    if (iq_drive >  iq_lim) iq_drive =  iq_lim;
                    if (iq_drive < -iq_lim) iq_drive = -iq_lim;
                }
                /* ★★ 2026-10-11w 近目标温柔档(见 POS_SOFT_ZONE_DEG 注释):
                   位置分辨率只有 15.5°, 而 2.5A 在 35ms 内就能把转子撺过 15.5°
                   ⇒ 近目标用满力 = 看不见的增益雪崩。这里把电流限到"刚好过破黏",
                   让它一次只走一个齿槽(末速 ~195°/s = 齿槽能擒住的 130eRPM)。 */
                if (!s_pos_flying && !s_tm_active && fabsf(err) < POS_SOFT_ZONE_DEG) {
                    if (flying_fast && fabsf(err) < POS_SOFT_BRAKE_DEG) {
                        /* ★★ 2026-10-11aj 最后一段**真刹车**(见 POS_DIR_RUN_MIN):
                           幅值按速度估计(自限: 转子一慢下来它就自己消失, 不会
                           把转子反过来推), 方向用 hall.dir(离散可靠)。
                           目标: 把“冲过齿槽再被推回来”改成“停在齿槽沙里”。 */
                        float mag = POS_KV_A_PER_ERPM * fabsf(s_pos_erpm_filt);
                        if (mag > POS_DAMP_MAX_A) mag = POS_DAMP_MAX_A;
                        iq_drive = ((int8_t)foc->hall.dir >= 0) ? -mag : mag;
                    } else {
                        /* 慢: 限到 POS_SOFT_HOLD_A(0.85A) < 破黏 1.15A ⇒ 环路
                           自己拽不动转子, 不可能自激; 起步交给爬行脉冲。 */
                        if (iq_drive >  POS_SOFT_HOLD_A) iq_drive =  POS_SOFT_HOLD_A;
                        if (iq_drive < -POS_SOFT_HOLD_A) iq_drive = -POS_SOFT_HOLD_A;
                    }
                }

                /* ★★ 2026-10-11j 温柔脱困: 近目标 + 基本没动 ⇒ 撤掉速度环驱动,
                   只靠积分慢慢顶(见 POS_GENTLE_ZONE_DEG 注释) */
                if (!s_tm_active &&
                    fabsf(err) < POS_GENTLE_ZONE_DEG &&
                    fabsf(v) < POS_INTEG_GATE_ERPM) {
                    iq_drive = 0.0f;
                }

                /* ★★ 2026-10-11x/y 近目标爬行(见 POS_SOFT_ON_DEG / POS_SOFT_TRAVEL_DEG
                   注释): 取代 10-11s 的"掰一格" —— 它的条件(since_edge>400ms)在
                   扇区边界抖动时永远不成立。
                   退出条件(2026-10-11y 关键修正): **按"推了多少位移"退, 不按时间/误差退**。 */
                {
                    /* ★★ 2026-10-11aa: "认输"不再是永久的。
                       实测(10-11y/z 日志) `t=42867..45847 Pos=195.47 err=-15.47
                       Iqref=-0.00 dte 1500→4481` —— 认输后环路**永久静默**,
                       一个扇区的静差就永远留在那里(而且日志里再也看不到任何
                       CREEP/GIVE UP, 完全无从判断)。现在认输后只安静
                       POS_SOFT_GIVE_RETRY_TICKS, 再从最小电流重新爬一轮。 */
                    if (s_pos_hop_give && s_pos_soft_end_tick != 0U &&
                        (foc->tick - s_pos_soft_end_tick) > POS_SOFT_GIVE_RETRY_TICKS) {
                        s_pos_hop_give = 0U;
                        s_pos_hop_cnt  = 0U;
                        LOGD("[POS] CREEP retry after give-up (err=%dmdeg)\r\n",
                             (int)(err * 1000.0f));
                    }
                    if (s_pos_soft_on) {
                        float travel = (pos_now_deg - s_pos_soft_start_deg)
                                       * (float)s_pos_soft_dir;
                        /* ★ 2026-10-11ah 脉宽与电流成反比(近似"等冲量"): 电流
                           上爬到 2.0A 时 80ms 的行程会从 4° 涨到 30°(≈两格) ——
                           "推一下、冲两格、再推"就是这么来的。 */
                        float amp0 = POS_SOFT_IQ_A + POS_SOFT_STEP_A * (float)s_pos_hop_cnt;
                        uint32_t pulse_max = POS_SOFT_PUSH_TICKS;
                        if (amp0 > POS_SOFT_IQ_A) {
                            pulse_max = (uint32_t)((float)POS_SOFT_PUSH_TICKS *
                                                   (POS_SOFT_IQ_A / amp0));
                        }
                        if (pulse_max < 400U) pulse_max = 400U;   /* 至少 20ms */
                        if (fabsf(err) < POS_SOFT_OFF_DEG ||
                            travel > POS_SOFT_TRAVEL_DEG ||
                            (foc->tick - s_pos_soft_tick) > pulse_max) {
                            s_pos_soft_on       = 0U;
                            s_pos_soft_end_tick = foc->tick;
                            /* ★ 10-11z/aa: 结算本次推动 —— "到底有没有真的动"由
                               **霍尔边沿**裁决(位置只能数到半个扇区, 而边沿是
                               物理事件): 推动期间出现过任何一条被接受的边沿
                               ⇒ 转子真的挪过一格 ⇒ 配额清零; 一条都没有 ⇒
                               齿槽太深/读数是外推虚值 ⇒ 失败次数 +1、电流上爬。 */
                            if (foc->hall.edge_tick != s_pos_soft_etick0 ||
                                foc->angle.sect_run  != s_pos_soft_sect0) {
                                s_pos_hop_cnt = 0U;   /* 动了 ⇒ 配额清零 */
                            } else {
                                if (s_pos_hop_cnt < 250U) s_pos_hop_cnt++;
                                if (s_pos_hop_cnt > POS_HOP_MAX &&
                                    !s_pos_hop_give) {
                                    s_pos_hop_give = 1U;
                                    LOGW("[POS] CREEP give up: %u x push at err=%dmdeg with NO hall edge (even %dmA) -> quiet %ums, then retry.\r\n",
                                         (unsigned)POS_HOP_MAX, (int)(err * 1000.0f),
                                         (int)(POS_SOFT_IQ_MAX_A * 1000.0f),
                                         (unsigned)(POS_SOFT_GIVE_RETRY_TICKS / FOC_TICKS_PER_MS));
                                }
                            }
                        }
                    } else if (!s_tm_active && !s_pos_hop_give && !s_pos_flying &&
                               (s_pos_soft_end_tick == 0U ||
                                (foc->tick - s_pos_soft_end_tick) > POS_SOFT_RETRY_TICKS) &&
                               (foc->tick - foc->hall.edge_tick) > POS_SOFT_QUIET_TICKS &&
                               fabsf(err) >= POS_SOFT_ON_DEG &&
                               fabsf(err) <  POS_SOFT_ZONE_DEG) {
                        s_pos_soft_on        = 1U;
                        s_pos_soft_dir       = (err > 0.0f) ? 1 : -1;
                        s_pos_soft_tick      = foc->tick;
                        s_pos_soft_start_deg = pos_now_deg;
                        s_pos_soft_sect0     = foc->angle.sect_run;
                        s_pos_soft_etick0    = foc->hall.edge_tick;
                        LOGD("[POS] CREEP(%u): %dmdeg off, push %dmA\r\n",
                             (unsigned)s_pos_hop_cnt, (int)(err * 1000.0f),
                             (int)((POS_SOFT_IQ_A + POS_SOFT_STEP_A
                                    * (float)s_pos_hop_cnt) * 1000.0f));
                    }
                    if (s_pos_soft_on) {
                        /* 电流按"连续失败次数"分档上爬: 1.30 / 1.40 / ... / 上限 2.00A
                           (实测各齿槽破黏门槛在 1.15~1.6A 之间变化)。
                           因为退出按位移算, 电流大只会走得更快, 不会推得更远。 */
                        float amp = POS_SOFT_IQ_A + POS_SOFT_STEP_A * (float)s_pos_hop_cnt;
                        if (amp > POS_SOFT_IQ_MAX_A) amp = POS_SOFT_IQ_MAX_A;
                        iq_drive    = (float)s_pos_soft_dir * amp;
                        s_pos_integ = 0.0f;
                    }
                }
                if (fabsf(err) < POS_HOLD_IN_DEG) {
                    s_pos_hop_cnt = 0U;   /* 到位 ⇒ 配额清零 */
                }
            }

            /* 位置积分: 只在"确实动不了"时才增长。三个条件必须同时成立:
               ① 规划器已持续要求运动 300ms(开闸, 防"换目标就靠积分冲");
               ② 滤波速度接近 0;  ③ 150ms 没有霍尔边沿(真没动)。
               正常运动时①②③至少一个不成立 ⇒ 积分冻结/按 0.1s 泄放。
               ★ 10-11w: 近目标温柔档内**冻结积分** —— 那一档已经由 1.30A 前馈
               负责"把转子从一个齿槽里挤出去", 再加上积分(≤0.8A)就变成 2.1A,
               又把"看不见的增益雪崩"带回来了。 */
            if (!s_pos_flying && fabsf(err) < POS_SOFT_ZONE_DEG) {
                s_pos_integ = 0.0f;
            }
            if (hop == 0U && s_pos_arm_tick != 0U &&
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

            /* ★★ 2026-10-11ap: 轨迹激活期间**跳过**下面全部"实测速度驱动"的
               旁路逻辑(脱困刹车 / 超速保险 / 破黏脉冲 / 分档限流) ——
               它们全部依赖 vsp, 而本机这个速度段的 vsp 是毛刺。
               实测(`8` = 360°/1s) 中段 `Iqref` 在 **±2.50A 之间打摆**、
               位置读数在 299↔314↔320 之间来回: 就是"超速保险"
               (`s_pos_flying && |vsp|>250eRPM ⇒ 满刹车`) 在按**假速度**砸电流
               —— 而转子当时明明还在规划后面(tv=+36°)。轨迹律自己有完整
               限幅(驱动 ≤POS_TM_IQ_DRIVE_MAX_A, 刹车 ≤POS_DAMP_MAX_A),
               这里直接跳过 ⇒ "规划 = 唯一权威"。 */
            if (!s_tm_active) {
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
                           fabsf(err) > POS_KICK_MIN_ERR_DEG &&
                           !s_pos_blocked_hard &&
                           (s_pos_kick_next_tick == 0U || foc->tick >= s_pos_kick_next_tick)) {
                    /* ★10-11l: 先结算上一次脉冲有没有效果(有没有出现过霍尔边沿) */
                    if (s_pos_kick_edge_tick != 0U &&
                        foc->hall.edge_tick == s_pos_kick_edge_tick) {
                        if (s_pos_kick_fail < 255U) s_pos_kick_fail++;
                    } else {
                        s_pos_kick_fail = 0U;
                    }
                    if (s_pos_kick_fail >= POS_KICK_MAX_FAIL) {
                        /* 连续多次全力脉冲 + 完全无霍尔边沿 ⇒ 机构推不动。
                           静默 POS_BLOCK_RETRY_TICKS 后由上面的看门狗自动清零重试;
                           按数字键/换目标也会立刻清零。 */
                        if (s_pos_blocked_hard == 0U) {
                            s_pos_blocked_hard = 1U;
                            s_pos_block_tick   = foc->tick;
                            LOGW("[POS] BLOCKED: %u kicks x %dmA produced NO hall edge (err=%dmdeg, since_edge=%lums).\r\n",
                                 (unsigned)s_pos_kick_fail, (int)(POS_KICK_A * 1000.0f),
                                 (int)(err * 1000.0f),
                                 (unsigned long)(since_edge / FOC_TICKS_PER_MS));
                            LOGW("[POS] -> torque is not reaching the rotor (mechanism jammed/loaded, or commutation angle wrong).\r\n");
                            LOGW("[POS] -> hold %us then retry automatically (or press a digit key / 'r').\r\n",
                                 (unsigned)(POS_BLOCK_RETRY_TICKS / FOC_TICKS_PER_S));
                        }
                    } else {
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
                        LOGW("[POS] STICTION kick #%u: %dmA for %ums (err=%dmdeg, stalled %lums)\r\n",
                             (unsigned)s_pos_kick_cnt, (int)(POS_KICK_A * 1000.0f),
                             (unsigned)(POS_KICK_TICKS / FOC_TICKS_PER_MS),
                             (int)(err * 1000.0f), (unsigned long)(since_edge / FOC_TICKS_PER_MS));
                    }
                }

                if (s_pos_brake_end != 0U && foc->tick < s_pos_brake_end && foc->hall.dir != 0) {
                    /* 刚脱困: 强制全刹车(方向与刚走的方向相反), 停在下一个齿槽 */
                    iq_target = -(float)foc->hall.dir * POS_DAMP_MAX_A;
                } else if (s_pos_flying && fabsf(s_pos_erpm_filt) > POS_OVERSPEED_ERPM) {
                    /* ★10-11m 超速保险(最后一道): 无论规划器怎么说, 只要速度
                       真的冲上去了就闭眼满刹车。
                       ★★ 2026-10-11w 必须加 `s_pos_flying`: 实测近目标处位置读数
                       跳一个齿槽就能让 vsp 假报到 500~600eRPM ⇒ 超速保险每次
                       都被假尖刺触发、砸 2.5A(那是"近目标持续摆动"的主力之一)。
                       而真的跑飞一定会持续跨扇区 ⇒ s_pos_flying=1, 保护依然有效。 */
                    iq_target = (s_pos_erpm_filt > 0.0f) ? -POS_DAMP_MAX_A
                                                         :  POS_DAMP_MAX_A;
                } else if (kicking) {
                    iq_target = (err > 0.0f) ? POS_KICK_A : -POS_KICK_A;
                } else {
                    /* ★10-11l: 已判定机构推不动(连续全力脉冲都无霍尔边沿)
                       ⇒ 完全撤力, 不再白烧电流/不再刷屏 */
                    /* ★★ 2026-10-11aa: 只在"没有温和爬行"时才撤力。
                       原来这条把爬行的输出也一起清零(爬行块在前面已经把 iq_drive
                       写好了) ⇒ 一旦判定"机构推不动", 环路就**永久**一点力都不出,
                       连一个齿槽的静差都修不了。爬行本身是限幅的(≤2.0A、单次
                       ≤600ms、按位移退出), 让它继续工作; 被这条挡住的只是
                       2.0A 冲量脉冲。 */
                    if (s_pos_blocked_hard && !s_pos_soft_on) {
                        iq_target   = 0.0f;
                        s_pos_integ = 0.0f;
                    }
                    /* ★ 10-11k 抖动抑制: 近目标且误差符号刚翻过 ⇒ 撤掉一切电流
                       (位置读数分辨率只有半个扇区, 此时驱动就是自激振动的源)
                       ★★ 2026-10-11m 关键修正: 必须加 `|v| < 40`! 否则会变成
                       "转子高速穿过目标 → 误差翻号 → 撤力 → 惯性冲过去 30° →
                       150ms 后满力反推 → 再穿过来" = 大摆继电器环。实测
                       `t=23879 Pos=74.24 err=+15.8 vsp=-37 Iqref=0.00` 就是它。
                       只有"已经基本停住"时才需要这个去抖。 */
                    if (fabsf(err) < POS_HOLD_BAND_DEG &&
                        fabsf(v) < POS_ARRIVE_V_ERPM &&
                        s_pos_err_flip_tick != 0U &&
                        (foc->tick - s_pos_err_flip_tick) < POS_ERR_HYST_TICKS) {
                        iq_target   = 0.0f;
                        s_pos_integ = 0.0f;
                    }
                    /* ★ 10-11k "半个扇区内卡住" ⇒ 静默接受(见 POS_ACCEPT_BAND_DEG) */
                    if (fabsf(err) < POS_ACCEPT_BAND_DEG &&
                        (foc->tick - foc->hall.edge_tick) > POS_INTEG_STALL_TICKS) {
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
            }   /* ← 2026-10-11ap: 关闭"轨迹期间跳过旁路逻辑" */
        }

        /* ★★ 2026-10-11ab 近目标"驱动电流"硬限 —— **本轮抖动的直接主因**:
           实测(10-11aa 日志) `t=50715 Pos=163.52 err=16.48 Iqref=+2.50 vsp=-236`
           —— 转子在 16° 外用 2.5A 往回推: 净 1.32A ⇒ 13200°/s² ⇒ 到目标时速度
           660°/s, **远超齿槽能擒住的 195°/s** ⇒ 冲过目标 1~2 个齿槽、再被反碰
           ⇒ `156↔195` 的持续摆动(用户: "4 还是抖动")。
           而 2.5A 会出现, 是因为位置环把 `v` 强制当 0(没在飞) ⇒ 判不出
           "这是驱动还是刹车" ⇒ 走了驱动档 2.5A(见上面 v 的注释)。
           物理规律: 近目标要"刚好过破黏"就够 —— 距离 d 越大所需的净加速度
           越小(a = v_cap²/2d), 所以 **任何 d 下 1.30A 都刚好**:
             d=15.5° ⇒ 1.30A ⇒ 末速 195°/s ✓ ; d=31° ⇒ 1.24A ✓
           于是:
             · 转子**停着**(≥400ms 无边沿) ⇒ 允许到 POS_SOFT_IQ_MAX_A(1.6A,
               破黏最深的一格) —— 这是唯一能让它离开齿槽的手段;
             · 转子**一动** ⇒ 驱动方向一律 ≤1.30A(再大只会积累冲过目标的动能);
             · 电流**反着**运动方向(真刹车) ⇒ 仍可到 2.5A。
           这条放在最后(压过阻尼/脉冲/脱困刹车/积分), 保证近目标不会"越推越摆"。 */
        if (!s_tm_active && fabsf(err) < POS_SOFT_ZONE_DEG && !s_pos_flying) {
            uint8_t quiet   = ((foc->tick - foc->hall.edge_tick) > POS_SOFT_QUIET_TICKS)
                              ? 1U : 0U;
            uint8_t braking = ((fabsf(v) >= POS_CAPTURE_ERPM) &&
                               ((v > 0.0f && iq_target < 0.0f) ||
                                (v < 0.0f && iq_target > 0.0f))) ? 1U : 0U;
            float   lim     = quiet ? POS_SOFT_IQ_MAX_A : POS_SOFT_IQ_A;
            if (braking) lim = POS_DAMP_MAX_A;
            if (iq_target >  lim) iq_target =  lim;
            if (iq_target < -lim) iq_target = -lim;
        }
    }

    /* ????? */
    /* ★10-11k 兜底: 任何非有限/离谱的值都不许进电流环(实测日志出现过
       Iqref=-637.50 的一瞬间 ⇒ 电流环立刻饱和、相电流冲到 9.5A 过流跳闸) */
    if (!(iq_target >= -POS_IQ_CAP_A && iq_target <= POS_IQ_CAP_A)) {
        iq_target = 0.0f;
    }
    /* ★★ 2026-10-11v 见 POS_IQ_STEP_PRELOAD_A: 指令电流大阶跃 ⇒ 直接把电流环的
       q 积分预置到稳态值, 使它 1~5ms 内到位。不预置的话, 位置环想要的
       2.5A 刹车要 250ms 才出得来, 而整个暂态只有几十毫秒 = 等于没刹车。 */
    if (fabsf(iq_target - s_pos_iq_prev) > POS_IQ_STEP_PRELOAD_A) {
        CurrentLoop_SetQInteg(&foc->current_loop,
                              iq_target * (float)foc->torque_dir
                              * POS_FF_DUTY_PER_A);
    }
    s_pos_iq_prev = iq_target;
    foc->current_loop.id_ref = 0.0f;
    foc->current_loop.iq_ref = iq_target;

    /* ★ 位置律计时结束(不含 FOC_UpdateCurrent) */
    {
        uint32_t cyc = DWT->CYCCNT - s_law_t0;
        s_law_cyc_sum += cyc;
        s_law_cyc_cnt++;
        if (cyc > s_law_cyc_max) { s_law_cyc_max = cyc; }
    }

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
        if (foc->tick - foc->ol_last_warn > (200U * FOC_TICKS_PER_MS)) {   /* 200ms 最多一条 */
            foc->ol_last_warn = foc->tick;
            LOGW("[MAN] |I|=%.2fA > %.1fA -> v down to %.4f\n",
                 i_amp, OL_MAN_ILIM, foc->ol_man_v);
        }
    }

    /* 硬保护: 持续大电流 → 关 PWM。即使 v=0 也会触发(下管击穿时的回流路径)。 */
    if (i_amp > OL_MAN_TRIP_A) {
        s_man_oc_cnt++;
        if (s_man_oc_cnt > (OL_MAN_TRIP_MS * FOC_TICKS_PER_MS)) {
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
                                              FOC_TICKS_PER_S,
                                              1.0f,
                                              1,
                                              &erpm);
    if (erpm_out != NULL) *erpm_out = erpm;
}

/* ---- ????? + SVPWM?? ---- */
/* ---- ★ 2026-10-24 到位静音判定(见 PWM_IDLE_OFF_* 的说明) ----
   返回 1 = 当前处于"已关断"。
   抽成函数的两个目的:
     ① 关断期间可以**整段跳过** Clarke/Park/PI/逆Park/SVPWM —— PWM 已经关了,
        没有电流可调, 那 8.3us 全是白花的; 重新使能时会 CurrentLoop_ResetInteg
        清积分, 不会残留旧积分。
     ② 日志改成"只置事件标志", 由主循环打印 —— 实测在中断里 LOGD
        (vsnprintf + RTT 写入) 会把那一拍从 8.3us 拉到 17us。 */
static uint8_t FOC_PwmIdleEval(FOC_Control_t *foc)
{
    uint8_t want_off = 0U;
    float   aerr     = fabsf(foc->position_loop.pos_err_deg);
    /* 滞环: 已关断时用更宽的门才能退出, 防止在门限附近 off/on 反复 */
    float   elim     = (s_pwm_idle_off != 0U) ? PWM_IDLE_OFF_EXIT_DEG
                                              : PWM_IDLE_OFF_DEG;

    if ((s_pwm_idle_en != 0U) &&
        (foc->state == FOC_STATE_POSITION_LOOP) &&
        (foc->position_loop.arrived != 0U) &&
        (s_tm_active == 0U) &&
        (fabsf(foc->current_loop.iq_ref) < PWM_IDLE_OFF_CUR_A) &&
        (aerr < elim)) {
        if (s_pwm_idle_tick == 0U) {
            s_pwm_idle_tick = foc->tick;
        } else if ((foc->tick - s_pwm_idle_tick) >= PWM_IDLE_OFF_ARM_TICKS) {
            want_off = 1U;
        }
    } else {
        s_pwm_idle_tick = 0U;
    }

    if ((want_off != 0U) && (s_pwm_idle_off == 0U)) {
        s_pwm_idle_off = 1U;
        PWM_Output_Disable();
        s_pwm_on = 0U;      /* 关断期间采样无意义 ⇒ 不参与过流判断 */
        s_pwm_evt     = 1U; /* 交给主循环打印 */
        s_pwm_evt_pos = (int32_t)(foc->position_loop.pos_now_deg * 1000.0f);
        s_pwm_evt_err = 0;
    } else if ((want_off == 0U) && (s_pwm_idle_off != 0U)) {
        s_pwm_idle_off = 0U;
        CurrentLoop_ResetInteg(&foc->current_loop);
        s_pwm_evt     = 2U;
        s_pwm_evt_pos = (int32_t)(foc->position_loop.pos_now_deg * 1000.0f);
        s_pwm_evt_err = (int32_t)(foc->position_loop.pos_err_deg * 1000.0f);
    }
    return s_pwm_idle_off;
}

static void FOC_UpdateCurrent(FOC_Control_t *foc)
{
    float ialpha, ibeta, id, iq;
    float valpha, vbeta;
    uint32_t pt0 = DWT->CYCCNT;   /* ★ 拆分计时的 CUR 段起点 */

    /* ★★ 已关断 ⇒ 只做"要不要重新打开"的滞环判定, 电流环整段跳过。
       实测: 到位保持时(po=1) CUR 从 8.34us 降到 ~0.3us,
             整拍从 21.3us 降到 ~13us, 中断负载 43% → ~26%。 */
    if (s_pwm_idle_off != 0U) {
        (void)FOC_PwmIdleEval(foc);
        if (s_pwm_idle_off != 0U) {
            FOC_ProfAcc(DWT->CYCCNT - pt0,
                        &s_cur_cyc_sum, &s_cur_cyc_max, &s_cur_cyc_cnt);
            return;
        }
    }

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
        /* ★ 2026-10-24: dt_s = 1/f 是常量, 原来每拍要除一次 */
        if (s_cur_dt_s <= 0.0f) { s_cur_dt_s = 1.0f / foc->config.pwm_freq; }
        CurrentLoop_Update(&foc->current_loop, id, iq, s_cur_dt_s);
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

    /* ---- ★ 2026-10-24 到位静音判定(见 PWM_IDLE_OFF_* / FOC_PwmIdleEval) ---- */
    if (FOC_PwmIdleEval(foc) != 0U) {
        FOC_ProfAcc(DWT->CYCCNT - pt0,
                    &s_cur_cyc_sum, &s_cur_cyc_max, &s_cur_cyc_cnt);
        return;                 /* 关断中: 不再下发占空比, 也不置 s_pwm_on */
    }

    PWM_Output_Apply(&foc->pwm);
    if (!s_pwm_on) s_pwm_on_tick = foc->tick;   /* 记录开通时刻, 用于过流消隐 */
    s_pwm_on = 1;   /* 三相输出已使能, 之后的采样才有效, 可做过流判断 */

    FOC_ProfAcc(DWT->CYCCNT - pt0,
                &s_cur_cyc_sum, &s_cur_cyc_max, &s_cur_cyc_cnt);
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
    /* ★ 2026-10-11ae: 任何"设目标"(数字键/0~9/z/t)都取消正在跑的定时轨迹 */
    s_tm_active = 0U;
    s_tm_phase  = 0U;
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
    s_pos_vdiff_deg      = AngleObserver_GetMechDeg(&foc->angle, foc->config.pole_pairs);
    /* 2026-10-11r/t: 位置差基准/边沿计数/动-静历史也全部重起 */
    s_pos_win10_edges    = 0U;
    s_pos_edge_seen      = foc->hall.edge_tick;
    s_pos_sect_prev      = foc->angle.sect_run;
    s_pos_sect_tick      = 0U;
    s_pos_move           = 0U;
    s_pos_flying         = 0U;
    s_pos_fly_cnt        = 0U;
    s_pos_jump_cnt       = 0U;
    s_pos_hop_give       = 0U;
    s_pos_hop_cnt        = 0U;
    s_pos_soft_on        = 0U;
    s_pos_soft_dir       = 0;
    s_pos_soft_tick      = 0U;
    s_pos_soft_start_deg = 0.0f;
    s_pos_soft_end_tick  = 0U;
    s_pos_soft_sect0     = foc->angle.sect_run;
    s_pos_soft_etick0    = foc->hall.edge_tick;
    s_pos_iq_prev        = 0.0f;
    s_pos_esc_stalled    = 0U;
    s_pos_brake_end      = 0U;
    s_pos_err_sign       = 0;
    s_pos_err_flip_tick  = 0U;
    s_pos_hold           = 0U;     /* 新目标 ⇒ 退出到位死区 */
    s_pos_hold_in_tick   = 0U;
    s_pos_kick_fail      = 0U;     /* 新目标 ⇒ 允许重新尝试破黏 */
    s_pos_blocked_hard   = 0U;
    if (foc->state == FOC_STATE_POSITION_LOOP) {
        PositionLoop_ResetArrived(&foc->position_loop);
    }
}

/* ---- 2026-10-11ae 定时移动: 目标角度 + "到达所需时间(秒)" ----------
   命令语法(终端/ RTT): `t<角度> d<秒>` + 回车, 例:
     t180 d2      → 走到 180°, 全程 2s(1/9 加速 + 7/9 匀速 + 1/9 减速)
     t-45 d1.5    → 走到 -45°, 1.5s
     t180d2       → 同上(空格可省)
     t180         → 不给 d = 普通移动(不过轨迹)
   用时过短(会超过巡航/加速度上限)时 **自动延长** 并打 [TM] 警告 ——
   与其硬冲过去撞齿槽(前几轮实测: 1000eRPM 撞进齿槽 = 抖动), 不如花够时间。 */
void FOC_Control_SetTimedTarget(FOC_Control_t *foc, float target_deg, float dur_s)
{
    float pos0, dist, D, T, Tmin_a, Tmin_v;

    FOC_Control_SetPosTarget(foc, target_deg);   /* 先走普通换目标(复位一切) */
    if (!(dur_s > 0.0f)) {
        LOGD("[TM] no duration given -> normal move\r\n");
        return;                                   /* 没给 d ⇒ 普通移动 */
    }
    pos0 = AngleObserver_GetMechDeg(&foc->angle, foc->config.pole_pairs);
    dist = foc->pos_target_deg_cmd - pos0;
    D    = fabsf(dist);
    if (D < 1.0f) {
        LOGD("[TM] already at target (D=%.2fdeg) -> normal hold\r\n", D);
        return;
    }
    /* 最短用时: ① 加速度上限 ② 巡航上限。
       ★ 2026-10-11an: 改成 4 段(加速2/9+匀速3/9+减速2/9+逼近2/9)后
       vc ≈ 1.8·D/T、峰值加速度 ≈ 12.7·D/T² ⇒ 下面两个系数就是该近似。 */
    Tmin_a = sqrtf(12.7f * D / POS_TM_ACC_MAX_DEG_S2);
    Tmin_v = 1.8f * D / POS_TM_VMAX_MECH_DEG_S;
    T = (Tmin_a > Tmin_v) ? Tmin_a : Tmin_v;
    if (dur_s < T) {
        /* ★ cal28: 降为 LOGD —— d<合理值 时每按一次都会提示, 算正常行为,
           不该每次刷一条 WARN。真要查就开 verbose。 */
        LOGD("[TM] %.2fs too short for %.1fdeg -> use %.2fs (accel/capture limit)\r\n",
             dur_s, D, T);
    } else {
        T = dur_s;
    }
    s_tm_t0     = 0U;        /* 0 = 还没起步(第一拍位置环里锚定时间轴) */
    s_tm_Ttick  = (uint32_t)(T * (float)FOC_CTRL_HZ + 0.5f);
    s_tm_deg0   = pos0;
    s_tm_sgn    = (dist >= 0.0f) ? 1 : -1;
    /* ★ 2026-10-11an 4 段: 加速 2/9 + 匀速 3/9 + 减速 2/9 + 逼近 2/9。
       由 D = vc·(ta/2+tc+td/2) + vapp·(td/2+tapp) 反解 vc。
       ★★ 2026-10-17 短行程修正: 以前 vc 低于 vapp 时直接把 vc 钳到 vapp,
       但那样**总行程就超过 D 了** —— 例: T=1.5s、D=45° 时钳位后实际走
       40×(ta/2+tc+td/2) + 40×(td/2+tapp) = 40×1.333 = 53.3° > 45°,
       即轨迹自己会多冲 8°。现在改成“短行程时按比例降 vapp”, 两边同时缩,
       保证总行程严格等于 D。 (vc=vapp 时的最小行程 = vapp×1.333s) */
    {
        float ta = T * POS_TM_TA_FRAC;
        float tc = T * POS_TM_TC_FRAC;
        float td = T * POS_TM_TD_FRAC;
        float tapp = T - ta - tc - td;
        float den  = ta * 0.5f + tc + td * 0.5f;
        /* ★ cal19 同理: 逼近段改为 vapp 线性减到 0, 其平均速度是 vapp/2 ⇒
           总行程里的 vapp 系数从 (td/2 + tapp) 变成 (td/2 + tapp/2)。
           dmin_k(= vc=vapp 时的最小行程 / vapp) = ta/2 + tc + td + tapp/2。 */
        float dmin_k = ta * 0.5f + tc + td + tapp * 0.5f;
        float vapp = POS_TM_VAPP_DEG_S;
        float vc;

        if (D < vapp * dmin_k) {
            vapp = D / dmin_k;                          /* 短行程: 等比例降逼近速度 */
            if (vapp < 1.0f) vapp = 1.0f;
        }
        vc = (D - vapp * (td * 0.5f + tapp * 0.5f)) / den;
        if (vc < vapp) vc = vapp;
        s_tm_vapp = vapp;
        s_tm_vc   = vc;
    }
    s_tm_phase  = 1U;
    s_tm_active = 1U;
    s_tm_iq           = 0.0f;   /* 2026-10-11am 模型前馈状态复位 */
    s_tm_vprev       = 0.0f;
    s_tm_vprev_valid = 0U;
    /* ★ 2026-10-11ao 速度观测器复位(位置也锚到当前读数, 免得第一拍就炸) */
    s_tm_vobs        = 0.0f;
    s_tm_posobs      = pos0;
    s_tm_dbg_verr    = 0.0f;   /* 轨迹诊断清零: 免得上一轮的残留值误导日志 */
    s_tm_dbg_ff      = 0.0f;
    s_tm_dbg_trim    = 0.0f;
    s_tm_max_tverr   = 0.0f;   /* ★ 本次轨迹的峰值指标 */
    s_tm_max_iq      = 0.0f;
    s_tm_max_drive   = POS_TM_IQ_DRIVE_DEF_A;
    LOGD("[TM] %.1f -> %.1fdeg (D=%.1f) in %.2fs | accel %.2fs + cruise %.2fs + decel %.2fs + creep %.2fs | v_cruise=%.1f -> v_app=%.1f deg/s\r\n",
         pos0, foc->pos_target_deg_cmd, D, T,
         T * POS_TM_TA_FRAC, T * POS_TM_TC_FRAC, T * POS_TM_TD_FRAC,
         T * (1.0f - POS_TM_TA_FRAC - POS_TM_TC_FRAC - POS_TM_TD_FRAC),
         s_tm_vc, s_tm_vapp);
}

/* 0 = 未在跑定时轨迹; 1..9 = 当前处于第几份(1=加速, 9=减速) */
uint8_t FOC_Control_GetTimedPhase(void)
{
    return s_tm_active ? s_tm_phase : 0U;
}

/* ★ 2026-10-11ao 轨迹诊断(仅日志用): tv=轨迹位置误差(°), ff=摩擦前馈(A),
   tr=位置微调+阻尼(A), vob=观测速度(°/s)。判断“环是不是又在饱和打摆”。 */
void FOC_Control_GetTrajDiag(float *verr, float *iq_ff, float *iq_trim, float *vobs)
{
    if (verr    != 0) *verr    = s_tm_dbg_verr;
    if (iq_ff   != 0) *iq_ff   = s_tm_dbg_ff;
    if (iq_trim != 0) *iq_trim = s_tm_dbg_trim;
    if (vobs    != 0) *vobs    = s_tm_vobs;
}

/* 位置环积分项(A) — 只给日志看, 判断"是不是靠积分顶着静摩擦" */
float FOC_Control_GetPosIntegA(void)
{
    return s_pos_integ;
}

/* ★ 2026-10-11r 诊断: 1 = 50ms 窗判定"真的在动"; 0 = 判定已停(速度被当 0 用) */
uint8_t FOC_Control_GetPosMove(void)
{
    return s_pos_move;
}

/* ★ 2026-10-11r 诊断: 位置读数被判为"跳变"而丢弃的次数(0 = 读数全程可信) */
uint16_t FOC_Control_GetPosJumpCnt(void)
{
    return s_pos_jump_cnt;
}

/* ★ 2026-10-11r 诊断: 50ms 窗口净位移测速(eRPM) — 唯一可信的速度 */
float FOC_Control_GetVelWinErpm(void)
{
    return s_pos_v_sect;
}

/* ★★ 2026-10-11aa 诊断: 三个"静默锁" —— 用来区分"环路在等"还是"环路已放弃"
   (实测日志里 Iqref=-0.00 + 一个齿槽静差时, 光看 MAIN 行分不清是哪一把锁)。 */
uint8_t FOC_Control_GetPosHoldFlag(void)
{
    return s_pos_hold;          /* 1 = 在到位死区里(撤力) */
}

uint8_t FOC_Control_GetPosGiveFlag(void)
{
    return s_pos_hop_give;      /* 1 = 爬行认输中 */
}

uint8_t FOC_Control_GetPosBlockFlag(void)
{
    return s_pos_blocked_hard;  /* 1 = 判定"力矩没加到转子上" */
}

/* ★ 2026-10-11ah 诊断: 1 = 正在"近目标松手静默"(跨过边界后强行撤力 250ms) */
uint8_t FOC_Control_GetPosSettleFlag(void)
{
    return s_pos_settling;
}

uint16_t FOC_Control_GetPosHopCnt(void)
{
    return (uint16_t)s_pos_hop_cnt;   /* 本轮爬行的连续失败次数 */
}

/* 位置环阻尼用的滤波速度(eRPM) — 只给日志看, 判断振荡/蠕动幅度 */
float FOC_Control_GetPosErpm(void)
{
    return s_pos_erpm_filt;
}

/* 2026-10-11o 诊断: “位置差分”那一路算出来的原始测速(eRPM)。
   专门用来定位“转子明明停住、而 vsp 却很大”到底是哪一路来的:
   观测器(ob)/位置差分(vp)/两者取最大后的 vsp, 一对比就清楚。 */
float FOC_Control_GetVelPosErpm(void)
{
    return s_pos_vpos_raw;
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
    uint32_t n = dur_ms * FOC_TICKS_PER_MS;   /* 控制拍 */
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
    LOGD("[FAULT] code=0x%04X iu=%dmA iv=%dmA iw=%dmA state=%d vb=%dmV tick=%u\n",
         code, (int)(foc->current.u * 1000.0f), (int)(foc->current.v * 1000.0f),
         (int)(foc->current.w * 1000.0f),
         (int)foc->state, (int)(foc->current.vbus * 1000.0f), foc->tick);
}

FOC_State_t FOC_Control_GetState(const FOC_Control_t *foc)
{
    return foc->state;
}

uint8_t FOC_Control_IsFault(const FOC_Control_t *foc)
{
    return foc->flags.fault;
}

/* ==========================================================================
 * ★★★ 2026-10-24 应用层接口实现(声明/说明见 foc_app.h)
 * --------------------------------------------------------------------------
 * 三层结构, 上层只需要最上面那三个:
 *   ① FOC_App_SetTuning()  设「力矩档 + 速度档」
 *   ② FOC_App_MoveTo()     发一条移动命令
 *   ③ FOC_App_GetHw()      读硬件链路实测参数(运放增益/量程/分辨率)
 * ========================================================================== */
static FOC_Control_t *s_app_foc = 0;

void FOC_App_Init(FOC_Control_t *foc)
{
    s_app_foc = foc;

    /* 回到默认档 —— 与改动前完全一致的数值 */
    FOC_App_SetIqCap(FOC_APP_TORQUE_DEF_A);
    FOC_App_SetSpeedInt(FOC_APP_SPEED_DEF_DEG_S);
}

uint8_t FOC_App_SetTuning(const FOC_AppTuning_t *cfg)
{
    if ((cfg == 0) || (s_app_foc == 0)) { return 0U; }

    if (cfg->torque_a > 0.0f)    { FOC_App_SetIqCap((float)cfg->torque_a); }
    if (cfg->speed_deg_s > 0.0f) { FOC_App_SetSpeedInt((float)cfg->speed_deg_s); }
    return 1U;
}

void FOC_App_GetTuning(FOC_AppTuning_t *out)
{
    if (out == 0) { return; }
    out->torque_a    = s_iq_cap_a;
    out->speed_deg_s = s_vmax_deg_s;
}

void FOC_App_SetTorque(float amp)  { FOC_App_SetIqCap(amp); }
float FOC_App_GetTorque(void)      { return s_iq_cap_a; }
void FOC_App_SetSpeed(float deg_s) { FOC_App_SetSpeedInt(deg_s); }
float FOC_App_GetSpeed(void)       { return s_vmax_deg_s; }

float FOC_App_GetIqLimitDrive(void) { return s_iq_drive_cap; }
float FOC_App_GetIqLimitTrim(void)  { return s_trim_cap; }
float FOC_App_GetIqLimitDamp(void)  { return s_tm_damp_cap; }
float FOC_App_GetIqLimitKick(void)  { return s_kick_cap; }

/* 取走上一段定时轨迹的客观指标(主循环调, 读即清)。
   返回 1 = 有新数据。单位: 0.001° / 0.001A */
uint8_t FOC_App_TakeMoveStats(int32_t *max_tv_mdeg, int32_t *max_iq_ma,
                              int32_t *band_ma, int32_t *drive_ma)
{
    uint8_t ev;

    __disable_irq();
    ev = s_tm_done_evt;
    s_tm_done_evt = 0U;
    if (max_tv_mdeg != 0) { *max_tv_mdeg = s_tm_done_tv; }
    if (max_iq_ma    != 0) { *max_iq_ma    = s_tm_done_iq; }
    if (band_ma      != 0) { *band_ma      = s_tm_done_band; }
    if (drive_ma     != 0) { *drive_ma     = s_tm_done_drive; }
    __enable_irq();
    return ev;
}

/* 按「行程 + 速度」反算总时长。
   4 段轨迹(加速2/9 + 匀速3/9 + 减速2/9 + 逼近2/9)的巡航速度满足
       vc ≈ 1.8 × 行程 / 总时长
   ⇒ 总时长 T = 1.8 × 行程 / 速度; 再与"加速度上限"给出的最短用时取大。
   若给定速度 > 速度档, 用速度档(避免超出机械/齿槽能力)。 */
float FOC_App_EstimateTime(float dist_deg, float speed_deg_s)
{
    float D = fabsf(dist_deg);
    float v = (speed_deg_s > 0.0f) ? speed_deg_s : s_vmax_deg_s;
    float T, Tmin_a;

    if (D < 1.0f) { return 0.0f; }          /* 已在目标上 */
    if (v < 1.0f) { v = 1.0f; }
    if (v > s_vmax_deg_s) { v = s_vmax_deg_s; }

    T      = 1.8f * D / v;
    Tmin_a = sqrtf(12.7f * D / s_accmax_deg_s2);
    if (T < Tmin_a) { T = Tmin_a; }
    if (T < 0.20f)  { T = 0.20f; }          /* 再短就没有意义了 */
    return T;
}

uint8_t FOC_App_MoveTo(float target_deg, float speed_deg_s, float dur_s)
{
    float vmax_saved, acc_saved;
    float pos0, D, T;

    if (s_app_foc == 0) { return 0U; }
    if (!((target_deg > -100000.0f) && (target_deg < 100000.0f))) { return 0U; }

    vmax_saved = s_vmax_deg_s;
    acc_saved  = s_accmax_deg_s2;
    if (speed_deg_s > 0.0f) {
        FOC_App_SetSpeedInt((float)speed_deg_s);   /* 本动临时改速度档 */
    }

    if (dur_s > 0.0f) {
        /* 给了时长 ⇒ 时长优先(与原 `t<deg> d<sec>` 语义一致) */
        FOC_Control_SetTimedTarget(s_app_foc, target_deg, dur_s);
    } else {
        pos0 = AngleObserver_GetMechDeg(&s_app_foc->angle,
                                        s_app_foc->config.pole_pairs);
        D = fabsf(target_deg - pos0);
        T = FOC_App_EstimateTime(D, 0.0f);          /* 用当前速度档 */
        if (T <= 0.0f) {
            /* 读数上已经在目标上 ⇒ 仍然设目标(交给普通律去微调/保持),
               但返回 0 让上层知道"没有启动一段轨迹"。 */
            FOC_Control_SetPosTarget(s_app_foc, target_deg);
            s_vmax_deg_s    = vmax_saved;
            s_accmax_deg_s2 = acc_saved;
            return 0U;
        }
        FOC_Control_SetTimedTarget(s_app_foc, target_deg, T);
    }

    s_vmax_deg_s    = vmax_saved;                    /* 恢复速度档 */
    s_accmax_deg_s2 = acc_saved;

    /* 一键启动: 不在闭环里就自动走一次 ALIGN(与 app 层 AutoStartIfIdle 一致) */
    if ((s_app_foc->state == FOC_STATE_IDLE) ||
        (s_app_foc->state == FOC_STATE_FAULT)) {
        s_app_foc->flags.fault   = 0U;
        s_app_foc->fault_code    = 0U;
        s_app_foc->flags.enabled = 1U;
        s_app_foc->align_tick    = 0U;
        s_app_foc->startup_tick  = 0U;
        FOC_Control_SetState(s_app_foc, FOC_STATE_ALIGN);
    }
    return 1U;
}

void FOC_App_GetHw(uint16_t zero_counts, FOC_AppHw_t *out)
{
    float shunt, gain, vref, res, v_per_a, v_zero;

    if (out == 0) { return; }
    if (s_app_foc == 0) { return; }

    shunt = s_app_foc->config.shunt_r;
    gain  = s_app_foc->config.op_gain;
    vref  = s_app_foc->config.vref;
    res   = (float)s_app_foc->config.adc_resolution;

    v_per_a = shunt * gain;              /* V/A */
    if (v_per_a < 1.0e-6f) { v_per_a = 1.0e-6f; }
    v_zero  = ((float)zero_counts) * vref / (res - 1.0f);

    out->shunt_ohm = shunt;
    out->amp_gain  = gain;
    out->v_per_a   = v_per_a;
    out->a_per_lsb = vref / (res - 1.0f) / v_per_a;
    out->v_zero    = v_zero;
    /* 运放输出可摆到接近 0V 和接近 Vref(轨到轨), 所以:
         正方向可测 = (Vref − V_zero) / (V/A)
         负方向可测 =  V_zero        / (V/A)            */
    out->range_neg_a = v_zero / v_per_a;
    out->range_pos_a = (vref - v_zero) / v_per_a;
}


