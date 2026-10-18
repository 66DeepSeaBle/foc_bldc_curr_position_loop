/**
  ******************************************************************************
  * @file    app.c
  * @brief   应用层: 参数装配、零电流标定、按键、日志上报、霍尔验证、波形节拍。
  ******************************************************************************
  *
  *  这里**不出现任何 HAL 函数或寄存器名** —— 硬件访问全部走 bsp.h,
  *  位置/速度全部走 sensor.h。所以换芯片只改 bsp_xxx.c, 换传感器只改
  *  sensor_xxx.c, 本文件不用动。
  *
  *  结构(自上而下):
  *    App_Init()            ← main() 调一次
  *    App_Loop()            ← main() 的 while(1) 里反复调
  *      ├ App_PollKeys()    按键 + t<角度>d<秒> 命令缓冲
  *      ├ App_ReportSlow()  [ADCR]/[MAN]/[OL] 低频日志
  *      ├ HallVerify_Poll() 手动盘轴霍尔验证(可选模式)
  *      └ App_ReportMain()  [MAIN] 状态行 + [ANG]
  *    App_OnCurrentSample() ← 20kHz 中断钩子
  *    App_OnWaveTick()      ← 1kHz  中断钩子
  *
  ******************************************************************************
  */
#include "app.h"
#include "board_config.h"
#include "bsp.h"
#include "sensor.h"
#include "foc_control.h"
#include "vofa.h"
#include "log.h"
#include "SEGGER_RTT.h"

#include <math.h>
#include <stdio.h>

/* 固件版本标签: 开机打一次, 并跟着 [ADCR] 每秒打一次 —— 只截日志中段也能确认
   跑的是哪一版(反复吃过"烧的不是当前源码"的亏)。 */
#define FW_BUILD_TAG  "2026-10-18 cal30 quiet120 vapp28 tm085"

/* 零电流标定采样点数 */
#define APP_CAL_SAMPLES   128U

/* ==========================================================================
 * 应用状态
 * ========================================================================== */
static FOC_Control_t g_foc;

/* 相电流零点(ADC 码)与"标定完成"标志 */
static uint16_t s_off_u = 0U, s_off_v = 0U, s_off_w = 0U;
static uint8_t  s_off_ok = 0U;

/* 20kHz 控制环门控: 零电流标定完成前不跑(否则会拿全 0 偏置去算电流 → 误报过流) */
static volatile uint8_t s_foc_loop_en = 0U;

/* 日志节流状态 */
static uint32_t s_last_main_ms = 0U;
static uint32_t s_last_adcr_ms = 0U;
static uint32_t s_last_ol_ms   = 0U;
static uint32_t s_last_ang_ms  = 0U;
static uint32_t s_log_seq      = 0U;
static float    s_last_mech    = 1.0e9f;
static float    s_last_tgt     = 1.0e9f;
static uint8_t  s_last_hall    = 0U;
static uint8_t  s_log_verbose  = 0U;

/* 手动盘轴霍尔验证模式(电机不通电) */
static volatile uint8_t s_hall_verify = 0U;

/* 相序邻接表(与 angle_observer.c 一致) */
static const uint8_t H_FWD[8] = { 0, 3, 6, 2, 5, 1, 4, 0 };
static const uint8_t H_BWD[8] = { 0, 5, 3, 1, 6, 4, 2, 0 };

typedef struct {
    uint8_t  prev_st;      /* 上一次霍尔扇区 */
    uint32_t t_cnt;        /* 总跳变次数 */
    uint32_t fwd_cnt;      /* 落在正向邻格的次数 */
    uint32_t bwd_cnt;      /* 落在反向邻格的次数 */
    uint32_t bad_cnt;      /* 0/7 或跨格/丢步次数 */
    float    total_deg;    /* 按判定方向累计电角度(deg) */
    uint32_t last_live;    /* 上次实时行打印时间 */
} HallVerify_t;
static HallVerify_t g_hv;

/* `t<角度> d<秒>` 命令缓冲 */
static char    s_cmd_buf[24];
static uint8_t s_cmd_len     = 0U;
static uint8_t s_cmd_pending = 0U;   /* 1 = 正在收命令(首字符 t/T) */

/* ==========================================================================
 * 1) 参数装配 —— 全部来自 board_config.h
 * ========================================================================== */
static void App_ConfigDefaults(FOC_Config_t *config)
{
    config->pwm_freq      = CFG_PWM_FREQ_HZ;
    config->vref          = CFG_ADC_VREF;
    config->shunt_r       = CFG_SHUNT_R_OHM;
    config->op_gain       = CFG_OPAMP_GAIN;
    config->adc_resolution = CFG_ADC_RESOLUTION;
    config->current_scale = CFG_SHUNT_R_OHM * CFG_OPAMP_GAIN;
    config->current_sign  = CFG_CURRENT_SIGN;
    config->vbus_divider  = CFG_VBUS_DIVIDER;
    config->pole_pairs    = CFG_MOTOR_POLE_PAIRS;
    config->hall_dir_rev  = CFG_HALL_DIR_REV;
    config->hall_offset_deg = 0.0f;
    config->overcurrent_amp = CFG_OVER_CURRENT_A;
    config->align_current   = CFG_ALIGN_CURRENT_A;
    config->startup_current = CFG_STARTUP_CURRENT_A;
    config->startup_hz      = CFG_STARTUP_HZ;
    config->iq_ramp_rate    = CFG_IQ_RAMP_A_S;

    /* 死区补偿 = 正反馈陷阱, 实测必须关掉(拿测到的电流符号去修正电压, 会自激):
       实测 v=0(三相同占空比, 绕组短接)时 du/dv/dw 被修成 0.520/0.520/0.480,
       且 |I| 从 8.75A 涨到 17.9A ⇒ 一律禁用。 */
    config->deadtime_ns = 0.0f;
}

/* 开机关键标定值打印: 一眼确认烧的是哪一版、哪个分压比/极对数 */
static void App_PrintCalib(void)
{
    LOGD("[FW] vbus_div=%.1f vref=%.2f shunt=%.4fOhm gain=%.1f pp=%d dir_rev=%d align=%.1fA startup=%.1fA\r\n",
         g_foc.config.vbus_divider, g_foc.config.vref, g_foc.config.shunt_r,
         g_foc.config.op_gain, (int)g_foc.config.pole_pairs,
         (int)g_foc.config.hall_dir_rev,
         g_foc.config.align_current, g_foc.config.startup_current);
}

/* ==========================================================================
 * 2) 零电流偏置标定
 *    必须在 PWM / ADC 注入 / 运放都启动之后: 0% 占空比 = 三个下管全通, 把
 *    采样电阻下端接到 GND, 得到确定的 0A 状态, 运放输出即零点偏置(≈ADC 中值)。
 * ========================================================================== */
static void App_CalibrateOffsets(void)
{
    uint16_t ru = 0U, rv = 0U, rw = 0U;
    uint32_t sum_u = 0U, sum_v = 0U, sum_w = 0U;
    uint16_t i;

    BSP_PWM_SetDuty3(0.0f, 0.0f, 0.0f);   /* 下管制动: 0A 基准 */
    BSP_Time_DelayMs(50);                 /* 等运放/采样稳定 */

    for (i = 0U; i < APP_CAL_SAMPLES; i++) {
        BSP_ADC_GetCurrentRaw(&ru, &rv, &rw);
        sum_u += ru;
        sum_v += rv;
        sum_w += rw;
        BSP_Time_DelayMs(1);
    }
    s_off_u = (uint16_t)(sum_u / APP_CAL_SAMPLES);
    s_off_v = (uint16_t)(sum_v / APP_CAL_SAMPLES);
    s_off_w = (uint16_t)(sum_w / APP_CAL_SAMPLES);
    s_off_ok = 1U;

    LOGD("[Calib] Offset: U=%u V=%u W=%u (zero-current)\r\n",
         s_off_u, s_off_v, s_off_w);
}

/* ==========================================================================
 * 3) 一键启动: 不在闭环里(含 IDLE/FAULT)时自动做一次 "r"
 * ========================================================================== */
static void AutoStartIfIdle(void)
{
    if (g_foc.state == FOC_STATE_IDLE || g_foc.state == FOC_STATE_FAULT) {
        g_foc.flags.fault   = 0U;
        g_foc.fault_code    = 0U;
        g_foc.flags.enabled = 1U;
        g_foc.align_tick    = 0U;      /* 让 DoAlign 重新计时 */
        g_foc.startup_tick  = 0U;
        /* 注意: 不能清 pos_target_user_set —— 用户刚设的目标要保留 */
        FOC_Control_SetState(&g_foc, FOC_STATE_ALIGN);
        LOGD("[CMD] auto-start -> ALIGN (or press 'r')\n");
    }
}

/* ==========================================================================
 * 4) 手动盘轴霍尔验证模式(纯手动, 电机不通电)
 *    作用: ① U/V/W 与合成扇区是否正常(不应出现 0/7);
 *          ② 跳变是否都在合法相序表上(bad 应为 0);
 *          ③ 电角度方向对不对(elec 递增方向 ↔ 期望正转方向);
 *          ④ 手转一整圈后 pole_pairs = (fwd+bwd)/6, 可核对极对数。
 * ========================================================================== */
static void HallVerify_Enter(void)
{
    g_foc.flags.fault   = 0U;
    g_foc.flags.enabled = 0U;
    g_foc.fault_code    = 0U;
    g_foc.state         = FOC_STATE_IDLE;
    PWM_Output_Disable();                  /* 关断驱动, 确保盘轴时不通电 */
    s_hall_verify       = 1U;

    g_hv.prev_st   = BSP_Hall_ReadSector(0, 0, 0);
    g_hv.t_cnt     = 0U;
    g_hv.fwd_cnt   = 0U;
    g_hv.bwd_cnt   = 0U;
    g_hv.bad_cnt   = 0U;
    g_hv.total_deg = 0.0f;
    g_hv.last_live = 0U;

    LOGD("\r\n[HALL] ==== Manual hall-spin verify (drive DISABLED) ====\r\n");
    LOGD("[HALL] Spin the rotor slowly & evenly by hand.\r\n");
    LOGD("[HALL] Watch st=1..6 sequence and U V W; no 0/7 expected.\r\n");
    LOGD("[HALL] elec = angle, current code convention (deg, 0..360)\r\n");
    LOGD("[HALL] rev  = mirrored angle if hall order were reversed\r\n");
    LOGD("[HALL] total = accumulated elec deg; spin EXACTLY one turn and\r\n");
    LOGD("[HALL]         pole_pairs = (fwd+bwd)/6  -> set cfg pole_pairs\r\n");
    LOGD("[HALL] bad should stay 0 (0/7 or skipped sector = wiring/order).\r\n");
    LOGD("[HALL] ONLY 'q' works in this mode -> press 'q' to exit/re-align.\r\n");
}

static void HallVerify_Exit(void)
{
    s_hall_verify       = 0U;
    g_foc.flags.fault   = 0U;
    g_foc.flags.enabled = 1U;
    FOC_Control_SetState(&g_foc, FOC_STATE_ALIGN);
    LOGD("[HALL] Verify exit -> ALIGN again.\r\n");
}

static void HallVerify_Poll(uint32_t now)
{
    const float R2D = 57.29577951f;
    uint8_t ru = 0U, rv = 0U, rw = 0U, st, p;
    float elec, rev;

    st = BSP_Hall_ReadSector(&ru, &rv, &rw);

    /* 观测电角度: 与正式运行共用同一套观测器(蠕动已抑制) */
    elec = g_sensor.GetElecThetaRad(&g_foc) * R2D;
    rev  = 360.0f - elec;
    if (rev >= 360.0f) rev -= 360.0f;
    if (rev < 0.0f)    rev += 360.0f;

    if (st != g_hv.prev_st) {
        p = g_hv.prev_st;
        g_hv.t_cnt++;
        if (st == 0U || st == 7U || p == 0U || p == 7U) {
            g_hv.bad_cnt++;
            LOGD("[HALL] #%u U%uV%uW%u st=%u  BAD(0/7)\r\n",
                 (unsigned)g_hv.t_cnt, ru, rv, rw, st);
        } else if (st == H_FWD[p]) {
            g_hv.fwd_cnt++;
            g_hv.total_deg += 60.0f;
            LOGI("[HALL] #%u U%uV%uW%u st=%u  %u->%u FWD elec=%.1f\r\n",
                 (unsigned)g_hv.t_cnt, ru, rv, rw, st, p, st, elec);
        } else if (st == H_BWD[p]) {
            g_hv.bwd_cnt++;
            g_hv.total_deg -= 60.0f;
            LOGI("[HALL] #%u U%uV%uW%u st=%u  %u->%u BWD elec=%.1f\r\n",
                 (unsigned)g_hv.t_cnt, ru, rv, rw, st, p, st, elec);
        } else {
            g_hv.bad_cnt++;
            LOGI("[HALL] #%u U%uV%uW%u st=%u  %u->%u GAP(bad)\r\n",
                 (unsigned)g_hv.t_cnt, ru, rv, rw, st, p, st);
        }
        g_hv.prev_st = st;
    }

    if ((now - g_hv.last_live) >= 250U) {
        g_hv.last_live = now;
        LOGD("[HALL] U%uV%uW%u st=%u syn=%d elec=%.1f rev=%.1f fwd=%u bwd=%u bad=%u total=%.0fdeg\r\n",
             ru, rv, rw, st, (int)g_foc.angle.synced, elec, rev,
             (unsigned)g_hv.fwd_cnt, (unsigned)g_hv.bwd_cnt,
             (unsigned)g_hv.bad_cnt, g_hv.total_deg);
    }
}

/* ==========================================================================
 * 5) 命令解析: `t<角度> d<秒>` + 回车
 *    必须"回车才解析": t 与 d 是同一时刻的参数。若边收边执行, t 先到就会按
 *    普通移动起步, 等 d 到了再改时长 = 边走边改, 轨迹就不再是一条曲线了。
 * ========================================================================== */
static const char *Cmd_Num(const char *p, float *out)
{
    float sgn = 1.0f, val = 0.0f, frac = 0.0f, div = 1.0f;

    if (*p == '+') {
        p++;
    } else if (*p == '-') {
        sgn = -1.0f;
        p++;
    }
    while (*p >= '0' && *p <= '9') {
        val = val * 10.0f + (float)(*p - '0');
        p++;
    }
    if (*p == '.') {
        p++;
        while (*p >= '0' && *p <= '9') {
            div *= 10.0f;
            frac += (float)(*p - '0') / div;
            p++;
        }
    }
    *out = sgn * (val + frac);
    return p;
}

/* 1 = 是定时命令(已填 tgt/dur); 0 = 不是(例如只按了 't') */
static uint8_t Cmd_ParseTimed(const char *buf, float *tgt, float *dur)
{
    const char *p = buf;
    uint8_t have_t = 0U;

    *tgt = 0.0f;
    *dur = 0.0f;
    while (*p != '\0') {
        char c = *p;
        if (c == 't' || c == 'T' || c == 'd' || c == 'D') {
            const char *q  = p + 1;
            const char *q0 = q;
            float v = 0.0f;
            if (*q == '+' || *q == '-') q++;
            if ((*q >= '0' && *q <= '9') || *q == '.') {
                q = Cmd_Num(q0, &v);
                if (c == 't' || c == 'T') {
                    *tgt    = v;
                    have_t  = 1U;
                } else {
                    *dur = v;
                }
                p = q - 1;              /* 循环末尾会 p++ */
            }
        }
        p++;
    }
    return have_t;
}

/* 把字符喂给命令缓冲。返回 1 = 已消费(不再走单键处理) */
static uint8_t App_FeedCommand(char ch)
{
    if (!s_cmd_pending && !s_hall_verify && (ch == 't' || ch == 'T')) {
        s_cmd_pending = 1U;
        s_cmd_len     = 0U;
        s_cmd_buf[0]  = '\0';
    }
    if (!s_cmd_pending) {
        return 0U;
    }

    if (ch == '\r' || ch == '\n') {
        float tgt = 0.0f, dur = 0.0f;
        if (Cmd_ParseTimed(s_cmd_buf, &tgt, &dur)) {
            LOGD("[CMD] timed: t%.1f d%.2f\r\n", tgt, dur);
            FOC_Control_SetTimedTarget(&g_foc, tgt, dur);
            AutoStartIfIdle();
        } else if (s_cmd_len == 1U && (s_cmd_buf[0] == 't' || s_cmd_buf[0] == 'T')) {
            /* 只按 't' + 回车 = 原来的"停止开环测试"语义 */
            if (g_foc.ol_active) {
                FOC_Control_StopOpenLoopTest(&g_foc, "key T");
            } else {
                LOGD("[CMD] no open-loop test running\n");
            }
        } else if (s_cmd_len > 0U) {
            LOGD("[CMD] unknown command \"%s\" (use: t<deg> d<sec> + Enter)\r\n",
                 s_cmd_buf);
        }
        s_cmd_pending = 0U;
        s_cmd_len     = 0U;
        s_cmd_buf[0]  = '\0';
        return 1U;
    }
    if ((ch == 8 || ch == 127) && s_cmd_len > 0U) {   /* 退格 */
        s_cmd_buf[--s_cmd_len] = '\0';
        return 1U;
    }
    if (ch >= 32 && ch < 127 && s_cmd_len < (uint8_t)(sizeof(s_cmd_buf) - 1U)) {
        s_cmd_buf[s_cmd_len++] = ch;
        s_cmd_buf[s_cmd_len]   = '\0';
    }
    return 1U;
}

/* ==========================================================================
 * 6) 单键处理(每个键一个分支, 保持"一键一功能")
 * ========================================================================== */
static void App_HandleKey(int ch)
{
    if (s_hall_verify) {
        if (ch == 'q' || ch == 'Q') {
            HallVerify_Exit();
        }
        return;                      /* 验证模式下只认 'q' */
    }

    if (ch == 'c' || ch == 'C') {
        HallVerify_Enter();
    } else if (ch == 'v' || ch == 'V') {
        /* 详细日志开关: 默认 500ms/条(不动) / 100ms/条(在动), v 切 25ms/条 */
        s_log_verbose = s_log_verbose ? 0U : 1U;
        LOGD("[CMD] verbose log %s (%ums/line)\r\n",
             s_log_verbose ? "ON" : "OFF",
             (unsigned)(s_log_verbose ? CFG_LOG_VERBOSE_MS : CFG_LOG_IDLE_MS));
    } else if (ch == 'w' || ch == 'W') {
        /* VOFA 波形输出开关 */
        VOFA_SetEnable(VOFA_GetEnable() ? 0U : 1U);
        LOGD("[CMD] VOFA wave %s (frames=%lu drops=%lu)\r\n",
             VOFA_GetEnable() ? "ON" : "OFF",
             (unsigned long)VOFA_GetFrameCount(),
             (unsigned long)VOFA_GetDropCount());
    } else if (ch == 'D') {
        /* 只认大写 'D': 小写 d 与命令 `t180 d2` 里的 d 同字, 误触发会把整个
           方向翻反(实测 ALIGN 偏移从 156.1° 变 203.9°, 之后全部失控)。 */
        g_foc.angle.hall_dir_rev  = g_foc.angle.hall_dir_rev ? 0U : 1U;
        g_foc.config.hall_dir_rev = g_foc.angle.hall_dir_rev;
        LOGD("[CMD] dir_rev -> %d (torque=%d)\n",
             (int)g_foc.angle.hall_dir_rev, (int)g_foc.torque_dir);
    } else if (ch == 's' || ch == 'S') {
        FOC_Control_StartOpenLoopTest(&g_foc, CFG_OL_TEST_HZ, CFG_OL_TEST_AMP_A,
                                      CFG_OL_TEST_MS);
    } else if (ch == 't' || ch == 'T') {
        if (g_foc.ol_active) {
            FOC_Control_StopOpenLoopTest(&g_foc, "key T");
        } else {
            LOGD("[CMD] no open-loop test running\n");
        }
    } else if (ch == 'y' || ch == 'Y') {
        g_foc.torque_dir = (int8_t)-g_foc.torque_dir;
        LOGD("[CMD] torque_dir -> %d (dir=%d)\n",
             (int)g_foc.torque_dir, (int)g_foc.angle.hall_dir_rev);
    } else if (ch == 'o' || ch == 'O') {
        FOC_Control_StartManualTest(&g_foc);
    } else if (ch == 'u' || ch == 'U') {
        FOC_Control_ManualAdjust(&g_foc, +0.01f, 0.0f);
    } else if (ch == 'i' || ch == 'I') {
        FOC_Control_ManualAdjust(&g_foc, -0.01f, 0.0f);
    } else if (ch == 'n' || ch == 'N') {
        FOC_Control_ManualAdjust(&g_foc, +0.05f, 0.0f);
    } else if (ch == 'm' || ch == 'M') {
        FOC_Control_ManualAdjust(&g_foc, -0.05f, 0.0f);
    } else if (ch == 'h' || ch == 'H') {
        FOC_Control_ManualAdjust(&g_foc, 0.0f, +1.0f);
    } else if (ch == 'l' || ch == 'L') {
        FOC_Control_ManualAdjust(&g_foc, 0.0f, -1.0f);
    } else if (ch == 'p' || ch == 'P') {
        FOC_Control_StartPwmSmokeTest(&g_foc);      /* 三相 50%, 不转, 按 t 停 */
    } else if (ch == 'f' || ch == 'F') {
        FOC_Control_FlipOpenLoopDir(&g_foc);
    } else if (ch == 'r' || ch == 'R') {
        /* 清故障 + 重新 ALIGN。单独按 'r' = "原地重新对齐并保持"(目标清零) */
        g_foc.flags.fault   = 0U;
        g_foc.fault_code    = 0U;
        g_foc.flags.enabled = 1U;
        g_foc.align_tick    = 0U;
        g_foc.startup_tick  = 0U;
        g_foc.pos_target_user_set = 0U;
        FOC_Control_SetState(&g_foc, FOC_STATE_ALIGN);
        LOGD("[CMD] fault cleared -> ALIGN (target reset to hold)\n");
    } else if (ch == 'z' || ch == 'Z') {
        FOC_Control_SetTimedTarget(&g_foc, 0.0f, FOC_POS_DEFAULT_TM_S);
        LOGD("[CMD] Zero position\n");
        AutoStartIfIdle();
    } else if (ch == 'j' || ch == 'J') {
        FOC_Control_SetTorqueProbe(&g_foc, +2.5f, 600);   /* +Iq 探针 */
    } else if (ch == 'k' || ch == 'K') {
        FOC_Control_SetTorqueProbe(&g_foc, -2.5f, 600);   /* -Iq 探针 */
    } else if (ch >= '0' && ch <= '9') {
        /* 数字键 = 45°×n。
           ★★★ cal20 (2026-10-18) —— **一律走定时轨迹**。
           原来 `|de| > 360.5°` 就退回 `SetPosTarget`(老的单扇区爬行律,
           Iqref 在 ±2.5A 之间打摆)。而转子在 360° 附近几乎总是停在
           360.6~361.6°(超调), 于是按 `0` 时 de = -361.x° **必然** > 360.5
           ⇒ 每次都掉进老律 —— 这就是"输入 0 转动不平滑"的唯一原因。
           日志铁证(好几个版本都一样):
             [CMD] Set target 0.0deg | now=360.65 err=-360.65 -> moving
           上面**没有 [TM] 行** ⇒ 轨迹根本没启动。
           SetTimedTarget 自己会处理 D<1°(already at target) 与 T 太短的情况,
           所以不需要这个门。 */
        float target = (float)(ch - '0') * 45.0f;
        float now_deg = g_sensor.GetMechDeg(&g_foc);
        float de = target - now_deg;
        /* ★ cal20: 时间按行程比例给, 让"巡航速度"大致恒定, 而不是固定 1.5s
           (固定 1.5s 时 360° 只有 416°/s、45° 只有 38°/s, 都在齿槽"一顿一顿"
            的区间; 速度越慢越一顿)。分档参考:
              |de|=360° ⇒ 1.00s ⇒ v≈632°/s    ← 用户实测"平滑很多"
              |de|= 90° ⇒ 0.35s ⇒ v≈410°/s
              |de|= 45° ⇒ 0.35s ⇒ v≈250°/s
           短行程到不了高速是"4 段轨迹 + 加速度上限 8000°/s²"的物理限制,
           不是参数能解决的(要根治只能加齿槽补偿表或换磁编码器)。 */
        {
            float dur = fabsf(de) / 360.0f;
            if (dur < 0.35f) dur = 0.35f;
            if (dur > FOC_POS_DEFAULT_TM_S) dur = FOC_POS_DEFAULT_TM_S;
            FOC_Control_SetTimedTarget(&g_foc, target, dur);
        }
        /* 把按键那一瞬的位置/误差一起打出来: 反复出现的"按了没动"多数是
           "本来就在容差内"(位置环在 |err|<容差 时故意撤力矩)。 */
        LOGD("[CMD] Set target %.1fdeg | now=%.2f err=%.2f -> %s\r\n",
             target, now_deg, de,
             (fabsf(de) < g_foc.position_loop.arrive_tol_deg)
                 ? "ALREADY at target (no motion)" : "moving");
        AutoStartIfIdle();
    }
}

/* ==========================================================================
 * 7) 按键扫描(RTT 键盘) —— 每个字符先回显, 再交给命令缓冲/单键处理
 * ========================================================================== */
static void App_PollKeys(void)
{
    int ch;

    while ((ch = SEGGER_RTT_GetKey()) >= 0) {
        /* 每个收到的字符都回显: 一眼分清"主机没把键送进来"和"收到但控制没动" */
        LOGD("[KEY] '%c' (0x%02X) st=%d flt=%d\r\n",
             (ch >= 32 && ch < 127) ? ch : '.', (unsigned)ch,
             (int)g_foc.state, (int)g_foc.flags.fault);

        if (App_FeedCommand((char)ch)) {
            continue;
        }
        App_HandleKey(ch);
    }
}

/* ==========================================================================
 * 8) 低频日志: [ADCR] / [MAN] / [OL]
 * ========================================================================== */
static void App_ReportSlow(uint32_t now)
{
    uint16_t ru = 0U, rv = 0U, rw = 0U;

    /* ⚠ 必须每次进来都取一次最新原始码: [MAN]/[OL] 两条日志也要用它。
       (2026-10-17: 重构时这里写成了"只在 [ADCR] 分支里取", 结果 [OL] 打出
        raw=0/0/0 —— 旧代码用的是文件级静态量, 永远是最新值。) */
    BSP_ADC_GetCurrentRaw(&ru, &rv, &rw);

    /* ---- ADC 原始值诊断 ---- */
    if ((now - s_last_adcr_ms) >= (s_log_verbose ? 1000U : CFG_LOG_ADC_MS)) {
        float vbus_v = BSP_ADC_ReadVbusVolts();
        s_last_adcr_ms = now;

        if (vbus_v >= 0.0f) {
            g_foc.current.vbus = vbus_v;
        }
        LOGD("[ADCR] fw=%s vbraw=%u vb=%.1fV | cntU=%u cntV=%u cntW=%u | offU=%u offV=%u offW=%u | Iu=%.2f Iv=%.2f Iw=%.2f | shunt=%.4f gain=%.1f\r\n",
             FW_BUILD_TAG, (unsigned)BSP_ADC_GetVbusRaw(), vbus_v,
             (unsigned)ru, (unsigned)rv, (unsigned)rw,
             (unsigned)s_off_u, (unsigned)s_off_v, (unsigned)s_off_w,
             g_foc.current.u, g_foc.current.v, g_foc.current.w,
             g_foc.config.shunt_r, g_foc.config.op_gain);

        /* 负载下母线塌陷是"电流环顶到限幅也拉不到目标"的头号嫌疑 */
        if (vbus_v >= 0.0f && vbus_v < 8.0f && g_foc.state != FOC_STATE_IDLE) {
            LOGW("[VBUS] LOW %.1fV while driving (raw=%u) - supply current-limit / sag?!\r\n",
                 vbus_v, (unsigned)BSP_ADC_GetVbusRaw());
        }
    }

    /* ---- 手动开环日志(200ms) ---- */
    if (g_foc.ol_active && g_foc.ol_manual && (now - s_last_ol_ms) >= 200U) {
        s_last_ol_ms = now;
        LOGD("[MAN] st=%d v=%.4f Vll=%.2fV f=%.1fHz dir=%+d th=%6.1f | |I|=%.2f (Iu=%+.2f Iv=%+.2f Iw=%+.2f) raw=%u/%u/%u | du=%.3f dv=%.3f dw=%.3f | vb=%.1fV\r\n",
             (int)g_foc.state, g_foc.ol_man_v,
             1.7320508f * g_foc.ol_man_v * g_foc.current.vbus,
             g_foc.ol_man_hz, (int)g_foc.ol_dir,
             g_foc.ol_theta * 57.29577951f, g_foc.ol_i_meas,
             g_foc.current.u, g_foc.current.v, g_foc.current.w,
             (unsigned)ru, (unsigned)rv, (unsigned)rw,
             PWM_Output_GetDu(&g_foc.pwm),
             PWM_Output_GetDv(&g_foc.pwm),
             PWM_Output_GetDw(&g_foc.pwm),
             g_foc.current.vbus);
    }

    /* ---- 自动开环测试日志(200ms) ---- */
    if (g_foc.ol_active && !g_foc.ol_manual && (now - s_last_ol_ms) >= 200U) {
        s_last_ol_ms = now;
        LOGD("[OL] t=%.2fs ph=%d dir=%+d th=%6.1f v=%.4f Vll=%.2fV | |I|=%.2f (Iu=%+.2f Iv=%+.2f Iw=%+.2f) raw=%u/%u/%u | du=%.3f dv=%.3f dw=%.3f | vb=%.1fV\r\n",
             (float)g_foc.ol_tick / g_foc.config.pwm_freq,
             (int)g_foc.ol_phase, (int)g_foc.ol_dir,
             g_foc.ol_theta * 57.29577951f, g_foc.ol_v_amp,
             1.7320508f * g_foc.ol_v_amp * g_foc.current.vbus,
             g_foc.ol_i_meas,
             g_foc.current.u, g_foc.current.v, g_foc.current.w,
             (unsigned)ru, (unsigned)rv, (unsigned)rw,
             PWM_Output_GetDu(&g_foc.pwm),
             PWM_Output_GetDv(&g_foc.pwm),
             PWM_Output_GetDw(&g_foc.pwm),
             g_foc.current.vbus);
    }
}

/* ==========================================================================
 * 9) 主状态日志: [MAIN](按变化节流) + [ANG]
 * ========================================================================== */
static void App_ReportMain(uint32_t now)
{
    uint32_t print_ms;
    float    mech = g_sensor.GetMechDeg(&g_foc);
    uint8_t  hh   = g_sensor.GetSector(&g_foc);
    float    tgt  = g_foc.pos_target_deg_cmd;
    uint8_t  moving = (fabsf(mech - s_last_mech) > 0.02f) ||
                      (hh != s_last_hall) ||
                      (fabsf(tgt - s_last_tgt) > 0.001f);

    /* 按"内容有没有变"定节拍: 变了 100ms, 完全不动 500ms, 详细模式 25ms。
       原来"err 超差就 25ms 一条"会在堵转时每秒 40 条把 RTT 上行缓冲打满,
       NO_BLOCK_SKIP 会直接丢日志(连 [CMD]/[KEY] 回显都丢)。 */
    print_ms = s_log_verbose ? CFG_LOG_VERBOSE_MS
                             : (moving ? CFG_LOG_MOVING_MS : CFG_LOG_IDLE_MS);
    s_last_mech = mech;
    s_last_hall = hh;
    s_last_tgt  = tgt;

    if ((now - s_last_main_ms) >= print_ms) {
        float td_verr, td_ff, td_trim, td_vobs;

        s_last_main_ms = now;
        FOC_Control_GetTrajDiag(&td_verr, &td_ff, &td_trim, &td_vobs);
        LOGD("[MAIN] t=%lums st=%d flt=%d fc=0x%04X h=%d arr=%d Pos=%.2f Tgt=%.2f err=%.2f th=%.1f Id=%.2f Iq=%.2f Iqref=%.2f vd=%.3f vq=%.3f du=%.3f spd=%.0f fb=%.0f Ipos=%.2f vsp=%.0f | ob=%.0f vp=%.0f adv=%.2f dte=%.0f | mv=%d jp=%u vs=%.0f | hl=%d gv=%d bh=%d cr=%u tm=%u sl=%u | tv=%.1f ff=%.2f tr=%.2f vob=%.0f\r\n",
             (unsigned long)now,
             g_foc.state, g_foc.flags.fault, (unsigned)g_foc.fault_code,
             (int)g_foc.hall.state,
             (int)g_foc.position_loop.arrived,
             mech, tgt, g_foc.position_loop.pos_err_deg,
             g_foc.angle.theta * 57.29577951f,   /* 电流矢量用的电角度 */
             g_foc.current_loop.id,
             g_foc.current_loop.iq,
             g_foc.current_loop.iq_ref * (float)g_foc.torque_dir,  /* 实际施加值 */
             g_foc.current_loop.vd,
             g_foc.current_loop.vq,
             PWM_Output_GetDu(&g_foc.pwm),
             g_foc.speed_loop.ref_erpm,
             g_foc.speed_loop.erpm_now,
             FOC_Control_GetPosIntegA(),
             FOC_Control_GetPosErpm(),
             g_sensor.GetErpm(&g_foc),
             FOC_Control_GetVelPosErpm(),
             g_foc.angle.adv * 57.29577951f,
             (float)(g_foc.tick - g_foc.hall.edge_tick) /
                 (g_foc.config.pwm_freq / 1000.0f),
             (int)FOC_Control_GetPosMove(),
             (unsigned)FOC_Control_GetPosJumpCnt(),
             FOC_Control_GetVelWinErpm(),
             (int)FOC_Control_GetPosHoldFlag(),
             (int)FOC_Control_GetPosGiveFlag(),
             (int)FOC_Control_GetPosBlockFlag(),
             (unsigned)FOC_Control_GetPosHopCnt(),
             (unsigned)FOC_Control_GetTimedPhase(),
             (unsigned)FOC_Control_GetPosSettleFlag(),
             td_verr, td_ff, td_trim, td_vobs);
    }

    /* ---- [ANG] 10ms(INFO 级, 默认不打印; 要看把 log.h 的等级调到 INFO) ---- */
    if ((now - s_last_ang_ms) >= 10U) {
        if (g_foc.state == FOC_STATE_ALIGN ||
            g_foc.state == FOC_STATE_STARTUP ||
            g_foc.state == FOC_STATE_CURRENT_LOOP ||
            g_foc.state == FOC_STATE_SPEED_LOOP ||
            g_foc.state == FOC_STATE_POSITION_LOOP) {
            s_last_ang_ms = now;
            LOGI("[ANG] %u,%d,%d,%d,%.2f,%.4f,%.1f\r\n",
                 (unsigned)s_log_seq++,
                 (int)g_foc.state,
                 (int)g_foc.hall.state,
                 (int)g_foc.angle.synced,
                 g_foc.angle.theta * 57.29577951f,
                 mech,
                 g_sensor.GetErpm(&g_foc));
        }
    }
}

/* ==========================================================================
 * 10) ISR 钩子
 * ========================================================================== */

/* 20kHz: 三相电流到齐 ⇒ 换算电流 → 传感器轮询 → 控制环 */
void App_OnCurrentSample(void)
{
    const float scale = g_foc.config.shunt_r * g_foc.config.op_gain;
    const float vref  = g_foc.config.vref;
    const float res   = (float)g_foc.config.adc_resolution;
    uint16_t ru = 0U, rv = 0U, rw = 0U;

    BSP_ADC_GetCurrentRaw(&ru, &rv, &rw);

    if (s_off_ok) {
        float iu = CFG_CURRENT_SIGN * ((float)ru - (float)s_off_u)
                   * vref / (res - 1.0f) / scale;
        float iv = CFG_CURRENT_SIGN * ((float)rv - (float)s_off_v)
                   * vref / (res - 1.0f) / scale;
        g_foc.current.u = iu;
        g_foc.current.v = iv;
        g_foc.current.w = -(iu + iv);
    }

    if (s_foc_loop_en) {
        g_sensor.Poll(&g_foc);                       /* 霍尔轮询/边沿检测(20kHz) */
        FOC_Control_Run(&g_foc, 1.0f / CFG_CTRL_FREQ_HZ);
    }
}

/* 1kHz: VOFA 波形一帧(通道: Iu, Iv, Iw, Spd(°/s), Pos(°)) */
void App_OnWaveTick(void)
{
    if (VOFA_GetEnable()) {
        /* ★ 速度: FOC_Control_GetPosErpm() 是 eRPM; 本机 4 对极 ⇒
           1 eRPM = 360°/60s/pp = 1.5 机械°/s, 所以 ×1.5 就是 °/s。
           想改看 Iq/Iqref 就把下面任一个换掉(它们才是能看出控制动作的量)。 */
        VOFA_Send5(g_foc.current.u, g_foc.current.v, g_foc.current.w,
                   FOC_Control_GetPosErpm() * 1.5f,
                   g_sensor.GetMechDeg(&g_foc));
    }
}

/* ==========================================================================
 * 11) 初始化 / 主循环
 * ========================================================================== */
static void App_PrintBanner(void)
{
    LOGD("\r\n===== FOC Control System =====\r\n");
    LOGD("MCU: STM32G431RBT6, 170MHz (platform layer: bsp_stm32g4.c)\r\n");
    LOGD("PWM: %.0fkHz Center-Aligned, control loop: %.0fkHz\r\n",
         (double)(CFG_PWM_FREQ_HZ / 1000.0f), (double)(CFG_CTRL_FREQ_HZ / 1000.0f));
    LOGD("[FW] build %s\r\n", FW_BUILD_TAG);
    LOGD("[FW] sensor=%s\r\n", g_sensor.name);
}

static void App_PrintKeys(void)
{
    LOGD("[MAIN] FOC ready (IDLE). dir_rev=%d torque_dir=%d. 's'=open-loop test  'c'=manual hall verify\r\n",
         (int)g_foc.config.hall_dir_rev, (int)g_foc.torque_dir);
    LOGD("[MAIN] Keys: o=MANUAL  u/i=V+/-0.01  n/m=V+/-0.05  h/l=freq +/-1Hz  f=dir  t=stop\r\n");
    LOGD("[MAIN] Keys: s=auto V/f test  p=PWM smoke test  r=re-align  D=dir_rev(cap)  y=torque\r\n");
    LOGD("[MAIN] Keys: j=+Iq pulse  k=-Iq pulse  z=target 0  0~9=digit*45deg(auto-start)  c=hall verify\r\n");
    LOGD("[MAIN] Keys: t<deg> d<sec> + Enter = TIMED move (accel/cruise/decel/creep = 2/9,3/9,2/9,2/9)\r\n");
    LOGD("[MAIN] Keys: 0~9/z default timed move: any angle in %.1fs (ramp-to-zero)\r\n",
         (double)FOC_POS_DEFAULT_TM_S);
    LOGD("[MAIN] Keys: v=verbose log(25ms)  w=VOFA wave on/off\r\n");
    LOGD("[ANG] idx,state,hall,synced,elec_deg,mech_deg,erpm\r\n");
}

/* 开机 5s 窗口: 等 'c'(手动盘轴验证) 或 's'(立即开环测试), 超时则待机 */
static void App_BootWindow(void)
{
    uint32_t boot = BSP_Time_Ms();
    uint8_t  start_ol = 0U;

    while (!s_hall_verify && !start_ol && (BSP_Time_Ms() - boot) < 5000U) {
        int ch;
        while ((ch = SEGGER_RTT_GetKey()) >= 0) {
            if (ch == 'c' || ch == 'C')      HallVerify_Enter();
            else if (ch == 's' || ch == 'S') start_ol = 1U;
        }
        BSP_Time_DelayMs(10);
    }
    if (start_ol) {
        FOC_Control_StartOpenLoopTest(&g_foc, CFG_OL_TEST_HZ, CFG_OL_TEST_AMP_A,
                                      CFG_OL_TEST_MS);
    }
}

void App_Init(void)
{
    FOC_Config_t config;

    (void)s_log_seq;      /* [ANG] 是 INFO 级(默认编译掉) ⇒ 序号可能没人用 */

    App_ConfigDefaults(&config);
    FOC_Control_Init(&g_foc, &config);
    App_PrintBanner();
    App_PrintCalib();

    /* 功率级与采样: 先 PWM, 再注入采样, 再运放前端, 最后才标定零点 */
    BSP_PWM_Start();
    BSP_ADC_StartCurrentSampling();
    BSP_Analog_Start();
    App_CalibrateOffsets();
    s_foc_loop_en = 1U;            /* 标定完成 ⇒ 放行 20kHz 控制环 */

    /* 开机**不自动进 ALIGN**(维修阶段自动驱动桥臂会把带故障的半桥一直烧着),
       必须按键才启动: s=自动 V/f / o=手动开环 / p=PWM 烟测 */
    g_foc.flags.enabled = 0U;
    FOC_Control_SetState(&g_foc, FOC_STATE_IDLE);
    App_PrintKeys();

    App_BootWindow();

    if (!s_hall_verify && !g_foc.ol_active) {
        LOGD("[MAIN] IDLE - NOT auto-started. press 'o'(manual) / 's'(auto V/f) / 'p'(pwm test)\r\n");
    }

    /* VOFA 波形输出: 标定完成后才启动, 免得标定期的乱流先刷一屏 */
    VOFA_Init();
    if (!BSP_WaveTimer_Start()) {
        LOGW("[VOFA] wave timer start failed -> no waveform output\r\n");
    } else {
        LOGD("[VOFA] wave ON: UART @ %ubps, FireWater %uch = Iu,Iv,Iw,Spd(deg/s),Pos, %uHz DMA (text, CRLF)\r\n",
             (unsigned)CFG_UART_BAUD, (unsigned)VOFA_CHANNELS,
             (unsigned)CFG_WAVE_RATE_HZ);
    }
}

/* ==========================================================================
 * 中断负载率报告(见 bsp.h 的 BSP_Prof_*)
 * --------------------------------------------------------------------------
 * 只走 RTT, 不上 VOFA。每秒一条。
 *   负载 = 该中断在窗口内花费的周期 / 窗口总周期。
 *   ⚠ 若 TIM6 优先级高于 ADC, 且两者相邻触发, ADC 测得的时间会包含 TIM6
 *     的耗时(保守偏大)。看"总和"是否 <100% 就能判断有没有溢出。
 * ========================================================================== */
static uint32_t App_CycToUsX100(uint32_t cyc)
{
    /* 周期数 → 1/100 微秒。 SystemCoreClock = 170000000 ⇒ 170 周期/us */
    uint32_t mhz = (uint32_t)(SystemCoreClock / 1000000U);
    if (mhz == 0U) { mhz = 170U; }
    return (uint32_t)(((unsigned long long)cyc * 100ULL) / (unsigned long long)mhz);
}

static void App_ReportProf(uint32_t now)
{
    static uint32_t s_pf_last_ms = 0U;
    static uint32_t s_pf_drop    = 0U;      /* ★ cal29: 上一窗口的 RTT 丢行数 */
    BSP_Prof_t p;

    if ((now - s_pf_last_ms) < 1000U) {
        return;
    }
    s_pf_last_ms = now;

    BSP_Prof_Take(&p);
    if ((p.win_cycles == 0U) || (p.isr_count == 0U)) {
        return;
    }
    {
        unsigned long long w   = (unsigned long long)p.win_cycles;
        uint32_t isr_pm = (uint32_t)((unsigned long long)p.isr_cycles * 10000ULL / w);
        uint32_t wav_pm = (uint32_t)((unsigned long long)p.wav_cycles * 10000ULL / w);
        uint32_t idl_pm = (isr_pm + wav_pm < 10000U) ? (10000U - isr_pm - wav_pm) : 0U;
        uint32_t clk1000 = (uint32_t)(SystemCoreClock / 1000U);
        uint32_t isr_avg = (p.isr_count != 0U) ? (p.isr_cycles / p.isr_count) : 0U;
        uint32_t wav_avg = (p.wav_count != 0U) ? (p.wav_cycles / p.wav_count) : 0U;
        uint32_t rtt_now = BSP_Log_GetRttDrop();
        uint32_t rtt_drop = rtt_now - s_pf_drop;   /* 本窗口(1s)丢掉的行数 */
        s_pf_drop = rtt_now;

        LOGD("[PROF] win=%lums | FOC@20k n=%lu load=%lu.%02lu%% tot=%lums avg=%lu.%02luus max=%lu.%02luus"
             " | VOFA@1k n=%lu load=%lu.%02lu%% tot=%lums avg=%lu.%02luus max=%lu.%02luus"
             " | idle=%lu.%02lu%% | rttdrop=%lu\r\n",
             (unsigned long)(p.win_cycles / clk1000),
             (unsigned long)p.isr_count,
             (unsigned long)(isr_pm / 100U), (unsigned long)(isr_pm % 100U),
             (unsigned long)(p.isr_cycles / clk1000),
             (unsigned long)(App_CycToUsX100(isr_avg) / 100U),
             (unsigned long)(App_CycToUsX100(isr_avg) % 100U),
             (unsigned long)(App_CycToUsX100(p.isr_max) / 100U),
             (unsigned long)(App_CycToUsX100(p.isr_max) % 100U),
             (unsigned long)p.wav_count,
             (unsigned long)(wav_pm / 100U), (unsigned long)(wav_pm % 100U),
             (unsigned long)(p.wav_cycles / clk1000),
             (unsigned long)(App_CycToUsX100(wav_avg) / 100U),
             (unsigned long)(App_CycToUsX100(wav_avg) % 100U),
             (unsigned long)(App_CycToUsX100(p.wav_max) / 100U),
             (unsigned long)(App_CycToUsX100(p.wav_max) % 100U),
             (unsigned long)(idl_pm / 100U), (unsigned long)(idl_pm % 100U),
             (unsigned long)rtt_drop);
    }
}

void App_Loop(void)
{
    uint32_t now = BSP_Time_Ms();

    App_PollKeys();
    BSP_Log_DeferFlush();      /* ★ cal25: 把中断里攒下的日志一次性写出 */
    App_ReportSlow(now);

    /* 手动盘轴验证: 高频采样(2ms), 不打常规状态行 */
    if (s_hall_verify) {
        HallVerify_Poll(now);
        BSP_Time_DelayMs(2);
        return;
    }

    App_ReportMain(now);
    App_ReportProf(now);
    BSP_Time_DelayMs(10);
}
