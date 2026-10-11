/**
  * @file    foc_types.h
  * @brief   FOC ???????????????
  */
#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdint.h>
#include <stddef.h>

/* ---- ???? ---- */
typedef enum {
    FOC_STATE_IDLE = 0,          // ??(?????)
    FOC_STATE_ALIGN,             // ????
    FOC_STATE_STARTUP,           // I-F ??
    FOC_STATE_CURRENT_LOOP,      // ????(????)
    FOC_STATE_SPEED_LOOP,        // ??? + ???
    FOC_STATE_POSITION_LOOP,     // ??? + ??? + ???
    FOC_STATE_BRAKE,             // ??(????)
    FOC_STATE_FAULT,             // ??(???)
    FOC_STATE_OPENLOOP           // 开环速度测试(S): theta 自行旋转, 电流环钳 1A
} FOC_State_t;

/* ---- ???? ---- */
typedef struct {
    uint8_t enabled;             // 1=??
    uint8_t aligned;             // 1=?????
    uint8_t hall_synced;         // 1=?????
    uint8_t fault;               // 1=??
    uint8_t overcurrent;         // 1=??
    uint8_t calibrating;         // 1=???
} FOC_Flags_t;

/* ---- ??????? ---- */
typedef struct {
    float u;                     // U??? (A)
    float v;                     // V??? (A)
    float w;                     // W??? (A)
    float vbus;                  // ???? (V)
    uint16_t offset_u;           // U??? (ADC code)
    uint16_t offset_v;
    uint16_t offset_w;
} ThreePhase_Current_t;

/* ---- ??????? ---- */
typedef struct {
    uint8_t u;                   // ??U?? (0/1)
    uint8_t v;                   // ??V?? (0/1)
    uint8_t w;                   // ??W?? (0/1)
    uint8_t state;               // ???? (1~6)
    uint8_t prev_state;          // ?????
    uint32_t edge_tick;          // ?????? (20kHz tick)
    uint32_t prev_dt;            // ?????? (tick)
    uint8_t dir;                 // ???? (+1/-1)
} Hall_State_t;

/* ---- ??????? ---- */
typedef struct {
    float theta;                 // ????? (??, 0~2p)
    float theta_base;            // ????????
    float theta_halfwidth;       // ?????? (??)
    /* 扇区内进角估计(rad, ∈[-halfwidth,+halfwidth]): 每次霍尔边沿重置到该扇区
       的入口边界, 之后按速度逐步累加。它是"逐步累加"而不是"用 dt 重算",
       所以停转时保持不动(不再往入口边界塌缩), 位置读数在扇区边界也连续。 */
    float adv;
    float omega;                 // ??? (rad/tick)
    float omega_filt;            // ????? (rad/tick)
    float omega_prev;            // ????? (?????)
    float alpha;                 // ???? (rad/tick^2)
    float alpha_filt;            // ??????
    float offset;                // hall mount offset (rad)
    uint8_t hall_dir_rev;        // 1 = reverse hall sector order (dir_rev)
    float raw_theta;             // raw hall angle (no offset)
    float elec_acc_deg;          // accumulated elec angle (multi-turn, from edges)
    uint8_t synced;              // 1 = hall synced (rotation detected)
    uint32_t glitch_cnt;         // glitch counter

    /* 多边沿测速: 单边沿 omega=扇区宽/dt 每个电周期只有 6 个点, 转子摩擦爬行
       ("冲一下、停一会")时会在几十~上千 eRPM 之间乱跳. 这里存最近 4 次边沿的
       时刻与连续电角度, 用"总电角度/总时间"做平均测速. */
    uint32_t edge_hist_tick[4];
    float    edge_hist_acc[4];
    uint8_t  edge_hist_idx;
    /* GetTheta 最后一次被调用的时刻: 用它判断 elec_acc_deg 是否"正在推进"。
       ALIGN/STARTUP 期间 GetTheta 不被调用 ⇒ elec_acc_deg 冻结 ⇒ 窗口测速
       没意义(必须退回单边沿估计, 否则 STARTUP 同步判据永远不成立)。 */
    uint32_t last_get_tick;

    /* ---- 2026-10-11c: 位置改成"扇区序号 + 扇区内插值"的绝对算法 ----
       sect_run   : 沿正向环序累计走过的扇区数(整型, 只在边沿更新, 不会漂)
       pos_abs_rad: 当前绝对电角度 = 360°×圈数 + 扇区入口角 + 扇区内进角
       pos_ref_rad: 位置零点(交接时对齐, 使 elec_acc_deg = 0)
       pos_seeded : 第一条边沿时把 sect_run 对齐到当前扇区 */
    int32_t  sect_run;
    float    pos_abs_rad;
    float    pos_ref_rad;
    uint8_t  pos_seeded;
    /* 上一次"相邻跳变"确认过的转动方向(±1, 0=未知): 对跖(180°)跳变时用它
       决断步数符号, 比瞬时 omega 可靠 */
    int8_t   last_dir;
    /* 20kHz 轮询边沿检测用: 连续多少次采样拿到"非相邻状态"(噪声计数) */
    uint16_t bad_cnt;
} Angle_Observer_t;

/* ---- PID??? ---- */
typedef struct {
    float kp;                    // ????
    float ki;                    // ????
    float kd;                    // ???? (??)
    float integ;                 // ?????
    float prev_err;              // ???? (????)
    float lim;                   // ???? (??)
    float lim_integ;             // ???? (??,???lim)
} PID_Controller_t;

/* ---- ????? ---- */
typedef struct {
    float y_prev;                // ????
    float tau_s;                 // ???? (?)
} LowPassFilter_t;

/* ---- ????? ---- */
typedef struct {
    float id_ref;                // D????? (A)
    float iq_ref;                // Q????? (A)
    float id;                    // D????? (A)
    float iq;                    // Q????? (A)
    float vd;                    // D????? (??)
    float vq;                    // Q????? (??)
    float vlim;                  // ???? (??)
    PID_Controller_t pid_d;      // D?PI???
    PID_Controller_t pid_q;      // Q?PI???
} CurrentLoop_t;

/* ---- ????? ---- */
typedef struct {
    float ref_erpm;              // ???? (eRPM)
    float ref_target;            // ???? (???)
    float erpm_now;              // ?????? (eRPM)
    float kp;                    // ????
    float ki;                    // ????
    float integ;                 // ???
    float iq_cap;                // ??IQ?? (A)
    float iq_max_normal;         // ???IQ?? (A)
    float iq_max_zone;           // ???IQ?? (A)
    float settle_err_erpm;       // ?????? (eRPM)
    uint32_t settle_ms;          // ?????? (ms)
    uint32_t settled_tick;       // ?????tick
    uint8_t settled;             // 1=??? (?????)
    float ramp_rate;             // ?????? (eRPM/s)
    uint32_t tick;               // ??tick (????)
} SpeedLoop_t;

/* ---- ????? ---- */
typedef struct {
    float target_deg;            // ???? (????)
    float pos_now_deg;           // ???? (????)
    float pos_err_deg;           // ???? (????)
    float kp;                    // ????
    float speed_max_erpm;        // ???? (eRPM)
    float decel_erpm_s;          // ??? (eRPM/s)
    uint8_t arrived;             // 1=???
    uint32_t arrived_tick;       // ???????tick
    uint32_t arrive_ms;          // ?????? (ms)
    float arrive_tol_deg;        // ???? (????)
    float travel_limit_deg;      // ???? (????)
    uint8_t pole_pairs;          // ???
} PositionLoop_t;

/* ---- SVPWM?? ---- */
typedef struct {
    float du;                    // U????
    float dv;                    // V????
    float dw;                    // W????
    float valpha;                // Alpha??? (??)
    float vbeta;                 // Beta??? (??)
    float dt_comp_duty;          // ???????
} SVPWM_t;

/* ---- ???? ---- */
typedef struct {
    float pwm_freq;              // PWM?? (Hz)
    float vref;                  // ADC???? (V)
    float shunt_r;               // ???? (O)
    float op_gain;               // ????
    float deadtime_ns;           // 死区(ns). 0 = 关闭死区补偿(见 PWM_Output_Init)
    float current_scale;         // ??????
    float current_sign;          // ???? (??)
    float vbus_divider;          // ?????
    uint16_t adc_resolution;     // ADC???
    uint8_t pole_pairs;          // ?????
    uint8_t hall_dir_rev;        // ??????
    float hall_offset_deg;       // ?????? (?)
    float overcurrent_amp;       // ???? (A)
    float align_current;         // ???? (A)
    float startup_current;       // I-F 拖转电流 (A), 与 align 解耦(原来用 align*0.5)
    float startup_hz;            // I-F???? (Hz)
    float iq_ramp_rate;          // ?????? (A/s)
} FOC_Config_t;

#endif

