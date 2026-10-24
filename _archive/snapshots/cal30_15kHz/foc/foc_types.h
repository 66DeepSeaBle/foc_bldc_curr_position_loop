/**
  ******************************************************************************
  * @file    foc_types.h
  * @brief   FOC 各模块共用的数据类型定义(纯数据结构, 不含任何平台依赖)。
  ******************************************************************************
  */
#ifndef FOC_TYPES_H
#define FOC_TYPES_H

#include <stdint.h>
#include <stddef.h>

/* ==========================================================================
 * ★ 控制拍频(唯一真源) —— 改控制/PWM 频率只需要改这一个数
 * --------------------------------------------------------------------------
 * ADC 注入中断由 TIM1 的更新事件触发 ⇒ **控制频率 ≡ PWM 频率**。
 * 改频时还必须同步改 tim.c 的 htim1.Init.Period(中心对齐):
 *     Period = 170e6 / (2 * FOC_CTRL_HZ) - 1
 *       20 kHz ⇒ 4249     15 kHz ⇒ 5666
 * 所有"毫秒/秒 → 拍数"的常量一律写成  NNN * FOC_TICKS_PER_MS  /
 * FOC_TICKS_PER_S, 不要再写裸数字(20U / 20000), 否则改频后所有超时时长
 * 会跟着跑偏。★ 要求 FOC_CTRL_HZ 是 1000 的整数倍(否则 FOC_TICKS_PER_MS 不精确)。
 * ========================================================================== */
#define FOC_CTRL_HZ          15000U                  /* ★ 试用: 20k → 15k(查静止啸叫) */
#define FOC_TICKS_PER_MS     (FOC_CTRL_HZ / 1000U)   /* 20k→20, 15k→15 */
#define FOC_TICKS_PER_S      (FOC_CTRL_HZ)

/* ---- 状态机状态 ---- */
typedef enum {
    FOC_STATE_IDLE = 0,          /* 待机(三相不驱动) */
    FOC_STATE_ALIGN,             /* 对齐: 强制电角度 0° 注入 Id, 把转子拉到已知位置 */
    FOC_STATE_STARTUP,           /* I-F 开环强拖(等霍尔同步) */
    FOC_STATE_CURRENT_LOOP,      /* 纯电流环(力矩模式) */
    FOC_STATE_SPEED_LOOP,        /* 速度环 + 电流环 */
    FOC_STATE_POSITION_LOOP,     /* 位置环 + 速度环 + 电流环 */
    FOC_STATE_BRAKE,             /* 刹车(三相短路/下管制动) */
    FOC_STATE_FAULT,             /* 故障(关驱动, 按 'r' 复位) */
    FOC_STATE_OPENLOOP           /* 开环速度测试('s'): theta 自行旋转, 电流环钳 1A */
} FOC_State_t;

/* ---- 运行标志 ---- */
typedef struct {
    uint8_t enabled;             /* 1 = 允许输出 */
    uint8_t aligned;             /* 1 = 已对齐 */
    uint8_t hall_synced;         /* 1 = 霍尔已同步 */
    uint8_t fault;               /* 1 = 故障 */
    uint8_t overcurrent;         /* 1 = 过流 */
    uint8_t calibrating;         /* 1 = 正在标定 */
} FOC_Flags_t;

/* ---- 三相电流/母线电压 ---- */
typedef struct {
    float u;                     /* U 相电流 (A) */
    float v;                     /* V 相电流 (A) */
    float w;                     /* W 相电流 (A) */
    float vbus;                  /* 母线电压 (V) */
    uint16_t offset_u;           /* U 相零电流偏置 (ADC 码) */
    uint16_t offset_v;           /* V 相零电流偏置 (ADC 码) */
    uint16_t offset_w;           /* W 相零电流偏置 (ADC 码) */
} ThreePhase_Current_t;

/* ---- 数字霍尔状态 ---- */
typedef struct {
    uint8_t u;                   /* U 相霍尔电平 (0/1) */
    uint8_t v;                   /* V 相霍尔电平 (0/1) */
    uint8_t w;                   /* W 相霍尔电平 (0/1) */
    uint8_t state;               /* 合成扇区号 (1~6) */
    uint8_t prev_state;          /* 上一扇区号 */
    uint32_t edge_tick;          /* 上一条边沿的时刻 (20kHz tick) */
    uint32_t prev_dt;            /* 与上一条边沿的间隔 (tick) */
    uint8_t dir;                 /* 转动方向 (+1/-1; 注意是 uint8_t, 判负要转 (int8_t)) */
} Hall_State_t;

/* ---- 角度观测器(霍尔版: 扇区 + 扇区内插值 + 模型外推) ---- */
typedef struct {
    float theta;                 /* 输出电角度 (rad, 0~2π) */
    float theta_base;            /* 当前扇区中心角 */
    float theta_halfwidth;       /* 当前扇区半宽 (rad) */
    /* 扇区内进角估计(rad, ∈[-halfwidth,+halfwidth]): 每次霍尔边沿重置到该扇区
       的入口边界, 之后按速度逐步累加。它是"逐步累加"而不是"用 dt 重算",
       所以停转时保持不动(不再往入口边界塌缩), 位置读数在扇区边界也连续。 */
    float adv;
    float omega;                 /* 单边沿估速 (rad/tick) */
    float omega_filt;            /* 滤波后估速 (rad/tick) */
    float omega_prev;            /* 上一拍估速 (用于算角加速度) */
    float alpha;                 /* 角加速度 (rad/tick^2) */
    float alpha_filt;            /* 滤波后角加速度 */
    float offset;                /* 霍尔安装零点偏移 (rad), ALIGN 时自动标定 */
    uint8_t hall_dir_rev;        /* 1 = 反转霍尔扇区顺序(dir_rev) */
    float raw_theta;             /* 未加 offset 的原始霍尔角 */
    float elec_acc_deg;          /* 相位展开后的累计电角度(多圈, deg) */
    uint8_t synced;              /* 1 = 已同步(测到过转动) */
    uint32_t glitch_cnt;         /* 毛刺计数 */

    /* 多边沿测速: 单边沿 omega=扇区宽/dt 每个电周期只有 6 个点, 转子摩擦爬行
       ("冲一下、停一会")时会在几十~上千 eRPM 之间乱跳。这里存最近 4 次边沿的
       时刻与连续电角度, 用"总电角度/总时间"做平均测速。 */
    uint32_t edge_hist_tick[4];
    float    edge_hist_acc[4];
    uint8_t  edge_hist_idx;
    /* GetTheta 最后一次被调用的时刻: 用它判断 elec_acc_deg 是否"正在推进"。
       ALIGN/STARTUP 期间 GetTheta 不被调用 ⇒ elec_acc_deg 冻结 ⇒ 窗口测速
       没意义(必须退回单边沿估计, 否则 STARTUP 同步判据永远不成立)。 */
    uint32_t last_get_tick;
    /* ★★ 2026-10-11ac 边沿间**外推速度**(rad/tick) —— 与 omega_filt 分开:
       omega_filt 是"上报给环路看"的模型速度; pos_ext_w 只用来推进 adv,
       并在每条边沿重载实测值、之后按 POS_MODEL_TAU_S 累积衰减
       (不分开的话衰减会被 omega_filt 的写回顺序吃掉 ⇒ 停转后读数会一直爬到
        扇区钳位, 凭空造出一个"差一个齿槽"的假误差 —— 10-11aa 日志实测)。 */
    float    pos_ext_w;
    uint32_t pos_ext_tick;
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

/* ---- 通用 PID ---- */
typedef struct {
    float kp;                    /* 比例增益 */
    float ki;                    /* 积分增益 */
    float kd;                    /* 微分增益 (按秒计) */
    float integ;                 /* 积分累计量 */
    float prev_err;              /* 上一拍误差 (算微分用) */
    float lim;                   /* 输出限幅 (对称) */
    float lim_integ;             /* 积分限幅 (对称, 通常小于 lim) */
} PID_Controller_t;

/* ---- 一阶低通 ---- */
typedef struct {
    float y_prev;                /* 上一拍输出 */
    float tau_s;                 /* 时间常数 (s) */
} LowPassFilter_t;

/* ---- 电流环状态 ---- */
typedef struct {
    float id_ref;                /* D 轴电流指令 (A) */
    float iq_ref;                /* Q 轴电流指令 (A) */
    float id;                    /* D 轴电流实测 (A) */
    float iq;                    /* Q 轴电流实测 (A) */
    float vd;                    /* D 轴电压输出 (占空比量) */
    float vq;                    /* Q 轴电压输出 (占空比量) */
    float vlim;                  /* 输出电压限幅 (占空比量) */
    PID_Controller_t pid_d;      /* D 轴 PI 调节器 */
    PID_Controller_t pid_q;      /* Q 轴 PI 调节器 */
} CurrentLoop_t;

/* ---- 速度环状态 ---- */
typedef struct {
    float ref_erpm;              /* 速度指令 (eRPM) */
    float ref_target;            /* 斜坡后的目标速度 (eRPM) */
    float erpm_now;              /* 转速反馈 (eRPM) */
    float kp;                    /* 比例增益 */
    float ki;                    /* 积分增益 */
    float integ;                 /* 积分累计量 */
    float iq_cap;                /* 当前 Iq 限幅 (A) */
    float iq_max_normal;         /* 稳态 Iq 限幅 (A) */
    float iq_max_zone;           /* 运动时 Iq 限幅 (A) */
    float settle_err_erpm;       /* 判定"已稳定"的速度误差门限 (eRPM) */
    uint32_t settle_ms;          /* 判定"已稳定"的持续时间 (ms) */
    uint32_t settled_tick;       /* 进入稳定的时刻 */
    uint8_t settled;             /* 1 = 已稳定(可降限幅) */
    float ramp_rate;             /* 指令斜坡速率 (eRPM/s) */
    uint32_t tick;               /* 内部计时 tick */
} SpeedLoop_t;

/* ---- 位置环状态 ---- */
typedef struct {
    float target_deg;            /* 目标机械角 (deg, 多圈) */
    float pos_now_deg;           /* 当前机械角 (deg, 多圈) */
    float pos_err_deg;           /* 位置误差 (deg) */
    float kp;                    /* 位置比例增益 (eRPM/deg) */
    float speed_max_erpm;        /* 限速 (eRPM) */
    float decel_erpm_s;          /* 减速度限制 (eRPM/s) */
    uint8_t arrived;             /* 1 = 已到位 */
    uint32_t arrived_tick;       /* 到位时刻 */
    uint32_t arrive_ms;          /* 到位耗时 (ms) */
    float arrive_tol_deg;        /* 到位容差 (deg) */
    float travel_limit_deg;      /* 行程软限位 (deg) */
    uint8_t pole_pairs;          /* 极对数 */
} PositionLoop_t;

/* ---- SVPWM 输出 ---- */
typedef struct {
    float du;                    /* U 相占空比 */
    float dv;                    /* V 相占空比 */
    float dw;                    /* W 相占空比 */
    float valpha;                /* Alpha 轴电压 (占空比量) */
    float vbeta;                 /* Beta 轴电压 (占空比量) */
    float dt_comp_duty;          /* 死区补偿量(占空比) */
} SVPWM_t;

/* ---- FOC 配置(由 board_config.h 装配) ---- */
typedef struct {
    float pwm_freq;              /* PWM 频率 (Hz) */
    float vref;                  /* ADC 参考电压 (V) */
    float shunt_r;               /* 采样电阻 (Ω) */
    float op_gain;               /* 运放增益 */
    float deadtime_ns;           /* 死区(ns). 0 = 关闭死区补偿(见 PWM_Output_Init) */
    float current_scale;         /* 电流换算系数(shunt_r × op_gain) */
    float current_sign;          /* 电流采样极性 (±1) */
    float vbus_divider;          /* 母线分压比 */
    uint16_t adc_resolution;     /* ADC 满量程码值 */
    uint8_t pole_pairs;          /* 极对数 */
    uint8_t hall_dir_rev;        /* 霍尔相序方向 */
    float hall_offset_deg;       /* 霍尔零点偏移 (deg) */
    float overcurrent_amp;       /* 过流门限 (A) */
    float align_current;         /* 对齐电流 (A) */
    float startup_current;       /* I-F 拖转电流 (A), 与 align 解耦(原来用 align*0.5) */
    float startup_hz;            /* I-F 拖转电频率 (Hz) */
    float iq_ramp_rate;          /* q 轴电流指令斜坡 (A/s) */
} FOC_Config_t;

#endif
