/**
  * @file    angle_observer.c
  * @brief   ?????????
  */
#include "angle_observer.h"
#include <math.h>

/* ---- ??????? (???,?????) ---- */
static const float SECTOR_CENTER[8] = {
    0.0f,           // 0 ??
    4.79092880f,    // h1 274.500�
    2.72533163f,    // h2 156.150�
    3.73267749f,    // h3 213.867�
    0.54017940f,    // h4  30.950�
    5.80321976f,    // h5 332.500�
    1.66184434f,    // h6  95.217�
    0.0f            // 7 ??
};

/* ---- ?????? (??) ---- */
static const float SECTOR_HALFWIDTH[8] = {
    0.52359878f,    // 0 ??,??????
    0.53290720f,    // h1 30.533�
    0.48200176f,    // h2 27.617�
    0.52534410f,    // h3 30.100�
    0.54017940f,    // h4 30.950�
    0.47938377f,    // h5 27.467�
    0.58148553f,    // h6 33.317�
    0.52359878f     // 7 ??,??????
};

/* ---- ??/????? ---- */
static const uint8_t FWD_NEXT[8] = { 0, 3, 6, 2, 5, 1, 4, 0 };
static const uint8_t BWD_NEXT[8] = { 0, 5, 3, 1, 6, 4, 2, 0 };

/* ---- ??????(??????) ---- */
static float GetSectorCenter(uint8_t state, uint8_t dir_rev)
{
    float a = SECTOR_CENTER[state];
    if (dir_rev) {
        a = 6.28318531f - a;
        if (a >= 6.28318531f) a -= 6.28318531f;
    }
    return a;
}

/* ---- ??? ---- */
void AngleObserver_Init(Angle_Observer_t *obs, float hall_offset_deg,
                        uint8_t hall_dir_rev)
{
    obs->theta = 0.0f;
    obs->theta_base = 0.0f;
    obs->theta_halfwidth = 0.52359878f;
    obs->omega = 0.0f;
    obs->omega_filt = 0.0f;
    obs->omega_prev = 0.0f;
    obs->alpha = 0.0f;
    obs->alpha_filt = 0.0f;
    obs->offset = hall_offset_deg * 0.01745329f;
    obs->hall_dir_rev = hall_dir_rev;
    obs->raw_theta = 0.0f;
    obs->elec_acc_deg = 0.0f;
    obs->synced = 0;
    obs->glitch_cnt = 0;
    /* 多边沿测速历史清零: 未填满时自动退回单边沿估计 */
    for (int i = 0; i < 4; i++) {
        obs->edge_hist_tick[i] = 0U;
        obs->edge_hist_acc[i]  = 0.0f;
    }
    obs->edge_hist_idx = 0U;
}

/* ---- ????????? ---- */
void AngleObserver_OnHallEdge(Angle_Observer_t *obs, Hall_State_t *hall,
                              uint32_t now_tick, float pwm_freq,
                              float min_dt_ticks, float omega_clamp_erpm,
                              float alpha_max)
{
    uint8_t h = hall->state;
    uint32_t dt;
    int8_t dir;

    if (h == 0 || h == 7) {
        obs->synced = 0;
        return;
    }

    /* ---- ???? ---- */
    if (hall->prev_state >= 1 && hall->prev_state <= 6) {
        if (h == FWD_NEXT[hall->prev_state]) {
            dir = 1;
        } else if (h == BWD_NEXT[hall->prev_state]) {
            dir = -1;
        } else {
            /* 非相邻跳变(丢步/多步): 沿用上一次的转动方向 */
            dir = (obs->omega >= 0.0f) ? 1 : -1;
        }
    } else {
        dir = 1;
    }

    dt = now_tick - hall->edge_tick;

    /* ---- ???? ---- */
    if (dt < min_dt_ticks) {
        obs->glitch_cnt++;
        obs->theta_base = GetSectorCenter(h, obs->hall_dir_rev);
        obs->theta_halfwidth = SECTOR_HALFWIDTH[h];
        hall->dir = dir;
        return;
    }

    if (dt > 0 && dt < 20000U) {  // 1???
        float sector_width = (hall->prev_state >= 1 && hall->prev_state <= 6) ?
                              (2.0f * SECTOR_HALFWIDTH[hall->prev_state]) : 1.04719755f;

        obs->omega = (float)dir * sector_width / (float)dt;

        /* ---- 多边沿(窗口)测速 ----
           单边沿 omega=扇区宽/dt 每个电周期只有 6 个点; 转子在静摩擦下"冲一下、
           停一会"时, 冲的那次 dt 很小 → 单点估计冲到几百~上千 eRPM, 速度环看到
           就满力矩反拉(积分被打到反向限幅), 然后要 ~1s 才退回来 → 转子停住等
           积分爬回来, 再冲一次 = 一顿一顿(实测每步 2.3° 机械角、停 0.4~1.4s).
           改用最近 3 个边沿(约 180° 电角度)的总电角度/总时间: 抖动按 1/3 降,
           而且用连续电角度之差, 天然带方向, 不依赖 dir 判定. */
        obs->edge_hist_tick[obs->edge_hist_idx] = now_tick;
        obs->edge_hist_acc[obs->edge_hist_idx]  = obs->elec_acc_deg;
        obs->edge_hist_idx = (uint8_t)((obs->edge_hist_idx + 1U) & 0x03U);
        {
            uint32_t t0 = obs->edge_hist_tick[obs->edge_hist_idx]; /* 最旧一条 */
            float    a0 = obs->edge_hist_acc[obs->edge_hist_idx];
            float    dacc = obs->elec_acc_deg - a0;
            /* 只有 elec_acc_deg 确实在推进时才采用窗口估计。
               ALIGN/STARTUP/手动开环期间 GetTheta 不被调用(FOC_DoAlign /
               FOC_DoStartup 直接调 FOC_UpdateCurrent), 于是 elec_acc_deg 被冻结
               → dacc==0 → omega 被算成 0 → omega_filt 衰减到 0 →
               GetOmegaFiltErpm()==0 → STARTUP 的同步判据 |erpm|>8 永远不成立。
               实测现象: 霍尔明明在跳(转子被拖动了), 却每次都
               [STARTUP] Timeout, hall not synced → FAULT 0x0002。
               此时保留上面的单边沿估计(扇区宽/dt)。 */
            if (t0 != 0U && now_tick > t0 &&
                (dacc > 0.01f || dacc < -0.01f)) {
                obs->omega = dacc * 0.01745329f / (float)(now_tick - t0);
            }
            /* t0==0 → 历史还没填满(刚同步/刚交接), 保留上面的单边沿估计 */
        }

        /* ---- ???? ---- */
        float omega_clamp = omega_clamp_erpm * 6.28318531f / 60.0f / pwm_freq;
        if (obs->omega >  omega_clamp) obs->omega =  omega_clamp;
        if (obs->omega < -omega_clamp) obs->omega = -omega_clamp;

        /* ---- ?????? ---- */
        obs->alpha = (obs->omega - obs->omega_prev) / (float)dt;
        if (obs->alpha >  alpha_max) obs->alpha =  alpha_max;
        if (obs->alpha < -alpha_max) obs->alpha = -alpha_max;
        obs->alpha_filt = 0.5f * obs->alpha_filt + 0.5f * obs->alpha;
        obs->omega_prev = obs->omega;

        /* ---- ?????? ---- */
        obs->omega_filt = 0.5f * obs->omega_filt + 0.5f * obs->omega;
        obs->synced = 1;
    }

    /* ---- ???? ---- */
    hall->prev_state = h;
    hall->edge_tick = now_tick;
    hall->prev_dt = dt;
    hall->dir = dir;
    obs->theta_base = GetSectorCenter(h, obs->hall_dir_rev);
    obs->theta_halfwidth = SECTOR_HALFWIDTH[h];
}

/* ---- ?????? (?PWM????) ---- */
float AngleObserver_GetTheta(Angle_Observer_t *obs, const Hall_State_t *hall,
                             uint32_t now_tick, float pwm_freq,
                             uint32_t timeout_ticks, float creep_hz,
                             uint8_t creep_suppress, float *erpm_out)
{
    uint32_t dt = now_tick - hall->edge_tick;
    float adv;
    float theta_out;
    float erpm = 0.0f;

    /* ---- ???? (??) ---- */
    if (!obs->synced || dt >= timeout_ticks) {
        obs->omega = 0.0f;
        obs->omega_filt = 0.0f;
        obs->alpha_filt = 0.0f;

        if (!creep_suppress) {
            float creep_step = 2.0f * 3.14159265f * creep_hz / pwm_freq;
            obs->theta += (float)hall->dir * creep_step;
        }
        theta_out = obs->theta;
        goto wrap_and_exit;
    }

    /* ---- 速度估计随时间衰减(关键) ----
       obs->omega 只在霍尔边沿更新一次, 之后会被一直"保持"到 1 秒 timeout.
       实测: 转子一次窜动(5ms 冲过 1 个扇区 ≈2000eRPM)后停住, fb 仍恒定
       1948/2019 长达 300~600ms → 速度环以为在飞车, 积分以 ki*|err|≈400/s
       打到反向满幅 → 满力矩反拉 → 转子反向冲 → 再反拉 … 表现为"很快的
       左右摆动, 然后停止, 又重复".
       物理上限: 自上次边沿起已经过了 dt 拍还没出现新边沿 ⇒ 这段时间的平均
       速度不可能超过 (一个扇区宽)/dt. 停转时该上限按 1/dt 收敛到 0, 估计值
       自然衰减; 正常转动时 dt≈边沿间隔, 上限≈实测值(留 1.2 倍余量给抖动). */
    if (dt > 0U && obs->theta_halfwidth > 0.0f) {
        float omega_cap = 1.2f * 2.0f * obs->theta_halfwidth / (float)dt;
        if (obs->omega >  omega_cap) obs->omega =  omega_cap;
        if (obs->omega < -omega_cap) obs->omega = -omega_cap;
    }

    /* ---- ???? ---- */
    adv = obs->omega_filt * (float)dt
        + 0.5f * obs->alpha_filt * (float)dt * (float)dt
        - (float)hall->dir * obs->theta_halfwidth;

    if (adv >  obs->theta_halfwidth) adv =  obs->theta_halfwidth;
    if (adv < -obs->theta_halfwidth) adv = -obs->theta_halfwidth;

    theta_out = obs->theta_base + adv + obs->offset;

    erpm = obs->omega * pwm_freq / 6.28318531f * 60.0f;

wrap_and_exit:
    /* ---- 折叠到 0~2π ---- */
    while (theta_out >= 6.28318531f) theta_out -= 6.28318531f;
    while (theta_out < 0.0f)         theta_out += 6.28318531f;

    /* ---- 连续电角度累加(相位展开) ----
       theta 本身就是按"相邻扇区边界重合"构造的连续角度(扇区中心/半宽做成
       不等宽, 正是为了让相连扇区的边界角度相等), 所以只要把相邻两拍的增量
       按 ±180° 展开后累加, 就得到不折叠的连续电角度; 机械角 = 连续电角度 /
       pole_pairs 天然连续.
       取代原先"霍尔边沿按扇区宽累加 + GetMechDeg 里 floor 找整圈数 k":
       那种做法里 k 的跳变点(D=acc-theta 越过 (n+0.5)*360)与 theta 的折叠点
       并不重合, 于是每个电周期机械角都会整圈跳变一次 360/pole_pairs 度
       (本机 26 极对 = 13.85°). 位置环把它当成"走过头了" → 反向猛推 →
       转子每转一个电周期就来回摇摆一次(实测 ~1s 一摇, 正对应"一动一停").
       霍尔在扇区边界抖动时 theta 的 ±60° 跳变正反对称, 净累加为 0, 不漂移. */
    {
        float d = theta_out - obs->theta;
        if (d >  3.14159265f) d -= 6.28318531f;
        if (d < -3.14159265f) d += 6.28318531f;
        obs->elec_acc_deg += d * 57.29577951f;
    }
    obs->theta = theta_out;

    if (erpm_out != NULL) *erpm_out = erpm;
    return theta_out;
}

/* ---- ???? (eRPM) ---- */
float AngleObserver_GetOmegaErpm(const Angle_Observer_t *obs, float pwm_freq)
{
    return obs->omega * pwm_freq / 6.28318531f * 60.0f;
}

float AngleObserver_GetOmegaFiltErpm(const Angle_Observer_t *obs, float pwm_freq)
{
    return obs->omega_filt * pwm_freq / 6.28318531f * 60.0f;
}

/* ---- 真实机械角度 (deg, 多圈) ----
   elec_acc_deg 是相位展开后的连续(不折叠)电角度(deg), 直接除以极对数即为
   多圈机械角. 停转时 GetTheta 的增量恒为 0 → 机械角稳定不漂移. */
float AngleObserver_GetMechDeg(const Angle_Observer_t *obs, float pole_pairs)
{
    if (pole_pairs <= 0.0f) pole_pairs = 1.0f;
    return obs->elec_acc_deg / pole_pairs;
}

/* ---- 用对齐结果自动校正霍尔表零点 ----
   对齐阶段 theta 被强制到 0°, 转子 d 轴就停在该电角度上; 读当时的霍尔状态 h0,
   由 GetSectorCenter(h0) + offset ≡ 0 反解 offset(规整到 [0,2π)).
   这样霍尔扇区表的零点偏差/接线差异会被自动吸收, 无需手调 hall_offset_deg. */
void AngleObserver_CalibrateOffsetAtZero(Angle_Observer_t *obs, uint8_t hall_state)
{
    float c;
    if (hall_state < 1U || hall_state > 6U) {
        return;                     /* 无效扇区(0/7)不校正, 保留原 offset */
    }
    c = GetSectorCenter(hall_state, obs->hall_dir_rev);
    obs->offset = -c;
    while (obs->offset >= 6.28318531f) obs->offset -= 6.28318531f;
    while (obs->offset < 0.0f)         obs->offset += 6.28318531f;
}

/* ---- ???? ---- */
void AngleObserver_SetOffset(Angle_Observer_t *obs, float offset_deg)
{
    obs->offset = offset_deg * 0.01745329f;
}

