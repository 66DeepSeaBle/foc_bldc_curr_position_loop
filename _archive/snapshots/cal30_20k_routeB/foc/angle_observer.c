/**
  * @file    angle_observer.c
  * @brief   角度观测器实现(数字霍尔: 扇区 + 扇区内插值 + 模型外推)
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

/* =====================================================================
 * 2026-10-11c: 位置计算改用"扇区序号 + 扇区内插值"的绝对算法
 * ---------------------------------------------------------------------
 * 旧做法: 把折叠后的 theta 每拍做 ±180° 相位展开后再累加(积分器)。
 * 致命缺陷: 本机霍尔环序里存在**正好相差 180° 的对跖对**(h1↔h2, h3↔h4,
 * h5↔h6 —— 表中中心角相差约 180°)。一旦漏掉一次边沿(EXTI 丢掉/受 PWM 干扰)
 * 使状态一次跳 3 个扇区, “折叠角的前后差”就正好落在 ±180° 附近, 展开成了
 * **猜正负号**: 猜错一次, 位置就整体偏 ±360° 电角度(= ±90° 机械角)且永不
 * 恢复。2026-10-11 实测日志里同一个 (h=4, th=94.3) 一会儿 Pos=0.00 一会儿
 * Pos=90.00(两者恰好差 90°机械 = 1 个电周期 360°电), 位置环于是一会儿正推
 * 一会儿反推 ⇒ 在 0~105°(Tgt=90) 之间以 ±2.5A 剧烈来回摆。
 * 新做法: 位置完全由当前(扇区, 扇区内进角)算出, 不再有任何积分器:
 *     电角度 = 360°×圈数 + 本扇区入口角 + (adv + 半扇区宽)
 * 圈数由一个**整型**计数器 sect_run(沿环序走过的扇区数)给出 —— 整型不漂、
 * 多扇区跳变的个数由环序差直接得出(无需猜正负号)。
 * ===================================================================== */

/* 扇区在"电角度递增"方向上的环序序号(0..5)。
   dir_rev=1 时递增顺序(按下面两表的入口边界角排):
     h5(0.03°) → h1(54.97°) → h3(116.04°) → h2(176.23°) → h6(231.47°) → h4(298.10°)
     ⇒ RING[h5]=0, RING[h1]=1, RING[h3]=2, RING[h2]=3, RING[h6]=4, RING[h4]=5.
   dir_rev=0 时 theta 与扇区的对应镜像了 ⇒ 环序反向: 直接查下面第二张表。
   ⚠ 环序必须沿"theta 递增"(而不是沿物理 FWD 链), 因为位置环的正方向就是
   theta 增大的方向 —— 这样“位置增量”与“闭环命令正负”天然一致。 */
static const uint8_t RING_REV1[8] = { 0U, 1U, 3U, 2U, 5U, 0U, 4U, 0U };
static const uint8_t RING_REV0[8] = { 0U, 0U, 4U, 5U, 2U, 1U, 3U, 0U };

static int32_t RingOfState(uint8_t h, uint8_t dir_rev)
{
    if (h < 1U || h > 6U) return 0;
    return (int32_t)(dir_rev ? RING_REV1[h] : RING_REV0[h]);
}

/* 本扇区"入口边界"的绝对电角度(rad, 已含 dir_rev 镜像) */
static float SectorEntryRad(uint8_t h, uint8_t dir_rev)
{
    float e = GetSectorCenter(h, dir_rev) - SECTOR_HALFWIDTH[h];
    while (e < 0.0f)         e += 6.28318531f;
    while (e >= 6.28318531f) e -= 6.28318531f;
    return e;
}

/* ---- ??? ---- */
void AngleObserver_Init(Angle_Observer_t *obs, float hall_offset_deg,
                        uint8_t hall_dir_rev)
{
    obs->theta = 0.0f;
    obs->theta_base = 0.0f;
    obs->theta_halfwidth = 0.52359878f;
    obs->adv = 0.0f;
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
    obs->last_get_tick = 0U;
    obs->pos_ext_w    = 0.0f;   /* 2026-10-11ac 边沿间外推速度 */
    obs->pos_ext_tick = 0U;
    obs->sect_run    = 0;
    obs->pos_abs_rad = 0.0f;
    obs->pos_ref_rad = 0.0f;
    obs->pos_seeded  = 0U;
    obs->last_dir    = 0;
    obs->bad_cnt     = 0U;
}

/* =====================================================================
 * 2026-10-11f: 20kHz 轮询式霍尔边沿检测(取代 EXTI 触发)
 * ---------------------------------------------------------------------
 * 为什么换: 10-11d/e 日志里位置读数会整体跳 ±90°(机械) = ±1 个电周期,
 * 而 90°机械 = 6 个扇区, 只可能来自"扇区序号一次少/多算 6" —— 即**漏掉了
 * 边沿**(EXTI 丢中断/被高优先级 ISR 挤掉/PWM 干扰合并), 之后算法只能靠
 * "猜方向"补, 猜错就是整整一个电周期(位置环于是正反猛推: ±60° 大摆 +
 * ±2.5A 来回, 测速也被同源的假差分污染成 ±1300eRPM 尖刺)。
 *
 * 本函数由 20kHz 的 ADC 中断调用(那里本来就在读三个霍尔脚):
 *  · 50µs 采样一次, 转子跨过 1 个扇区(15.5°机械)最快也要 ~200 个采样点
 *    ⇒ **状态序列一定是相邻链**, 不可能跳过中间态;
 *  · 所以"非相邻跳变"在物理上不可能(50µs 跨两个扇区 = 30 万°/s)
 *    ⇒ 一律当**噪声丢弃**, 不去动位置;
 *  · 两次接受的边沿间隔还要求 ≥ min_dt_ticks(4 拍 = 200µs, 同样物理上
 *    不可能更快) ⇒ 接触抖动/干扰脉冲全滤掉。
 * 这样扇区序号只会在确认的相邻跳变上 ±1, 结构上不可能再整圈漂。
 * ⚠ EXTI 那条路径已停用(否则同一跳变会被计两次)。 */
uint8_t AngleObserver_PollHall(Angle_Observer_t *obs, Hall_State_t *hall,
                               uint8_t h_new, uint32_t now_tick, float pwm_freq,
                               float min_dt_ticks, float omega_clamp_erpm,
                               float alpha_max)
{
    uint8_t  prev = hall->prev_state;
    uint32_t dt;

    if (h_new < 1U || h_new > 6U) {        /* 0/7 非法态: 不接受边沿 */
        return 0U;
    }
    if (h_new == hall->state) return 0U;   /* 没变化 */

    dt = now_tick - hall->edge_tick;

    if (prev >= 1U && prev <= 6U) {
        if (!((h_new == FWD_NEXT[prev] || h_new == BWD_NEXT[prev]) &&
              dt >= (uint32_t)min_dt_ticks)) {
            /* 非相邻 / 太快 ⇒ 先当噪声, **不改 hall->state**(下次采样再判同一个
               读数), 持续到 10ms 才认它(防干扰卡死位置) */
            obs->bad_cnt++;
            if (obs->bad_cnt < 200U) return 0U;
        }
    }

    obs->bad_cnt  = 0U;
    hall->state   = h_new;
    AngleObserver_OnHallEdge(obs, hall, now_tick, pwm_freq,
                             min_dt_ticks, omega_clamp_erpm, alpha_max);
    return 1U;
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
    int8_t prev_dir;

    if (h == 0 || h == 7) {
        obs->synced = 0;
        return;
    }

    prev_dir = obs->last_dir;        /* 进入本次边沿前的"上次确认方向" */

    /* ---- ???? ---- */
    if (hall->prev_state >= 1 && hall->prev_state <= 6) {
        if (h == FWD_NEXT[hall->prev_state]) {
            dir = 1;
            obs->last_dir = 1;          /* 相邻跳变确认方向(供对跖情况用) */
        } else if (h == BWD_NEXT[hall->prev_state]) {
            dir = -1;
            obs->last_dir = -1;
        } else {
            /* 非相邻跳变(丢步/多步): 沿用上一次确认过的方向; 没记录过才退回 omega 符号 */
            dir = (obs->last_dir != 0) ? obs->last_dir
                                       : ((obs->omega >= 0.0f) ? 1 : -1);
        }
    } else {
        dir = 1;
    }

    /* ---- 扇区序号累加(2026-10-11c, 位置算法的核心) ----
       位置不再靠“折叠角积分”, 而是“整型扇区序号 + 扇区内进角”, 所以这里
       必须把“跨了几个扇区、往哪边”记准: 相邻跳变的方向由 FWD/BWD 表直接
       给出; 非相邻跳变(漏边沿/受干扰)用 omega 的符号定方向, 再按环序取该
       方向的步数 —— 180° 的对跖跳变必须由方向定, 不能靠 ±180° 就近展开
       (那正是 2026-10-11 实测“位置整搬 90°”的根源)。 */
    if (obs->pos_seeded == 0U) {
        obs->sect_run  = RingOfState(h, obs->hall_dir_rev);
        obs->pos_seeded = 1U;
    } else if (hall->prev_state >= 1U && hall->prev_state <= 6U) {
        int32_t r0 = RingOfState(hall->prev_state, obs->hall_dir_rev);
        int32_t r1 = RingOfState(h, obs->hall_dir_rev);
        int32_t d  = r1 - r0;
        /* ★ 步数取"环序最短路径"(±3 以内) —— 绝不能用 dir 去强行定符号:
           dir 在非相邻跳变时只能猜(omega/上一次方向), 猜错会把 2 步变成 -4 步
           (= 整整 6 个扇区 = ±90° 机械角的位置大跳)。最短路径则是确定的:
             漏 1 条边(跳 2 扇区) → |d|=2 ✓ 唯一
             漏 2 条边(跳 3 扇区) → |d|=3（正好对跖, 两种方向等价于 ±3）
           ⇒ 只有 |d|==3 这种对跖情况才必须靠方向决断, 用上一次**相邻**跳变
              确认过的方向(last_dir) —— 比瞬时 omega 可靠得多。 */
        if (d >  3) d -= 6;
        if (d < -3) d += 6;
        if (d == 3 || d == -3) {
            d = (obs->last_dir >= 0) ? 3 : -3;
        }
        obs->sect_run += d;
    }

    /* ★★ 2026-10-11af 方向反转 ⇒ "出去又回来", 净位移≈0 ----------------------
       现象(10-11ae 日志): 转子停在齿槽=霍尔边界上, 一点机械抖动就让霍尔状态
       来回翻。每条边沿都按 `扇区宽/dt` 算出 **±600~700eRPM 的假速度**
       (实测 ob=-690), 而位置环经 10-11ad 的"最近有边沿就相信 v"直接采信 ⇒
         ① 环路以为转子在高速倒转 ⇒ 拿 2.5A 去"补偿" ⇒ 真的把转子甩出去
            (实测 t45 d0.8 被毁: vsp 冲到 -1000eRPM = 规划的 10 倍);
         ② 模型进角 adv 被反复重载(τ=5ms 的衰减来不及生效) ⇒ 一路爬到扇区
            钳位(adv=+33.32) ⇒ 电角度差 60° ⇒ 力矩只剩一半 ⇒ 日志前段那段
            "2.3A 顶 4.6 秒转子一动不动"就是这么来的。
       判据: **相邻两条边沿方向相反** = 转子出去又回来 ⇒ 平均速度≈0。
       处理: 测速/加速度/进角外推全清零, adv 重置到入口, 并清掉窗口历史
       (否则窗口估计会把"半去半回"平均成一个非零值); **扇区序号也还回去**
       (否则读数会凭空整跳一个扇区—— 10-11af 实测日志里 `h=2 Pos=45.94`
       与 `h=2 Pos=32.13` 交替出现就是这个, 而 h 根本没变)。
       注意这是**真边沿**(不是 glitch): edge_tick 照常更新,
       dte/停转判据/边沿计数都不受影响。 */
    if (prev_dir != 0 && dir != prev_dir) {
        /* ⚠⚠ 2026-10-11ai **绝不能在这里动 sect_run**(10-11ag 的 `sect_run -= dsect`
           是本轮 1/2/4/8 全部抖动的真正根因, 已删)。
           理由: 位置公式是
               pos = 2π·((sect_run - Ring(state)) / 6) + entry + local
           那个 `/6` 是**整型除法**, 整段算法依赖一条隐含不变量
               sect_run ≡ Ring(hall.state)  (mod 6)
           —— 只有同余成立时除出来的才是精确的圈数。
           而“边沿累加”那一步(obs->sect_run += d, d = Ring(new)-Ring(old)) **恰好**
           在维持这条不变量。10-11ag 在方向反转时把 d 还回去 ⇒ 不变量被打破
           ⇒ 整数除法向零截断 ⇒ 位置凭空多/少 **1/6 圈 = 15.4°机械**(连
           两次反转就 30.8°)。实测(10-11ah 日志)完全对得上:
             · 同一个转子读数在 -60.79(h=1,adv=-30.53) 与 -34.16(h=3,adv=+15.31)
               之间以 20~40Hz 来回跳(差 26.6° ≈ 15.4 + 3.7 外推);
             · `jp`(“物理上不可能的位置跳变”计数) 2.4 秒里 0→137;
             · err=+34.16 却给 Iqref=-1.30, 而 err=+60.79 给 +2.50 —— 同一个
               误差符号给出**相反**的电流符号, 说明读数本身已经不是一个
               连续的位置通道; 环路于是 20~40Hz 地正反推 = 用户听到的抖动。
           现在: 反转只清测速历史/外推速度(那是它本来要解决的问题), 位置
           继续由 (扇区, 进角) 按不变量算出 ⇒ 反转时 adv = -dir·hw 正好落在
           两个相邻扇区**共享的那条边界**上 ⇒ 读数连续 ✓。 */
        for (int i = 0; i < 4; i++) {
            obs->edge_hist_tick[i] = 0U;
            obs->edge_hist_acc[i]  = 0.0f;
        }
        obs->edge_hist_idx   = 0U;
        obs->omega           = 0.0f;
        obs->omega_filt      = 0.0f;
        obs->omega_prev      = 0.0f;
        obs->alpha           = 0.0f;
        obs->alpha_filt      = 0.0f;
        obs->pos_ext_w       = 0.0f;
        obs->pos_ext_tick    = now_tick;   /* 别让 GetTheta 再用旧速度重载 */
        hall->prev_state     = h;
        hall->edge_tick      = now_tick;
        hall->prev_dt        = 0U;         /* 抖动 ⇒ dt 无意义 */
        hall->dir            = dir;
        obs->theta_base      = GetSectorCenter(h, obs->hall_dir_rev);
        obs->theta_halfwidth = SECTOR_HALFWIDTH[h];
        obs->adv             = -(float)dir * obs->theta_halfwidth;
        return;
    }

    dt = now_tick - hall->edge_tick;

    /* ---- ???? ---- */
    if (dt < min_dt_ticks) {
        obs->glitch_cnt++;
        obs->theta_base = GetSectorCenter(h, obs->hall_dir_rev);
        obs->theta_halfwidth = SECTOR_HALFWIDTH[h];
        obs->adv = -(float)dir * obs->theta_halfwidth;
        hall->dir = dir;
        /* ⚠ 2026-10-11c: 这里必须更新 prev_state。原来不更新 ⇒ 下一次边沿
           拿“两条边沿前”的旧状态去算方向, 得到“非相邻”→ dir 只能猜, 且
           上一次边沿造成的扇区序号变化会算错(位置多跳/少跳一个扇区).
           edge_tick 仍不更新(抖时间仍按真实边沿算, 测速不受影响)。 */
        hall->prev_state = h;
        return;
    }

    if (dt > 0 && dt < FOC_TICKS_PER_S) {  /* 1s 上限 */
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
            /* 判据: GetTheta 最近(1ms 内)真的在跑 ⇒ 窗口测速可信。
               2026-10-01 改动(★解决到位后的嗡嗡振动): 原判据是"dacc 必须
               非零", 本意是防 ALIGN/STARTUP 期间 elec_acc_deg 冻结。 但在闭环
               里转子停在霍尔边界上时 dacc 也恰好 ≈0(来回掉), 于是窗口估计
               被跳过、保留了**单边沿**估计 sector_width/dt —— 边沿一密就
               变成 ±200~3000eRPM 的尖刺。 该尖刺乘 kd(0.006) 就是 ±1.5A
               的抖动电流, 真的把机构摇起来, 而位置反馈又因 adv 限幅而
               "看不见"这个抖动 ⇒ **自激嗡嗡声**(实测到位后 err=0 但
               Iqref 在 ±1.7A 来回、vsp=±250)。
               改用 last_get_tick 判"GetTheta 是否在跑": 闭环里永远用窗口
               估计(转子真不动时 dacc≈0 ⇒ omega≈0 ⇒ 不注入噪声);
               ALIGN/STARTUP 期还退出单边沿估计 ⇒ 同步判据不受影响。 */
            /* ⚠ 2026-10-11g: 加 dt >= 1000(50ms) 条件 —— 只在\"慢\"的时候用窗口平均.
               窗口跨 3 个扇区(~45°机械), 高速/加速时严重滞后(实测转子 674°/s 而
               上报 84eRPM, 差 8 倍 ⇒ 环路继续正推 ⇒ 冲过目标 15~80°). 快的时候
               用上面的单边沿估计(只滞后 1 个扇区), 慢的时候才用窗口平滑。 */
            if (t0 != 0U && now_tick > t0 && dt >= (50U * FOC_TICKS_PER_MS) &&
                (now_tick - obs->last_get_tick) < (20U * FOC_TICKS_PER_MS)) {
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
        /* ---- ★★ 2026-10-11j: 长时间没边沿(堵转/极慢)之后的**第一条**边沿, 不
           能用"含堵转时间段的窗口"去平均测速。
           实测(10-11i 日志): 卡在齿槽里 400ms 后一脱困, 窗口跨的那一段大部分
           是"停着不动", dacc/Δt 被拉低到几十分之一 —— 环路以为 "vsp=-14eRPM
           几乎没动", 于是继续用 1.87A 猛推, 一路推 40~46° 才看到高速去反拉。
           所以: 把这些历史清掉, ω_filt 直接取本次单边沿实测值(不用旧值平均),
           下一条边沿起才恢复窗口平滑。 */
        if (dt > (50U * FOC_TICKS_PER_MS)) {
            for (int i = 0; i < 4; i++) {
                obs->edge_hist_tick[i] = 0U;
                obs->edge_hist_acc[i]  = 0.0f;
            }
            obs->edge_hist_idx = 0U;
            obs->omega_filt = obs->omega;          /* 不用陈旧值平均 */
        } else {
            obs->omega_filt = 0.5f * obs->omega_filt + 0.5f * obs->omega;
        }
        obs->synced = 1;
    }

    /* ---- ???? ---- */
    hall->prev_state = h;
    hall->edge_tick = now_tick;
    hall->prev_dt = dt;
    hall->dir = dir;
    obs->theta_base = GetSectorCenter(h, obs->hall_dir_rev);
    obs->theta_halfwidth = SECTOR_HALFWIDTH[h];
    /* 刚进入(h 为当前)扇区 ⇒ 进角重置到入口边界(+1 入时在入口下边界, -1 入时
       在入口上边界). 因为相邻扇区的边界角度是相等的(扇区表按此构造), 这一步
       不会给 theta 带来跳变 ⇒ 位置读数在边界处连续。 */
    obs->adv = -(float)dir * obs->theta_halfwidth;
}

/* ---- ?????? (?PWM????) ---- */
/* 无新边沿时模型速度的衰减时间常数(秒): 2026-10-11h 新增, 见 GetTheta 内注释。
   ★ 2026-10-11k 0.15→0.08: 实测交接点后模型还带着 I-F 拖转的速度(189eRPM)
   继续"爬"了 9.4°机械(= 0.6 个扇区)才停, 而转子其实早就被摩擦停住了
   (减速约 290°/s², 200ms 就停) ⇒ 位置读数过量最多达半个扇区, 位置环
   于是以为"还差 15°"去反推一个本来就到位的转子 —— 顶在齿槽壁上持续
   振动, 顶久了还会塴出去。缩短到 80ms 后过量 ≤ v×0.08s(≈2~4°), 
 * 而真在慢速爬行时每条边沿都会把它重置, 不受影响。
 * ★ 2026-10-11v 0.08→0.035: 实测(10-11u 日志) 转子已经停住、`dte` 稳定、
 *   `mv=0`(100ms 内没跨扇区), 但读数在 500ms 里 **自己爬了 12°**
 *   (Pos 182→195, adv -19.95→+28.83, 全程 h 不变 ⇒ 转子物理上没动), 而模型
 *   外推量就是 v_last×τ: 最后一次边沿测到 77eRPM(=115°/s) ⇒ 0.08×115 = 9.2°
 *   ⇒ 刚好把 err 从 3° 推到 15° ⇒ 又触发"掰一格"⇒ 又一次飞出去/摆回来。
 *   改 35ms 后同样的假速度只推 4°(< 半个齿槽), 跨不过"掰一格"的 13° 门槛。
 *   真在动时每条边沿都会重置它, 不受影响(高速时 23ms 一条边沿, 35ms
 *   仍能覆盖大部分区间)。
 * ★★ 2026-10-11aa 0.035→0.010 —— 实测证明 35ms **还是一次走满一个扇区**:
 *   10-11z 日志 `t=35536..41146 Pos=195.47 Tgt=180.00 err=-15.47 Iqref=-0.00
 *   adv=30.95 dte 2000→7600 h=4(=不动)`: 转子明明停住了, 而 adv 钉在 +30.95°(电)
 *   = **正好走完一个扇区**(读数从 180.00 爬到 195.47 = 15.47°机械), 且这 15.47°
 *   与"退出到位死区"的 15.5° 只差 0.03° ⇒ 环路永久撤力、差一个齿槽修不回来。
 *   外推量的物理含义就是"最后速度 × τ": 要它小于半个齿槽(7.7°机械 ≈ 31°电),
 *   就需要 v_last × τ < 31°电; 而近目标实测最后一条边沿的速度可到 200~300eRPM
 *   (=1800~2700°/s电) ⇒ τ 必须 ≤ 11~17ms。取 10ms(外推 ≤ 270°×0.01 = 2.7°电
 *   = 0.7°机械, 完全无害)。真在动时每条边沿都会重置它; 慢速爬行时边沿间隔
 *   ~80ms, 外推不足的部分由下一条边沿的"重置到入口边界"补上(读数落在
 *   入口~半个扇区之间, 这是本方案分辨率的固有极限, 方向上是**偏保守**的:
 *   读数只会落后、绝不虚前, 环路因此不会去修一个不存在的误差)。 */
#define POS_MODEL_TAU_S   0.005f

float AngleObserver_GetTheta(Angle_Observer_t *obs, const Hall_State_t *hall,
                             uint32_t now_tick, float pwm_freq,
                             uint32_t timeout_ticks, float creep_hz,
                             uint8_t creep_suppress, float *erpm_out)
{
    uint32_t dt = now_tick - hall->edge_tick;
    float dt_s = (dt > 0U) ? ((float)dt / pwm_freq) : 0.0f;
    float theta_out;
    float erpm = 0.0f;
    uint8_t live = (obs->synced && dt < timeout_ticks) ? 1U : 0U;

    /* ==== 扇区内进角估计: 逐步累加, 不在超时/停转时塌缩到扇区入口 ====
       2026-10-01 v3 重写. 旧实现是 adv = omega*dt - dir*hw(用 dt 重算),
       带来两个致命副作用:
        (1) 停转时 omega 衰减到 0 ⇒ adv 塔缩到 -dir*hw(扇区入口边界) ⇒
            扇区内位置信息全部丢失, 位置读数被量化到 15°机械的格点上;
            若目标不在格点上, 闭环永远进不了容差。
        (2) 上一次为了防"卡死"又把 adv 限幅成 ±0.5×hw, 结果破坏了"相邻扇区
            边界角度相等"这个连续性 —— 转子停在边界上时霍尔抖动, 读数在
            两个相邻扇区之间跳 ±7.5°机械, 误差正负交替 ⇒ 电流 ±0.9A 来回
            推 ⇒ 永久限幅环(实测: Tgt=0 时 h 在 4/5 之间抖, Pos 在 -7.74/
            +7.31 之间跳, err 永远 7.3~7.7°, arr 永远 0; Tgt=45/90 同样,
            因为 45/90 都是 15°的整数倍 ⇒ 正好落在霍尔边界上)。
       现在: obs->adv 在每次霍尔边沿重置为 -dir*hw(入口边界, 与相邻扇区
       边界角度相等 ⇒ 连续), 之后每拍 adv += omega_filt/pwm_freq 并限幅到
       ±hw.
        · 停转/超时: adv 保持不动 ⇒ 保留扇区内真实位置 ⇒ 读数连续、准确
          (不再塌缩到格点);
        · 边界抖动: 正向抖动进新扇区 adv=-hw, 反向抖动回旧扇区 adv=+hw,
          两者算出的 theta 都是同一条边界 ⇒ 读数不跳 ⇒ 无限幅环;
        · 不会卡死: 磁平衡点 θc=θest+90° = 真实位置+90°(±60°电误差),
          sin 始终 ≥0.5 且指向正方向 ⇒ 必然越过霍尔边界。
       物理上限: 自上次边沿起过了 dt 还没跳边沿 ⇒ 平均速度不可能超过
       1.5 个扇区宽/dt; 用它限制 omega_filt, 停转时 adv 自然不会继续冲。 */
    if (!live) {
        obs->omega = 0.0f;
        obs->omega_filt = 0.0f;
        obs->pos_ext_w  = 0.0f;                 /* 2026-10-11ac 外推速度也归零 */
        obs->pos_ext_tick = hall->edge_tick;
        obs->alpha_filt = 0.0f;
        if (!creep_suppress) {
            float creep_step = 2.0f * 3.14159265f * creep_hz / pwm_freq;
            obs->adv += (float)hall->dir * creep_step;
        }
    } else {
        float w = obs->omega_filt;
        if (dt_s > 1.0e-6f) {
            /* ⚠ 单位: omega/omega_filt 的单位是 rad 每 tick(见 OnHallEdge 的
               `omega = sector_width/dt_ticks`, 以及那里的
               `omega_clamp = erpm*2π/60/pwm_freq`) ⇒ "rad/s"的物理上限
               要再 /pwm_freq 才换算成 rad/tick。
               原来漏了这个 /pwm_freq(20000) ⇒ 限幅永不生效 ⇒ 停转后 w 一直
               保持上一次边沿的旧值(实测卡住时 vsp 冻在 136eRPM 长达 0.7s),
               于是阻尼项 -kd*vsp 把 P 项正好抵消 ⇒ Iqref 只剩 -0.01A, 转子
               推不动, 要等 1s 超时(live=0)才恢复; kd 调得越大这个假速度
               吃掉的力越多。 */
            /* 速度物理上限: 自上一条霍尔边沿已过 dt 仍未跳边沿 ⇒ 平均速度不可能
               超过(半扇区宽×系数)/dt。系数由 3.0 降到 1.0(2026-10-09h):
               卡在齿槽里(400ms 没边沿、转子确实没动)时, 残留 omega_filt 只按
               1/dt 缓慢衰减, 实测还有 40~90eRPM ⇒ ① 位置环积分门控(|v|<40)
               一直不打开 ⇒ Ipos 恒 0, 只剩 P 的 0.6A, 推不出齿槽、只能等破黏;
               ② 更早以前还会经阻尼抵消 P 项。系数 1.0 后 dt=150ms 就降到 <40eRPM,
               积分门控能自己打开。只在"停转"区间收紧, 正常运转(dt 小)不受影响。 */
            float wmax = 2.0f * obs->theta_halfwidth / (dt_s * pwm_freq);
            if (w >  wmax) w =  wmax;
            if (w < -wmax) w = -wmax;
            /* ★★ 2026-10-11ac **外推速度与上报速度分开**(本轮抖动的主因):
               原来只有一份 `omega_filt`, 而且衰减写在 `omega_filt = w` **之后**
               ⇒ 衰减只作用在"这一拍的增量"上, 下一拍又从 omega_filt 拿回
               未衰减的原值 ⇒ **衰减根本不累积**。
               实测(10-11aa 日志铁证): 停转后 `ob` 不按指数衰减, 而是 **∝1/dt**
                 (279/147/97/73/58/48/42/36/32/29/27 @ dte 50/85/120/…/390ms,
                  恰好就是物理上限 2×半扇区/dt), 于是 `adv` 以 1/dt 的速率继续
                 爬 ⇒ **读数在 ~400ms 里从扇区入口一路爬到扇区钳位 (+15.47°机械)**
                 ⇒ 静止时凭空造出"差一个齿槽"的假误差 ⇒ 温和爬行每 ~450ms 推一次
                 (1.3A, 而且每次都真的把转子推过一个边界) ⇒ 用户看到"到达目标后
                  一直轻微摆动"(2Hz 小幅爬行→回弹 极限环)。
               修法: 外推单独立一个状态 `pos_ext_w` —— 每条霍尔边沿重载实测速度,
               之后按 τ **累积**衰减 ⇒ 停转时 adv 最多再走 v_last×τ
               (τ=5ms ⇒ ≤5°电 = 1.25°机械) ⇒ 读数立刻停在真实位置(刚走过的那个
               齿槽边界) ⇒ err 落进 ±6° 死区 ⇒ 真正安静。
               上报给环路的 omega_filt 保持原样(不动速度环的既有标定)。 */
            obs->omega_filt = w;
            if (hall->edge_tick != obs->pos_ext_tick) {
                obs->pos_ext_tick = hall->edge_tick;
                obs->pos_ext_w    = obs->omega;   /* 新边沿 ⇒ 用实测速度重载 */
            }
            if (obs->pos_ext_w >  wmax) obs->pos_ext_w =  wmax;
            if (obs->pos_ext_w < -wmax) obs->pos_ext_w = -wmax;
            {
                float tau_ticks = POS_MODEL_TAU_S * pwm_freq;
                if (tau_ticks > 1.0f) obs->pos_ext_w *= (1.0f - 1.0f / tau_ticks);
            }
        }
        /* 同单位 bug: w 是 rad/tick, 每拍就应 += w(原来 += w/pwm_freq, 小 20000
           倍) ⇒ adv 实际上钉在扇区入口边界不动 ⇒ **位置读数被量化到扇区
           边界**(实测 Pos 反复只出现 44.48/59.53/73.34/105.48 这类格点值,
           目标 45/90 恰好落在格点上所以能到位, 但看到的"过冲 15°"其实主要是
           越过边界后读数跳到下一个格点, 不是真的冲了 15°)。
           ★ 2026-10-11ac: 用**外推速度** pos_ext_w(按 τ 累积衰减), 不再用
           omega_filt(那里存的是给环路看的速度, 不衰减)。 */
        obs->adv += obs->pos_ext_w;
        /* 2026-10-01: 上报速度也用**限幅后**的 w。 原来用 obs->omega, 它在
           两个边沿之间是"冻结"的旧值 ⇒ 停转后 dt 越拉越长、物理上不可能
           还保持那个速度, 但 vsp 会一直显示旧值(实测停在 404 不动), 
           而阻尼项 -kd*vsp 就白推了 ±1A 的电流。 */
        erpm = w * pwm_freq / 6.28318531f * 60.0f;
    }

    if (obs->adv >  obs->theta_halfwidth) obs->adv =  obs->theta_halfwidth;
    if (obs->adv < -obs->theta_halfwidth) obs->adv = -obs->theta_halfwidth;

    theta_out = obs->theta_base + obs->adv + obs->offset;

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
    {   /* ---- 位置: 由 (扇区序号, 扇区内进角) 直接算出, 不做积分 ----
           2026-10-11c 重写(原因见 RingOfState 上方的长注释): 不再对折叠后的
           theta 做 ±180° 展开累加 —— 对跖扇区(h1↔h2, h3↔h4, h5↔h6 相差
           180°)被漏掉一次边沿时那一步展开就会猜错符号, 位置整体偏 ±360°电
           (±90°机械)且永不恢复, 闭环于是正推/反推交替 = 剧烈来回摆。
           现在位置 = 360°×圈数(sect_run 整型推算) + 本扇区入口角 +
           (adv + 半扇区宽), 完全由当前状态决定, 没有可漂的积分器;
           多扇区跳变按环序差一步到位(不需猜符号, 方向由 dir 给出)。 */
        uint8_t hs = hall->state;
        if (hs >= 1U && hs <= 6U && obs->pos_seeded != 0U) {
            int32_t r    = RingOfState(hs, obs->hall_dir_rev);
            float   local = obs->adv + SECTOR_HALFWIDTH[hs];        /* 0 .. 2hw */
            float   entry = SectorEntryRad(hs, obs->hall_dir_rev);
            /* 圈数用 (sect_run - r)/6 算: sect_run ≡ r (mod 6) 由构造保证,
               所以这个差恰好是 6 的整数倍(不会出现除不尽的情况)。
               ★ 2026-10-11ai: 这里改用**四舍五入**而不是 C 的向零截断 ——
               万一哪条路径意外打破了同余(例如算法以后再改), 四舍五入给出
               的是“最近的那一圈”, 误差最多半个电周期(45°电=11°机械并随后被
               下一次边沿纠正); 而向零截断会把 -1/6 圈直接算成 0 圈 ⇒
               凭空 +15.4°机械的位置偏置(这次的就是它)。 */
            float turns = (float)(obs->sect_run - r) / 6.0f;
            float pos_abs = 6.28318531f * lroundf(turns)
                            + entry + local;
            obs->pos_abs_rad  = pos_abs;
            obs->elec_acc_deg = (pos_abs - obs->pos_ref_rad) * 57.29577951f;
        }
    }
    obs->theta = theta_out;
    obs->last_get_tick = now_tick;   /* 标记"GetTheta 在跑", 供窗口测速判据用 */

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

/* ---- 把当前位置定为 0(交接用) ----
   2026-10-11c: 位置不再是“可累加的积分器”, 而是由 (扇区, 进角) 算出的绝对值,
   所以“清零”= 把参考点搬到当前位置(不能再去写 elec_acc_deg, 那样下一拍就被
   覆盖)。 */
void AngleObserver_ZeroPos(Angle_Observer_t *obs)
{
    obs->pos_ref_rad  = obs->pos_abs_rad;
    obs->elec_acc_deg = 0.0f;
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

