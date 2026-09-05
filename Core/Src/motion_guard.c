/**
  ******************************************************************************
  * @file    motion_guard.c
  * @brief   位置/速度级联控制的防冲击护栏模块实现，见motion_guard.h说明。
  ******************************************************************************
  */
#include "motion_guard.h"
#include <math.h>

/* 真正撑屏障的公共实现：给定severity(已经钳过范围)，算出限速/限流并生效。 */
static void MotionGuard_ArmBarrierWithSeverity(MotionGuard_t *mg, uint32_t now, float severity)
{
  const MotionGuardConfig_t *cfg = &mg->cfg;

  mg->barrier_active = 1;
  mg->barrier_start_ms = now;
  mg->barrier_speed_cap_erpm = cfg->barrier_base_erpm / severity;
  mg->barrier_iq_cap = cfg->barrier_base_iq / severity;
}

/* 按观测到的转速幅度支起(或续期)屏障：severity=|omega|/breakaway_omega_erpm，
   最低钳到1.0(哪怕omega本身没有很快，也至少给基准力度的屏障)，最高钳到
   severity_max防止极端尖峰把上限压到接近0。用于"直接观测到高速"这类
   场景(ISR单边沿骤停骤起、erpm_now持续超阈值)，omega_erpm本身就能真实
   反映挣脱力度。 ---- */
static void MotionGuard_ArmBarrier(MotionGuard_t *mg, uint32_t now, float omega_erpm)
{
  const MotionGuardConfig_t *cfg = &mg->cfg;
  float severity = fabsf(omega_erpm) / cfg->breakaway_omega_erpm;

  if (severity < 1.0f) severity = 1.0f;
  if (severity > cfg->barrier_severity_max) severity = cfg->barrier_severity_max;

  MotionGuard_ArmBarrierWithSeverity(mg, now, severity);
}

/* 卡滞助推/giveup"诊断退出"这类场景专用：这两条路径的退出判据本身就是
   erpm_now刚过kick_erpm(30)这个很低的阈值，用它去算severity(=erpm_now/
   breakaway_omega_erpm(1500))必然远小于1.0、被下限钳死在1.0(最弱档)，
   算出来的barrier_iq_cap=barrier_base_iq/1.0=0.6A——跟卡滞助推自己的
   kick_iq(0.6A)完全一样，等于没收紧。但实测证实这类退出之后转子往往
   带着卡滞助推攒的动能继续冲(挣脱那一刻起stall_kicking/giveup标志位
   已经清零，后续这段真实位移全部被算成"不在卡滞助推/giveup期间发生"，
   date=25冲出约1380°那次就是这样，kick=0/giveup=0全程)——用erpm_now
   这个"退出判据本身很低"的量去算severity从根上就衡量不出真实挣脱力度，
   必须用固定的、比kick_iq本身更紧的severity，不能依赖观测到的瞬时转速。 */
void MotionGuard_ArmBarrierKickExit(MotionGuard_t *mg, uint32_t now)
{
  const MotionGuardConfig_t *cfg = &mg->cfg;
  float severity = cfg->kick_exit_barrier_severity;

  if (severity < 1.0f) severity = 1.0f;
  if (severity > cfg->barrier_severity_max) severity = cfg->barrier_severity_max;

  MotionGuard_ArmBarrierWithSeverity(mg, now, severity);
}

/* giveup基准转矩的"远近分区"：废除"giveup恒定用固定基准推力，只有
   stall_kicking才按剩余距离收紧"这个隔离设计(见giveup_iq在motion_guard.h
   里的说明)——实测证实这个隔离导致"不管离目标多近，giveup挣脱的位移量
   级都一样"，最终"收敛"变成"多轮不可控幅度的挣脱里恰好有一次落进容差"，
   不是真正意义上的逼近。dist_scale由main.c按剩余距离(度)换算成[0,1]的
   比例传入(远距离=1.0，近距离趋于0，见main.c里FOC_POS_GIVEUP_SCALE_
   DIST_DEG说明)：dist_scale=1时基准钉在giveup_iq(0.5A，保证兜底脱困
   能力不打折)，dist_scale趋于0时基准线性滑向giveup_iq_floor(离目标近，
   没必要再用大力气冲，柔化下来)。非位置环场景(main.c传固定1.0)行为
   等价于原来的"恒定基准"，不受影响。 */
static float MotionGuard_GiveupBaseIq(const MotionGuardConfig_t *cfg, float dist_scale)
{
  if (dist_scale < 0.0f) dist_scale = 0.0f;
  if (dist_scale > 1.0f) dist_scale = 1.0f;
  return cfg->giveup_iq_floor + (cfg->giveup_iq - cfg->giveup_iq_floor) * dist_scale;
}

/* giveup斜率自适应衰减：given "已经越过giveup_ms门槛多久"，在上面按距离
   算出的基准之上，再叠加"卡越久力度越收紧"这一层——前giveup_decay_
   step_ms这一档还是满额的距离基准(短时助推克服轻微卡滞)，之后每过一档
   下调giveup_decay_step_iq，直到giveup_iq_floor封底(长期卡死，主动
   降低静态转矩，减小一旦真正挣脱那一刻的初始加速度/动能，不再让转子在
   满额静推力下"蓄力顶死")。giveup_decay_step_ms==0表示不启用时间衰减，
   只保留距离基准(等价于只做远近分区、不做卡滞时长惩罚)。 */
static float MotionGuard_GiveupDecayedIq(const MotionGuardConfig_t *cfg, uint32_t ms_past_giveup,
                                          float dist_scale)
{
  float base_iq = MotionGuard_GiveupBaseIq(cfg, dist_scale);
  float iq;
  uint32_t steps;

  if (cfg->giveup_decay_step_ms == 0)
  {
    return base_iq;
  }

  steps = ms_past_giveup / cfg->giveup_decay_step_ms;
  iq = base_iq - (float)steps * cfg->giveup_decay_step_iq;
  if (iq < cfg->giveup_iq_floor) iq = cfg->giveup_iq_floor;
  return iq;
}

void MotionGuard_Init(MotionGuard_t *mg, const MotionGuardConfig_t *cfg)
{
  mg->cfg = *cfg;

  mg->stall_since_ms = 0;
  mg->stall_kicking = 0;
  mg->stall_phase_ms = 0;
  mg->stuck_episode_start_ms = 0;
  mg->giveup_active = 0;
  mg->giveup_dir = 1.0f;
  mg->giveup_exit_lock_until_ms = 0;
  mg->not_arrived_since_ms = 0;

  mg->stall_entry_edge_baseline = 0;
  mg->stall_exit_confirming = 0;
  mg->stall_exit_edge_baseline = 0;
  mg->post_stall_freeze_until_ms = 0;

  mg->pos_err_frozen = 0;
  mg->pos_err_frozen_deg = 0.0f;
  mg->last_pos_err_live_deg = 0.0f;
  mg->pos_err_output_deg = 0.0f;
  mg->last_getpos_calc_ms = 0;

  mg->breakaway_pending = 0;
  mg->breakaway_omega_pending = 0.0f;
  mg->barrier_active = 0;
  mg->barrier_start_ms = 0;
  mg->barrier_speed_cap_erpm = 0.0f;
  mg->barrier_iq_cap = 0.0f;
  mg->barrier_dir = 1.0f;

  mg->kick_exit_hold_until_ms = 0;
  mg->kick_exit_hold_dir = 1.0f;
}

float MotionGuard_Update(MotionGuard_t *mg, float erpm_now, float err,
                          uint32_t now, float settle_iq_cap, uint8_t arrived,
                          float giveup_dist_scale, uint32_t edge_cnt)
{
  const MotionGuardConfig_t *cfg = &mg->cfg;
  float iq_cap = settle_iq_cap;
  /* 屏障单向锁(用户主导设计，见barrier_dir声明处的说明)：先快照这一拍
     开始前barrier_active是不是已经在生效，函数末尾(所有可能撑屏障的
     分支都跑完之后)拿这个快照跟最终状态比较，只在0->1这一拍锁存方向。 */
  uint8_t barrier_was_active_this_tick = mg->barrier_active;

  /* ---- 长期未到位兜底：跟下面卡滞助推自己的stuck_episode_start_ms是
     两条独立的计时线，只认main.c传入的arrived状态，真正到位才清零，
     不会被卡滞助推那条"检测到erpm_now短暂超过kick_erpm就清零"的微观
     复位打断。实测发现有场景会反复"卡住->真实小跳一下(edges非0)->又
     卡住"，位置净进度趋近于0，但每次小跳都把stuck_episode_start_ms
     清零重来，5秒的giveup阈值永远攒不满，giveup_iq这条底线没机会
     生效、位置迟迟到不了目标。这里只在zone_erpm窗口内(近零速)生效，
     不影响正常巡航调速；stall_kick_disabled为真时也一并跳过，保持
     跟卡滞助推同一个总开关。只会把iq_cap抬高到giveup_iq，不会调低。 ---- */
  if (arrived)
  {
    mg->not_arrived_since_ms = 0;
  }
  else
  {
    if (mg->not_arrived_since_ms == 0) mg->not_arrived_since_ms = now;
    /* ↓↓↓ 门槛用kick_erpm而不是zone_erpm，见下面"卡滞助推"块里giveup_iq
       那段的同一处说明——真正开始动之后不能再继续加这条底线的力，否则
       会把"刚松动"的这一下越推越猛，实测复现过冲出3441°、伴随明显
       冲击/持续抖动感。 */
    if (!cfg->stall_kick_disabled &&
        fabsf(erpm_now) < cfg->kick_erpm &&
        (now - mg->not_arrived_since_ms) > cfg->giveup_ms)
    {
      /* 见MotionGuard_GiveupDecayedIq/GiveupBaseIq说明：卡越久这条底线本身
         越收紧，离目标越近基准也越低，不再恒定顶满giveup_iq。跟旧版一样，
         只在算出来的值仍然比当前iq_cap更高时才生效(只raise不lower，不会
         覆盖掉更严格的settle值)。 */
      float decayed_iq = MotionGuard_GiveupDecayedIq(cfg, (now - mg->not_arrived_since_ms) - cfg->giveup_ms,
                                                       giveup_dist_scale);
      if (decayed_iq > iq_cap) iq_cap = decayed_iq;
    }
  }

  /* ---- 挣脱冲击观测限速屏障：先消费ISR置的pending，再检查是否到期解除。
     severity按"挣脱瞬间转速/判定阈值"算，挣脱越猛屏障压得越死，用
     severity_max兜底防止极端尖峰把上限压到接近0(又制造一次新的卡滞)。 ---- */
  if (mg->breakaway_pending)
  {
    float omega = mg->breakaway_omega_pending;
    mg->breakaway_pending = 0;
    MotionGuard_ArmBarrier(mg, now, omega);
  }
  /* 直接观测判据：不管转子是不是刚从被诊断出的卡滞里挣脱出来，只要
     erpm_now本身已经超过挣脱判定阈值，就先把屏障(续期)撑住。上面两条
     (ISR单边沿骤停骤起 / MotionGuard自己诊断出的卡滞退出)覆盖的都是
     "从卡滞里冲出来"这个特定场景，堵不住"转子本来就已经在动、后面
     还在继续加速冲高"这种(实测log里出现过erpm_now越过阈值但mg=?/0没
     点亮屏障的情况，根子就是这几条判据都要求"之前诊断过卡滞"这个
     前提，这里补齐一条不需要这个前提、只看瞬时观测值的兜底判据)。
     只要erpm_now持续超阈值，这里会每周期续期，天然维持屏障；erpm_now
     一旦回落到阈值以下，才开始倒数上面的barrier_hold_ms。 */
  if (fabsf(erpm_now) > cfg->breakaway_omega_erpm)
  {
    MotionGuard_ArmBarrier(mg, now, erpm_now);
  }
  if (mg->barrier_active && (now - mg->barrier_start_ms) > cfg->barrier_hold_ms)
  {
    mg->barrier_active = 0;
  }

  /* ---- 卡滞助推状态机(方案A双条件触发 + 步骤1总时长限幅)，原样迁移自
     main.c原来FOC_Run()里的同名逻辑，行为不变。stall_kick_disabled为真时
     整体跳过(走else分支，把所有相关状态清零/保持关闭)，见该字段在
     motion_guard.h里的说明。 ---- */
  if (fabsf(erpm_now) < cfg->zone_erpm && !cfg->stall_kick_disabled)
  {
    if (!mg->stall_kicking)
    {
      /* arrived时不新开一轮助推：见motion_guard.h里MotionGuard_Update的
         说明——条件B只看转速、不看误差，静止在目标附近这条本身就满足
         条件B，不加这个判据会把"已经到位"永远误判成"卡滞"。 */
      if (!arrived && (fabsf(err) > cfg->err_enter_erpm || fabsf(erpm_now) < cfg->kick_erpm))
      {
        if (mg->stall_since_ms == 0)
        {
          mg->stall_since_ms = now;
          mg->stall_entry_edge_baseline = edge_cnt;   /* 去抖窗口起点的边沿基准，见下面滞回滤波判定 */
        }
        else if (now - mg->stall_since_ms > cfg->arm_ms)
        {
          /* 进入滞回滤波(用户主导设计，见motion_guard.h顶部第9点说明)：
             不能只看erpm_now这一个瞬时/滤波量就判定卡死——这arm_ms期间
             如果其实已经发生了不少真实边沿(只是每次都没能让erpm_now冲过
             kick_erpm)，说明其实已经在动，是erpm_now这个量失真，不是真
             卡死。边沿增量超过stall_entry_edge_max就不判定，把去抖窗口
             的起点往后挪到现在重新计时，而不是白等了这么久却被一次误判
             直接放弃继续观察。 */
          uint32_t edges_during_arm = edge_cnt - mg->stall_entry_edge_baseline;
          if (edges_during_arm <= cfg->stall_entry_edge_max)
          {
            mg->stall_kicking = 1;
            mg->stall_phase_ms = now;
            mg->stall_exit_confirming = 0;
            if (mg->stuck_episode_start_ms == 0) mg->stuck_episode_start_ms = now;

            /* Stall诊断联动位置误差冻结：进入助推的这一刻，把位置环最近
               一次观测到的实时误差锁存下来。助推期间位置环改用这个锁存值
               算目标转速，不再随卡滞时长继续增长，从源头卡住挣脱那一下
               的目标转速上限(呼应main.c位置环设计说明里诊断出的根子1)。 */
            mg->pos_err_frozen = 1;
            mg->pos_err_frozen_deg = mg->last_pos_err_live_deg;
          }
          else
          {
            mg->stall_since_ms = now;
            mg->stall_entry_edge_baseline = edge_cnt;
          }
        }
      }
      else
      {
        mg->stall_since_ms = 0;
      }
    }
    else
    {
      /* 退出边沿确认(用户主导设计，见motion_guard.h顶部第9点说明)：
         erpm_now单次越过kick_erpm不直接放行——可能只是一次孤立的、马上
         又会停下来的抖动，不代表真的持续动起来了。先进入"确认中"状态，
         持续累计边沿数，直到达到stall_exit_edge_count(4~6)个才真正认为
         这是一次可信的挣脱。kick_max_ms硬上限不受这个确认过程拖延——
         不管确认到哪一步，只要到点就强制退出这一轮(按"没追上、先歇一轮"
         处理，不触发屏障/单向续航/挣脱后积分冻结，这些都是"真挣脱"才
         需要的收尾)。 */
      uint8_t erpm_ok = fabsf(erpm_now) >= cfg->kick_erpm;
      uint8_t maxms_up = (now - mg->stall_phase_ms) > cfg->kick_max_ms;

      if (erpm_ok)
      {
        if (!mg->stall_exit_confirming)
        {
          mg->stall_exit_confirming = 1;
          mg->stall_exit_edge_baseline = edge_cnt;
        }
      }
      else
      {
        mg->stall_exit_confirming = 0;   /* erpm又跌回kick_erpm以下，重新开始确认 */
      }

      if (erpm_ok && mg->stall_exit_confirming &&
          (edge_cnt - mg->stall_exit_edge_baseline) >= cfg->stall_exit_edge_count)
      {
        /* 真正确认挣脱 */
        mg->stall_kicking = 0;
        mg->stall_since_ms = 0;
        mg->stall_exit_confirming = 0;
        mg->pos_err_frozen = 0;
        mg->stuck_episode_start_ms = 0;   /* 真的动起来了，这次发作算结束 */

        /* 卡滞诊断联动限速屏障：只要是从卡滞助推里真正挣脱出来的，就先
           把屏障支起来。原因：挣脱这一刻往往pos_err本身还很大(尤其是
           前面已经攒过好几轮失败的卡滞发作)，解冻后位置环会立刻拿这个
           仍然很大的实时误差重新算出接近满幅的v_max——真正的冲击很多
           时候不是"挣脱这一下"本身多快，而是解冻后被live pos_err重新
           顶起来、后续好几个edge里逐步冲上去的(实测log里出现过edges=
           130这类没有触发下面ISR急停判据、也没触发过旧版只靠ISR
           pending的屏障的大跳变，根子就在这)。比只挂在FOC_HallEdge()
           那个"单边沿骤停骤起"急停判据下更全面：后者只抓得住"单个edge
           内陡然发生"的挣脱，抓不住这种"解冻后连续好几个edge里逐步
           冲上去"的runaway。
           ↓↓↓ 用ArmBarrierKickExit(固定severity)而不是ArmBarrier(按
           erpm_now算severity)：这里的erpm_now就是刚过kick_erpm(30)那个
           很低的退出判据本身，拿它算severity必然落到1.0下限，barrier_
           iq_cap算出来正好等于kick_iq(0.6A)，等于没收紧。实测坐实过：
           退出这一刻mg从1/0/0变0/1/0，但edges/kick/giveup计数显示这次
           挣脱的真实位移(约1380°)全部发生在"不在卡滞助推/giveup期间"
           的窗口里——说明退出后转子带着攒的动能继续冲，而屏障因为severity
           下限被钳死并没有真正收紧。改成固定更紧的severity，不再依赖
           这个天生就很低的erpm_now。 */
        MotionGuard_ArmBarrierKickExit(mg, now);

        /* 卡滞/giveup挣脱后单向续航(见motion_guard.h顶部第8点说明，第二版)：
           第一版直接抬高对称的iq_cap，上机验证暴露回归——iq_target的正负
           钳位共用同一个iq_cap，抬高后连带放宽了反向修正的转矩权限，如果
           这时候转子已经冲过头、误差转负，"往回拉"变得比之前更猛(表现为
           "步伐中间会往回迈步")，正向推力窗口变长也让冲过行程限位前能
           积累的动能变多(表现为限位截停距离变大)。改成只记录这次挣脱的
           方向(erpm_now的符号)，不在这里碰iq_cap——真正的单向放宽挪到
           main.c里跟frag_step/breakout_suppress同一套"只挡/只放宽同方向，
           反方向完全不碰"的模式去做，不会给反向修正额外力度，也不绕过
           下面即将生效的行程软限位。窗口本身仍然接在屏障(0.3A/800ms)
           结束之后才由main.c判定生效，不跟屏障抢档期。 */
        mg->kick_exit_hold_until_ms = now + cfg->barrier_hold_ms + cfg->kick_exit_hold_ms;
        mg->kick_exit_hold_dir = (erpm_now >= 0.0f) ? 1.0f : -1.0f;

        /* 挣脱后继续冻结速度环积分(用户主导设计，见motion_guard.h顶部
           第9点说明)：卡滞期间积累的历史积分即便被冻结没有累加(见下面
           main.c那侧的应用)，一旦真正解冻，Kp*err这一刻本身也可能不小
           (erpm_now刚从接近0冲上来)——继续冻结积分这一段，只是不让
           "历史遗留的integral"参与进一步放大，Kp项该多大还是多大，只
           挡积分这一路，不影响正常的比例响应速度。 */
        mg->post_stall_freeze_until_ms = now + cfg->post_stall_integ_freeze_ms;
      }
      else if (maxms_up)
      {
        mg->stall_kicking = 0;
        mg->stall_since_ms = 0;
        mg->stall_exit_confirming = 0;
        mg->pos_err_frozen = 0;
        /* 不清stuck_episode_start_ms："没追上、先歇一轮"，这次发作没有
           结束，跟原逻辑一致。 */
      }
    }

    if (mg->stall_kicking)
    {
      if (mg->stall_exit_confirming)
      {
        /* 退出边沿确认期间(erpm_now已经越过kick_erpm，正在等
           stall_exit_edge_count个边沿才真正翻转stall_kicking)：转矩权限
           提前收回到settle水平(iq_cap保持函数入口处的settle_iq_cap，
           不再套用kick_iq/半档)，不等状态机正式翻转。
           根因(用户实测坐实，~506°超调，行程限位定标500°)：这个块原来
           只看stall_kicking这一个标志，确认期间它仍然是1，会继续按
           kick_iq(0.6A)/半档出力——等于"已经确认在动了"之后还要再等
           4~6个边沿(数十到上百ms)才松手，这段时间攒的额外动能直接带过
           了行程限位。只收紧这一段的出力幅度，不改变确认窗口本身的边沿
           计数逻辑(挂屏障/记方向/冻结积分这些退出收尾动作依旧等够edges
           才做，跟用户"需要连续N个边沿才清stall标志"的原设计不冲突)。 */
      }
      else
      {
        uint32_t stuck_dur = now - mg->stuck_episode_start_ms;
        if (stuck_dur > cfg->giveup_ms)
        {
          mg->stall_kicking = 0;   /* 卡太久了，放弃周期性助推 */
          mg->pos_err_frozen = 0;
        }
        else
        {
          iq_cap = (stuck_dur > cfg->deescalate_ms) ? (cfg->kick_iq * 0.5f) : cfg->kick_iq;
        }
      }
    }

    /* 放弃助推后的底线(见FOC_STUCK_GIVEUP_IQ/giveup_iq说明)：只要本次
       卡滞发作还没真正解除(stuck_episode_start_ms没被erpm_now>=kick_erpm
       清零过)，不管是刚放弃的这一拍还是放弃后一直蹭着的后续拍，都用
       这条比settle_iq_cap略高的底线，而不是继续用0.4A那档——齿槽转矩
       是电机固有的、跟外部负载无关的"卡位感"，纯settle_iq_cap可能长期
       压不过去，形成手感没有阻力却一直到不了目标的僵局。
       ↓↓↓ 退出门槛用kick_erpm而不是外层zone_erpm：之前这条底线一旦
       激活，会一直加力到erpm_now冲过zone_erpm(300)才罢手——但"齿槽松动、
       转子真的开始动"这个判据(跟卡滞助推自己退出用的判据完全一样)其实
       在kick_erpm(30)就该成立了。实测复现：giveup生效后转子挣脱，
       在erpm还没到300之前这条底线全程继续按giveup_iq加码，把刚松动的
       这一下越推越猛，最终冲出3441°，伴随明显冲击/持续抖动感——跟本
       session之前证伪过的"减速方向豁免"不是一回事(那次是放宽刹车方向
       上限，这次是收紧"已经真的动起来之后还要不要继续加力"这个判据本身，
       方向相反、风险模型不同)。真正挣脱这一刻同时清stuck_episode_start_ms
       (跟卡滞助推自己退出时的清零语义一致，避免残留计时让giveup在后续
       微小转速回落时又立刻重新激活)，并撑一次屏障(呼应卡滞助推退出时
       的"卡滞诊断联动限速屏障"，给这次真实挣脱同等力度的减震)。
       ↓↓↓ 这条底线本身也不再恒定顶满giveup_iq：实测复现过卡满giveup_ms
       后原地顶着满额giveup_iq蓄力很久，一旦齿槽真的松动就是edges=67
       (约4020°)这种量级的巨大单窗口挣脱——不能只靠事后的屏障/位置误差
       冻结收拾，从源头降低挣脱瞬间可用的静态转矩，见MotionGuard_
       GiveupDecayedIq说明。 */
    if (!mg->stall_kicking && mg->stuck_episode_start_ms != 0 &&
        (now - mg->stuck_episode_start_ms) > cfg->giveup_ms)
    {
      if (fabsf(erpm_now) < cfg->kick_erpm)
      {
        /* giveup单向锁(用户主导设计，见该字段声明处的说明)：只在0->1这
           一拍锁存方向，之后整段giveup期间不再跟着err的瞬时符号变化。 */
        if (!mg->giveup_active)
        {
          mg->giveup_dir = (err >= 0.0f) ? 1.0f : -1.0f;
        }
        /* 见MotionGuard_GiveupDecayedIq/GiveupBaseIq说明：这条底线同样不再
           恒定顶满，还接入了离目标距离这个维度。 */
        iq_cap = MotionGuard_GiveupDecayedIq(cfg, (now - mg->stuck_episode_start_ms) - cfg->giveup_ms,
                                              giveup_dist_scale);
        mg->giveup_active = 1;
      }
      else
      {
        mg->stuck_episode_start_ms = 0;
        mg->giveup_active = 0;
        /* 同上面卡滞助推退出处的说明：用固定severity而不是按这里同样
           很低的erpm_now(刚过kick_erpm)算severity，否则barrier_iq_cap
           会跟giveup_iq(0.5A)同量级，起不到真正收紧的作用。 */
        MotionGuard_ArmBarrierKickExit(mg, now);

        /* 同上面卡滞助推退出处的说明(第二版，单向续航)：giveup真正挣脱
           这一刻同样只记录方向，不碰iq_cap，见motion_guard.h顶部第8点；
           窗口同样接在屏障hold_ms之后再开始算，不跟屏障抢。 */
        mg->kick_exit_hold_until_ms = now + cfg->barrier_hold_ms + cfg->kick_exit_hold_ms;
        mg->kick_exit_hold_dir = (erpm_now >= 0.0f) ? 1.0f : -1.0f;

        /* 同上面卡滞助推退出处的说明：giveup真正挣脱同样是一次"猛地
           前窜"的风险点，一并接入挣脱后积分冻结，见motion_guard.h顶部
           第9点。 */
        mg->post_stall_freeze_until_ms = now + cfg->post_stall_integ_freeze_ms;

        /* giveup单向锁延续窗口(用户主导设计，见该字段声明处的说明)：
           giveup_active刚变0的这一刻转子往往还带着挣脱攒下的真实动能
           (实测eRPM远超轨迹此刻的目标速度)，正常闭环PI对这个巨大速度
           误差的刹车响应可能强到不只是刹停、而是把净位移反方向拉回去，
           表现为"挣脱之后紧接着一步往回踏"——这不是giveup状态机本身在
           跑，锁的判据(MotionGuard_IsGiveupActive)已经覆盖不到这一段。
           延续单向锁到屏障窗口结束(复用barrier_hold_ms，跟屏障同步到期，
           不需要单独再调一个参数)，让这段时间里的动能靠自然摩擦/齿槽
           阻力耗散，而不是被主动刹车指令推成反向位移。 */
        mg->giveup_exit_lock_until_ms = now + cfg->barrier_hold_ms;
      }
    }
    else
    {
      mg->giveup_active = 0;
    }
  }
  else
  {
    mg->stall_since_ms = 0;
    mg->stall_kicking = 0;
    mg->stall_exit_confirming = 0;
    mg->stuck_episode_start_ms = 0;
    mg->pos_err_frozen = 0;
    mg->giveup_active = 0;
  }

  /* 卡滞/giveup挣脱后单向续航窗口的到期清理：见motion_guard.h顶部第8点
     说明(第二版)——这里只负责到点(now超过kick_exit_hold_until_ms)清零
     状态，不碰iq_cap。真正"要不要放宽、放宽多少、只放宽哪个方向"这几
     个判断挪到main.c里跟frag_step/breakout_suppress同一套单向阻断/放宽
     模式去做(main.c能直接拿到foc_iq_ref的实际符号，比这里更适合判断
     "是不是同方向")，避免重蹈第一版对称抬高iq_cap连累反向修正力度的
     覆辙。 */
  if (mg->kick_exit_hold_until_ms != 0 && now >= mg->kick_exit_hold_until_ms)
  {
    mg->kick_exit_hold_until_ms = 0;
  }

  /* 挣脱后积分冻结窗口的到期清理：见motion_guard.h顶部第9点说明。这里
     只负责到点清零状态，真正"要不要跳过Ki*err*dt_s这步累加"的判断在
     main.c(main.c能直接摸到speed_integ这个变量，这里没有)。 */
  if (mg->post_stall_freeze_until_ms != 0 && now >= mg->post_stall_freeze_until_ms)
  {
    mg->post_stall_freeze_until_ms = 0;
  }

  /* giveup单向锁延续窗口的到期清理：见该字段声明处的说明，这里只负责
     到点清零状态，main.c据此判断这段窗口内是否继续禁止iq_target反向。 */
  if (mg->giveup_exit_lock_until_ms != 0 && now >= mg->giveup_exit_lock_until_ms)
  {
    mg->giveup_exit_lock_until_ms = 0;
  }

  /* 屏障对iq_cap做进一步收紧：只收不放，不会覆盖掉助推/settle算出的更
     严格的值。 */
  if (mg->barrier_active && mg->barrier_iq_cap < iq_cap)
  {
    iq_cap = mg->barrier_iq_cap;
  }

  /* 屏障单向锁(用户主导设计，见barrier_dir声明处的说明)：这一拍屏障
     从"未生效"变成"生效"(不管是哪条路径撑起来的——ISR单边沿骤停骤起、
     stall_kicking/giveup诊断退出、还是这里直接观测到erpm_now超阈值)，
     锁存这一刻的erpm_now符号，交给main.c在屏障整个生效期间用同一套
     "只钳到0、不允许反号"的硬闸挡住反向刹车力矩。跟giveup_dir是同一
     思路的推广：giveup_dir只覆盖giveup触发的那一条路径，这里覆盖屏障
     本身，不管由谁触发。 */
  if (!barrier_was_active_this_tick && mg->barrier_active)
  {
    mg->barrier_dir = (erpm_now >= 0.0f) ? 1.0f : -1.0f;
  }

  return iq_cap;
}

float MotionGuard_GetPosErrForSpeedCalc(MotionGuard_t *mg, float pos_err_live_deg, uint32_t now)
{
  const MotionGuardConfig_t *cfg = &mg->cfg;
  uint32_t dt_ms = (mg->last_getpos_calc_ms == 0) ? 0 : (now - mg->last_getpos_calc_ms);

  mg->last_pos_err_live_deg = pos_err_live_deg;
  mg->last_getpos_calc_ms = now;

  if (mg->pos_err_frozen)
  {
    /* 冻结态：直接锁定，不做任何平滑(冻结进入这一刻本身不产生新误差，
       用户没要求平滑这一侧，只要求平滑"解冻"那一侧)。 */
    mg->pos_err_output_deg = mg->pos_err_frozen_deg;
    return mg->pos_err_output_deg;
  }

  /* 解冻/未冻结态：从pos_err_output_deg(可能还停在刚解冻那一刻的冻结值)
     朝live值做一阶低通，而不是直接跳变——避免挣脱那一刻(冻结刚解除)
     立刻被可能仍然很大的live误差顶起一个大的v_max，见unfreeze_tau_s
     说明。dt_ms==0(从未调用过)时没有平滑基准，直接透传。 */
  if (dt_ms == 0)
  {
    mg->pos_err_output_deg = pos_err_live_deg;
  }
  else
  {
    float dt_s = (float)dt_ms * 0.001f;
    float alpha = dt_s / (cfg->unfreeze_tau_s + dt_s);
    mg->pos_err_output_deg += alpha * (pos_err_live_deg - mg->pos_err_output_deg);
  }
  return mg->pos_err_output_deg;
}

float MotionGuard_ClampSpeedRef(MotionGuard_t *mg, float speed_ref_target)
{
  if (!mg->barrier_active)
  {
    return speed_ref_target;
  }
  if (speed_ref_target >  mg->barrier_speed_cap_erpm) return  mg->barrier_speed_cap_erpm;
  if (speed_ref_target < -mg->barrier_speed_cap_erpm) return -mg->barrier_speed_cap_erpm;
  return speed_ref_target;
}

void MotionGuard_NotifyBreakaway(MotionGuard_t *mg, float omega_erpm)
{
  /* ISR安全：只做两次简单赋值，不做浮点状态机运算，真正的屏障力度计算
     推迟到下一次MotionGuard_Update(主循环上下文)里完成。 */
  mg->breakaway_pending = 1;
  mg->breakaway_omega_pending = omega_erpm;
}

uint8_t MotionGuard_IsGiveupActive(const MotionGuard_t *mg)
{
  return mg->giveup_active;
}
