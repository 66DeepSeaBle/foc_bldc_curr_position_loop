/**
  ******************************************************************************
  * @file    motion_guard.h
  * @brief   位置/速度级联控制的防冲击护栏模块。
  *
  * 背景：sensored BLDC/PMSM 级联三环(位置->速度->电流)在近零速卡滞后挣脱时，
  * 容易出现"卡滞期间攒的位置误差/转矩·时间越攒越大 -> 一旦挣脱就是全速冲刺"
  * 的失控式野蛮挣脱，"机械冲击绝对不可接受"这条硬约束下必须解决。这个模块
  * 把解决方案封装成与main.c控制流程解耦的结构体+纯函数，包含四部分：
  *
  *   1. 卡滞助推(双条件触发：err大 或 转速低，见MotionGuard_Update)——原
  *      main.c里的"方案A"，原样迁移。
  *   2. 卡滞发作总时长限幅(卡越久助推力度越收紧/放弃)——原main.c里的
  *      "位置环重新设计·步骤1"，原样迁移。
  *   3. Stall诊断联动位置误差冻结(新)：诊断为"卡滞助推中"期间，位置环
  *      用来算目标转速的误差冻结在刚进入卡滞判定那一刻的值，不再随卡滞
  *      时长继续增长，从源头卡住挣脱那一下的目标转速上限。
  *   4. 挣脱冲击观测限速屏障(新)：挣脱事件发生后，按观测到的瞬时转速幅度
  *      分级，对speed_ref/iq_cap施加一段时间的临时收紧上限，幅度越猛压得
  *      越死，到点自动解除。
  *   5. 卡滞蓄能抑制(新)：诊断为持续卡滞期间(卡滞助推/giveup)，位置环
  *      不再用冻结误差长期钉着一个恒定的高speed_ref目标(哪怕误差本身
  *      已冻结，算出来的目标转速仍是个不小的常数，长期喂给速度环相当于
  *      "预先蓄力"，挣脱那一刻会被这个已经攒了很久的目标顶飞)——诊断期
  *      间把speed_ref_target幅度压到很小(只保留方向)；挣脱解冻那一刻也
  *      不直接跳回可能很大的实时误差，改用一阶低通缓慢修正。这是从"挣脱
  *      前蓄能"这个源头压制最大冲出距离，跟第3/4点(误差冻结/屏障)是
  *      互补关系，不是替代。main.c后来改用轨迹发生器后，位置环这边不再
  *      调用这一点，但机制本身仍保留在这个模块里，可独立复用。
  *   6. giveup斜率自适应衰减(新)：放弃周期性助推后的giveup_iq底线不再
  *      恒定顶满——持续卡满giveup_ms门槛后，先按满额giveup_iq维持一小段
  *      时间(短时助推克服轻微卡滞)，仍未起步就按giveup_decay_step_ms/
  *      giveup_decay_step_iq阶梯下调，直到giveup_iq_floor封底(长期卡死，
  *      主动降低静态转矩，减小一旦真正挣脱那一刻的初始加速度/动能)。
  *      跟第3/4/5点是同一条因果链上不同的治理点：3/5管的是位置环喂给
  *      速度环的目标不要蓄力，这一点管的是电流环本身允许维持的静态转矩
  *      不要一直顶在高位。
  *   7. giveup基准远近分区(新，用户主导设计)：废除第6点"giveup恒定用
  *      固定基准giveup_iq"这个设计——实测证实不管离目标多近，giveup挣脱
  *      的位移量级都一样，多轮挣脱后"收敛"只是恰好落进容差、不是真正
  *      逼近目标。giveup_dist_scale(由main.c按剩余距离换算传入)=1.0
  *      (远)时基准仍是giveup_iq(保证兜底脱困能力)，趋于0.0(近)时基准
  *      线性滑向giveup_iq_floor(离目标近不需要大力气冲，主动柔化)；
  *      第6点的时长衰减在这个距离基准之上继续生效，两者是叠加关系。
  *   8. 卡滞/giveup挣脱平滑过渡(新，用户主导设计)：卡滞助推/giveup的
  *      退出判据都是"erpm_now刚越过kick_erpm(30)这个很低的阈值"——
  *      退出那一刻转子往往才刚从静止开始加速，动能积累还很少，若立刻
  *      把iq_cap从kick_iq/giveup_iq一步跌回危险区settle值(0.4A)，只要
  *      齿槽转矩在这个爬升初期仍然大于0.4A能提供的推力，转子会立刻
  *      失去动能再次停住——表现为"起始位置到目标位置被拆成好几个台阶"，
  *      每个台阶之间是一次完整的"卡住->700ms去抖->kick/giveup->刚越过
  *      阈值又跌回0.4A->再次卡住"循环，而不是一次连贯的加速-匀速-减速。
  *      退出那一刻额外撑起一段kick_exit_hold_ms的过渡期，期间iq_cap维持
  *      在kick_exit_hold_iq(高于settle、低于kick_iq/giveup_iq的中间值)，
  *      给转子足够的时间/转矩把这一下的加速度延续下去、真正冲出这一段
  *      齿槽爬升区，而不是拿到"刚够格退出"的转速就被立刻抽走继续加速的
  *      能力。只影响退出后的这一小段窗口，不改变卡滞助推/giveup本身
  *      "要不要触发""触发后给多大力"这两个判据，也不新增任何刹车方向的
  *      转矩权限——风险模型跟已经证伪的"减速方向豁免"不同。
  *   9. 卡滞进入滞回滤波+退出边沿确认+挣脱后积分冻结(新，用户主导设计)：
  *      第8点解决的是"退出之后怎么接上气"，但把碎步压制/脱困软上限/
  *      单向续航这三层全部关掉复测，台阶依旧存在，说明根子还要往前挖：
  *      (a) 进入判据只看erpm_now这一个瞬时/滤波量，可能被短暂噪声误判
  *      成"卡死"，触发不必要的一轮kick；(b) 退出判据是erpm_now单次越过
  *      kick_erpm(30)就立刻放行，可能只是一次孤立的、马上又停下来的
  *      抖动，不代表真的持续动起来了；(c) 卡滞期间err持续存在，速度环
  *      积分Ki*err*dt_s一直在正常累加、没有被冻结，一旦挣脱，这份历史
  *      积分会跟挣脱瞬间的Kp*err一起释放，容易"猛地前窜"。这一点新增
  *      三样东西：进入判据额外要求去抖窗口(arm_ms)期间边沿增量够低
  *      (stall_entry_edge_max)才真正判定卡死；退出判据额外要求erpm_now
  *      越过kick_erpm后再持续观测到stall_exit_edge_count(4~6)个边沿才
  *      真正确认挣脱；确认挣脱后继续冻结速度环积分post_stall_integ_
  *      freeze_ms(300~400ms)，只挡积分这一路、不影响Kp项的正常比例
  *      响应。三者都要靠main.c喂来的单调递增边沿计数(edge_cnt)才能算出
  *      "从某一刻到现在一共发生了几个边沿"这个跨任意时长的差值。
  *      ↓↓↓ 上机验证暴露副作用(用户实测反馈：台阶依旧+行程限位被冲过
  *      约6°，~506°)：退出边沿确认期间(stall_exit_confirming=1，erpm_now
  *      已经越过kick_erpm、正在等够edges)，stall_kicking标志还没翻转，
  *      按kick_iq/半档出力的那个判断块只看stall_kicking，会在整个确认
  *      窗口(4~6个边沿)里继续满额出力——排查过speed_integ本身不是根子
  *      (冻结期间不累加、且被±iq_cap钳死，没有真正意义上的windup)，
  *      真正的额外动能来自这里："已经确认在动"之后还继续按满额kick_iq
  *      喂电流数十到上百ms。修复：确认期间转矩权限提前收回到settle水平，
  *      只是"要不要真正翻转stall_kicking/挂屏障/记方向/冻结积分"这几个
  *      收尾动作依旧等够edges才做，边沿计数本身的设计不变。
  *
  * 每个实例(MotionGuard_t)自带一份配置，互不影响，方便同一逻辑复用到未来
  * 不同的电机/工况(比如带载夹爪)，各自喂各自的参数。
  ******************************************************************************
  */
#ifndef __MOTION_GUARD_H
#define __MOTION_GUARD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  /* ---- 卡滞助推(方案A + 步骤1)参数 ---- */
  float    zone_erpm;             /* |eRPM|低于此值才算"过零低速窗口"，助推只在窗口内评估 */
  float    kick_erpm;             /* 条件B阈值(|eRPM|<此值判定"真的没转起来")，也是统一退出阈值 */
  float    err_enter_erpm;        /* 条件A阈值：窗口内|err|超过此值才算"明显跟不上" */
  uint32_t arm_ms;                /* 条件A/B任一持续满足这么久才真正触发(去抖) */
  uint32_t kick_max_ms;           /* 单轮助推硬上限，不管有没有追上，到点强制退出 */
  float    kick_iq;               /* 助推期间的转矩上限(A) */
  uint32_t deescalate_ms;         /* 本次卡滞发作累计超过这么久，助推电流减半 */
  uint32_t giveup_ms;             /* 累计超过这么久，彻底放弃周期性助推 */
  float    giveup_iq;             /* 放弃助推后维持的底线转矩(A)：比危险区settle_iq_cap略高，
                                      专门用来压过电机自身齿槽转矩(cogging torque)——纯摩擦/惯性
                                      场景下settle_iq_cap够用，但齿槽转矩是电机固有的、跟外部
                                      负载无关的"卡位感"，光靠settle_iq_cap可能长期蹭不过去，
                                      形成明明手感没有阻力却怎么都到不了目标的僵局。用户主导
                                      设计：这是"远距离"档的基准值(MotionGuard_Update的
                                      giveup_dist_scale参数=1.0时生效)，不再是恒定顶满的唯一
                                      基准——实测证实"giveup用固定基准"会让每次挣脱的位移量级
                                      跟剩余距离脱钩，多轮挣脱后"收敛"只是恰好落进容差、不是
                                      真正逼近，见giveup_iq_floor说明。 */
  uint32_t giveup_decay_step_ms;  /* giveup这条底线生效后，每隔这么久(仍未起步)就把当前档位的
                                      基准iq下调一档giveup_decay_step_iq，直到giveup_iq_floor
                                      封底——不要一直顶满基准蓄力等齿槽松动，那一下的起步动能
                                      天然就大；0表示不启用时间衰减(只保留下面的距离分区)。 */
  float    giveup_decay_step_iq;  /* 每档下调多少安培 */
  float    giveup_iq_floor;       /* 衰减下限(A)，必须仍然明显高于危险区settle_iq_cap，否则
                                      退化回"齿槽压不过去、永远到不了目标"那个已修复的旧问题。
                                      同时也是giveup_dist_scale=0.0(离目标很近)时的基准值——
                                      远近分区用的是同一个下限，离目标越近基准越靠近这个值，
                                      不需要额外等时间衰减才能柔化下来。 */
  uint8_t  stall_kick_disabled;   /* 诊断/排查用总开关：置1彻底关闭整个卡滞助推+giveup机制
                                      (含条件A/B/去抖/减半/放弃底线全部不生效)，只保留挣脱冲击
                                      限速屏障。用于排查"到位后还会自己动"这类疑似卡滞助推
                                      误触发的问题——如果关掉后现象消失，就实锤是卡滞助推
                                      本身的问题；如果现象依旧，说明根子在别处。正常使用应
                                      置0，不是长期打开的选项。 */

  /* ---- 卡滞进入滞回滤波 + 退出边沿确认 + 挣脱后积分冻结(用户主导设计) ----
     背景：台阶排查到最后发现即便关掉碎步压制/脱困软上限/单向续航这三层
     "事后收尾"机制、台阶依旧存在，根子在卡滞助推状态机本身——(a)进入
     判据只看erpm_now这一个瞬时/滤波量，可能被短暂的量化噪声或插值误差
     误判为"卡死"；(b)退出判据是erpm_now单次越过kick_erpm(30)就立刻放行，
     可能只是一次孤立的、马上又会停下来的抖动，不代表真的开始持续运动；
     (c)卡滞期间err持续存在、速度环积分Ki*err*dt_s一直在正常累加(没有
     被冻结)，一旦挣脱，这份"卡滞期间攒下的历史积分"会跟挣脱瞬间的
     Kp*err一起释放，容易造成"猛地前窜"。这三点都会导致"进入太快->
     卡一下就退出->退出瞬间又因为历史积分猛冲->很快又卡住"这种短周期
     反复，表现为台阶/往回迈步。 */
  uint32_t stall_entry_edge_max;       /* 进入卡滞去抖(arm_ms)窗口期间，边沿增量必须不超过这个数
                                           才真正判定为卡死——如果这段时间其实已经有不少边沿在跳，
                                           说明erpm_now只是瞬时失真，不是真卡死，不判定，继续观察。 */
  uint32_t stall_exit_edge_count;      /* erpm_now越过kick_erpm之后，还需要再观测到这么多个连续
                                           边沿(4~6个)才真正确认挣脱、清除stall_kicking标志——不让
                                           单个孤立边沿的瞬时读数就判定"已经追上了"。 */
  uint32_t post_stall_integ_freeze_ms; /* 确认挣脱后，继续冻结速度环积分这么久(300~400ms)，防止
                                           卡滞期间积累的历史积分在挣脱瞬间一次性放出来、叠加当时
                                           本来就不小的Kp*err项，造成猛地前窜。只冻结积分这一路，
                                           Kp项该多大还是多大，不影响正常的比例响应。 */

  /* ---- 挣脱冲击观测限速屏障参数 ---- */
  float    breakaway_omega_erpm;  /* 挣脱判定阈值(eRPM)，与霍尔中断里的急停判据共用同一个量级 */
  float    barrier_base_erpm;     /* 屏障期基准限速(eRPM)，severity=1时的上限 */
  float    barrier_base_iq;       /* 屏障期基准限流(A)，severity=1时的上限 */
  float    barrier_severity_max;  /* 限速/限流力度按挣脱速度加严的上限倍数，防止极端尖峰把上限压到接近0 */
  uint32_t barrier_hold_ms;       /* 屏障维持时长，到点自动解除 */
  float    kick_exit_barrier_severity;  /* 卡滞助推/giveup"诊断退出"专用的固定severity(不按erpm_now
                                            算)——这两条退出路径本身的判据就是erpm_now刚过kick_erpm
                                            这个很低的阈值，拿它算severity必然落到1.0下限、barrier_iq_
                                            cap算出来会跟kick_iq/giveup_iq同量级，等于没收紧。实测坐实
                                            过这个漏洞：退出那一刻mg从1/0/0变0/1/0，但转子带着攒的动能
                                            继续冲了约1380°，全程edges的kick/giveup标记都是0(因为退出
                                            那一刻状态已经翻转)，屏障却没有真正收紧过。 */

  /* ---- 卡滞蓄能抑制(新)：解决"多次挣脱都超调、来回振荡不收敛"问题 ---- */
  float    unfreeze_tau_s;        /* pos_err解冻后的一阶低通时间常数(秒)：挣脱那一刻(pos_err_frozen
                                      1->0)不直接跳回可能很大的实时误差，而是缓慢修正，避免解冻瞬间
                                      立刻又被live pos_err顶起一个大的v_max，造成二次冲击。冻结进入
                                      (0->1)不做平滑，直接锁存(用户没要求，且冻结瞬间本身不产生
                                      新误差)。起点值待实测调整。 */

  /* ---- 卡滞/giveup挣脱平滑过渡(新，用户主导设计) ---- */
  uint32_t kick_exit_hold_ms;     /* 卡滞助推/giveup真正挣脱(erpm_now越过kick_erpm)那一刻起，
                                      额外维持这么久的过渡期，而不是立刻跌回settle_iq_cap，见
                                      本文件顶部第8点说明。 */
  float    kick_exit_hold_iq;     /* 过渡期用的iq_cap(A)，取在settle危险区上限和kick_iq/giveup_iq
                                      之间——只是"别一步跌到底"，不是重新给一次满额助推。 */
} MotionGuardConfig_t;

typedef struct
{
  MotionGuardConfig_t cfg;

  /* ---- 卡滞助推状态机 ---- */
  uint32_t stall_since_ms;         /* 0=未在去抖计时；条件A/B任一满足时开始计时 */
  uint8_t  stall_kicking;          /* 0=未在助推，1=正在助推 */
  uint32_t stall_phase_ms;         /* 本轮助推的起始时刻，配合kick_max_ms硬上限 */
  uint32_t stuck_episode_start_ms; /* 0=当前不在卡滞发作中；本次发作从第一次触发助推算起 */
  uint8_t  giveup_active;          /* 本周期iq_cap是否正在用giveup_iq这条底线，供main.c
                                       调整危险区电流斜坡速率用(见MotionGuard_IsGiveupActive) */
  float    giveup_dir;             /* giveup单向锁(用户主导设计)：giveup_active 0->1那一拍
                                       锁存err的符号(+1.0/-1.0)，之后整段giveup期间不再跟着
                                       err的瞬时符号变化——防止挣脱瞬间erpm超调/霍尔噪声导致
                                       err符号短暂翻转，把giveup强制爬升(target_integ)和最终
                                       iq_target一起带偏成反方向，表现为"中途反向踏步"。main.c
                                       两处都要用：giveup强制爬升的target_integ改用这个锁存
                                       方向而不是err瞬时符号；iq_target最终确定处再补一道硬闸，
                                       giveup生效期间绝不允许iq_target跟锁存方向反号(顶多钳到
                                       0，不会新增任何方向的转矩权限)。 */
  uint32_t giveup_exit_lock_until_ms; /* giveup单向锁延续窗口(用户主导设计)：giveup_active
                                       刚变0(真正挣脱)那一刻，转子往往还带着挣脱攒下的真实
                                       动能(实测eRPM远超轨迹此刻的目标速度)，正常闭环PI对这
                                       个巨大速度误差的刹车响应可能强到不只是刹停、而是把净
                                       位移反方向拉回去，表现为"挣脱之后紧接着一步往回踏"——
                                       giveup_active已经是0，MotionGuard_IsGiveupActive()判据
                                       覆盖不到这一段。这个窗口延续到屏障(barrier_hold_ms)
                                       结束为止，main.c据此继续用giveup_dir锁住iq_target的
                                       方向，让这段时间里的动能靠自然摩擦/齿槽阻力耗散，而
                                       不是被主动刹车指令推成反向位移。0=当前不在这个窗口内。 */
  uint32_t not_arrived_since_ms;   /* 0=当前已到位；main.c每拍传入的arrived状态从这里开始
                                       计时，只有真正到位才清零——跟stuck_episode_start_ms是
                                       两条独立的计时线，不会被卡滞助推自己"检测到瞬时真实
                                       转动就清零"这个微观复位打断。见MotionGuard_Update里
                                       "长期未到位兜底"那段说明：实测有场景反复"卡住->真实
                                       小跳一下->又卡住"，每次小跳都把stuck_episode_start_ms
                                       清零重来，giveup阈值永远攒不满，位置迟迟到不了目标。 */

  /* ---- 卡滞进入滞回滤波 + 退出边沿确认 + 挣脱后积分冻结，见cfg里
     stall_entry_edge_max/stall_exit_edge_count/post_stall_integ_freeze_ms
     的说明 ---- */
  uint32_t stall_entry_edge_baseline;  /* 进入去抖计时(stall_since_ms 0->非0)那一刻的边沿基准 */
  uint8_t  stall_exit_confirming;      /* 0=未在等待退出确认；1=erpm_now已越过kick_erpm，正在
                                           累计边沿数、等待达到stall_exit_edge_count */
  uint32_t stall_exit_edge_baseline;   /* 开始等待确认退出(stall_exit_confirming 0->1)那一刻的
                                           边沿基准 */
  uint32_t post_stall_freeze_until_ms; /* 0=不在挣脱后积分冻结期内；非0=冻结速度环积分直到这个
                                           时刻，main.c据此跳过Ki*err*dt_s那步累加 */

  /* ---- 位置误差冻结：与stall_kicking 1:1联动 ---- */
  uint8_t  pos_err_frozen;         /* 镜像stall_kicking，冻结态由位置环侧查询 */
  float    pos_err_frozen_deg;     /* stall_kicking 0->1那一刻锁存的位置误差(度) */
  float    last_pos_err_live_deg;  /* 本周期位置环观测到的实时误差，供冻结锁存取值用 */
  float    pos_err_output_deg;     /* GetPosErrForSpeedCalc()实际返回值的内部状态：冻结态直接等于
                                       pos_err_frozen_deg，解冻后朝live值一阶低通逼近(见unfreeze_tau_s) */
  uint32_t last_getpos_calc_ms;    /* 0=GetPosErrForSpeedCalc()从未被调用过，供算低通dt用 */

  /* ---- 挣脱冲击观测限速屏障 ---- */
  volatile uint8_t  breakaway_pending;       /* 挣脱检测(ISR中断上下文)置位，主循环消费 */
  volatile float    breakaway_omega_pending; /* 挣脱瞬间观测到的转速(eRPM)，用于按幅度分级 */
  uint8_t  barrier_active;
  uint32_t barrier_start_ms;
  float    barrier_speed_cap_erpm;
  float    barrier_iq_cap;
  float    barrier_dir;            /* 屏障单向锁(用户主导设计)：barrier_active 0->1那一拍锁存
                                       erpm_now的符号(+1.0f/-1.0f)，覆盖屏障生效期间不管由谁
                                       触发(ISR单边沿骤停骤起/stall_kicking或giveup诊断退出/
                                       直接观测erpm_now超阈值)。根因：屏障本身只收紧iq_cap这个
                                       上限，没有锁方向——转子冲过目标后正常PI的刹车响应(err
                                       深度为负)算出的iq_target即便被屏障压低的iq_cap钳过，仍
                                       然是同号的负值，屏障窗口内(数百ms)持续施加这份被压低但
                                       仍然存在的反向转矩，足以把净位移拉成明显的反向摆动，不
                                       只是单纯刹停——实测坐实(pos从545.4冲过目标后被拉回
                                       515.3/459.1，giveup全程未生效，只有屏障在跑)。main.c据
                                       此在barrier_active期间追加一道硬闸：iq_target绝不允许
                                       跟这个锁存方向反号(顶多钳到0，不新增任何方向的转矩权
                                       限)，让屏障窗口内的多余动能靠自然摩擦/齿槽阻力耗散，
                                       跟giveup_dir是同一个思路在更大范围(所有屏障触发场景)的
                                       推广。 */

  /* ---- 卡滞/giveup挣脱后单向续航窗口(第二版：只记方向，不碰iq_cap，
     真正的单向放宽由main.c参照foc_iq_ref符号去做，见本文件顶部第8点) ---- */
  uint32_t kick_exit_hold_until_ms; /* 0=不在窗口内；非0=维持到这个时刻，到点自动清零 */
  float    kick_exit_hold_dir;      /* 触发那一刻erpm_now的符号(+1.0f/-1.0f)：main.c只在
                                        iq_target跟这个方向同号时才放宽，反方向完全不碰 */
} MotionGuard_t;

/* 用cfg初始化(或复位)一个实例，运行时状态清零。 */
void MotionGuard_Init(MotionGuard_t *mg, const MotionGuardConfig_t *cfg);

/* 每个控制周期调用一次(主循环速度环那一拍)：喂当前转速反馈(erpm_now)、
   速度误差(err=目标-实测)、当前时间戳(ms)、位置环是否已判定到位(arrived)、
   giveup底线的远近分区比例(giveup_dist_scale，见下方说明)，内部完成：
     - 屏障pending的消费/超时解除
     - 卡滞助推状态机(条件A/B、去抖、MAX_MS退出)——arrived为真时不会
       触发新的一轮助推(已经稳稳停在目标附近，条件B只看转速不看误差，
       否则光是"静止"这一条就会被永远误判成卡滞、周期性给出0.6A电流，
       偶尔压过齿槽转矩把已经到位的转子重新推走)。已经在进行中的助推、
       屏障、giveup底线都不受这个新增判据影响，只挡"新开一轮"这一步；
       一旦真被外力顶离目标(main.c那边pos_arrived会重新变0)，判据立刻
       恢复正常。
     - 卡滞发作总时长限幅(减半/放弃)
     - 长期未到位兜底(not_arrived_since_ms，见该字段声明处的说明)：跟
       上面的卡滞发作计时是两条独立的线，不受"偶尔真实小跳一下"打断，
       持续未到位超过giveup_ms后同样给出giveup_iq这条底线，解决"反复
       卡住->小跳->又卡住，位置迟迟到不了目标"的僵局。
     - giveup基准的远近分区 + 斜率自适应衰减：上面两条giveup底线不再
       恒定顶满giveup_iq——giveup_dist_scale=1.0(离目标远)时基准还是
       giveup_iq，越接近0.0(离目标近)基准越滑向giveup_iq_floor，在这个
       距离基准之上再叠加"卡越久力度越收紧"这一层(giveup_decay_step_ms/
       _step_iq阶梯下调，同样封底giveup_iq_floor)。用户主导设计：废除
       "giveup恒定用固定基准、不看剩余距离"这个隔离，避免每次挣脱的
       位移量级跟目标远近脱钩("收敛"变成多轮随机挣脱恰好落进容差)。
       giveup_dist_scale由main.c按剩余距离(度)换算好传入，本模块不关心
       具体单位，非位置环场景固定传1.0即可(等价旧行为)。
     - 卡滞/giveup挣脱平滑过渡(见本文件顶部第8点说明)：卡滞助推/giveup
       退出(erpm_now刚越过kick_erpm)那一刻额外撑起kick_exit_hold_ms窗口，
       期间iq_cap维持在kick_exit_hold_iq，不立刻跌回settle_iq_cap，避免
       转子刚有点动能就被抽走继续加速的能力、又停回去，导致整个行程被
       拆成一段段"卡住->顶一下->又卡住"的台阶而不是连贯的加速-匀速-减速。
     - 卡滞进入滞回滤波+退出边沿确认(见cfg里stall_entry_edge_max/
       stall_exit_edge_count的说明)：进入判据额外要求去抖窗口期间边沿
       增量够低才真正判定卡死；退出判据额外要求erpm_now越过kick_erpm后
       再持续观测到足够多的边沿才真正确认挣脱，都要靠edge_cnt这个新增
       入参(main.c喂来的单调递增边沿计数)才能算增量。
     - 位置误差冻结的锁存/解锁(与stall_kicking联动)
   返回本周期应使用的iq上限(settle_iq_cap为呼叫方按现有settle-based逻辑
   算出的基线值，函数内部只会在此基础上收紧或按giveup逻辑抬高，不会
   无理由放宽)。edge_cnt：main.c维护的单调递增霍尔边沿计数(不会被任何
   诊断窗口清零)，供上面两条新增判据算"从某一刻到现在一共发生了几个
   边沿"这个跨任意时长的差值用。 */
float MotionGuard_Update(MotionGuard_t *mg, float erpm_now, float err,
                          uint32_t now, float settle_iq_cap, uint8_t arrived,
                          float giveup_dist_scale, uint32_t edge_cnt);

/* 供位置环调用：传入本周期实时算出的位置误差(度)+当前时间戳(ms)，返回
   应该实际用于计算目标转速(v_max)的误差——冻结态直接返回锁存值(同时
   刷新last_pos_err_live_deg供后续MotionGuard_Update锁存)；解冻态不是
   原样透传live值，而是从锁存值开始朝live值做一阶低通(时间常数见
   unfreeze_tau_s)，避免挣脱那一刻(冻结刚解除)立刻被可能仍然很大的
   live误差顶起一个大的v_max、造成二次冲击。 */
float MotionGuard_GetPosErrForSpeedCalc(MotionGuard_t *mg, float pos_err_live_deg, uint32_t now);

/* 供位置环/速度环调用：屏障激活时把speed_ref_target夹到屏障限速以内，
   未激活时原样返回。 */
float MotionGuard_ClampSpeedRef(MotionGuard_t *mg, float speed_ref_target);

/* 供挣脱检测调用(典型场景是霍尔跳变中断本身，反应要快)：只置一个轻量
   pending标志和观测到的瞬时转速，不做浮点状态机运算，可安全从ISR调用。
   真正的屏障力度计算/生效在下一次MotionGuard_Update里完成。 */
void MotionGuard_NotifyBreakaway(MotionGuard_t *mg, float omega_erpm);

/* 供main.c的电流斜坡速率选择处调用：本周期iq_cap是不是正在用giveup_iq
   这条底线。giveup_iq本身只抬高了上限，foc_iq_ref真正追上去还要经过
   main.c里危险区专用的慢速斜坡(原本是为了避免转矩突变造成冲击而故意
   放慢的)——但giveup场景下真正挣脱那一下的冲击已经由挣脱冲击观测限速
   屏障单独兜底，没必要再叠加这层慢速斜坡，让"卡死后要等很久才能挣脱"
   变相被两层慢速率拖长。main.c据此在giveup_active时改用专门更快的斜坡。 */
uint8_t MotionGuard_IsGiveupActive(const MotionGuard_t *mg);

/* 供main.c在"模式切换本身有风险、但没有可靠的观测转速可以拿来算severity"
   这类场景调用：固定用cfg->kick_exit_barrier_severity(不看erpm_now)撑
   屏障，原本只给卡滞助推/giveup退出这两处内部场景用(见motion_guard.c
   里的完整说明)，现在放开给main.c，蠕行模式(FOC_POS_CREEP_*)退出切回
   闭环那一刻同样适用——真正的风险不是"退出时观测到的转速有多快"，是
   "foc_theta的角度来源要在这一拍从蠕行自己的开环推算值瞬间切回霍尔
   实测值"，只要两者有残留偏差，切换本身就可能瞬间改变实际输出转矩的
   方向/大小，跟观测到的转速快慢没有直接关系，所以必须用固定severity。 */
void MotionGuard_ArmBarrierKickExit(MotionGuard_t *mg, uint32_t now);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_GUARD_H */
