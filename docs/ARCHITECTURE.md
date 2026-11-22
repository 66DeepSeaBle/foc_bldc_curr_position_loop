# BldcClamp 固件架构说明

> 面向"第一次读这套代码"的人：先看第 0、1、3 章就能跑通全貌，第 4 章之后是细节。
> 本文只描述**现在代码实际是什么样**，不描述"应该是什么样"。

---

## 0. 一页速览

| 项 | 值 |
|---|---|
| 目标芯片 | STM32G431RBT6，170 MHz（HSE 8 MHz → PLL N=85/P=2） |
| 控制频率 | **20 kHz**（= PWM 频率，中心对齐 TIM1） |
| 位置律频率 | **10 kHz**（`POS_LAW_DIV = 2`，见 6.3） |
| 位置反馈 | 3 路数字霍尔，6 扇区，4 对极 ⇒ 1 扇区 = **15.467° 机械** |
| 电流采样 | 0.01 Ω 采样电阻 + 内部 PGA ×8 ⇒ **0.080 V/A ⇒ 10.07 mA/LSB** |
| 代码规模 | `Core/` 下 22 个 .c，共约 **8 750 行**（含 CMSIS/HAL 样板约 560 行）；其中 `foc_control.c` 占 3 872 行 |
| 编译 | Keil MDK，ARMCC V5.06，`-O3`，C99 |
| 产物 | `_archive/builds/BldcClamp.hex`（版本台账见 `_archive/builds/README.md`） |

**一句话概括**：这是一个"ADC 注入中断里跑完整 FOC，主循环只做按键和日志"的裸机程序。
唯一的实时路径是 `TIM1 → ADC 注入 → HAL 回调 → App_OnCurrentSample() → FOC_Control_Run()`，
其余一切都是它的旁观者。

---

## 1. 分层与依赖规则

### 1.1 分层图

```
┌──────────────────────────────────────────────────────────────────┐
│ main.c  (36 行)                                                  │
│   BSP_Init() → App_Init() → while(1) App_Loop()                   │
└───────────────────────────┬──────────────────────────────────────┘
                            │ 只依赖 app.h
┌───────────────────────────▼──────────────────────────────────────┐
│ app.c / app.h  (1 076 行)   应用层                                │
│   按键、日志节流、零电流标定、开机窗口、VOFA 帧组装                │
│   ⚠ 不含任何寄存器操作                                           │
│                        ┌─────────────────────────────────────────┤
│                        │ vofa.c / vofa.h  (164 行)                │
│                        │   FireWater 文本协议 + 整数格式化         │
│                        │   发送走 BSP_UART_TxAsync（不碰 HAL）     │
└────────────────────────┴──────────────────┬──────────────────────┘
                                            │ 只依赖 .h
┌───────────────────────────────────────────▼──────────────────────┐
│ foc_control.c / .h  (3 872 行)   控制律（平台无关）               │
│   状态机 + 三个环的编排 + 定时轨迹 + 摩擦前馈 + 到位静音            │
│                                                                  │
│   ┌──────────────┬──────────────┬──────────────┬───────────────┐ │
│   │ angle_observer│ current_loop │ speed_loop   │ position_loop │ │
│   │  (700)        │  (55)        │  (103)       │  (89)         │ │
│   ├──────────────┼──────────────┼──────────────┼───────────────┤ │
│   │ pwm_output   │ transform    │ pid          │ foc_types.h   │ │
│   │  (73)        │  (107)       │  (75)        │  纯数据结构    │ │
│   └──────────────┴──────────────┴──────────────┴───────────────┘ │
│                                                                  │
│   ┌──────────────────────────────────────────────────────────┐   │
│   │ sensor.h  ── 位置传感器抽象（函数指针表）                  │   │
│   │   sensor_hall.c (75)  /  sensor_encoder.c (110)           │   │
│   └──────────────────────────────────────────────────────────┘   │
└───────────────────────────────────────────┬──────────────────────┘
                                            │ 只依赖 bsp.h
┌───────────────────────────────────────────▼──────────────────────┐
│ bsp_stm32g4.c / bsp.h  (612 行)   平台层 ★ 唯一碰 HAL/寄存器的地方 │
│   board_config.h  ── 唯一的功能配置文件（引脚/句柄/硬件参数）      │
│   ┌────────┬────────┬────────┬────────┬────────┐                 │
│   │ adc.c  │ tim.c  │ gpio.c │ opamp.c│ usart.c│  ← CubeMX 生成   │
│   └────────┴────────┴────────┴────────┴────────┘                 │
└──────────────────────────────────────────────────────────────────┘
```

### 1.2 依赖规则（实际检查结果）

| 规则 | 状态 | 说明 |
|---|---|---|
| 算法层（`foc/` 除 `foc_control.c`）只 include `foc_types.h` | ✅ 全部满足 | `pid` / `transform` / `current_loop` / `speed_loop` / `position_loop` / `pwm_output` / `foc_types` 一个 `HAL_` 都没有 |
| 只有 `bsp_*.c` 碰 HAL 和寄存器 | ⚠️ 有泄漏 | 第 3 条（DWT）在算法层；见 1.3 |
| 上层不 include `main.h` | ⚠️ 部分违反 | `app.h`、`vofa.h` include 了 `main.h` |
| 执行器输出走 BSP | ✅ | `pwm_output.c` 的 `BSP_PWM_SetDuty3` / `BSP_PWM_Off` |
| 串口输出走 BSP | ✅ | `vofa.c` 的 `BSP_UART_TxAsync`，注释明确写了"换芯片不影响本文件" |

### 1.3 四个移植泄漏点（都在意料之内，但要知道）

| # | 位置 | 内容 | 影响 | 修法（可选） |
|---|---|---|---|---|
| 1 | `app.c:31,631,978` | `#include "SEGGER_RTT.h"` + `SEGGER_RTT_GetKey()` | 换平台必须改 `app.c` | 抽成 `BSP_Debug_GetKey()` |
| 2 | `app.h:10`、`vofa.h:30` | `#include "main.h"` | `main.h` 会引入 `stm32g4xx_hal.h` ⇒ 上层间接依赖具体芯片 | 改成 `<stdint.h>`（`uint8_t` 够用） |
| 3 | `foc_control.c:1840…` 共 10 处 | `DWT->CYCCNT` | DWT 是 **Cortex-M 内核**通用外设，非 STM32G4 专有，但仍是"寄存器级"，且未走 BSP | 抽成 `BSP_Prof_Cycles()` |
| 4 | `board_config.h` | 直接写 `htim1` / `huart2` / `USART2` / `GPIOB` / `GPIO_PIN_3`，并 include `main.h` | 该文件本身就是**平台相关**的，不能当"纯配置"跨平台复用 | 换芯片时改这一个文件 + 新写 `bsp_xxx.c` |

> **结论**：分层是真实的、不是摆设。前 3 条加起来约 15 行代码，属于"清尾"工作，
> 不影响现在的可维护性；第 4 条是设计意图（配置文件允许平台相关）。

---

## 2. 目录与文件职责

| 文件 | 行数 | 层 | 职责（一句话） |
|---|---:|---|---|
| `Core/Src/main.c` | 36 | 入口 | 三行调用，别加逻辑 |
| `Core/app/app.c` | 1 076 | 应用 | 按键/日志/标定/开机窗口/VOFA 帧组装 + 2 个中断钩子 |
| `Core/app/vofa.c` | 164 | 应用 | FireWater 帧格式化（整数运算，避免中断里用浮点 printf） |
| `Core/foc/foc_control.c` | 3 872 | 控制 | **状态机 + 三环编排 + 轨迹 + 前馈**（全工程唯一的大文件） |
| `Core/foc/angle_observer.c` | 700 | 算法 | 霍尔扇区 → 连续电角度/机械角/转速（**全工程最复杂的一块**） |
| `Core/foc/pwm_output.c` | 73 | 算法 | SVPWM 扇区计算 + 死区补偿 + 下发 |
| `Core/foc/current_loop.c` | 55 | 算法 | d/q 双 PI |
| `Core/foc/speed_loop.c` | 103 | 算法 | 速度 PI + 指令斜坡 + Iq 限幅切换 |
| `Core/foc/position_loop.c` | 89 | 算法 | 位置 P + 限速 + 到位判定 |
| `Core/foc/transform.c` | 107 | 算法 | Clarke / Park / 逆 Park / 逆 Clarke + **512 点正弦查表** |
| `Core/foc/pid.c` | 75 | 算法 | 通用 PID + 一阶低通 |
| `Core/foc/sensor_hall.c` | 75 | 传感器实现 | 把 `angle_observer` 包成 `Sensor_Driver_t` |
| `Core/foc/sensor_encoder.c` | 110 | 传感器实现 | 编码器骨架（`CFG_SENSOR_TYPE` 选） |
| `Core/bsp/bsp_stm32g4.c` | 612 | 平台 | 时钟/启动/日志队列/RTT/中断回调分发/负载计时 |
| `Core/bsp/adc.c` `tim.c` `gpio.c` `opamp.c` `usart.c` | 287/346/70/195/144 | 平台 | CubeMX 生成 + 手改的 `USER CODE` 段 |
| `Core/Inc/log.h` | 64 | 基础设施 | `LOGI/D/W/E` 分级宏, 统一走 `LOG_Printf` |
| `Core/foc/FOC_APP_API.md` | — | 文档 | **上层调用接口**（力矩档/速度档/MoveTo），与本文互补 |
| `docs/CONTROL_COMMANDS.md` | — | 文档 | **调试控制命令手册**（按键、移动命令、档位、自检、日志读法） |
| `docs/ARCHITECTURE.md` | — | 文档 | 本文 |

---

## 3. 三种时间尺度（理解全部代码的钥匙）

这个程序只有三个"心脏"，其它都是它们的旁观者。

### 3.1 20 kHz —— 唯一实时路径

```
TIM1（中心对齐，ARR=4249）
   │  计数器下降经过 CCR4 = 98.85% × ARR
   ▼
OC4REF 上升沿 ──► TIM1 TRGO (TIM_TRGO_OC4REF)
   │              （此刻三相下管全通，分流电阻上都有电流）
   ├──────────────► ADC1 注入组：rank1 = U (CH3)，rank2 = W (CH12)
   └──────────────► ADC2 注入组：rank1 = V (CH3)
                       │
                       ▼  ADC2 转换完成
        HAL_ADCEx_InjectedConvCpltCallback(hadc2)      [bsp_stm32g4.c:586]
          ├─ 读 ADC1 rank1/rank2 → s_adc_iu_raw / s_adc_iw_raw  ← ADC1 先完成
          ├─ 读 ADC2 rank1       → s_adc_iv_raw
          ├─ DWT 打点 t0
          └─ App_OnCurrentSample()                       [app.c:808]
               ├─ BSP_ADC_GetCurrentRaw(&ru,&rv,&rw)
               ├─ 换安培：iu = (ru − off_u) × s_i_k，iv 同理
               ├─ current.w = −(iu + iv)        ← 强制三相和 = 0
               ├─ g_sensor.Poll(&g_foc)          → Hall_Poll → AngleObserver_PollHall
               └─ FOC_Control_Run(&g_foc, 1/20000)
                    ├─ 过流检查（开通消隐 100 ms + 连续 4 次去抖）
                    ├─ foc->tick++
                    └─ switch(state) → FOC_DoIdle / Align / Startup / …
             ← DWT 打点，交给 BSP_Prof_RecordIsr()
```

> ⚠ **`App_OnCurrentSample()` 里不要打任何日志**。`log.h` 已经做了"中断里只写环形队列"
> 的兜底，但仍建议保持这条路径干净。

### 3.2 1 kHz —— 波形输出

```
TIM6 更新中断 → HAL_TIM_PeriodElapsedCallback      [bsp_stm32g4.c]
   → App_OnWaveTick()                              [app.c]
      → VOFA_Send5(ch0..ch4)  → VOFA_Send()        [vofa.c]
         → 整数格式化进 s_frame[]
         → BSP_UART_TxAsync() → USART2 + DMA1_Channel1
```

**非阻塞**：上一帧没发完直接丢弃（`VOFA_GetDropCount()` 计数），绝不等待。

### 3.3 主循环 —— 其余一切

```
while (1) App_Loop()                               [app.c:1162]
   ├─ App_PollKeys()          轮询 RTT 键盘
   ├─ App_PollPwmEvent()      打印中断里攒下的 PWM 通断事件
   ├─ BSP_Log_DeferFlush()    把中断里入队的日志写出 ← 必须紧跟在按键之后
   ├─ App_ReportSlow()        [ADCR] / [MAN] / [OL]
   ├─ (霍尔验证模式则 2 ms 节拍早退)
   ├─ App_ReportMain()        [MAIN] / [ANG]
   ├─ App_ReportProf()        [PROF] 中断负载率
   └─ BSP_Time_DelayMs(10)
```

---

## 4. 一次采样 → 一次 PWM 的完整数据流

以 **POSITION_LOOP** 状态、且本拍轮到外层位置律为例：

```
① 电流   u/v/w ──Clarke──► iα/iβ ──Park(θ)──► id/iq
② 误差   id_ref = 0 ；iq_ref = 位置律输出 × torque_dir
③ 电流环 CurrentLoop_Update() → vd/vq        （单位：占空比量，不是伏特！）
④ 逆Park vα/vβ = f(vd, vq, θ)
⑤ SVPWM  vα/vβ → 三相占空比 du/dv/dw（含死区补偿 dt_comp_duty）
⑥ 下发   PWM_Output_Apply() → BSP_PWM_SetDuty3() → TIM1->CCR1..3
```

### 4.1 单位约定（最容易搞混的地方）

| 量 | 单位 | 备注 |
|---|---|---|
| `current.u/v/w` | A | 已扣零点、已乘极性 |
| `current_loop.id/iq` | A | d/q 轴 |
| `current_loop.id_ref/iq_ref` | A | 存的是**未乘 torque_dir** 的值 |
| `current_loop.vd/vq` | **占空比量**（0~±0.5） | 不是伏特。`POS_FF_DUTY_PER_A = 0.028` ⇒ 1 A ≈ 0.028 duty |
| `pwm.du/dv/dw` | 0~1 | 直接写 `CCR` |
| `angle.theta` | rad | 电角度，0~2π |
| `angle.adv` | rad | **扇区内进角**，被钳在 ±`theta_halfwidth` |
| `tick` / `edge_tick` / `*_ticks` | 20 kHz 拍 | 毫秒换算一律用 `FOC_TICKS_PER_MS` |
| `*_erpm` | eRPM | 4 对极 ⇒ 1 eRPM = 1.5 机械 °/s |
| 位置（`GetMechDeg`） | 机械 ° | 多圈累加 |

### 4.2 为什么 ADC 采样点定在 98.85% × ARR

见 `Core/bsp/tim.c` 的 TIM1 初始化段(约 150~215 行)。要点：

- 中心对齐 PWM 在**峰顶**时三相下管全通 ⇒ 此时分流电阻上都有电流，能同时测三相
- 谷底时上管在通 ⇒ 测不到
- `CCR4 = 98.85% × ARR` 让 OC4REF 正好落在峰顶之后不久
- **CCR4 必须跟着 ARR 走**（硬编码 4200 曾在 25 kHz 下导致 ADC 中断彻底停摆）

---

## 5. 状态机

```
        ┌──────┐  按 s/o/p       ┌───────┐
        │ IDLE │────────────────►│ ALIGN │
        └──────┘                 └───┬───┘
           ▲                         │ 对齐完成 + 零点标定
           │                         ▼
           │                    ┌─────────┐
           │                    │ STARTUP │  I-F 开环强拖，等霍尔同步
           │                    └────┬────┘
           │                         │ hall_synced
           │                    ┌────▼──────────┐
           │                    │ CURRENT_LOOP  │◄── j/k 力矩探针
           │                    └────┬──────────┘
           │                         │
           │              ┌──────────▼────────┐
           │              │   SPEED_LOOP      │
           │              └──────────┬────────┘
           │                         │
           │              ┌──────────▼────────┐
           │              │  POSITION_LOOP    │◄── 数字键 0~9 / z / t<deg>
           │              └──────────┬────────┘
           │                         │
           └─────────────────────────┘  到位 + 静音
                 BRAKE / FAULT / OPENLOOP 是旁支，可随时进入
```

实现：`FOC_Control_Run()` 里一个 `switch(foc->state)`，每个状态一个 `FOC_DoXxx()` 静态函数。
**没有状态表、没有函数指针数组**——就是硬编码 switch，故意做得一眼能看完。

| 状态 | 关键动作 |
|---|---|
| IDLE | 关输出，等按键 |
| ALIGN | 强制 θ=0 注入 Id，把转子拉到已知位置；结束时调 `g_sensor.CalibrateAtZero()` 反解霍尔零点 |
| STARTUP | θ 自行旋转（I-F），电流环钳 `startup_current`，等 `hall_synced` |
| CURRENT_LOOP | 只跑电流环，`iq_ref` 来自按键 |
| SPEED_LOOP | `SpeedLoop_Update()` → `iq_ref` → 电流环 |
| POSITION_LOOP | 见第 6.3 节 |
| BRAKE | 三相下管同时导通（短路制动） |
| OPENLOOP | `s` 键 V/f 自检，θ 由软件积分 |
| FAULT | 关输出，按 `r` 复位 |

---

## 6. 三个环

### 6.1 电流环 —— 20 kHz

`CurrentLoop_Update()` → 两个独立 PI（`pid_d` / `pid_q`），输出限幅 `vlim`（占空比量）。

**注意 `torque_dir` 的处理**：方向修正在**指令侧**做（`iq_cmd = iq_ref × torque_dir`），
反馈保持正常负反馈。取反反馈会变正反馈，q 轴电流跑飞。

### 6.2 速度环 —— 20 kHz

`SpeedLoop_Update()` 里除了 PI 还有：
- 指令斜坡（`ramp_rate_erpm_s`）
- `iq_max_normal` ↔ `iq_max_zone` 的限幅切换（稳定后降限幅）

### 6.3 位置环 —— **10 kHz**（`POS_LAW_DIV = 2`）

`FOC_DoPositionLoop()` 内部按 1.3 的顺序分成三段：

```
PRE 段（每拍 20kHz）
    FOC_UpdateAngle()      ← 角度观测器，Park 角度必须每拍更新
    动/静判定、看门狗、跳变检测
        │
        ▼  分频门（每 2 拍放行一次，dt_s ×= POS_LAW_DIV）
LAW 段（10 kHz）—— 全部在 foc_control.c 的 POS_TM_* 宏区
    ① 定时轨迹 4 段：加速 2/9 + 匀速 3/9 + 减速 2/9 + 逼近 2/9
    ② 摩擦前馈  ff = Stribeck(FRIC_STATIC 1.45A, FRIC_SLIDE 0.35A)
    ③ 微调阻尼  tr = P_TRIM(0.030 A/°) × verr + DAMP × (v_eff − vobs)
    ④ 破黏/收尾  hw=1（起步）、hw=2（掉队）
    ⑤ 输出 iq_ref
        │
        ▼
CUR 段（每拍 20kHz）  FOC_UpdateCurrent()  ← 见第 4 章
```

**踩过的坑（别重复）**：
- `POS_LAW_DIV = 2` ⇒ 位置律心跳是 10 kHz，**不是** ADC tick。按"位置律心跳"累加的
  计数器除以 `FOC_CTRL_HZ` 会得到一半速度（A2h 的 bug）。
- 任何按**阈值**切换前馈/增益强弱的写法都会退化成 bang-bang（A2e、A2h 两次踩）。
- 被控对象速度是 15.47° 阶梯 ⇒ 任何"让参考等转子"的做法都会把参考也变成阶梯。

---

## 7. 三个关键抽象（移植的支点）

### 7.1 `bsp.h` —— 平台接口

共 **9 组 22 个函数**，全部无参或参数为基本类型/数组，**不出现任何 HAL 类型**：

```c
BSP_Init()                          /* 时钟/GPIO/ADC/TIM/OPAMP/UART */
BSP_PWM_Start / SetDuty3 / Off
BSP_ADC_StartCurrentSampling / GetCurrentRaw / ReadVbusVolts / GetVbusRaw
BSP_Analog_Start
BSP_Hall_ReadSector                 /* 返回 1..6，0/7 = 非法 */
BSP_Time_Ms / DelayMs
BSP_UART_IsTxReady / TxAsync
BSP_WaveTimer_Start
BSP_Prof_*                          /* DWT 周期计时 */
BSP_Log_DeferFlush / GetRttDrop / GetRttBad
```

**换芯片 = 新写一个 `bsp_gd32.c` 实现这 22 个函数**，其余零改动。

### 7.2 `Sensor_Driver_t` —— 位置传感器抽象

```c
typedef struct {
    const char *name;
    void    (*Poll)(FOC_Control_t *);            /* ★ 有副作用，每周期只调一次 */
    float   (*GetMechDeg)(const FOC_Control_t *);
    float   (*GetElecThetaRad)(const FOC_Control_t *);
    float   (*GetErpm)(const FOC_Control_t *);
    uint8_t (*GetSector)(const FOC_Control_t *);
    void    (*CalibrateAtZero)(FOC_Control_t *, uint8_t sector);
} Sensor_Driver_t;

extern const Sensor_Driver_t g_sensor;   /* 由 CFG_SENSOR_TYPE 编译期二选一 */
```

**接口约定**（写在 `sensor.h` 里，很重要）：
- `Poll()` **有副作用**，只在 `App_OnCurrentSample()` 里调一次
- 其余 `Get*()` **只读**，随便在哪调；若它们也推进观测器，一个周期里会重复外推，位置就漂了

### 7.3 `board_config.h` —— 唯一的功能配置文件

11 个分节：

| 节 | 内容 |
|---|---|
| 1 | 平台选择（STM32G4 / GD32） |
| 2 | 传感器选择（HALL / ENCODER） |
| 3 | 串口（实例、引脚、AF、DMA、波特率） |
| 4 | 功率级 / PWM（频率、死区、最大占空比） |
| 5 | ADC 句柄分配 |
| 6 | 定时器任务（TIM6 = 波形节拍，TIM3 = 霍尔时间戳） |
| 7 | 霍尔引脚 |
| 8 | 电机 / 硬件标定（极对数、R、shunt、增益、极性、分压比） |
| 9 | 控制默认参数（对齐/起步电流、斜坡） |
| 10 | 自检测试参数 |
| 11 | 日志节流周期 |

---

## 8. 参数到底改在哪（三层，别改错地方）

| 想改什么 | 改哪 | 备注 |
|---|---|---|
| 引脚 / 串口 / 采样电阻 / 极对数 / 分压比 | `Core/bsp/board_config.h` | 改完要重编 |
| 日志频率 | `board_config.h` 第 11 节 `CFG_LOG_*_MS` | |
| 日志级别 | `Core/Inc/log.h` 的 `LOG_MIN_LEVEL` | INFO 默认编译掉 |
| 轨迹/前馈/破黏/到位静音等调参量 | `foc_control.c` **顶部 `POS_TM_*` / `PWM_IDLE_OFF_*` 宏区** | 这是调参主战场 |
| 力矩档 / 速度档默认值 | `Core/foc/foc_app.h` 的 `FOC_APP_*` | 运行期也能用 `[` `]` `{` `}` 调 |
| 控制/PWM 频率 | `foc_types.h` 的 `FOC_CTRL_HZ` **+** `tim.c` 的 `htim1.Init.Period` | 两处必须同步，公式见 7.3 注释 |
| VOFA 通道内容 | `app.c` 的 `App_OnWaveTick()` | 只影响显示 |

---

## 9. 移植 / 替换操作清单

### 9.1 换芯片

1. 新建 `Core/bsp/bsp_<chip>.c`，实现 `bsp.h` 全部 22 个函数
2. `board_config.h` 第 1 节改 `CFG_PLATFORM`，第 3~7 节改句柄与引脚
3. 修 1.3 的 4 个泄漏点（`SEGGER_RTT_GetKey`、`main.h`、`DWT->CYCCNT`）
4. `Core/foc/` 整个目录**不动**

### 9.2 换位置传感器

1. 实现 `sensor.h` 的 6 个函数，填一张 `Sensor_Driver_t`
2. `board_config.h` 第 2 节改 `CFG_SENSOR_TYPE`
3. `app.c` / `foc_control.c` **不动**（`ALIGN` 零点标定已经走 `g_sensor.CalibrateAtZero`）

### 9.3 换电机

只改 `board_config.h` 第 8 节：`CFG_MOTOR_POLE_PAIRS` / `CFG_MOTOR_RS_OHM` /
`CFG_SHUNT_R_OHM` / `CFG_OPAMP_GAIN` / `CFG_CURRENT_SIGN` / `CFG_VBUS_DIVIDER`。
然后重新做零电流标定与 `ALIGN`。

### 9.4 改 PWM 频率

`foc_types.h: FOC_CTRL_HZ` + `tim.c: htim1.Init.Period = 170e6/(2·f) − 1`。
两处必须同步；`CCR4` 会自动跟着 `ARR` 走（已修）。改完核对 `[PROF]` 里的 `load=`。

---

## 10. 日志与波形

> 完整按键表、`t<deg> d<sec>` 语法、档位派生限幅、自检流程、字段速查，
> 见配套文档 **[`docs/CONTROL_COMMANDS.md`](CONTROL_COMMANDS.md)**。

### 10.1 日志分级与节流

| 标签 | 周期 | 内容 |
|---|---|---|
| `[MAIN]` | 运动 100 ms / 静止 500 ms / 详细 25 ms | 全状态一行（最有用的一条） |
| `[ADCR]` | 5 s（详细 1 s） | ADC 原始码 / 零点 / 换算后电流 / 固件 tag |
| `[PROF]` | 1 s | 中断负载率 + 各段耗时 + `rttdrop` / `rttbad` |
| `[ANG]` | 10 ms | **INFO 级，默认编译掉** |
| `[TM]` / `[POS]` / `[PWM]` / `[CMD]` / `[KEY]` | 事件驱动 | |

### 10.2 `[PROF]` 怎么读

```
FOC@20k  load=54.4%  avg=27.4us max=38.7us    ← 整个 20kHz 中断
VOFA@1k  load=1.7%   avg=17.3us max=48.5us    ← TIM6 组帧
ANG      avg=3.02us                            ← FOC_UpdateAngle
PRE      avg=7.05us                            ← 分频门之前
LAW      avg=11.8us                            ← 外层位置律（10kHz，所以 avg 看着低）
CUR      avg=9.5us                             ← Clarke/Park/PI/逆Park/SVPWM
idle=43%                                       ← CPU 空闲
```

`[MAIN]` 字段速查见 `FOC_APP_API.md` 与 `app.c:747` 的格式串。

---

## 11. 已知技术债（按严重度排序）

| # | 问题 | 影响 | 建议 |
|---|---|---|---|
| 1 | **残留编码问题（已大幅修复）**：`Core/foc/` 下 14 文件 77 处被写成 `?` 的中文注释已于 2026-11-22 全部重写（二进制 SHA256 不变，纯注释改动）。仍残留 2 行含**非法字节**：`foc_control.c` 的 `theta_align = 1.57079633f;` / `= 0.0f;` 行尾的 `°` 是原始 GBK 残留（显示为 `�`） | 只影响这 2 行的可读性，语义已由上方新加的注释覆盖 | 需做一次整体编码规范化（UTF-8）才能彻底清掉 |
| 2 | `foc_control.c` 3 872 行（占全工程 44%），`FOC_DoPositionLoop()` 单个函数超 1 100 行 | 定位困难，改一处要通读 | 建议后续按"轨迹 / 前馈 / 爬行脱困 / 输出"拆成 4 个 `static` 段函数，**但改动风险高，必须单独一版** |
| 3 | `app.h` / `vofa.h` include `main.h` | 上层间接依赖 HAL | 改 `<stdint.h>` |
| 4 | `app.c` 直接用 `SEGGER_RTT_GetKey()` | 换平台要改应用层 | 抽 `BSP_Debug_GetKey()` |
| 5 | `foc_control.c` 用 `DWT->CYCCNT` | 未走 BSP | 抽 `BSP_Prof_Cycles()` |
| 6 | `foc_control.c` 里大量"实验日志摘录式"注释 | 信息量高但噪音也高 | 保留结论，把过程日志移到 `docs/` |
| 7 | 到位静音（`po=1`）时 `FOC_UpdateCurrent()` 提前返回 ⇒ `[MAIN]` 的 `Id/Iq/du` 是**冻结旧值** | 极易误读（"明明没电流却显示 Iq=-0.35"） | 在 `po=1` 时把这三个字段打 0 或加标记 |

---

## 12. 建议阅读顺序

1. `board_config.h`（134 行，头部注释约 35 行讲清分层）→ 通读整个文件
2. `main.c`（36 行）
3. `app.c` 的 `App_Init()` / `App_OnCurrentSample()` / `App_Loop()`
4. `bsp.h`（94 行，讲清平台契约）
5. `foc_types.h`（208 行，纯数据结构，看一遍就懂全局状态）
6. `foc_control.c` 的 `FOC_Control_Run()` + `FOC_UpdateCurrent()`（约 100 行，是核心骨架）
7. `pwm_output.c` / `transform.c` / `current_loop.c`（三个小文件，10 分钟看完）
8. `angle_observer.c`（700 行，**最后看**，它最复杂且被大量实机教训注释包裹）
9. `foc_control.c` 的位置律部分（`POS_TM_*` 宏区 + `FOC_DoPositionLoop`）
10. `FOC_APP_API.md`（上层调用接口）
11. `docs/CONTROL_COMMANDS.md`（调试命令手册，实际动手时对着看）

---

*本文与代码同源维护。改分层、改调用链、增删模块时请同步更新本文与 `FOC_APP_API.md`。*
