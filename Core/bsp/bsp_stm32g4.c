/**
  ******************************************************************************
  * @file    bsp_stm32g4.c
  * @brief   STM32G4 平台的 BSP 实现: 时钟、PWM、ADC、霍尔引脚、串口、调试输出。
  ******************************************************************************
  *
  *  这里是"换芯片时唯一要重写"的文件。上面几层(app.c / sensor_*.c / 算法)
  *  只通过 bsp.h 访问硬件, 所以端口到 GD32 时不需要碰它们。
  *
  *  代码来源: 原先散在 main.c 里的 MotorSetDuty3 / MotorAllOff / Vbus_ReadVolts
  *  / 霍尔引脚读取 / HAL 回调, 原样搬到这里(行为完全不变), 只是把裸寄存器访问
  *  集中到这一处。
  *
  ******************************************************************************
  */
#include "bsp.h"
#include "app.h"          /* 只为两个 ISR 钩子: App_OnCurrentSample / App_OnWaveTick */
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "log.h"
#include "SEGGER_RTT.h"
#include <stdio.h>
#include <stdarg.h>       /* ★ LOG_Printf 用 va_list/vsnprintf */

/* 上游 CubeMX 生成的外设句柄(定义在 adc.c / tim.c / usart.c 里) */
extern ADC_HandleTypeDef  hadc1;
extern ADC_HandleTypeDef  hadc2;
extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim6;
extern TIM_HandleTypeDef  htim3;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_tx;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

/* CubeMX 生成的初始化函数 */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_TIM1_Init(void);
void MX_TIM6_Init(void);
void MX_TIM3_Init(void);
void MX_USART2_UART_Init(void);
void MX_OPAMP1_Init(void);
void MX_OPAMP2_Init(void);
void MX_OPAMP3_Init(void);

/* ---- 本文件私有状态 ---- */
static uint16_t s_adc_iw_raw = 0;      /* ADC1 注入 rank2 = W 相(与 U 同一片 ADC) */
static uint16_t s_adc_iu_raw = 0;      /* ADC1 注入 rank1 = U 相 */
static uint16_t s_adc_iv_raw = 0;      /* ADC2 注入 rank1 = V 相 */
static uint16_t s_vbus_raw   = 0;
static uint8_t  s_pwm_off_logged = 0;

/* ==========================================================================
 * 初始化
 * ========================================================================== */
void BSP_Init(void)
{
    HAL_Init();
    SystemClock_Config();          /* 170MHz: HSE + PLL(见文件末尾), APB1/2 = 85MHz */

    /* 顺序有意义: GPIO 先配好(霍尔 EXTI), TIM3 的输入捕获要在其后追加 AF 复用 */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_TIM1_Init();
    MX_TIM6_Init();
    MX_TIM3_Init();
    MX_USART2_UART_Init();
    MX_OPAMP1_Init();
    MX_OPAMP2_Init();
    MX_OPAMP3_Init();

    SEGGER_RTT_Init();             /* 调试输出(RTT, 不需要串口) */

    BSP_Prof_Init();               /* DWT 周期计数器: 中断负载率测量 */
}

/* ==========================================================================
 * 中断负载率测量(DWT->CYCCNT)
 * --------------------------------------------------------------------------
 * CYCCNT 是 32 位、每 HCLK 周期 +1。170MHz 下 25.3s 溢出一次 —— 本测量窗口
 * 只有 1s, 而且全部用"无符号减法", 所以溢出不会出错。
 * 累加变量都是 volatile, 中断里写、主循环里带中断屏蔽读并清零 ⇒ 无竞态。
 * ========================================================================== */
static volatile uint32_t s_pf_isr_sum = 0U, s_pf_isr_max = 0U, s_pf_isr_cnt = 0U;
static volatile uint32_t s_pf_wav_sum = 0U, s_pf_wav_max = 0U, s_pf_wav_cnt = 0U;
static volatile uint32_t s_pf_win_start = 0U;

void BSP_Prof_Init(void)
{
    /* 必须先开 TRCENA, 否则 DWT 的寄存器不可访问/不计数 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    s_pf_isr_sum = 0U; s_pf_isr_max = 0U; s_pf_isr_cnt = 0U;
    s_pf_wav_sum = 0U; s_pf_wav_max = 0U; s_pf_wav_cnt = 0U;
    s_pf_win_start = DWT->CYCCNT;
}

void BSP_Prof_RecordIsr(uint32_t cyc)
{
    s_pf_isr_sum += cyc;
    s_pf_isr_cnt++;
    if (cyc > s_pf_isr_max) { s_pf_isr_max = cyc; }
}

void BSP_Prof_RecordWave(uint32_t cyc)
{
    s_pf_wav_sum += cyc;
    s_pf_wav_cnt++;
    if (cyc > s_pf_wav_max) { s_pf_wav_max = cyc; }
}

void BSP_Prof_Take(BSP_Prof_t *out)
{
    if (out == 0) { return; }
    __disable_irq();
    out->isr_cycles = s_pf_isr_sum;  s_pf_isr_sum = 0U;
    out->isr_max    = s_pf_isr_max;  s_pf_isr_max = 0U;
    out->isr_count  = s_pf_isr_cnt;  s_pf_isr_cnt = 0U;
    out->wav_cycles = s_pf_wav_sum;  s_pf_wav_sum = 0U;
    out->wav_max    = s_pf_wav_max;  s_pf_wav_max = 0U;
    out->wav_count  = s_pf_wav_cnt;  s_pf_wav_cnt = 0U;
    out->win_cycles = DWT->CYCCNT - s_pf_win_start;
    s_pf_win_start  = DWT->CYCCNT;
    __enable_irq();
}

/* ==========================================================================
 * 功率级(PWM)
 * ========================================================================== */
void BSP_PWM_Start(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

/* 三相占空比下发。TIM1->ARR 是周期值, CCR 用同一计数范围; 上下桥同时使能。 */
void BSP_PWM_SetDuty3(float du, float dv, float dw)
{
    uint32_t arr = CFG_PWM_TIM->ARR + 1U;

    if (du < 0.0f) du = 0.0f;
    if (du > CFG_PWM_MAX_DUTY) du = CFG_PWM_MAX_DUTY;
    if (dv < 0.0f) dv = 0.0f;
    if (dv > CFG_PWM_MAX_DUTY) dv = CFG_PWM_MAX_DUTY;
    if (dw < 0.0f) dw = 0.0f;
    if (dw > CFG_PWM_MAX_DUTY) dw = CFG_PWM_MAX_DUTY;

    CFG_PWM_TIM->CCR1 = (uint32_t)(du * (float)arr);
    CFG_PWM_TIM->CCR2 = (uint32_t)(dv * (float)arr);
    CFG_PWM_TIM->CCR3 = (uint32_t)(dw * (float)arr);

    CFG_PWM_TIM->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE |
                          TIM_CCER_CC2E | TIM_CCER_CC2NE |
                          TIM_CCER_CC3E | TIM_CCER_CC3NE);

    s_pwm_off_logged = 0;         /* 下次关断可以再报一次 */
}

/* 关断三相输出。日志只在每次"开→关"跳变时打一条, 免得故障态反复刷屏。 */
void BSP_PWM_Off(void)
{
    CFG_PWM_TIM->CCR1 = 0;
    CFG_PWM_TIM->CCR2 = 0;
    CFG_PWM_TIM->CCR3 = 0;
    CFG_PWM_TIM->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                           TIM_CCER_CC2E | TIM_CCER_CC2NE |
                           TIM_CCER_CC3E | TIM_CCER_CC3NE);

    if (!s_pwm_off_logged) {
        LOGD("[PWM] MotorAllOff\r\n");
        s_pwm_off_logged = 1;
    }
}

/* ==========================================================================
 * 采样
 * ========================================================================== */
void BSP_ADC_StartCurrentSampling(void)
{
    /* 注入通道(相电流)用中断方式在 20kHz 触发, 回调里跑 FOC 控制环 */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADC_Start(&hadc1);        /* 规则通道(母线电压), 仅使能, 由 BSP_ADC_ReadVbusVolts 软件触发 */
}

void BSP_ADC_GetCurrentRaw(uint16_t *u, uint16_t *v, uint16_t *w)
{
    if (u) *u = s_adc_iu_raw;
    if (v) *v = s_adc_iv_raw;
    if (w) *w = s_adc_iw_raw;
}

/* 三相电流运放前端: 必须在 ADC 采样之前启动, 否则采到的是悬空电平(实测恒 0) */
void BSP_Analog_Start(void)
{
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);
}

/* 母线电压: ADC1 规则通道。ADC1 已被注入通道占用, HAL_ADC_Start() 会返回
   HAL_BUSY 且不触发转换 ⇒ 这里直接写 ADSTART 位做一次软件触发, 读 DR。
   (注入结果在 JDRx, 规则结果在 DR, 互不干扰。) */
float BSP_ADC_ReadVbusVolts(void)
{
    uint32_t guard = 0;
    float    v;

    CFG_ADC_VBUS_INSTANCE->ISR = ADC_ISR_EOC;          /* 清残留 EOC, 免读到旧值 */
    CFG_ADC_VBUS_INSTANCE->CR |= ADC_CR_ADSTART;       /* 软件触发一次规则转换 */
    while ((CFG_ADC_VBUS_INSTANCE->ISR & ADC_ISR_EOC) == 0U) {
        if (++guard > 200000U) return -1.0f;           /* 超时: 返回负值便于识别 */
    }
    s_vbus_raw = (uint16_t)CFG_ADC_VBUS_INSTANCE->DR;  /* 读 DR 会清 EOC */

    v = (float)s_vbus_raw * CFG_ADC_VREF / (float)(CFG_ADC_RESOLUTION - 1);
    return v * CFG_VBUS_DIVIDER;
}

uint16_t BSP_ADC_GetVbusRaw(void)
{
    return s_vbus_raw;
}

/* ==========================================================================
 * 数字霍尔引脚
 * ========================================================================== */
uint8_t BSP_Hall_ReadSector(uint8_t *u, uint8_t *v, uint8_t *w)
{
    uint8_t hu = (uint8_t)HAL_GPIO_ReadPin(CFG_HALL_U_PORT, CFG_HALL_U_PIN);
    uint8_t hv = (uint8_t)HAL_GPIO_ReadPin(CFG_HALL_V_PORT, CFG_HALL_V_PIN);
    uint8_t hw = (uint8_t)HAL_GPIO_ReadPin(CFG_HALL_W_PORT, CFG_HALL_W_PIN);

    if (u) *u = hu;
    if (v) *v = hv;
    if (w) *w = hw;
    return (uint8_t)((~((hu << 2) | (hv << 1) | hw)) & 0x07U);
}

/* ==========================================================================
 * 时间 / 串口
 * ========================================================================== */
uint32_t BSP_Time_Ms(void)
{
    return HAL_GetTick();
}

void BSP_Time_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

uint8_t BSP_UART_IsTxReady(void)
{
    /* gState 只有 DMA 搬完 + 移位寄存器发完(TC)之后才回 READY。
       ⚠ USART 的 TC 中断必须在 NVIC 里使能, 否则永远卡在 BUSY_TX。 */
    return (CFG_UART_HANDLE.gState == HAL_UART_STATE_READY) ? 1U : 0U;
}

uint8_t BSP_UART_TxAsync(const uint8_t *buf, uint16_t len)
{
    if (CFG_UART_CONSOLE == 0) return 0U;
    if (buf == 0 || len == 0U) return 0U;
    if (!BSP_UART_IsTxReady()) return 0U;              /* 上一帧没发完 ⇒ 让调用者丢帧 */
    return (HAL_UART_Transmit_DMA(&CFG_UART_HANDLE, (uint8_t *)buf, len) == HAL_OK)
               ? 1U : 0U;
}

/* ==========================================================================
 * 调试输出: printf → RTT(缓冲 0)
 * ========================================================================== */
/* ==========================================================================
 * 统一日志出口(★ cal25, 声明在 log.h)
 * --------------------------------------------------------------------------
 * 中断上下文里**不做 RTT 写**: printf→fputc→RTT_PutChar 是逐字符写, 一行要
 * 20~120us, 而 20kHz 控制中断的周期只有 50us ⇒ 实测 max=116us, 直接掉拍。
 * 现在中断里只 vsnprintf 到环形队列(2~6us), 由主循环 BSP_Log_DeferFlush()
 * 一次性写出。主循环里的日志仍直接写, 保证顺序不乱。
 * ⚠ 两个中断都可能是生产者(ADC=0 最高, TIM6=2), 用 busy 旗标互斥;
 *   撞上就丢掉那一条(日志丢一条无所谓, 控制掉拍才是要命的)。
 * ========================================================================== */
#define LOGQ_N     8U
#define LOGQ_LEN   128U

static char              s_logq[LOGQ_N][LOGQ_LEN];
static volatile uint8_t  s_logq_head = 0U;
static volatile uint8_t  s_logq_tail = 0U;
static volatile uint8_t  s_logq_busy = 0U;

/* ★ cal29: RTT 丢行计数 + 行缓冲(定义提前, 因为 BSP_Log_DeferFlush 也要用) */
#define RTT_LINE_MAX   384U

static char              s_rtt_line[RTT_LINE_MAX];
static unsigned          s_rtt_line_len = 0U;
static volatile uint32_t s_rtt_drop     = 0U;   /* 被 SKIP 掉的**行**数 */
static volatile uint32_t s_rtt_bad      = 0U;   /* ★ 2026-10-25 A2e: 乱码行(被丢弃)数 */

/* ---- 极简整数格式化(★ cal27, 只给中断路径用) ----------------------------
   cal26 发现: 把中断日志的 %.1f 换成 %dmA 之后 max 还是 108us —— 说明贵的不
   是浮点, 而是 **ARM 的 vsnprintf 本身**(通用格式引擎要 50~100us)。
   中断日志只用 %d/%u/%x/%X(带 0 填充宽度)、%c、%s, 所以这里自己写一个,
   约 3~6us。**不支持 %f**: 中断日志里出现 %f 会打成 "?f" 提醒你改掉。
   (%f 仍然可以在主循环日志里用 —— 那条路走 vprintf。) */

static int LogFmt_UInt(char *d, int cap, uint32_t v, uint32_t base,
                       uint8_t upper, int width, uint8_t zero)
{
    char tmp[12];
    int  n = 0, i;
    const char *dig = (upper != 0U) ? "0123456789ABCDEF" : "0123456789abcdef";

    do {
        tmp[n++] = dig[(unsigned)(v % base)];
        v /= base;
    } while ((v != 0U) && (n < 11));
    while ((n < width) && (n < 11)) {
        tmp[n++] = (zero != 0U) ? '0' : ' ';
    }
    if (n > (cap - 1)) { n = cap - 1; }
    for (i = 0; i < n; i++) {
        d[i] = tmp[n - 1 - i];
    }
    return n;
}

static int LogFmt(char *d, int cap, const char *fmt, va_list ap)
{
    int n = 0;

    if ((d == 0) || (cap <= 1)) { return 0; }

    while ((*fmt != '\0') && (n < (cap - 2))) {
        if (*fmt != '%') {
            d[n++] = *fmt++;
            continue;
        }
        fmt++;
        {
            uint8_t zero  = 0U;
            uint8_t lng   = 0U;
            int     width = 0;
            char    conv;
            int     w;

            if (*fmt == '0') { zero = 1U; fmt++; }
            while ((*fmt >= '0') && (*fmt <= '9')) {
                width = width * 10 + (int)(*fmt - '0');
                fmt++;
            }
            if (*fmt == 'l') { lng = 1U; fmt++; }
            conv = *fmt;
            if (conv == '\0') { break; }
            fmt++;

            switch (conv) {
            case '%':
                d[n++] = '%';
                break;
            case 'c':
                d[n++] = (char)va_arg(ap, int);
                break;
            case 's':
                {
                    const char *s = va_arg(ap, const char *);
                    if (s == 0) { s = "(null)"; }
                    while ((*s != '\0') && (n < (cap - 2))) { d[n++] = *s++; }
                }
                break;
            case 'd':
            case 'i':
                {
                    int32_t v = (lng != 0U) ? (int32_t)va_arg(ap, long)
                                            : (int32_t)va_arg(ap, int);
                    if (v < 0) {
                        d[n++] = '-';
                        v = -v;
                    }
                    w = LogFmt_UInt(&d[n], cap - n, (uint32_t)v, 10U, 0U, width, zero);
                    n += w;
                }
                break;
            case 'u':
                {
                    uint32_t v = (lng != 0U) ? (uint32_t)va_arg(ap, unsigned long)
                                             : (uint32_t)va_arg(ap, unsigned int);
                    w = LogFmt_UInt(&d[n], cap - n, v, 10U, 0U, width, zero);
                    n += w;
                }
                break;
            case 'x':
            case 'X':
                {
                    uint32_t v = (lng != 0U) ? (uint32_t)va_arg(ap, unsigned long)
                                             : (uint32_t)va_arg(ap, unsigned int);
                    w = LogFmt_UInt(&d[n], cap - n, v, 16U,
                                    (conv == 'X') ? 1U : 0U, width, zero);
                    n += w;
                }
                break;
            default:
                /* 不支持(尤其 %f) ⇒ 打标记提醒, 同时**吞掉**那个参数不现实,
                   所以这里只提示"中断日志里不能用这个转换"。 */
                d[n++] = '?';
                d[n++] = conv;
                break;
            }
        }
    }
    d[n] = '\0';
    return n;
}

int LOG_Printf(const char *fmt, ...)
{
    va_list ap;
    int     n;
    uint8_t in_isr = ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0U) ? 1U : 0U;

    if (fmt == 0) { return 0; }

    if (in_isr == 0U) {
        /* ★ cal27: 主循环走 vprintf → fputc → RTT, **不限行长、不截断**。
           cal26 用 96 字节的 vsnprintf 缓冲, 结果 [MAIN]/[PROF]/[ADCR] 这些
           200~300 字节的行全被砍掉一半, 日志里到处是半句拼接。 */
        va_start(ap, fmt);
        n = vprintf(fmt, ap);
        va_end(ap);
        return n;
    }

    /* ---- 中断上下文: 用极简整数格式化器入队 ---- */
    if (s_logq_busy != 0U) { return 0; }
    s_logq_busy = 1U;
    {
        uint8_t next = (uint8_t)((s_logq_head + 1U) & (LOGQ_N - 1U));
        if (next == s_logq_tail) {          /* 队列满 ⇒ 丢 */
            s_logq_busy = 0U;
            return 0;
        }
        va_start(ap, fmt);
        n = LogFmt(s_logq[s_logq_head], (int)LOGQ_LEN, fmt, ap);
        va_end(ap);
        if (n > 0) {
            s_logq_head = next;
        }
    }
    s_logq_busy = 0U;
    return n;
}

void BSP_Log_DeferFlush(void)
{
    /* ★ cal26: 每次最多写 2 条。
       原来一次把队列里(最多 8 条)全倒出去, 会把 RTT 上行缓冲瞬间打满 ⇒
       NO_BLOCK_SKIP 丢尾巴, 日志里大量出现半句拼接。主循环 10ms 一圈,
       2 条/圈 = 200 条/s, 足够且不会洪泛。 */
    uint8_t n = 2U;

    while ((n-- > 0U) && (s_logq_tail != s_logq_head)) {
        /* ★ cal29: 整串写入是原子的(SKIP 模式要么全进要么全丢),
           返回 0 就是这条被丢了 ⇒ 计数。 */
        if (SEGGER_RTT_WriteString(0, s_logq[s_logq_tail]) == 0U) {
            s_rtt_drop++;
        }
        s_logq_tail = (uint8_t)((s_logq_tail + 1U) & (LOGQ_N - 1U));
    }
}

uint32_t BSP_Log_GetRttDrop(void)
{
    return s_rtt_drop;
}

/* ★ 2026-10-25 A2e: 被判定为“乱码”而丢弃的行数(见 fputc 里的扫描) */
uint32_t BSP_Log_GetRttBad(void)
{
    return s_rtt_bad;
}

/* ==========================================================================
 * ★★★ cal29: fputc 改成“整行缓冲” —— 一行日志 = 一次 RTT 写入
 * --------------------------------------------------------------------------
 * 之前 fputc 是逐字节 SEGGER_RTT_PutChar, 而 AC5 的 vprintf 是**每个字符调
 * 一次 fputc**。⇒ 一行 200 字节的 [PROF] 就是 200 次独立的“缓冲够不够”判定,
 * 而 RTT 处在 NO_BLOCK_SKIP 模式(见 SEGGER_RTT.c 的 SKIP 分支):
 *
 *     Avail < NumBytes  ⇒  这次写入**整次丢弃**
 *
 * 于是缓冲将满时: 前 N 个字符已写进 ring, 中间一段被丢, J-Link 取走几个字节
 * 后 Avail 又 >0 ⇒ 后面的字符**又能写进去** ⇒
 * **一行中间出现一个“洞”**, 表现为:
 *     [PROF] win=1000ms | FO ???乱码??? C@20k load=52.11%
 * 这就是日志里那些“同长度前缀被换成乱码”的真正原因 ——
 * **不是覆盖**(SEGGER_RTT_PutChar 里的 `WrOff != pRing->RdOff` 保证了它从
 * 不覆盖未读数据), **是行内字节被丢**。
 *
 * 改后: 每个字符先攒进 s_rtt_line, 遇 '\n'(或超长)时**一次性**
 * SEGGER_RTT_Write(0, s_rtt_line, len) ⇒ 每行只有 1 次 SKIP 判定:
 *   · 要么整行完整进 buffer;
 *   · 要么整行一条都不留(**只丢整行, 绝不出现半行/洞**)。
 * 顺带把每行 200 次 BASEPRI 加/解锁砉成 1 次。
 *
 * ⚠ 不换掉 vprintf ⇒ %f / %lu 的链接行为完全不动, 零回归风险。
 * ⚠ 中断里万一有人 printf: 直接逐字节直写, 不碰 s_rtt_line
 *   (否则会踩花主循环攒到一半的那行)。
 * ========================================================================== */

int fputc(int ch, FILE *f)
{
    (void)f;

    /* 中断上下文: 不走行缓冲, 逐字节直写 */
    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0U) {
        (void)SEGGER_RTT_PutChar(0, (char)ch);
        return ch;
    }

    if ((s_rtt_line_len >= (RTT_LINE_MAX - 1U)) || (ch == '\n')) {
        unsigned len, i;
        uint8_t  bad = 0U;

        s_rtt_line[s_rtt_line_len++] = (char)ch;
        len = s_rtt_line_len;

        /* ★★ 2026-10-25 A2e: 整行健全性检查。
           日志里偶发出现 "€E33S@ / 33S@$" 这种乱码片段(两份日志各有 1~2 处)，
           而 `rttdrop` 始终为 0 ⇒ **不是整行被 SKIP**，而是行缓冲 s_rtt_line
           在写出前内容就被改掉了，或者根本是上位机(RTT Viewer)的显示问题。
           我们的日志全部是可打印 ASCII + \r\n\t，出现其它字节就判定为坏行：
             · 整行丢弃(宁可再生一行，不把乱码送给上位机)
             · 计数进 s_rtt_bad，并在 [PROF] 里打出 rttbad=N
           判读：
             · rttbad 跟着乱码一起涨  ⇒ 目标端内存/缓冲被踩（继续查）
             · rttbad 一直为 0 但仍有乱码 ⇒ 问题在上位机 / J-Link，与固件无关
           代价: 每行一次 ~300 字节扫描，100ms 一次，可以忽略。 */
        for (i = 0U; i < len; i++) {
            unsigned char c = (unsigned char)s_rtt_line[i];
            if ((c >= 0x80U) || (c == 0x7FU) ||
                ((c < 0x20U) && (c != (unsigned char)'\r') &&
                 (c != (unsigned char)'\n') && (c != (unsigned char)'\t'))) {
                bad = 1U;
                break;
            }
        }

        if (bad != 0U) {
            s_rtt_bad++;                        /* 乱码行: 丢弃 */
        } else if (SEGGER_RTT_Write(0, s_rtt_line, len) != len) {
            s_rtt_drop++;                       /* 整行被 SKIP */
        }
        s_rtt_line_len = 0U;
        return ch;
    }
    s_rtt_line[s_rtt_line_len++] = (char)ch;
    return ch;
}

/* 波形节拍定时器(TIM6): 只启动“更新中断”, 具体发什么由 App_OnWaveTick 决定 */
uint8_t BSP_WaveTimer_Start(void)
{
    return (HAL_TIM_Base_Start_IT(&CFG_WAVE_TIM_HANDLE) == HAL_OK) ? 1U : 0U;
}

/* ==========================================================================
 * HAL 中断回调 —— 只做"搬运 + 转调应用层钩子", 不含任何控制逻辑
 * ========================================================================== */

/* 相电流采样完成(每个 PWM 周期两次: ADC1、ADC2)。
   ADC1 先回(读 U/W 两相), ADC2 后回(读 V 相) ⇒ 在 ADC2 这一拍算完三相电流,
   再通知应用层跑 20kHz 控制环。 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == CFG_ADC_CUR_HANDLE_A.Instance) {
        s_adc_iu_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&CFG_ADC_CUR_HANDLE_A,
                                                            ADC_INJECTED_RANK_1);
        s_adc_iw_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&CFG_ADC_CUR_HANDLE_A,
                                                            ADC_INJECTED_RANK_2);
    } else if (hadc->Instance == CFG_ADC_CUR_HANDLE_B.Instance) {
        s_adc_iv_raw = (uint16_t)HAL_ADCEx_InjectedGetValue(&CFG_ADC_CUR_HANDLE_B,
                                                            ADC_INJECTED_RANK_1);
        /* ★ 中断负载率: 包住整个 20kHz 控制环(霍尔轮询 + FOC 位置/电流环) */
        {
            uint32_t t0 = DWT->CYCCNT;
            App_OnCurrentSample();          /* 三相都到齐了 ⇒ 跑控制环 */
            BSP_Prof_RecordIsr(DWT->CYCCNT - t0);
        }
    }
}

/* 霍尔边沿: 边沿检测已改成 20kHz 轮询(见 sensor_hall.c), EXTI 这条路径必须停用,
   否则同一个跳变会被计两次(扇区序号一次加 2 = 位置整体跑掉)。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    (void)GPIO_Pin;
}

/* TIM6(1kHz): 波形发送节拍 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == CFG_WAVE_TIM_HANDLE.Instance) {
        /* ★ 中断负载率: 包住 FireWater 组帧 + 启动 DMA */
        {
            uint32_t t0 = DWT->CYCCNT;
            App_OnWaveTick();
            BSP_Prof_RecordWave(DWT->CYCCNT - t0);
        }
    }
}

/* ==========================================================================
 * 系统时钟: 170MHz(HSE 8MHz → PLL N=85 / P=2), APB1/APB2 = 85MHz
 * ========================================================================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    uint32_t hse_retry;

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN       = 85;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR       = RCC_PLLR_DIV2;

    /* HSE 偶发起振失败(板上晶振/负载问题), 最多重试 10 次: 每次先关再开 */
    for (hse_retry = 0; hse_retry < 10U; hse_retry++) {
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK) {
            break;
        }
        __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
        HAL_Delay(5);
    }
    if (hse_retry >= 10U) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }

    /* ★★★ 2026-10-24 性能关键(削峰第一步): 打开 Flash 预取 + I-Cache + D-Cache
       ------------------------------------------------------------------
       HAL_RCC_ClockConfig 只写了 4 个等待周期(LATENCY), **没有**打开缓存。
       本工程 SystemClock_Config 是手写的(CubeMX 那种会带这三句), 于是一直以
       "4WS + 无缓存 + 无预取" 在 170MHz 跑: 每条取指都要等好几拍,
       常量表(sin/cos 512 点表, 2KB)每拍被读 4 次也全走 Flash。

       这和实测对得上: [PROF] 里 IDLE 时中断 4.1us(几乎不执行代码),
       一旦进位置环就 23~30us —— 差 6~7 倍, 而代码量并没有增加 6 倍。

       这三句是纯性能开关, 对程序完全透明, 不改任何控制逻辑/时序:
         · ICEN  = 指令缓存(1KB), 循环体/热点函数不再反复等 Flash
         · DCEN  = 数据缓存(1KB), sin/cos 表、PID 系数等常量读命中
         · PRFTEN= 预取缓冲
       回归方法: 把这三句注释掉就回到原状态。 */
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    __HAL_FLASH_DATA_CACHE_ENABLE();
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
}
