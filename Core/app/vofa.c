/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vofa.c
  * @brief   VOFA+ 波形输出(**FireWater 文本协议**), USART2(PB3/PB4) + DMA 发送。
  ******************************************************************************
  * @attention
  *
  * 协议(FireWater): 一帧 = 文本一行, 通道用 ',' 分隔, 行尾 '\r\n'。
  *   例:  "0.123,-0.045,0.870,360.000\r\n"
  *   VOFA+ 里选 "FireWater" 协议、串口选这个 COM 口、波特率对上即可。
  *   ⚠ 之前是 JustFloat(二进制 + 00 00 80 7F 帧尾), 如果上位机还按 JustFloat
  *     解析, 现在会看到一堆乱码 —— 记得把协议切到 FireWater。
  *
  * 为什么用 DMA + 定时器中断:
  *   1kHz 的波形数据如果放在主循环里发, 会跟"每 330ms 一次的 Vbus 阻塞
  *   ADC 轮询""RTT 日志"这些慢活串在一起 ⇒ 发送节奏抖, VOFA 上看着就是
  *   忽快忽慢的延迟感; 放在 1kHz 的 TIM6 中断里, 节奏只取决于定时器本身。
  *   文本比二进制长: 4 通道 ≈ 30 字节/帧 ⇒ 1kHz = 30kB/s = 300kbps,
  *   在 921600 下占 ~33%(二进制 JustFloat 只要 160kbps)。
  *
  * 数据源(见 bsp_stm32g4.c 的 HAL_TIM_PeriodElapsedCallback):
  *   ch0/ch1/ch2 = g_foc.current.u/v/w  (三相电流, A, 20kHz 在 ADC 中断里刷新)
  *   ch3         = 机械位置 Pos (deg, 多圈)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "vofa.h"
#include "bsp.h"        /* 串口发送走 BSP, 不直接碰 HAL/句柄(换串口/换芯片不影响本文件) */

/* USER CODE BEGIN 0 */

/* 发送缓冲。⚠ 必须住 .bss(不是 const/Flash), 且**只有**在 DMA 空闲时改写。
   文本模式最坏一帧 = 4×(符号+6位整数+小数点+3位小数) + 3个逗号 + CRLF
                   = 4×11 + 3 + 2 = 49 字节, 64 留了充足余量。 */
static uint8_t  s_frame[VOFA_FRAME_MAX];
static uint8_t  s_enable = 1U;
static uint32_t s_frames = 0U;
static uint32_t s_drops  = 0U;

/* ★ FireWater 文本格式化器 —— 特意**不用 sprintf("%.3f")**:
   ① 1kHz 中断里 ARM 的浮点 printf 要 15~25µs, 而且会把半个 C 库拖进镜像;
   ② 更关键: printf 系列不是重入安全的。主循环的 LOGD 正在 printf 时,
      1kHz 中断插进来再 sprintf, 库内部状态会被改写 ⇒ 偶发乱码/卡死。
      这里用纯整数运算(乘 1000 + 四舍五入), 约 1~2µs, 无副作用。 */

/* 无符号整数 → 十进制, 返回写入字节数 */
static uint8_t Vofa_PutU32(uint8_t *p, uint32_t v)
{
    uint8_t buf[10];
    uint8_t n = 0U;
    uint8_t i;
    do {
        buf[n++] = (uint8_t)('0' + (uint8_t)(v % 10U));
        v /= 10U;
    } while (v != 0U);
    for (i = 0U; i < n; i++) {
        p[i] = buf[n - 1U - i];
    }
    return n;
}

/* float → "-1234.567" 文本(3 位小数, 四舍五入), 返回写入字节数(≤11) */
static uint8_t Vofa_PutFix3(uint8_t *p, float x)
{
    uint8_t  n = 0U;
    uint32_t mag, ip, fp;

    if (x != x) { x = 0.0f; }                 /* NaN → 0 */
    if (x >  999999.0f) { x =  999999.0f; }   /* 同时挡住 +Inf */
    if (x < -999999.0f) { x = -999999.0f; }   /* 同时挡住 -Inf */
    if (x < 0.0f) {
        p[n++] = (uint8_t)'-';
        x = -x;
    }
    mag = (uint32_t)(x * 1000.0f + 0.5f);     /* 千分位整数(已四舍五入) */
    ip  = mag / 1000U;
    fp  = mag % 1000U;

    n += Vofa_PutU32(&p[n], ip);
    p[n++] = (uint8_t)'.';
    p[n++] = (uint8_t)('0' + (uint8_t)(fp / 100U));
    p[n++] = (uint8_t)('0' + (uint8_t)((fp / 10U) % 10U));
    p[n++] = (uint8_t)('0' + (uint8_t)(fp % 10U));
    return n;
}

/* USER CODE END 0 */

void VOFA_Init(void)
{
    s_enable = 1U;
    s_frames = 0U;
    s_drops  = 0U;
}

void VOFA_SetEnable(uint8_t en)
{
    s_enable = (en != 0U) ? 1U : 0U;
}

uint8_t VOFA_GetEnable(void)
{
    return s_enable;
}

uint32_t VOFA_GetFrameCount(void)
{
    return s_frames;
}

uint32_t VOFA_GetDropCount(void)
{
    return s_drops;
}

void VOFA_Send(const float *ch, uint8_t n)
{
    uint8_t  i;
    uint8_t  w;
    uint16_t len = 0U;

    if ((s_enable == 0U) || (ch == 0)) {
        return;
    }
    if ((n == 0U) || (n > VOFA_CHANNELS)) {
        return;
    }
    /* ⚠ 必须在**写缓冲之前**判断上一帧有没有发完:
       DMA 还在从 s_frame 往外搬数据时改写它, 会把正在发的帧搞成半新半旧
       (VOFA 上看到的就是乱跳的毛刺)。上一帧没发完就丢掉这一帧 ——
       波形少一个点无所谓, 数据错乱才是要命的。 */
    if (!BSP_UART_IsTxReady()) {
        s_drops++;
        return;
    }

    /* FireWater: ch0,ch1,...,chN\r\n */
    for (i = 0U; i < n; i++) {
        if ((uint16_t)(len + 12U) > VOFA_FRAME_MAX) {   /* 预留 11 字节 + 1 个逗号 */
            s_drops++;
            return;
        }
        if (i != 0U) {
            s_frame[len++] = (uint8_t)',';
        }
        w = Vofa_PutFix3(&s_frame[len], ch[i]);
        len = (uint16_t)(len + w);
    }
    s_frame[len++] = (uint8_t)'\r';
    s_frame[len++] = (uint8_t)'\n';

    if (BSP_UART_TxAsync(s_frame, len)) {
        s_frames++;
    } else {
        s_drops++;
    }
}

void VOFA_Send4(float c0, float c1, float c2, float c3)
{
    float v[4];
    v[0] = c0;
    v[1] = c1;
    v[2] = c2;
    v[3] = c3;
    VOFA_Send(v, 4U);
}

void VOFA_Send5(float c0, float c1, float c2, float c3, float c4)
{
    float v[5];
    v[0] = c0;
    v[1] = c1;
    v[2] = c2;
    v[3] = c3;
    v[4] = c4;
    VOFA_Send(v, 5U);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
