/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vofa.c
  * @brief   VOFA+ 波形输出(JustFloat 协议), USART2(PB3/PB4) + DMA 发送。
  ******************************************************************************
  * @attention
  *
  * 为什么用 DMA + 定时器中断:
  *   1kHz 的波形数据如果放在主循环里发, 会跟"每 330ms 一次的 Vbus 阻塞
  *   ADC 轮询""RTT 日志"这些慢活串在一起 ⇒ 发送节奏抖, VOFA 上看着就是
  *   忽快忽慢的延迟感; 放在 1kHz 的 TIM6 中断里, 节奏只取决于定时器本身。
  *   DMA 让 CPU 只做"组帧 + 启动传输"(几微秒), 字节搬运由 DMA 完成,
  *   1kHz × 20 字节 = 160kbps, 在 921600 下只占 ~17% 带宽。
  *
  * 数据源(见 main.c 的 HAL_TIM_PeriodElapsedCallback):
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

/* JustFloat 帧尾 = float 的 +inf(小端 00 00 80 7F), VOFA+ 靠它识别帧边界 */
static const uint8_t VOFA_TAIL[4] = { 0x00U, 0x00U, 0x80U, 0x7FU };

/* 发送缓冲。⚠ 必须住 .bss(不是 const/Flash), 且**只有**在 DMA 空闲时改写。 */
static uint8_t  s_frame[VOFA_FRAME_MAX];
static uint8_t  s_enable = 1U;
static uint32_t s_frames = 0U;
static uint32_t s_drops  = 0U;

/* float → 小端 4 字节。Cortex-M4 本身就是小端, 这里显式逐字节写,
   免得依赖编译器/对齐规则; union 避免浮点格式化(%f)那套慢代码。 */
static void Vofa_PutFloatLE(uint8_t *p, float x)
{
    union { float f; uint32_t u; } cvt;
    cvt.f = x;
    p[0] = (uint8_t)(cvt.u);
    p[1] = (uint8_t)(cvt.u >>  8);
    p[2] = (uint8_t)(cvt.u >> 16);
    p[3] = (uint8_t)(cvt.u >> 24);
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
    uint16_t len;

    if ((s_enable == 0U) || (ch == 0)) {
        return;
    }
    if ((n == 0U) || ((uint16_t)n > ((VOFA_FRAME_MAX - 4U) / 4U))) {
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

    for (i = 0U; i < n; i++) {
        Vofa_PutFloatLE(&s_frame[(uint16_t)i * 4U], ch[i]);
    }
    len = (uint16_t)((uint16_t)n * 4U);
    s_frame[len + 0U] = VOFA_TAIL[0];
    s_frame[len + 1U] = VOFA_TAIL[1];
    s_frame[len + 2U] = VOFA_TAIL[2];
    s_frame[len + 3U] = VOFA_TAIL[3];
    len = (uint16_t)(len + 4U);

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

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
