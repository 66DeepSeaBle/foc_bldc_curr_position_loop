/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vofa.h
  * @brief   VOFA+ 波形输出(JustFloat 协议) —— 通过 USART2(PB3=TX / PB4=RX,
  *          DMA 发送)把电机的三相电流等变量发给电脑上的 VOFA+ 画波形。
  ******************************************************************************
  * @attention
  *
  * 协议(JustFloat): 一帧 = N 个 float32(小端) + 4 字节帧尾 00 00 80 7F,
  * 那 4 个字节正好是 float 的 +inf, VOFA+ 用它来切帧, 所以帧内数据是什么
  * 都无所谓。VOFA+ 里选 "JustFloat" 协议、串口选这个 COM 口、波特率对上即可。
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VOFA_H__
#define __VOFA_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* 通道数(1~15)。一帧字节数 = VOFA_CHANNELS*4 + 4。 */
#define VOFA_CHANNELS      4U
/* 帧缓冲上限(足够 15 通道) */
#define VOFA_FRAME_MAX     64U

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* 初始化(必须在 MX_USART2_UART_Init() 之后调用)。默认打开输出。 */
void     VOFA_Init(void);

/* 波形输出开关(默认开)。关掉后 VOFA_Send 直接返回, 完全不碰串口。 */
void     VOFA_SetEnable(uint8_t en);
uint8_t  VOFA_GetEnable(void);

/* 把 n 个 float 组一帧 JustFloat 交给 DMA 发走。
   **非阻塞**: 上一帧还没发完时这一帧直接丢弃(drop +1), 绝不等待 ——
   所以可以(也只应该)从中断里调用。 */
void     VOFA_Send(const float *ch, uint8_t n);

/* 4 通道便捷入口(当前 main.c 用: Iu, Iv, Iw, Pos) */
void     VOFA_Send4(float c0, float c1, float c2, float c3);

/* 诊断计数: 已发帧数 / 丢弃帧数 */
uint32_t VOFA_GetFrameCount(void);
uint32_t VOFA_GetDropCount(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
