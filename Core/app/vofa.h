/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    vofa.h
  * @brief   VOFA+ 波形输出(**FireWater 文本协议**) —— 通过 USART2(PB3=TX /
  *          PB4=RX, DMA 发送)把电机的三相电流等变量发给电脑上的 VOFA+ 画波形。
  ******************************************************************************
  * @attention
  *
  * 协议(FireWater, 文本): 一帧 = 一行 ASCII, 通道间用 ',' 分隔, 行尾 '\r\n'。
  *      例: "0.123,-0.045,0.870,360.000\r\n"
  *   每个数字固定 3 位小数。VOFA+ 里选 "FireWater" 协议、串口选这个 COM 口、
  *   波特率对上、8/N/1 即可。
  *
  *   4 通道 ≈ 30 字节/帧, 1kHz ⇒ 30kB/s(300kbps), 921600 下占 ~33%。
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

/* 通道数(1~15)。当前 5 路: Iu, Iv, Iw, Spd(°/s), Pos(°)。 */
#define VOFA_CHANNELS      5U
/* 帧缓冲上限。5 路文本最坏 5×11+4+2 = 61 字节，96 留余量。 */
#define VOFA_FRAME_MAX     96U

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

/* 4 通道便捷入口(旧版用: Iu, Iv, Iw, Pos) */
void     VOFA_Send4(float c0, float c1, float c2, float c3);
/* 5 通道便捷入口(当前用: Iu, Iv, Iw, Spd(°/s), Pos(°)) */
void     VOFA_Send5(float c0, float c1, float c2, float c3, float c4);

/* 诊断计数: 已发帧数 / 丢弃帧数 */
uint32_t VOFA_GetFrameCount(void);
uint32_t VOFA_GetDropCount(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __VOFA_H__ */
