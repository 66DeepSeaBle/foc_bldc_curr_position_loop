/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "board_config.h"   /* 串口实例/波特率/引脚/DMA/中断号全部来自这里 */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USART2 init function
   PB3=USART2_TX / PB4=USART2_RX（AF7）。板上这个口直连USB转TTL模块到电脑，
   点对点全双工，不是RS485（原来接收发器时留的USART1/RS485切换逻辑已去掉）。
   注意：PB3/PB4 复位后默认是 JTAG(JTDO/NJTRST)，配成 AF7 即可覆盖，
   SWD 调试用的是 PA13/PA14，不受影响。
   TX 挂了DMA：VOFA波形帧靠 HAL_UART_Transmit_DMA 发送，主循环不用等
   传输完成，能更及时地跑速度环/读霍尔。
   ★ 本函数里的硬件常量全部来自 board_config.h 的 CFG_UART_*, 换串口只改那里。 */
void MX_USART2_UART_Init(void)
{
  CFG_UART_HANDLE.Instance = CFG_UART_INSTANCE;
  CFG_UART_HANDLE.Init.BaudRate = CFG_UART_BAUD;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==CFG_UART_INSTANCE)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

    /* 串口时钟源: 必须与实际使用的串口对应(换 USART1 时这里自动跟着变) */
#if (CFG_UART_NUM == 2)
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
#elif (CFG_UART_NUM == 1)
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
#else
#error "board_config.h: 这个串口实例还没在本文件里加时钟源分支"
#endif
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();   /* 注: gpio.c 已把 A/B/C/F 时钟全开了, 这里保留无害 */
    /**USART2 GPIO Configuration (引脚/复用号来自 board_config.h)
    PB3     ------> USART2_TX
    PB4     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = CFG_UART_TX_PIN|CFG_UART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = CFG_UART_AF;
    HAL_GPIO_Init(CFG_UART_GPIO_PORT, &GPIO_InitStruct);

    /* USART2_TX DMA Init */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_usart2_tx.Instance = CFG_UART_DMA_INSTANCE;
    hdma_usart2_tx.Init.Request = CFG_UART_DMA_REQUEST;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart2_tx);

    HAL_NVIC_SetPriority(CFG_UART_DMA_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CFG_UART_DMA_IRQn);

    /* USART2全局中断：DMA把最后一个字节搬进TDR之后，HAL会转去开
       USART自己的"发送完成"(TC)中断，等移位寄存器真正把最后一位推出去
       才把gState翻回READY——这个中断必须在NVIC里使能，否则gState永远
       卡在BUSY，第一帧发完之后 vofa.c 的 VOFA_Send 就会一直判断"没就绪"
       而跳过，串口看起来像是卡死/只收到一帧。 */
    HAL_NVIC_SetPriority(CFG_UART_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(CFG_UART_IRQn);

  /* USER CODE END USART2_MspInit 0 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PB3     ------> USART2_TX
    PB4     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3|GPIO_PIN_4);

    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);

  /* USER CODE END USART2_MspDeInit 0 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

