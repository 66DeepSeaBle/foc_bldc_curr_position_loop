/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;
/* TIM6：基本定时器，1kHz周期中断，专门用来定时触发VOFA波形发送，
   不占PWM通道。见 tim.c 里 MX_TIM6_Init() 的说明。 */
extern TIM_HandleTypeDef htim6;
/* TIM3：输入捕获，专用于霍尔边沿高精度时间戳(用户主导设计)。CH1/CH2/CH3
   分别接HALL_U/V/W(PC6/PC7/PC8，AF2)，双边沿捕获，1MHz计数(1µs分辨率)，
   跟原有EXTI触发的FOC_HallEdge()并存、不冲突——见 tim.c 里 MX_TIM3_Init()
   的说明。阶段1只做诊断旁路，不接入控制逻辑。 */
extern TIM_HandleTypeDef htim3;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM6_Init(void);
void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

