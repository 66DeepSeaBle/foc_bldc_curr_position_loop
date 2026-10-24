/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim3;

/* TIM6 init function
   基本定时器，1kHz周期中断，专门用来定时触发VOFA波形发送(Vofa_SendAvg)。
   原来这个调用挂在主循环里，跟其他任何一次可能变慢的主循环代码
   (比如每330ms一次的Vbus阻塞ADC轮询)排在一起执行，主循环一慢，
   发送节奏跟着抖动，VOFA上看着就是忽快忽慢的延迟感。
   换成独立定时器中断后，发送节奏只取决于这个1kHz中断本身，
   跟主循环在干什么完全无关，延迟更稳定、更低。
   170MHz(APB1定时器时钟) / 170 = 1MHz计数，再 /1000 = 1kHz。 */
void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 169;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM3 init function（用户主导设计：霍尔边沿高精度时间戳，见项目plan文件
   "霍尔边沿高精度时间戳：定时器输入捕获"一节的完整说明）。
   CH1/CH2/CH3输入捕获，双边沿(TIM_ICPOLARITY_BOTHEDGE)，接HALL_U/V/W
   (PC6/PC7/PC8，见HAL_TIM_IC_MspInit里的GPIO配置)。170MHz(APB1定时器
   时钟，跟已经在用的TIM6同一个时钟域)/170=1MHz计数，1µs分辨率，比现有
   基于foc_tick(20kHz中断计数)的50µs分辨率细50倍，且是硬件在电平跳变
   那一刻直接锁存CNT到CCRx，不受EXTI软件响应延迟/抖动影响。
   ARR=0xFFFF(16位定时器最大值)，1µs下约65.5ms回绕一次——这个阶段只做
   诊断旁路(见main.c里HAL_TIM_IC_CaptureCallback())，不接入控制逻辑，
   回绕处理留到阶段2按plan文件里的"精细dt只在安全范围内采信、否则回退
   现有粗时钟"这套设计做，不在这里处理。
   ↓↓↓ 关键设计：GPIO配置见HAL_TIM_IC_MspInit()——PC6/7/8已经在gpio.c
   MX_GPIO_Init()里配置过EXTI双边沿中断(GPIO_MODE_IT_RISING_FALLING)，
   这里初始化TIM3必须晚于MX_GPIO_Init()调用(main()里已经是这个顺序)，
   让EXTI先配置好，这里只追加AF复用功能(GPIO_MODE_AF_PP+AF2/TIM3)，
   不清除EXTI寄存器配置——STM32的EXTI是从管脚物理电平经SYSCFG直接引出、
   跟管脚是否处于AF复用模式无关(只要不是模拟模式)，两者天然可以并存，
   原有FOC_HallEdge()走的EXTI路径完全不受影响、行为不变。 */
void MX_TIM3_Init(void)
{
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 169;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity  = TIM_ICPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter    = 4;   /* 起点值，起消抖作用，跟真实抓取的边沿间隔数据比对后可以再调 */
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  /* ★ 中心对齐: f = 170MHz / (2 * (Period+1))
     20kHz ⇒ 4249 ; 15kHz ⇒ 5666 ; 16kHz ⇒ 5311 ; 25kHz ⇒ 3399
     16kHz: 170e6/(2*5312) = 16001.5Hz。**改频必须与 foc_types.h 的 FOC_CTRL_HZ 同步**。 */
  htim1.Init.Period = 4249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* ADC注入采样的触发源。
     原为 TRGO_UPDATE：中心对齐下更新事件在谷底和峰顶各发生一次 ——
     峰顶时三个下管全通（低边分流能测到电流），谷底时上管在通（测不到），
     于是一半采样是废的，这也是六步下电流波形很脏的原因之一。
     改为 OC4REF：由 CCR4 精确指定采样时刻，固定落在峰顶之后，
     此时三相下管均导通，可同时准确采到三相电流（FOC 的前提）。 */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF_RISING_OC6REF_FALLING;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  /* 必须为 LOW：IR2103 的 LIN 是反相输入（LO = NOT(LIN)），这是它区别于 IR2101 之处。
     用 HIGH 时 CH1N 在 OCREF 为低的 92% 时间里输出高 -> LIN 高 -> 下管关，
     结果上下管同时关断 92%，绕组无导通回路，实测无电流、无阻力，
     且 LO 平均电压恰为 12V×8%≈1V（与占空比相同），已实测确认。
     改为 LOW 后 CH1N 反相：LIN 低 92% -> 下管导通 92%，死区期间两管均关。 */
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* CH4 不接引脚，只用来产生 OC4REF 作为 ADC 触发。
     PWM1 模式下 OCREF 在 CNT<CCR 时为高，中心对齐下降计数经过 CCR4 时产生上升沿。
     取 4200（ARR=4249）使该上升沿落在峰顶之后不久：此刻凡占空比低于
     4200/4250≈98% 的相，上管都已关断、下管导通，三路分流电阻同时有电流。
     注意：占空比很高时下管导通窗口会缩短，需保证窗口长于ADC采样时间
     （47.5+12.5=60个ADC周期 ≈ 2.8us/通道）。 */
  /* ★★★ 2026-10-24 严重 bug 修正: CCR4 必须**跟着 ARR 走**。
     原来硬编码 4200 是 20kHz(ARR=4249) 专用值。改成 25kHz(ARR=3399) 后
     4200 > ARR ⇒ CCR4 永远匹配不到 ⇒ OC4REF 不再产生边沿 ⇒
     **ADC 注入中断彻底停摆**(实测: cntU/V/W=0, offU/V/W=0, 卡在 ALIGN)。
     改成 98.85% × ARR（= 20kHz 时正好 4199，与原来的 4200 等价）:
       · 仍落在峰顶之后（< ARR）
       · 仍大于最大占空比 CFG_PWM_MAX_DUTY(0.95) 对应的 CCR ⇒ 采样时上管已全关
     20kHz⇒4199  15kHz⇒5601  25kHz⇒3359 */
  sConfigOC.Pulse = (uint32_t)((float)htim1.Init.Period * 0.9885f);
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 85;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 1;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspInit 0 */

  /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM1_MspInit 1 */

  /* USER CODE END TIM1_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(timHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspPostInit 0 */

  /* USER CODE END TIM1_MspPostInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PC3     ------> TIM1_CH4
    PB13     ------> TIM1_CH1N
    PB14     ------> TIM1_CH2N
    PB15     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM1_MspPostInit 1 */

  /* USER CODE END TIM1_MspPostInit 1 */
  }

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM1)
  {
  /* USER CODE BEGIN TIM1_MspDeInit 0 */

  /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  /* USER CODE BEGIN TIM1_MspDeInit 1 */

  /* USER CODE END TIM1_MspDeInit 1 */
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM6)
  {
    /* TIM6 clock enable */
    __HAL_RCC_TIM6_CLK_ENABLE();

    /* TIM6 interrupt Init */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);   /* 比TIM1/DMA/USART优先级低，不抢控制环 */
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM6)
  {
    __HAL_RCC_TIM6_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM6_DAC_IRQn);
  }
}

/* HAL_TIM_IC_Init()内部会调用这个弱函数做外设相关的时钟/GPIO/NVIC初始化，
   风格跟上面HAL_TIM_PWM_MspInit/HAL_TIM_Base_MspInit一致(按Instance分发)。
   用户主导设计：见MX_TIM3_Init()声明处的完整说明。 */
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{
  if(tim_icHandle->Instance==TIM3)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC6/PC7/PC8(HALL_U/V/W)追加AF2/TIM3复用——这三个管脚已经在
       gpio.c的MX_GPIO_Init()里配置过GPIO_MODE_IT_RISING_FALLING(EXTI
       双边沿中断)，这里只重新配置MODER/AFR(切到AF模式)，不碰EXTI/
       SYSCFG寄存器(HAL_GPIO_Init只在Mode带EXTI标志位时才会碰那些
       寄存器，GPIO_MODE_AF_PP不带)，所以原有EXTI配置原样保留、继续
       生效，FOC_HallEdge()走的EXTI路径完全不受这次调用影响。上拉
       保持跟原配置一致(霍尔传感器开漏输出，需要上拉)。 */
    GPIO_InitStruct.Pin       = HALL_U_Pin | HALL_V_Pin | HALL_W_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);   /* 跟原EXTI9_5_IRQn同一优先级，纯诊断旁路不需要抢占控制环 */
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{
  if(tim_icHandle->Instance==TIM3)
  {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

