/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "SEGGER_RTT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define R_SHUNT       0.01f    // 采样电阻 0.01Ω
#define OP_GAIN        8        // PGA增益8（16会因偏置0.2V导致输出饱和，见opamp.c注释）
// ADC参考电压：VREF+(MCU pin28)接的是3V3_REF网络，实测只有2.6V（TL431因R52供流不足未进入稳压区）
#define ADC_VREF       2.6f
#define ADC_RESOLUTION 4096     // ADC分辨率12位
// 零电流偏置不再用固定常数，改为开机自动标定（见main中的[Calib]段），三相各自独立
#define CURRENT_SCALE  (R_SHUNT * OP_GAIN)  // 电流计算系数 = 0.08 V/A
#define VBUS_DIVIDER   13.4f    // 母线分压比: (R5 12.4K + R8 1K) / R8 1K
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc1_inj_val[2];  // ADC1注入通道值 [U相, W相]
uint16_t adc2_inj_val[1];  // ADC2注入通道值 [V相]
// 零电流偏置标定值（开机时采样得到），三相各自独立，补偿运放失调与电阻误差
uint16_t adc_offset_u = 0, adc_offset_v = 0, adc_offset_w = 0;
uint8_t  offset_calibrated = 0;  // 标定完成前不计算电流
float i_u = 0, i_v = 0, i_w = 0;  // 三相电流
float v_bus = 0;                   // 母线电压
uint8_t hall_u = 0, hall_v = 0, hall_w = 0;  // Hall状态
uint8_t hall_state = 0;
uint32_t test_step = 0;  // 测试步骤计数
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// printf重定向到RTT（通过调试口SWD输出）。改用MicroLIB后printf直接调用fputc，
// 不会再走标准库那套 _initio/freopen/_sys_open 半主机初始化流程，
// 避免了 standalone 运行时卡在 BKPT 半主机指令、以及 _sys_write 和库里 sys_io.o 符号冲突的问题
int fputc(int ch, FILE *f)
{
    SEGGER_RTT_PutChar(0, (char)ch);
    return ch;
}

// ADC注入转换完成中断回调
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 电流计算: I = (ADC读数 - 零电流标定值) × LSB电压 / (R_shunt × 增益)
    // 标定值由开机自动标定得到，三相独立，未标定完成前不计算
    if (hadc->Instance == ADC1)
    {
        // ADC1注入: Rank1=PA2(U相), Rank2=PB1(W相)
        adc1_inj_val[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        adc1_inj_val[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        if (offset_calibrated)
        {
            i_u = ((float)adc1_inj_val[0] - (float)adc_offset_u) * ADC_VREF / (ADC_RESOLUTION - 1) / CURRENT_SCALE;
            i_w = ((float)adc1_inj_val[1] - (float)adc_offset_w) * ADC_VREF / (ADC_RESOLUTION - 1) / CURRENT_SCALE;
        }
    }
    else if (hadc->Instance == ADC2)
    {
        // ADC2注入: Rank1=PA6(V相)
        adc2_inj_val[0] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        if (offset_calibrated)
        {
            i_v = ((float)adc2_inj_val[0] - (float)adc_offset_v) * ADC_VREF / (ADC_RESOLUTION - 1) / CURRENT_SCALE;
        }
    }
}

// Hall传感器外部中断回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    hall_u = HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);
    hall_v = HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);
    hall_w = HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
    hall_state = (hall_u << 2) | (hall_v << 1) | hall_w;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  /* USER CODE BEGIN 2 */

  // 启动内部运放 OPA1/OPA2/OPA3
  HAL_StatusTypeDef st_opa1 = HAL_OPAMP_Start(&hopamp1);
  HAL_StatusTypeDef st_opa2 = HAL_OPAMP_Start(&hopamp2);
  HAL_StatusTypeDef st_opa3 = HAL_OPAMP_Start(&hopamp3);

  // 启动ADC注入转换（TIM1 TRGO触发）
  HAL_StatusTypeDef st_adc1_inj = HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_StatusTypeDef st_adc2_inj = HAL_ADCEx_InjectedStart_IT(&hadc2);

  // 启动ADC规则转换（母线电压PA0）
  HAL_StatusTypeDef st_adc1_reg = HAL_ADC_Start(&hadc1);

  // 启动TIM1三相互补PWM
  HAL_StatusTypeDef st_pwm1 = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_StatusTypeDef st_pwm2 = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_StatusTypeDef st_pwm3 = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_StatusTypeDef st_pwmn1 = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_StatusTypeDef st_pwmn2 = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_StatusTypeDef st_pwmn3 = HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  // 不再只在开机时打印一次（容易被RTT非阻塞缓冲区覆盖导致错过），
  // 状态码放到下面 while(1) 循环里跟其他数据一起每200ms打印一次，随时连RTT都能看到

  // 初始占空比设为0（安全）
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  // RS485初始为接收模式
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);

  SEGGER_RTT_Init();  // 初始化RTT，printf通过调试口输出

  printf("\r\n\r\n===== BldcClamp FOC Board Test =====\r\n");
  printf("MCU: STM32G431RBT6, 170MHz\r\n");
  printf("PWM: TIM1 20kHz Center-Aligned, DeadTime=85\r\n");
  printf("ADC: Injected CH, TIM1_TRGO trigger\r\n");
  printf("OPA: PGA Mode, Gain=%d, VREF=%.2fV\r\n", OP_GAIN, ADC_VREF);
  printf("===================================\r\n\r\n");

  // ---- 零电流偏置自动标定 ----
  // 此时PWM占空比为0、电机未通电，流过采样电阻的电流必为零，
  // 采样若干次取平均即为各相的零点。三相独立标定以补偿运放失调和电阻误差。
  {
    const uint16_t CAL_SAMPLES = 128;
    uint32_t sum_u = 0, sum_v = 0, sum_w = 0;

    HAL_Delay(50);  // 等待运放和ADC稳定
    for (uint16_t i = 0; i < CAL_SAMPLES; i++)
    {
      sum_u += adc1_inj_val[0];
      sum_w += adc1_inj_val[1];
      sum_v += adc2_inj_val[0];
      HAL_Delay(1);
    }
    adc_offset_u = (uint16_t)(sum_u / CAL_SAMPLES);
    adc_offset_w = (uint16_t)(sum_w / CAL_SAMPLES);
    adc_offset_v = (uint16_t)(sum_v / CAL_SAMPLES);
    offset_calibrated = 1;

    printf("[Calib] Zero-current offset: U=%u V=%u W=%u  (%.3fV %.3fV %.3fV)\r\n",
           adc_offset_u, adc_offset_v, adc_offset_w,
           adc_offset_u * ADC_VREF / (ADC_RESOLUTION - 1),
           adc_offset_v * ADC_VREF / (ADC_RESOLUTION - 1),
           adc_offset_w * ADC_VREF / (ADC_RESOLUTION - 1));
    printf("[Calib] Sensitivity: %.1f mV/A, Range: +%.1fA / %.1fA\r\n\r\n",
           CURRENT_SCALE * 1000.0f,
           ((ADC_RESOLUTION - 1 - adc_offset_u) * ADC_VREF / (ADC_RESOLUTION - 1)) / CURRENT_SCALE,
           -(adc_offset_u * ADC_VREF / (ADC_RESOLUTION - 1)) / CURRENT_SCALE);
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 读取Hall传感器状态
    hall_u = HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);
    hall_v = HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);
    hall_w = HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
    hall_state = (hall_u << 2) | (hall_v << 1) | hall_w;

    // 读取母线电压（规则通道PA0）
    // 注意：ADC1规则通道是单次转换模式(ContinuousConvMode=DISABLE)，必须每次重新触发，
    // 否则 HAL_ADC_GetValue() 读到的一直是开机那一次转换的陈旧值。
    uint16_t bus_raw = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
      bus_raw = HAL_ADC_GetValue(&hadc1);
    }
    v_bus = ADC_VREF * (float)bus_raw / (ADC_RESOLUTION - 1) * VBUS_DIVIDER;

    // 打印所有测试数据
    printf("[%lu] Hall: U=%d V=%d W=%d St=%d | ", HAL_GetTick(), hall_u, hall_v, hall_w, hall_state);
    printf("Vbus=%.2fV | Iu=%+.3fA Iv=%+.3fA Iw=%+.3fA | ADC1=[%d,%d] ADC2=[%d] | TIM1_CNT=%lu | ",
           v_bus, i_u, i_v, i_w,
           adc1_inj_val[0], adc1_inj_val[1], adc2_inj_val[0],
           (unsigned long)__HAL_TIM_GET_COUNTER(&htim1));
    printf("OPA=%d,%d,%d INJ=%d,%d REG=%d PWM=%d,%d,%d,%d,%d,%d ADCst=0x%08lX,0x%08lX\r\n",
           st_opa1, st_opa2, st_opa3, st_adc1_inj, st_adc2_inj, st_adc1_reg,
           st_pwm1, st_pwm2, st_pwm3, st_pwmn1, st_pwmn2, st_pwmn3,
           (unsigned long)HAL_ADC_GetState(&hadc1), (unsigned long)HAL_ADC_GetState(&hadc2));

    // LED闪烁指示系统运行
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

    HAL_Delay(200);  // 200ms打印一次
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  uint32_t hse_retry;

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  // HSE 冷启动偶发起振失败（实测偶尔要重试2次才成功），失败后关闭HSE再重试，而不是直接卡死
  for (hse_retry = 0; hse_retry < 10; hse_retry++)
  {
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK)
    {
      break;
    }
    __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
    HAL_Delay(5);
  }
  if (hse_retry >= 10)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
