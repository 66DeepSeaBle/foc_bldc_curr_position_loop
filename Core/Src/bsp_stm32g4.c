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
 * ========================================================================== */int fputc(int ch, FILE *f)
{
    (void)f;
    SEGGER_RTT_PutChar(0, (char)ch);
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
        App_OnCurrentSample();          /* 三相都到齐了 ⇒ 跑控制环 */
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
        App_OnWaveTick();
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
}
