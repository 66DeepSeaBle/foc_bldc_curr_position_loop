/**
  * @file    main.c
  * @brief   ???:??? + ?????
  */
#include "main.h"
#include "adc.h"
#include "opamp.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "foc_control.h"
#include <stdio.h>
#include <math.h>
#include "SEGGER_RTT.h"
#include "log.h"

/* ---- ??FOC???? ---- */
static FOC_Control_t g_foc;
static uint8_t motor_pwm_off_logged = 0;

/* ---- ????????? (RTT) ---- */
#define ANGLE_LOG_MS   10U          /* 角度采样打印周期 (ms) */
static uint32_t last_angle_print = 0;
static uint32_t g_log_seq = 0;

/* 20kHz FOC 控制环门控: 零电流标定完成前不运行, 否则 ISR 里的 IDLE 会
   立刻关断 PWM, 且用未标定(全0)偏置算出的电流会误触发过流保护. */
static volatile uint8_t foc_loop_en = 0;

/* ---- 手动盘轴霍尔验证模式 ---- */
static volatile uint8_t hall_verify_mode = 0;   /* 1=验证中(电机不通电) */
/* 相序邻接表(与 angle_observer.c 一致): H_FWD = 代码判定的正向下一扇区 */
static const uint8_t H_FWD[8] = { 0, 3, 6, 2, 5, 1, 4, 0 };
static const uint8_t H_BWD[8] = { 0, 5, 3, 1, 6, 4, 2, 0 };

typedef struct {
    uint8_t  prev_st;      /* 上一次霍尔扇区 */
    uint32_t t_cnt;        /* 总跳变次数 */
    uint32_t fwd_cnt;      /* 落在正向邻格的次数 */
    uint32_t bwd_cnt;      /* 落在反向邻格的次数 */
    uint32_t bad_cnt;      /* 0/7 或跨格/丢步次数 */
    float    total_deg;    /* 按判定方向累计电角度(deg) */
    uint32_t last_live;    /* 上次实时行打印时间 */
} HallVerify_t;
static HallVerify_t g_hv;

/* ---- ??????? ---- */
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

  // HSE ?????????(???????2????),?????HSE???,???????
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

/**
  * @brief  Apply three-phase PWM duty cycles to TIM1.
  * @param  du Phase U duty cycle, normalized to 0.0 ... 1.0.
  * @param  dv Phase V duty cycle, normalized to 0.0 ... 1.0.
  * @param  dw Phase W duty cycle, normalized to 0.0 ... 1.0.
  * @note   The duty cycle is limited to 95%% to preserve switching margin.
  * @note   Both high-side and low-side outputs are enabled after updating CCR1-CCR3.
  */
void MotorSetDuty3(float du, float dv, float dw)
{
    /* TIM1->ARR is the timer period value; CCR uses the corresponding count range. */
    uint32_t arr1 = TIM1->ARR + 1;

    /* Keep all phase commands inside the safe output range. */
    if (du < 0.0f) du = 0.0f;
    if (du > 0.95f) du = 0.95f;
    if (dv < 0.0f) dv = 0.0f;
    if (dv > 0.95f) dv = 0.95f;
    if (dw < 0.0f) dw = 0.0f;
    if (dw > 0.95f) dw = 0.95f;

    /* Convert normalized duty cycles to TIM1 compare values. */
    TIM1->CCR1 = (uint32_t)(du * (float)arr1);
    TIM1->CCR2 = (uint32_t)(dv * (float)arr1);
    TIM1->CCR3 = (uint32_t)(dw * (float)arr1);

    /* Enable both complementary outputs for all three motor phases. */
    TIM1->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC1NE |
                   TIM_CCER_CC2E | TIM_CCER_CC2NE |
                   TIM_CCER_CC3E | TIM_CCER_CC3NE);

    /* Allow the next MotorAllOff() call to report a new shutdown event. */
    motor_pwm_off_logged = 0;
}

/**
  * @brief  Disable all three motor PWM phases.
  * @note   The shutdown log is emitted once per PWM-off event to avoid
  *         flooding the log when the fault state is repeatedly serviced.
  */
void MotorAllOff(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E | TIM_CCER_CC3NE);

    if (!motor_pwm_off_logged) {
      LOGD("[PWM] MotorAllOff\r\n");
      motor_pwm_off_logged = 1;
    }
}

/* ---- ?????FOC?? ---- */

/* ??????? */
static uint16_t adc1_inj_val[2];
static uint16_t adc2_inj_val[1];
static uint16_t adc_offset_u = 0, adc_offset_v = 0, adc_offset_w = 0;
static uint8_t offset_calibrated = 0;

/**
  * @brief  ADC??????????
  * @note   20kHz,??????????FOC???
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    const float ADC_VREF = 3.3f;
    const uint16_t ADC_RESOLUTION = 4096;
    const float R_SHUNT = 0.01f;
    const float OP_GAIN = 8.0f;
    const float CURRENT_SCALE = R_SHUNT * OP_GAIN;
    const float CURRENT_SIGN = -1.0f;
    float i_u, i_v;

    if (hadc->Instance == ADC1) {
        adc1_inj_val[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        adc1_inj_val[1] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    } else if (hadc->Instance == ADC2) {
        adc2_inj_val[0] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);

        if (offset_calibrated) {
            i_u = CURRENT_SIGN * ((float)adc1_inj_val[0] - (float)adc_offset_u) *
                  ADC_VREF / (ADC_RESOLUTION - 1) / CURRENT_SCALE;
            i_v = CURRENT_SIGN * ((float)adc2_inj_val[0] - (float)adc_offset_v) *
                  ADC_VREF / (ADC_RESOLUTION - 1) / CURRENT_SCALE;

            g_foc.current.u = i_u;
            g_foc.current.v = i_v;
            g_foc.current.w = -(i_u + i_v);
        }

        /* ??FOC??? (20kHz) - 零电流标定完成前不运行 */
        if (foc_loop_en) {
            float dt_s = 1.0f / 20000.0f;
            FOC_Control_Run(&g_foc, dt_s);
        }
    }
}

/**
  * @brief  ????????
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint8_t u, v, w;

    (void)GPIO_Pin;

    u = (uint8_t)HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);
    v = (uint8_t)HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);
    w = (uint8_t)HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);

    g_foc.hall.u = u;
    g_foc.hall.v = v;
    g_foc.hall.w = w;
    g_foc.hall.state = (uint8_t)((~((u << 2) | (v << 1) | w)) & 0x07);

    /* ??????? */
    AngleObserver_OnHallEdge(&g_foc.angle, &g_foc.hall,
                             g_foc.tick,
                             g_foc.config.pwm_freq,
                             4U, 3000.0f, 1.5e-5f);
}

/* ---- printf??? ---- */
int fputc(int ch, FILE *f)
{
    (void)f;
    SEGGER_RTT_PutChar(0, (char)ch);
    return ch;
}

/* ---- ??? ---- */
/* =====================================================================
 * 手动盘轴霍尔验证模式 (纯手动, 电机不通电)
 * ---------------------------------------------------------------------
 * 进入: 开机 5s 内或运行中按 'c'/'C';  退出: 'q'/'Q' (重新 ALIGN).
 * 作用: 不加电, 用手盘动转子, 检查霍尔:
 *   1) U V W 三相与合成扇区 st(1~6) 是否正常 (正常盘一整圈不应出现 0/7);
 *   2) 每次跳变是否落在合法相序表上 (bad_cnt 应保持 0);
 *   3) 电角度方向: elec = 当前代码约定电角度; rev = 反转约定后的镜像角.
 *      若按"期望正转"的方向盘轴时 elec 递减而 rev 递增, 说明霍尔相序与
 *      SVPWM 相反(方向反了), 需把 config.hall_dir_rev 置 1 处理.
 *      total_deg 为按判定方向累计电角度: 转完整一圈机械角 /360 约等于
 *      极对数×6, 可用来核对极对数与霍尔是否覆盖所有磁极.
 * ===================================================================== */
static uint8_t HallVerify_ReadState(void)
{
    uint8_t u = (uint8_t)HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);
    uint8_t v = (uint8_t)HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);
    uint8_t w = (uint8_t)HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
    return (uint8_t)((~((u << 2) | (v << 1) | w)) & 0x07);
}

static void HallVerify_Enter(void)
{
    /* 关断驱动, 确保盘轴时电机不通电 */
    g_foc.flags.fault   = 0;
    g_foc.flags.enabled = 0;
    g_foc.fault_code    = 0;
    g_foc.state         = FOC_STATE_IDLE;
    PWM_Output_Disable();
    hall_verify_mode    = 1;

    g_hv.prev_st   = HallVerify_ReadState();
    g_hv.t_cnt     = 0;
    g_hv.fwd_cnt   = 0;
    g_hv.bwd_cnt   = 0;
    g_hv.bad_cnt   = 0;
    g_hv.total_deg = 0.0f;
    g_hv.last_live = 0;

    LOGI("\r\n[HALL] ==== Manual hall-spin verify ====\r\n");
    LOGI("[HALL] Drive disabled. Spin the rotor slowly & evenly by hand.\r\n");
    LOGI("[HALL] Watch st=1..6 sequence and U V W; no 0/7 expected.\r\n");
    LOGI("[HALL] elec = angle, current code convention (deg, 0..360)\r\n");
    LOGI("[HALL] rev  = mirrored angle if hall order were reversed\r\n");
    LOGI("[HALL] If spinning the intended-forward way makes elec go DOWN\r\n");
    LOGI("[HALL] (while rev goes UP), hall order is REVERSED -> flip\r\n");
    LOGI("[HALL] config.hall_dir_rev (0<->1) in main.c, then rebuild.\r\n");
    LOGI("[HALL] Press 'q' to exit/re-align.\r\n");
}

static void HallVerify_Exit(void)
{
    hall_verify_mode = 0;
    g_foc.flags.fault   = 0;
    g_foc.flags.enabled = 1;
    FOC_Control_SetState(&g_foc, FOC_STATE_ALIGN);
    LOGI("[HALL] Verify exit -> ALIGN again.\r\n");
}

static void HallVerify_Poll(uint32_t now)
{
    static const float R2D = 57.29577951f;
    uint8_t ru, rv, rw, st, p;
    float elec, rev, erpm = 0.0f;

    ru = (uint8_t)HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);
    rv = (uint8_t)HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);
    rw = (uint8_t)HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
    st = (uint8_t)((~((ru << 2) | (rv << 1) | rw)) & 0x07);

    /* 观测电角度: 与正式运行共用同一套 AngleObserver (蠕动已抑制) */
    elec = AngleObserver_GetTheta(&g_foc.angle, &g_foc.hall,
                                  g_foc.tick, g_foc.config.pwm_freq,
                                  20000U, 1.0f, 1, &erpm) * R2D;
    rev = 360.0f - elec;                 /* 反转约定后的镜像角 */
    if (rev >= 360.0f) rev -= 360.0f;
    if (rev < 0.0f)    rev += 360.0f;

    /* 跳变检测 */
    if (st != g_hv.prev_st) {
        p = g_hv.prev_st;
        g_hv.t_cnt++;
        if (st == 0 || st == 7 || p == 0 || p == 7) {
            g_hv.bad_cnt++;
            LOGI("[HALL] #%u U%uV%uW%u st=%u  BAD(0/7)\r\n",
                 g_hv.t_cnt, ru, rv, rw, st);
        } else if (st == H_FWD[p]) {
            g_hv.fwd_cnt++;
            g_hv.total_deg += 60.0f;
            LOGI("[HALL] #%u U%uV%uW%u st=%u  %u->%u FWD elec=%.1f\r\n",
                 g_hv.t_cnt, ru, rv, rw, st, p, st, elec);
        } else if (st == H_BWD[p]) {
            g_hv.bwd_cnt++;
            g_hv.total_deg -= 60.0f;
            LOGI("[HALL] #%u U%uV%uW%u st=%u  %u->%u BWD elec=%.1f\r\n",
                 g_hv.t_cnt, ru, rv, rw, st, p, st, elec);
        } else {
            g_hv.bad_cnt++;
            LOGI("[HALL] #%u U%uV%uW%u st=%u  %u->%u GAP(bad)\r\n",
                 g_hv.t_cnt, ru, rv, rw, st, p, st);
        }
        g_hv.prev_st = st;
    }

    /* 周期实时行 */
    if ((now - g_hv.last_live) >= 250) {
        g_hv.last_live = now;
        LOGI("[HALL] U%uV%uW%u st=%u syn=%d elec=%.1f rev=%.1f fwd=%u bwd=%u bad=%u total=%.0fdeg\r\n",
             ru, rv, rw, st, (int)g_foc.angle.synced, elec, rev,
             g_hv.fwd_cnt, g_hv.bwd_cnt, g_hv.bad_cnt, g_hv.total_deg);
    }
}

int main(void)
{
    FOC_Config_t config;
    uint32_t last_print = 0;

    HAL_Init();
    SystemClock_Config();

    /* ????? */
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

    SEGGER_RTT_Init();

    LOGD("\r\n===== FOC Control System =====\r\n");
    LOGD("MCU: STM32G431RBT6, 170MHz\r\n");
    LOGD("PWM: 20kHz Center-Aligned\r\n");
    LOGD("[FW] build 2026-09-12 spd-decay ref-scale\r\n");

    /* ---- ??FOC ---- */
    config.pwm_freq = 20000.0f;
    config.vref = 3.3f;
    config.shunt_r = 0.01f;
    config.op_gain = 8.0f;
    config.current_scale = 0.01f * 8.0f;
    config.current_sign = -1.0f;
    config.vbus_divider = 13.4f;
    config.adc_resolution = 4096;
    config.pole_pairs = 26;
    /* 霍尔方向: 实测(逐组切换验证) 只有 dir_rev=1 + torque_dir=-1 一组
       能“不振动、稳定停住”，其余组合要么剧烈振动要么跑飞，故默认 1.
       运行时可用 'd' 键翻转(会与 torque_dir 组合成另一类符号). */
    config.hall_dir_rev = 1;
    config.hall_offset_deg = 0.0f;
    config.overcurrent_amp = 6.0f;
    config.align_current = 2.0f;
    config.startup_hz = 6.0f;
    config.iq_ramp_rate = 2.0f;

    /* ???FOC?? */
    FOC_Control_Init(&g_foc, &config);

    /* ---- ??PWM ---- */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    /* ??ADC */
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);
    HAL_ADC_Start(&hadc1);

    /* ???? */
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    /* ---- 零电流偏置标定 ----
       必须放在 PWM/ADC注入/运放 都启动之后(旧代码放在启动前, 采样到的
       adc1/2_inj_val 全是 0, 偏置=0, 一跑 FOC 就误报过流).
       标定时 0% 占空比 = 三个下管全开, 把采样电阻下端接到 GND, 得到
       确定的 0A 状态; 运放输出即为零电流偏置(约 ADC 中值). */
    {
        const uint16_t CAL_SAMPLES = 128;
        uint16_t val_u, val_v, val_w;
        uint32_t sum_u = 0, sum_v = 0, sum_w = 0;

        MotorSetDuty3(0.0f, 0.0f, 0.0f);   /* 下管制动: 0A 基准 */
        HAL_Delay(50);                     /* 等运放/采样稳定 */
        for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
            val_u = adc1_inj_val[0];
            val_w = adc1_inj_val[1];
            val_v = adc2_inj_val[0];
            sum_u += val_u;
            sum_v += val_v;
            sum_w += val_w;
            HAL_Delay(1);
        }
        adc_offset_u = (uint16_t)(sum_u / CAL_SAMPLES);
        adc_offset_v = (uint16_t)(sum_v / CAL_SAMPLES);
        adc_offset_w = (uint16_t)(sum_w / CAL_SAMPLES);
        offset_calibrated = 1;

        LOGD("[Calib] Offset: U=%u V=%u W=%u (zero-current)\r\n",
             adc_offset_u, adc_offset_v, adc_offset_w);
    }

    /* 标定完成, 放行 20kHz FOC 控制环 */
    foc_loop_en = 1;

    /* ??FOC: 先停在 IDLE. 开机 5s 内按 'c' 进入手动盘轴霍尔验证
       (不加电), 或按 's' 立即开始; 超时则自动 ALIGN->STARTUP */
    g_foc.flags.enabled = 0;
    FOC_Control_SetState(&g_foc, FOC_STATE_IDLE);
    LOGD("[MAIN] FOC ready (IDLE). dir_rev=%d torque_dir=%d. 'c'=manual hall verify, 's'=start now\r\n",
         (int)g_foc.config.hall_dir_rev, (int)g_foc.torque_dir);
    LOGD("[MAIN] Keys: j=+Iq pulse  k=-Iq pulse  d=flip dir  t=flip torque\r\n");
    LOGD("[MAIN] Keys: z=target 0  0~9=target digit*45deg\r\n");
    LOGD("[ANG] idx,state,hall,synced,elec_deg,mech_deg,erpm\r\n");
    (void)g_log_seq;    /* [ANG] 已降为 LOGI(默认不打印), 序号不再被引用 */

    {
        uint32_t boot_wait = HAL_GetTick();
        uint8_t  start_now = 0;
        while (!hall_verify_mode && !start_now &&
               (HAL_GetTick() - boot_wait) < 5000U) {
            int ch;
            while ((ch = SEGGER_RTT_GetKey()) >= 0) {
                if (ch == 'c' || ch == 'C') HallVerify_Enter();
                else if (ch == 's' || ch == 'S') start_now = 1;
            }
            HAL_Delay(10);
        }
    }

    if (!hall_verify_mode) {
        g_foc.flags.enabled = 1;
        FOC_Control_SetState(&g_foc, FOC_STATE_ALIGN);
        LOGD("[MAIN] FOC started, state=ALIGN\r\n");
    }

    /* ---- ???: ??????? ---- */
    while (1) {
        uint32_t now = HAL_GetTick();
        uint32_t print_ms;
        int ch;

        /* RTT 键盘 */
        while ((ch = SEGGER_RTT_GetKey()) >= 0) {
            if (hall_verify_mode) {
                if (ch == 'q' || ch == 'Q') HallVerify_Exit();
            } else {
                if (ch == 'c' || ch == 'C') {
                    HallVerify_Enter();
                } else if (ch == 'd' || ch == 'D') {
                    /* 运行时翻转霍尔方向(dir_rev), 用于判断闭环换相是否反向 */
                    g_foc.angle.hall_dir_rev = g_foc.angle.hall_dir_rev ? 0 : 1;
                    g_foc.config.hall_dir_rev = g_foc.angle.hall_dir_rev;
                    LOGD("[CMD] dir_rev -> %d (torque=%d)\n",
                         (int)g_foc.angle.hall_dir_rev, (int)g_foc.torque_dir);
                } else if (ch == 't' || ch == 'T') {
                    /* 运行时翻转力矩方向(torque_dir) */
                    g_foc.torque_dir = (int8_t)-g_foc.torque_dir;
                    LOGD("[CMD] torque_dir -> %d (dir=%d)\n",
                         (int)g_foc.torque_dir, (int)g_foc.angle.hall_dir_rev);
                } else if (ch == 'z' || ch == 'Z') {
                    FOC_Control_SetPosTarget(&g_foc, 0.0f);
                    LOGD("[CMD] Zero position\n");
                } else if (ch == 'j' || ch == 'J') {
                    /* 开环力矩探针: +Iq 脉冲, 测机械方向 */
                    FOC_Control_SetTorqueProbe(&g_foc, +0.7f, 500);
                } else if (ch == 'k' || ch == 'K') {
                    /* 开环力矩探针: -Iq 脉冲, 测机械方向 */
                    FOC_Control_SetTorqueProbe(&g_foc, -0.7f, 500);
                } else if (ch >= '0' && ch <= '9') {
                    /* ????:????????? */
                    float target = (float)(ch - '0') * 45.0f;
                    FOC_Control_SetPosTarget(&g_foc, target);
                    LOGD("[CMD] Set target %.1fdeg\n", target);
                }
            }
        }

        /* 手动盘轴验证: 高频采样(2ms), 不打常规状态行 */
        if (hall_verify_mode) {
            HallVerify_Poll(now);
            HAL_Delay(2);
            continue;
        }

        /* 日志节流: 到位/静止时 500ms 一条; 正在运动(pos_err 超差)时 100ms
           一条, 便于看清"动一下停一下"这类容差附近的行为, 又不会刷屏. */
        print_ms = (fabsf(g_foc.position_loop.pos_err_deg) >=
                    g_foc.position_loop.arrive_tol_deg) ? 100U : 500U;
        if ((now - last_print) >= print_ms) {
            last_print = now;
            LOGD("[MAIN] st=%d flt=%d h=%d arr=%d Pos=%.2f Tgt=%.2f err=%.2f Id=%.2f Iq=%.2f Iqref=%.2f vd=%.3f vq=%.3f du=%.3f spd=%.0f fb=%.0f I=%.2f\r\n",
                 g_foc.state, g_foc.flags.fault,
                 (int)g_foc.hall.state,
                 (int)g_foc.position_loop.arrived,
                 AngleObserver_GetMechDeg(&g_foc.angle, g_foc.config.pole_pairs),
                 g_foc.pos_target_deg_cmd,
                 g_foc.position_loop.pos_err_deg,
                 g_foc.current_loop.id,       /* d 轴电流: 大 = 电流跑进 d 轴, 不出力矩 */
                 g_foc.current_loop.iq,
                 g_foc.current_loop.iq_ref * (float)g_foc.torque_dir,
                 /* ↑ 打印的是实际施加给电流环的值(已乘 torque_dir), 应与 Iq 同号;
                    否则就是方向/取样有问题 */
                 g_foc.current_loop.vd,       /* 电流环输出(占空比量) */
                 g_foc.current_loop.vq,
                 PWM_Output_GetDu(&g_foc.pwm),/* 实际加到 U 相的占空比 */
                 g_foc.speed_loop.ref_erpm,
                 g_foc.speed_loop.erpm_now,   /* 速度环实际用的反馈(已低通) */
                 g_foc.speed_loop.integ);
        }

        /* ??RTT????????: ???? + ???? */
        if ((now - last_angle_print) >= ANGLE_LOG_MS) {
            /* 只在"真正在控制"的阶段打印; IDLE/BRAKE/FAULT 时 theta 不更新,
               打出来是 0/残留值, 对位置环调试无帮助 → 跳过. */
            if (g_foc.state == FOC_STATE_ALIGN ||
                g_foc.state == FOC_STATE_STARTUP ||
                g_foc.state == FOC_STATE_CURRENT_LOOP ||
                g_foc.state == FOC_STATE_SPEED_LOOP ||
                g_foc.state == FOC_STATE_POSITION_LOOP) {
            float elec_deg = g_foc.angle.theta * 57.29577951f;          /* 电角度 */
            last_angle_print = now;
            /* [ANG] 10ms 一条, 是日志里唯一的"刷屏源". 该问题(到位后电流
               不泄放/霍尔边界抖动)看 [MAIN] 的 Iqref/Iq/spd 就够, 故降到
               INFO 级(默认不打印). 要看回把 log.h 的 LOG_MIN_LEVEL 改成
               LOG_LEVEL_INFO 即可. */
            LOGI("[ANG] %u,%d,%d,%d,%.2f,%.4f,%.1f\r\n",
                 g_log_seq++,
                   (int)g_foc.state,                  /* FOC 状态 */
                   (int)g_foc.hall.state,             /* 霍尔扇区 1~6 */
                   (int)g_foc.angle.synced,           /* 是否已同步(测到转动) */
                   elec_deg,                          /* 电角度 (deg) */
                   AngleObserver_GetMechDeg(&g_foc.angle,
                                            g_foc.config.pole_pairs), /* 真实机械角(多圈) */
                   AngleObserver_GetOmegaFiltErpm(&g_foc.angle,
                                                  g_foc.config.pwm_freq));
            }
        }

        HAL_Delay(10);
    }
}
