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

/* 固件版本标签: 开机打一次, 并**每秒跟着 [ADCR] 打一次** —— 这样即使用户
   只截取日志中段, 也能一眼看出跑的是哪个版本(反复吃过"烧的不是当前源码"的亏)。 */
#define FW_BUILD_TAG  "2026-10-11ac extdecay-5ms"

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
static uint16_t g_vbus_raw = 0;

/* ---- 开环速度测试参数 (S 键) ----
   15Hz 电频率: 按 pp=26 算约 35rpm, 按 pp=4 算约 225rpm —— 两种都可能,
   但都能看出转子在不在转. 合成电流 1A(约 1/4 额定), 10s 后自动停. */
#define OL_TEST_HZ    15.0f
#define OL_TEST_AMP    1.0f
#define OL_TEST_MS  10000U

/* ---- 母线电压采集 ----
   ADC1 规则通道 IN1 上挂了分压网络(config.vbus_divider=13.4), MX_ADC1_Init 里
   配置了规则转换、HAL_ADC_Start 也调了, 但**从来没有读过 DR** —— 于是"母线到底
   几伏"一直是盲区, 而这恰恰是解释下面这个矛盾唯一的未知量:

     日志里 vd/vq 到 ±0.45(≈11V@24V母线), 而相电流只有 2~3A  → 隐含阻抗 ~4Ω;
     但 ALIGN 刚开通那一拍 |v|=0.10(≈2.5V) 却读到 12A → 隐含阻抗 ~0.2Ω.
     同一台机器、同一秒内, 等效阻抗差 20 倍 → 只能是母线在负载下塌了
     (电源被限流 / 供电线压降), 因为"占空比×母线电压"才是真实施加电压.

   注意: ADC1 已被注入通道(相电流)占用, HAL_ADC_Start() 此时返回 HAL_BUSY 且
   不触发转换, 所以这里直接写 ADSTART 位做一次软件触发规则转换. 注入通道结果
   在 JDRx, 规则通道结果在 DR, 互不干扰. */
static float Vbus_ReadVolts(void)
{
    uint32_t guard = 0;

    ADC1->ISR = ADC_ISR_EOC;            /* 清掉可能残留的 EOC(免读到旧值) */
    ADC1->CR |= ADC_CR_ADSTART;         /* 软件触发一次规则转换 */
    while ((ADC1->ISR & ADC_ISR_EOC) == 0U) {
        if (++guard > 200000U) return -1.0f;   /* 超时: 返回负值, 便于识别 */
    }
    g_vbus_raw = (uint16_t)ADC1->DR;    /* 读 DR 会清 EOC */
    return (float)g_vbus_raw * g_foc.config.vref / 4095.0f *
           g_foc.config.vbus_divider;
}

/**
  * @brief  ADC??????????
  * @note   20kHz,??????????FOC???
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /* 电流量纲单一来源: 统一取 foc 配置, 避免这里和 config 各写一份
       (原来这里硬编码 R_SHUNT=0.01/OP_GAIN=8, 与 config.shunt_r/op_gain
        重复; 一旦要按实际硬件改标定值, 漏改一处就会对不上) */
    const float CURRENT_SCALE = g_foc.config.shunt_r * g_foc.config.op_gain;
    const float ADC_VREF = g_foc.config.vref;
    const float ADC_RESOLUTION = (float)g_foc.config.adc_resolution;
    const float CURRENT_SIGN = g_foc.config.current_sign;
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
            /* 2026-10-01 ★每拍轮询霍尔引脚: hall.state 原来只在 EXTI 边沿里更新,
               停机时没有边沿 ⇒ 它一直停在初值 0 ⇒ ALIGN 末尾读到 hall=0、
               不标定 offset ⇒ 角度全错 ⇒ 位置环正反馈把电机抽飞(实测 Pos 跑到
               -39616°)。这里 20kHz 直接读引脚(不碰 prev_state/edge_tick,
               不影响边沿测速), 保证任何时刻 hall.state 都是真值。 */
            {   /* 20kHz 直接读引脚: ① 保证任何时刻 hall.state 都是真值; ② 边沿
                   检测也在这里做(不依赖 EXTI, 见 AngleObserver_PollHall)。 */
                uint8_t hu = (uint8_t)HAL_GPIO_ReadPin(HALL_U_GPIO_Port, HALL_U_Pin);
                uint8_t hv = (uint8_t)HAL_GPIO_ReadPin(HALL_V_GPIO_Port, HALL_V_Pin);
                uint8_t hw = (uint8_t)HAL_GPIO_ReadPin(HALL_W_GPIO_Port, HALL_W_Pin);
                uint8_t hs = (uint8_t)((~((hu << 2) | (hv << 1) | hw)) & 0x07);
                g_foc.hall.u = hu;
                g_foc.hall.v = hv;
                g_foc.hall.w = hw;
                (void)AngleObserver_PollHall(&g_foc.angle, &g_foc.hall, hs,
                                             g_foc.tick,
                                             g_foc.config.pwm_freq,
                                             4.0f, 3000.0f, 1.5e-5f);
            }
            FOC_Control_Run(&g_foc, dt_s);
        }
    }
}

/**
  * @brief  ????????
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* 2026-10-11f: 霍尔边沿已改成 ADC 中断里 20kHz 轮询检测
       (AngleObserver_PollHall) —— EXTI 这条路径**必须停用**, 否则同一个跳变
       会被计两次(扇区序号一次加 2 = 位置整体跑掉)。
       这里只清引脚/保留回调(由 HAL 清 EXTI pending 位)。 */
    (void)GPIO_Pin;
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

    LOGD("\r\n[HALL] ==== Manual hall-spin verify (drive DISABLED) ====\r\n");
    LOGD("[HALL] Spin the rotor slowly & evenly by hand.\r\n");
    LOGD("[HALL] Watch st=1..6 sequence and U V W; no 0/7 expected.\r\n");
    LOGD("[HALL] elec = angle, current code convention (deg, 0..360)\r\n");
    LOGD("[HALL] rev  = mirrored angle if hall order were reversed\r\n");
    LOGD("[HALL] total = accumulated elec deg; spin EXACTLY one turn and\r\n");
    LOGD("[HALL]         pole_pairs = (fwd+bwd)/6  -> set config.pole_pairs\r\n");
    LOGD("[HALL] bad should stay 0 (0/7 or skipped sector = wiring/order).\r\n");
    LOGD("[HALL] ONLY 'q' works in this mode -> press 'q' to exit/re-align.\r\n");
}

static void HallVerify_Exit(void)
{
    hall_verify_mode = 0;
    g_foc.flags.fault   = 0;
    g_foc.flags.enabled = 1;
    FOC_Control_SetState(&g_foc, FOC_STATE_ALIGN);
    LOGD("[HALL] Verify exit -> ALIGN again.\r\n");
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
            LOGD("[HALL] #%u U%uV%uW%u st=%u  BAD(0/7)\r\n",
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
        LOGD("[HALL] U%uV%uW%u st=%u syn=%d elec=%.1f rev=%.1f fwd=%u bwd=%u bad=%u total=%.0fdeg\r\n",
             ru, rv, rw, st, (int)g_foc.angle.synced, elec, rev,
             g_hv.fwd_cnt, g_hv.bwd_cnt, g_hv.bad_cnt, g_hv.total_deg);
    }
}

/* ---- 一键启动: 不在闭环里(含 IDLE/FAULT)时自动做一次 "r" ----
   2026-10-01: 原来在 IDLE 下按数字键只改目标、不启动, 实测用户按 '1'/'2' 后
   电机完全没反应(日志: st 恒 0/IDLE, th 恒 0), 极易误判成算法坏了.
   现在按数字键或 'z' 会自动清故障 + ALIGN + STARTUP, 按了就走;
   已经在闭环(POSITION/SPEED/CURRENT)里则只改目标, 不重新对齐. */
static void AutoStartIfIdle(void)
{
    if (g_foc.state == FOC_STATE_IDLE || g_foc.state == FOC_STATE_FAULT) {
        g_foc.flags.fault   = 0;
        g_foc.fault_code    = 0;
        g_foc.flags.enabled = 1;
        g_foc.align_tick    = 0;   /* 让 DoAlign 重新计时 */
        g_foc.startup_tick  = 0;
        /* 注意: 这里**不能**清 pos_target_user_set, 用户刚设的目标要保留 */
        FOC_Control_SetState(&g_foc, FOC_STATE_ALIGN);
        /* 字符串里不要放非 ASCII: ARMCC 会报 #870-D invalid multibyte sequence,
           而且 RTT 终端上原样打出 `?????`(见历史日志) —— 用纯英文更清楚 */
        LOGD("[CMD] auto-start -> ALIGN (or press 'r')\n");
    }
}

int main(void)
{
    FOC_Config_t config;
    uint32_t last_print = 0;
    uint32_t last_adc_print = 0;
    uint32_t last_ol_print = 0;
    /* 2026-10-11l 日志降噪用: 上一条 [MAIN] 的内容 + verbose 开关 */
    float    last_mech = 1.0e9f;
    float    last_tgt  = 1.0e9f;
    uint8_t  last_hall = 0U;
    uint8_t  log_verbose = 0U;

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
    LOGD("[FW] build %s\r\n", FW_BUILD_TAG);

    /* ---- ??FOC ---- */
    config.pwm_freq = 20000.0f;
    config.vref = 3.3f;
    config.shunt_r = 0.01f;
    config.op_gain = 8.0f;
    /* 死区补偿 = 正反馈陷阱, 实测必须关掉:
       PWM_Output_SVPWM 里按每相电流符号给 ±dt_comp(0.02 duty) 修正, 本意是
       补偿死区压降。但它是拿"测到的电流"当输入 → 一旦电流读数有偏差就会
       在两相之间造出一个真实电压差 → 又反过来维持那个读数(正反馈)。
       实锤: OL 测试段0 命令 v=0(三相同占空比 = 绕组短接, 本该 0A), 日志却是
       `du=0.520 dv=0.520 dw=0.480`(不对称!) 且 |I| 从 8.75A 涨到 17.9A。
       0.02 duty 只有 0.39V 量级, 去掉它损失很小, 先排除这个疑点。 */
    config.deadtime_ns = 0.0f;
    config.current_scale = 0.01f * 8.0f;
    config.current_sign = -1.0f;
    /* 母线分压比: 用户用万用表实测驱动器母线端子 = 24V, 而 divider=13.4 时
       固件算出来只有 19.5V(即 raw≈1804 → 1.454V)。要得到 24V:
         1.454V * 16.5 = 24.0V  → 分压比应为 16.5(原来是 13.4, 算小了 23%)。
       这个常数直接影响所有 "占空比 x 母线" 的电压换算, 所以必须对。 */
    config.vbus_divider = 16.5f;
    config.adc_resolution = 4096;
    /* 实测电机参数(用户提供): 4 对极, 相电阻 0.52Ω, 相电感 0.31mH.
       原来写 26 对极是错的, 会直接影响 GetMechDeg / 位置环 / I-F 起始频率的换算. */
    config.pole_pairs = 4;
    /* 霍尔方向: 实测(逐组切换验证) 只有 dir_rev=1 + torque_dir=-1 一组
       能“不振动、稳定停住”，其余组合要么剧烈振动要么跑飞，故默认 1.
       运行时可用 'd' 键翻转(会与 torque_dir 组合成另一类符号). */
    config.hall_dir_rev = 1;
    config.hall_offset_deg = 0.0f;
    config.overcurrent_amp = 6.0f;
    /* 对齐电流: 实测 2.0A 已能可靠把转子拉到 0°(42BLH90 齿槽转矩不大), 
       不必用 3.0A(减少发热与对功率级的压力). I-F 拖转另用 startup_current. */
    config.align_current = 2.0f;
    /* I-F 拖转电流: 原来用 align_current*0.5=1.0A, 实测只把转子拖到 ~13eRPM
       就滑脱、6s 超时进 FAULT(42BLH90 齿槽转矩较大). 提到 3.0A 后能拖动;
       现取 2.5A 留出对 6A 过流门限与 4A 额定电流的余量. */
    config.startup_current = 2.5f;
    config.startup_hz = 6.0f;
    config.iq_ramp_rate = 2.0f;

    /* ???FOC?? */
    FOC_Control_Init(&g_foc, &config);

    /* 关键标定值开机打印: 这个工程反复吃过"烧进去的不是当前源码"的亏(最近一次:
       日志打出 vb=1.7V, 而源码里 vbus_divider=16.5 —— 1.7V 对应的分压比是 13.4,
       说明烧的是老 hex)。有这行就能一眼确认烧的是哪一版、哪个分压比。 */
    LOGD("[FW] vbus_div=%.1f vref=%.2f shunt=%.4fOhm gain=%.1f pp=%d dir_rev=%d align=%.1fA startup=%.1fA\r\n",
         g_foc.config.vbus_divider, g_foc.config.vref, g_foc.config.shunt_r,
         g_foc.config.op_gain, (int)g_foc.config.pole_pairs,
         (int)g_foc.config.hall_dir_rev,
         g_foc.config.align_current, g_foc.config.startup_current);
    /* 母线采样自检: 这行在 PWM 启动前, 故它应该就是万用表量到的母线电压。
       若相差数倍 ⇒ 分压网络/PA0 或"母线根本没到分压点"(同一节点也喂功率级,
       所以这种断点会同时表现为"有占空比但相电流恒 0")。 */

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
    LOGD("[MAIN] FOC ready (IDLE). dir_rev=%d torque_dir=%d. 's'=open-loop test  'c'=manual hall verify\r\n",
         (int)g_foc.config.hall_dir_rev, (int)g_foc.torque_dir);
    LOGD("[MAIN] Keys: o=MANUAL  u/i=V+/-0.01  n/m=V+/-0.05  h/l=freq +/-1Hz  f=dir  t=stop\r\n");
    LOGD("[MAIN] Keys: s=auto V/f test  p=PWM smoke test  r=re-align  d=dir_rev  y=torque\r\n");
    LOGD("[MAIN] Keys: j=+Iq pulse  k=-Iq pulse  z=target 0  0~9=digit*45deg(auto-start)  c=hall verify\r\n");
    LOGD("[MAIN] Keys: v=verbose log(25ms/line, default 500ms idle / 100ms moving)\r\n");
    LOGD("[ANG] idx,state,hall,synced,elec_deg,mech_deg,erpm\r\n");
    (void)g_log_seq;    /* [ANG] 已降为 LOGI(默认不打印), 序号不再被引用 */

    {
        uint32_t boot_wait = HAL_GetTick();
        uint8_t  start_ol  = 0;
        while (!hall_verify_mode && !start_ol &&
               (HAL_GetTick() - boot_wait) < 5000U) {
            int ch;
            while ((ch = SEGGER_RTT_GetKey()) >= 0) {
                if (ch == 'c' || ch == 'C') HallVerify_Enter();
                else if (ch == 's' || ch == 'S') start_ol = 1;
            }
            HAL_Delay(10);
        }

        /* 's': 直接进开环速度测试(1A / 15Hz 电频 / 10s 后自动停), 不进闭环 */
        if (start_ol) {
            FOC_Control_StartOpenLoopTest(&g_foc, OL_TEST_HZ, OL_TEST_AMP,
                                          OL_TEST_MS);
        }
    }

    /* 开机**不再自动进 ALIGN**。
       原因: 板子在维修/排查阶段, 一上电就自动驱动桥臂, 会把带故障的半桥
       (实测 W 相下管击穿)一直烧着, 几十秒就 70~80°C。改成必须手动按键才启动:
         's' = 自动 V/f 测试   'o' = 手动开环(配万用表)   'p' = PWM 硬烟测
       这样上电后桥臂是关的, 可以安全地做断电/上电的电压测量。 */
    if (!hall_verify_mode && !g_foc.ol_active) {
        LOGD("[MAIN] IDLE - NOT auto-started. press 'o'(manual) / 's'(auto V/f) / 'p'(pwm test)\r\n");
    }

    /* ---- ???: ??????? ---- */
    while (1) {
        uint32_t now = HAL_GetTick();
        uint32_t print_ms;
        int ch;

        /* RTT 键盘 */
        while ((ch = SEGGER_RTT_GetKey()) >= 0) {
            /* ★ 2026-10-11l: 每个收到的字符都回显一行。
               反复出现"按了没反应"的困惑: 有了这行就能一眼分清
               ①主机根本没把按键送进来(没有 [KEY] 行) ⇔
               ②收到了但控制没动([KEY] 有, 后面的 [CMD]/[MAIN] 不动)。 */
            LOGD("[KEY] '%c' (0x%02X) st=%d flt=%d\r\n",
                 (ch >= 32 && ch < 127) ? ch : '.', (unsigned)ch,
                 (int)g_foc.state, (int)g_foc.flags.fault);
            if (hall_verify_mode) {
                if (ch == 'q' || ch == 'Q') HallVerify_Exit();
            } else {
                if (ch == 'c' || ch == 'C') {
                    HallVerify_Enter();                } else if (ch == 'v' || ch == 'V') {
                    /* 2026-10-11l: 详细日志开关。默认 500ms/条(不动)或 100ms/条
                       (在动); 按 v 切成 25ms/条(看细节用), 再按一下关。
                       原因: 堵转时 err 一直超差 ⇒ 25ms 一条 = 每秒 9KB,
                       RTT 上行缓冲写满后 NO_BLOCK_SKIP 会丢掉日志,
                       用户看到的 [CMD]/[KEY] 回显也可能被挤掉。 */
                    log_verbose = log_verbose ? 0U : 1U;
                    LOGD("[CMD] verbose log %s (%ums/line)\r\n",
                         log_verbose ? "ON" : "OFF",
                         log_verbose ? 25U : 500U);                } else if (ch == 'd' || ch == 'D') {
                    /* 运行时翻转霍尔方向(dir_rev), 用于判断闭环换相是否反向 */
                    g_foc.angle.hall_dir_rev = g_foc.angle.hall_dir_rev ? 0 : 1;
                    g_foc.config.hall_dir_rev = g_foc.angle.hall_dir_rev;
                    LOGD("[CMD] dir_rev -> %d (torque=%d)\n",
                         (int)g_foc.angle.hall_dir_rev, (int)g_foc.torque_dir);
                } else if (ch == 's' || ch == 'S') {
                    /* 开环速度测试(与开机 5s 内按 s 同一个动作) */
                    FOC_Control_StartOpenLoopTest(&g_foc, OL_TEST_HZ, OL_TEST_AMP,
                                                  OL_TEST_MS);
                } else if (ch == 't' || ch == 'T') {
                    /* 停止开环测试(T 键专门用于测试; torque 翻转移到 y) */
                    if (g_foc.ol_active) {
                        FOC_Control_StopOpenLoopTest(&g_foc, "key T");
                    } else {
                        LOGD("[CMD] no open-loop test running\n");
                    }
                } else if (ch == 'y' || ch == 'Y') {
                    /* 运行时翻转力矩方向(torque_dir) */
                    g_foc.torque_dir = (int8_t)-g_foc.torque_dir;
                    LOGD("[CMD] torque_dir -> %d (dir=%d)\n",
                         (int)g_foc.torque_dir, (int)g_foc.angle.hall_dir_rev);
                } else if (ch == 'o' || ch == 'O') {
                    /* 手动开环: 电压/频率靠按键一步步加, 配合万用表自检 */
                    FOC_Control_StartManualTest(&g_foc);
                } else if (ch == 'u' || ch == 'U') {
                    FOC_Control_ManualAdjust(&g_foc, +0.01f, 0.0f);
                } else if (ch == 'i' || ch == 'I') {
                    FOC_Control_ManualAdjust(&g_foc, -0.01f, 0.0f);
                } else if (ch == 'n' || ch == 'N') {
                    FOC_Control_ManualAdjust(&g_foc, +0.05f, 0.0f);
                } else if (ch == 'm' || ch == 'M') {
                    FOC_Control_ManualAdjust(&g_foc, -0.05f, 0.0f);
                } else if (ch == 'h' || ch == 'H') {
                    FOC_Control_ManualAdjust(&g_foc, 0.0f, +1.0f);
                } else if (ch == 'l' || ch == 'L') {
                    FOC_Control_ManualAdjust(&g_foc, 0.0f, -1.0f);
                } else if (ch == 'p' || ch == 'P') {
                    /* 硬件烟测: 三相 50% 占空比, 不旋转, 按 t 停 */
                    FOC_Control_StartPwmSmokeTest(&g_foc);
                } else if (ch == 'f' || ch == 'F') {
                    /* 开环测试旋转方向翻转 */
                    FOC_Control_FlipOpenLoopDir(&g_foc);
                } else if (ch == 'r' || ch == 'R') {
                    /* 清故障并重新对齐: 之前一进 FAULT(st=7) 就只能断电重启,
                       ALIGN/STARTUP 失败后按任何数字键都没反应就是卡在这里. */
                    g_foc.flags.fault   = 0;
                    g_foc.fault_code    = 0;
                    g_foc.flags.enabled = 1;
                    g_foc.align_tick    = 0;   /* 让 DoAlign 重新计时 */
                    g_foc.startup_tick  = 0;
                    /* 单独按 'r' = "原地重新对齐并保持" ⇒ 不带上一轮的目标;
                       若在 r 之后再按数字键, 那个新目标不会被交接清掉。 */
                    g_foc.pos_target_user_set = 0U;
                    FOC_Control_SetState(&g_foc, FOC_STATE_ALIGN);
                    LOGD("[CMD] fault cleared -> ALIGN (target reset to hold)\n");
                } else if (ch == 'z' || ch == 'Z') {
                    FOC_Control_SetPosTarget(&g_foc, 0.0f);
                    LOGD("[CMD] Zero position\n");
                    AutoStartIfIdle();
                } else if (ch == 'j' || ch == 'J') {
                    /* 开环力矩探针: +Iq 脉冲, 测机械方向 */
                    FOC_Control_SetTorqueProbe(&g_foc, +2.5f, 600);
                } else if (ch == 'k' || ch == 'K') {
                    /* 开环力矩探针: -Iq 脉冲, 测机械方向 */
                    FOC_Control_SetTorqueProbe(&g_foc, -2.5f, 600);
                } else if (ch >= '0' && ch <= '9') {
                    /* ????:????????? */
                    float target = (float)(ch - '0') * 45.0f;
                    float now_deg = AngleObserver_GetMechDeg(&g_foc.angle,
                                                             g_foc.config.pole_pairs);
                    float de = target - now_deg;
                    FOC_Control_SetPosTarget(&g_foc, target);
                    /* 把按键那一瞬的位置/误差一起打出来: 反复出现"按了没动"的疑问,
                       其实多数是"已经在目标容差内" —— 位置环在 |err|<容差 时本来就
                       故意撤力矩(靠静摩擦保持), 所以按同一个数字键不会有任何动作。 */
                    LOGD("[CMD] Set target %.1fdeg | now=%.2f err=%.2f -> %s\r\n",
                         target, now_deg, de,
                         (fabsf(de) < g_foc.position_loop.arrive_tol_deg)
                             ? "ALREADY at target (no motion)" : "moving");
                    AutoStartIfIdle();   /* IDLE/FAULT 时按了就走, 不用先按 'r' */
                }
            }
        }

        /* ---- ADC 原始值诊断(1s 一条) ----
           核对电流采样量纲: I = -(cnt-offset)*Vref/(4095)/(shunt_r*op_gain).
           例: 若 cnt 只比 offset 高 ~100 而代码显示 1.5A, 说明 shunt_r 写小了
           (实际电阻更小/没有外部分流), 实际电流远小于显示值 → 空载也不转. */
        if ((now - last_adc_print) >= (log_verbose ? 1000U : 5000U)) {
            float vbus_v = Vbus_ReadVolts();
            last_adc_print = now;
            if (vbus_v >= 0.0f) g_foc.current.vbus = vbus_v;
            LOGD("[ADCR] fw=%s vbraw=%u vb=%.1fV | cntU=%u cntV=%u cntW=%u | offU=%u offV=%u offW=%u | Iu=%.2f Iv=%.2f Iw=%.2f | shunt=%.4f gain=%.1f\r\n",
                 FW_BUILD_TAG,
                 g_vbus_raw, vbus_v,
                 adc1_inj_val[0], adc2_inj_val[0], adc1_inj_val[1],
                 adc_offset_u, adc_offset_v, adc_offset_w,
                 g_foc.current.u, g_foc.current.v, g_foc.current.w,
                 g_foc.config.shunt_r, g_foc.config.op_gain);

            /* 负载下母线塌陷是"电流环顶到限幅也拉不到目标电流"的头号嫌疑,
               单独用 WARN 打一条, 免得淹没在 [ADCR] 里. */
            if (vbus_v >= 0.0f && vbus_v < 8.0f &&
                g_foc.state != FOC_STATE_IDLE) {
                LOGW("[VBUS] LOW %.1fV while driving (raw=%u) - supply current-limit / sag?!\r\n",
                     vbus_v, g_vbus_raw);
            }
        }

        /* ---- 手动开环日志(200ms 一条) ---- */
        if (g_foc.ol_active && g_foc.ol_manual && (now - last_ol_print) >= 200U) {
            last_ol_print = now;
            LOGD("[MAN] st=%d v=%.4f Vll=%.2fV f=%.1fHz dir=%+d th=%6.1f | |I|=%.2f (Iu=%+.2f Iv=%+.2f Iw=%+.2f) raw=%u/%u/%u | du=%.3f dv=%.3f dw=%.3f | vb=%.1fV\r\n",
                 (int)g_foc.state,
                 g_foc.ol_man_v,
                 1.7320508f * g_foc.ol_man_v * g_foc.current.vbus,
                 g_foc.ol_man_hz, (int)g_foc.ol_dir,
                 g_foc.ol_theta * 57.29577951f,
                 g_foc.ol_i_meas,
                 g_foc.current.u, g_foc.current.v, g_foc.current.w,
                 adc1_inj_val[0], adc2_inj_val[0], adc1_inj_val[1],
                 PWM_Output_GetDu(&g_foc.pwm),
                 PWM_Output_GetDv(&g_foc.pwm),
                 PWM_Output_GetDw(&g_foc.pwm),
                 g_foc.current.vbus);
        }

        /* ---- 自动开环测试日志(200ms 一条) ----
           段0(ph=0/1/2)看 V/I, 算 R_eq; 段1(ph=3)看电机是否转。
           `Vll` = 命令的线电压, 段0 稳压时拿万用表量任意两相引出线,
           读数应该接近 Vll —— 若差很多倍, 说明电压根本没加到电机上。 */
        if (g_foc.ol_active && !g_foc.ol_manual && (now - last_ol_print) >= 200U) {
            last_ol_print = now;
            LOGD("[OL] t=%.2fs ph=%d dir=%+d th=%6.1f v=%.4f Vll=%.2fV | |I|=%.2f (Iu=%+.2f Iv=%+.2f Iw=%+.2f) raw=%u/%u/%u | du=%.3f dv=%.3f dw=%.3f | vb=%.1fV\r\n",
                 (float)g_foc.ol_tick / g_foc.config.pwm_freq,
                 (int)g_foc.ol_phase, (int)g_foc.ol_dir,
                 g_foc.ol_theta * 57.29577951f,
                 g_foc.ol_v_amp,
                 1.7320508f * g_foc.ol_v_amp * g_foc.current.vbus,
                 g_foc.ol_i_meas,
                 g_foc.current.u, g_foc.current.v, g_foc.current.w,
                 adc1_inj_val[0], adc2_inj_val[0], adc1_inj_val[1],
                 PWM_Output_GetDu(&g_foc.pwm),
                 PWM_Output_GetDv(&g_foc.pwm),
                 PWM_Output_GetDw(&g_foc.pwm),
                 g_foc.current.vbus);
        }

        /* 手动盘轴验证: 高频采样(2ms), 不打常规状态行 */
        if (hall_verify_mode) {
            HallVerify_Poll(now);
            HAL_Delay(2);
            continue;
        }

        /* 日志节流(2026-10-11l 重做):
           原来 "err 超差 ⇒ 25ms 一条"，一堵转就是每秒 40 条、每秒 ~9KB ——
           ① RTT 上行缓冲写满后 NO_BLOCK_SKIP 直接**丢日志**(所以用户会看到
              断头行, 甚至丢 [CMD]/[KEY] 回显); ② 调试信息根本看不清。
           现在按"内容有没有变"定节拍:
             位置/霍尔/目标 有变化 → 100ms;  完全不动(堵转/保持) → 500ms;
             按 'v' 切 verbose → 25ms(查细节时手动开)。 */
        {
            float   mech = AngleObserver_GetMechDeg(&g_foc.angle,
                                                    g_foc.config.pole_pairs);
            uint8_t hh   = g_foc.hall.state;
            float   tg   = g_foc.pos_target_deg_cmd;
            uint8_t moving = (fabsf(mech - last_mech) > 0.02f) ||
                             (hh != last_hall) ||
                             (fabsf(tg - last_tgt) > 0.001f);
            print_ms = log_verbose ? 25U : (moving ? 100U : 500U);
            last_mech = mech;
            last_hall = hh;
            last_tgt  = tg;
        }
        if ((now - last_print) >= print_ms) {
            last_print = now;
            LOGD("[MAIN] t=%lums st=%d flt=%d fc=0x%04X h=%d arr=%d Pos=%.2f Tgt=%.2f err=%.2f th=%.1f Id=%.2f Iq=%.2f Iqref=%.2f vd=%.3f vq=%.3f du=%.3f spd=%.0f fb=%.0f Ipos=%.2f vsp=%.0f | ob=%.0f vp=%.0f adv=%.2f dte=%.0f | mv=%d jp=%u vs=%.0f | hl=%d gv=%d bh=%d cr=%u\r\n",
                 (unsigned long)now,
                 g_foc.state, g_foc.flags.fault, g_foc.fault_code,
                 (int)g_foc.hall.state,
                 (int)g_foc.position_loop.arrived,
                 AngleObserver_GetMechDeg(&g_foc.angle, g_foc.config.pole_pairs),
                 g_foc.pos_target_deg_cmd,
                 g_foc.position_loop.pos_err_deg,
                 g_foc.angle.theta * 57.29577951f, /* 电流矢量用的电角度: 应随霍尔边沿 60° 跳变, 停转时冻结 */
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
                 FOC_Control_GetPosIntegA(),  /* 位置环积分(A): 大 = 正在顶静摩擦 */
                 FOC_Control_GetPosErpm(),    /* 位置环阻尼用速度(eRPM): 振荡幅度 */
                 /* ---- 2026-10-11o 诊断四件套: 定位"停住了但 vsp 很大"是哪一路 ---- */
                 AngleObserver_GetOmegaFiltErpm(&g_foc.angle, g_foc.config.pwm_freq),
                 FOC_Control_GetVelPosErpm(),
                 g_foc.angle.adv * 57.29577951f,
                 (float)(g_foc.tick - g_foc.hall.edge_tick) /
                     (g_foc.config.pwm_freq / 1000.0f),
                 (int)FOC_Control_GetPosMove(),
                 (unsigned)FOC_Control_GetPosJumpCnt(),
                 FOC_Control_GetVelWinErpm(),
                 /* ★★ 2026-10-11aa: 三个静默锁 + 爬行失败数(l=到位死区,
                    g=爬行认输, b=机构推不动, cr=本轮连续推不动次数) */
                 (int)FOC_Control_GetPosHoldFlag(),
                 (int)FOC_Control_GetPosGiveFlag(),
                 (int)FOC_Control_GetPosBlockFlag(),
                 (unsigned)FOC_Control_GetPosHopCnt());
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
