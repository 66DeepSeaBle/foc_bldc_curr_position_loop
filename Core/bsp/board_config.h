/**
  ******************************************************************************
  * @file    board_config.h
  * @brief   【唯一的功能配置文件】—— 换芯片 / 换串口 / 换传感器 / 换引脚 / 调参数
  *          都只改这一个文件, 上层应用(app.c)和纯算法(foc/pid/…)一个字都不用动。
  ******************************************************************************
  *
  *  分层结构(自下而上, 上层只依赖下层的 .h):
  *
  *    ┌──────────────────────────────────────────────┐
  *    │ main.c        只有 HAL_Init + 三个函数调用    │
  *    ├──────────────────────────────────────────────┤
  *    │ app.c/.h      应用层: 按键、日志、标定、状态机 │  ← 不含任何 HAL/寄存器
  *    ├──────────────────────────────────────────────┤
  *    │ sensor.h      sensor_hall.c / sensor_encoder.c │  ← 位置传感器抽象
  *    ├──────────────────────────────────────────────┤
  *    │ bsp.h         bsp_stm32g4.c (或 bsp_gd32.c)    │  ← 平台层: 只有这里碰 HAL/寄存器
  *    ├──────────────────────────────────────────────┤
  *    │ 算法(平台无关): foc_control / pid / transform  │
  *    │                current_loop / speed_loop / ... │
  *    └──────────────────────────────────────────────┘
  *
  *  换平台: 保留 board_config.h + app.c + sensor_*.c + 全部算法文件,
  *          只写一个新的 bsp_xxx.c 实现 bsp.h 里那几个函数即可。
  *  换传感器: 改 CFG_SENSOR_TYPE, 并实现 sensor.h 里的接口。
  *
  ******************************************************************************
  */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__
#include "main.h"
#include "foc_types.h"   /* 只为拿到 FOC_CTRL_HZ(控制拍频唯一真源) */

/* ==========================================================================
 * 1) 平台选择
 * ========================================================================== */
#define CFG_PLATFORM_STM32G4      180
#define CFG_PLATFORM_GD32         2
#define CFG_PLATFORM              CFG_PLATFORM_STM32G4

/* ==========================================================================
 * 2) 位置传感器选择
 *    CFG_SENSOR_HALL    : 数字霍尔(3 路, 当前使用的)
 *    CFG_SENSOR_ENCODER : 磁编码器(SPI/I2C/ABZ, 见 sensor_encoder.c 的骨架)
 * ========================================================================== */
#define CFG_SENSOR_HALL           1
#define CFG_SENSOR_ENCODER        2
#define CFG_SENSOR_TYPE           CFG_SENSOR_HALL

/* ==========================================================================
 * 3) 串口(调试/波形输出)
 *    当前: USART2 = PB3(TX)/PB4(RX), AF7, TX 走 DMA1_Channel1
 *    ⚠ 换串口只改这一段; usart.c 里的 MX_USART2_UART_Init 会照着它配。
 *    ⚠ 若换成 USART1(PB6/PB7), 同时要把 CubeMX 的 usart.c/中断向量对应上。
 * ========================================================================== */
#define CFG_UART_CONSOLE          1        /* 1 = 串口打印(VOFA/日志) 使能 */
#define CFG_UART_NUM              2        /* 实例编号: 1=USART1, 2=USART2(只用于预编译分支) */
#define CFG_UART_INSTANCE         USART2
#define CFG_UART_HANDLE           huart2
#define CFG_UART_BAUD             921600U  /* VOFA 波形: 1kHz×20B=160kbps, 9600 根本发不出来 */
#define CFG_UART_AF               GPIO_AF7_USART2
#define CFG_UART_GPIO_PORT        GPIOB
#define CFG_UART_TX_PIN           GPIO_PIN_3
#define CFG_UART_RX_PIN           GPIO_PIN_4
#define CFG_UART_DMA_HANDLE       hdma_usart2_tx
#define CFG_UART_DMA_INSTANCE     DMA1_Channel1
#define CFG_UART_DMA_REQUEST      DMA_REQUEST_USART2_TX
#define CFG_UART_IRQn             USART2_IRQn
#define CFG_UART_DMA_IRQn         DMA1_Channel1_IRQn

/* ==========================================================================
 * 4) 功率级 / PWM
 * ========================================================================== */
#define CFG_PWM_TIM_HANDLE        htim1
#define CFG_PWM_TIM               TIM1
/* ★ 2026-10-24: PWM / 控制频率统一由 foc_types.h 的 FOC_CTRL_HZ 决定
   (改频只改那一处; tim.c 的 htim1.Init.Period 也需同步改) */
#define CFG_PWM_FREQ_HZ           ((float)FOC_CTRL_HZ)  /* 中心对齐 PWM 频率 */
#define CFG_CTRL_FREQ_HZ          ((float)FOC_CTRL_HZ)  /* FOC 控制环频率(= ADC 注入中断频率) */
#define CFG_PWM_DEADTIME_NS       500       /* 死区, 与 tim.c 里 DeadTime 一致 */
#define CFG_PWM_MAX_DUTY          0.95f     /* 单相占空比上限(留开关裕量) */

/* ==========================================================================
 * 5) ADC / 采样
 * ========================================================================== */
#define CFG_ADC_CUR_HANDLE_A      hadc1     /* 相电流: ADC1 注入 rank1=U, rank2=W */
#define CFG_ADC_CUR_HANDLE_B      hadc2     /* 相电流: ADC2 注入 rank1=V       */
#define CFG_ADC_VBUS_INSTANCE     ADC1      /* 母线电压: ADC1 规则通道 IN1(寄存器访问用实例指针) */

/* ==========================================================================
 * 6) 定时器任务
 * ========================================================================== */
#define CFG_WAVE_TIM_HANDLE       htim6
#define CFG_WAVE_RATE_HZ          1000U     /* VOFA 波形帧率(1kHz) */
#define CFG_HALL_IC_TIM_HANDLE    htim3     /* 霍尔边沿高精度时间戳(诊断旁路) */

/* ==========================================================================
 * 7) 霍尔引脚
 * ========================================================================== */
#define CFG_HALL_U_PORT           HALL_U_GPIO_Port
#define CFG_HALL_U_PIN            HALL_U_Pin
#define CFG_HALL_V_PORT           HALL_V_GPIO_Port
#define CFG_HALL_V_PIN            HALL_V_Pin
#define CFG_HALL_W_PORT           HALL_W_GPIO_Port
#define CFG_HALL_W_PIN            HALL_W_Pin

/* ==========================================================================
 * 8) 电机 / 硬件标定参数(实测值, 换电机才需要改)
 * ========================================================================== */
#define CFG_MOTOR_POLE_PAIRS      4         /* 4 对极(42BLH90) */
#define CFG_MOTOR_RS_OHM          0.52f     /* 相电阻 */
#define CFG_SHUNT_R_OHM           0.01f     /* 采样电阻 */
#define CFG_OPAMP_GAIN            8.0f      /* 运放(内部 PGA)增益 */
#define CFG_ADC_VREF              3.3f
#define CFG_ADC_RESOLUTION        4096
#define CFG_CURRENT_SIGN          (-1.0f)   /* 电流采样极性 */
#define CFG_VBUS_DIVIDER          16.5f     /* 母线分压比(万用表实测 24V 标定) */
#define CFG_HALL_DIR_REV          1         /* 霍尔相序方向(实测唯一稳定组合) */
#define CFG_OVER_CURRENT_A        6.0f

/* ==========================================================================
 * 9) 控制默认参数
 * ========================================================================== */
#define CFG_ALIGN_CURRENT_A       2.0f
#define CFG_STARTUP_CURRENT_A     2.5f
#define CFG_STARTUP_HZ            6.0f
#define CFG_IQ_RAMP_A_S           2.0f      /* q 轴电流指令斜坡 */
/* 位置/速度环的调参量属于“纯算法参数”, 统一放在 foc_control.c 顶部的 POS_* 宏里
   (包括数字键默认时长 FOC_POS_DEFAULT_TM_S), 本文件只管“硬件/端口”。 */

/* ==========================================================================
 * 10) 自检快捷键用的测试参数
 * ========================================================================== */
#define CFG_OL_TEST_HZ            15.0f     /* 开环 V/f 测试电频率 */
#define CFG_OL_TEST_AMP_A         1.0f      /* 开环 V/f 合成电流 */
#define CFG_OL_TEST_MS            10000U    /* 自动停止时间 */

/* ==========================================================================
 * 11) 日志
 * ========================================================================== */
#define CFG_LOG_IDLE_MS           500U      /* 静止时 [MAIN] 周期 */
#define CFG_LOG_MOVING_MS         100U      /* 运动时 [MAIN] 周期 */
#define CFG_LOG_VERBOSE_MS        25U       /* 详细模式周期 */
#define CFG_LOG_ADC_MS            5000U     /* [ADCR] 周期 */

#endif /* __BOARD_CONFIG_H__ */
