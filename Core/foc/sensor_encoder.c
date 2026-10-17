/**
  ******************************************************************************
  * @file    sensor_encoder.c
  * @brief   磁编码器位置传感器实现(骨架/预留) —— 换编码器时照这个文件填。
  ******************************************************************************
  *
  *  用法: board_config.h 里把
  *        #define CFG_SENSOR_TYPE   CFG_SENSOR_ENCODER
  *        再把 CFG_ENC_* 几个参数按实际编码器填好。
  *
  *  与霍尔的差别(填这个文件时必须处理的三件事):
  *   1. 分辨率: 霍尔只有 6 扇区(15.47° 机械), 编码器一般是 12~14 bit
  *      (0.09~0.02°), 因此 position/speed 环的分辨率假设全部要重算 ——
  *      那些阈值集中在 foc_control.c 的 POS_* 宏里(它们是按 15.47° 粒度定的)。
  *   2. 方向: 编码器读数增大对应哪个机械方向, 用 CFG_ENC_DIR 修,
  *      不要沿用霍尔的 hall_dir_rev。
  *   3. 绝对零点: 霍尔靠 ALIGN 时读扇区反解 offset; 编码器可以直接记住
  *      "上电时的读数 = 当前位置", offset 由 CFG_ENC_ZERO_DEG 给。
  *
  *  接口见 sensor.h。下面所有 "TODO" 都是接编码器时要写的部分。
  *
  ******************************************************************************
  */
#include "sensor.h"
#include "board_config.h"
#include "bsp.h"

#if (CFG_SENSOR_TYPE == CFG_SENSOR_ENCODER)

/* ---- 编码器参数(换硬件时改这里, 或移到 board_config.h) ---- */
#define ENC_BITS            14U          /* 单圈分辨率 14bit = 16384 */
#define ENC_COUNTS          ((float)(1UL << ENC_BITS))
#define ENC_DIR             (1.0f)       /* 读数增大 → 机械正方向 ? 1 : -1 */

/* 编码器原始读数。TODO: 接 SPI/I2C 读取(如 AS5047/MT6701: 读 0x3FFF 角度寄存器) */
static uint32_t Enc_ReadRaw(void)
{
    /* TODO: 换成实际总线读取, 例如:
       uint16_t v = AS5047_ReadAngle();
       return (uint32_t)(v & 0x3FFF);                                  */
    return 0U;
}

/* 每个控制周期: 读一次原始角度 → 换算机械角 → 差分测速
   注意: 编码器是绝对量, 不需要"边沿检测", 这里只需要保存上一次的角度做微分。 */
static void Enc_Poll(FOC_Control_t *foc)
{
    static uint32_t s_last_raw  = 0U;
    static uint8_t  s_first     = 1U;
    static float    s_mech_deg  = 0.0f;   /* 多圈机械角(累计) */
    static float    s_erpm      = 0.0f;

    uint32_t raw = Enc_ReadRaw();
    float    one_turn_deg = 360.0f;

    if (s_first) {
        s_last_raw = raw;
        s_first    = 0U;
        /* TODO: 上电零点: s_mech_deg = 由 CFG_ENC_ZERO_DEG 与当前读数算绝对位置 */
    } else if (raw != s_last_raw) {
        /* 差分并处理过零回绕(单圈内 ±半圈视为连续) */
        int32_t d = (int32_t)raw - (int32_t)s_last_raw;
        if (d >  (int32_t)(ENC_COUNTS * 0.5f)) d -= (int32_t)ENC_COUNTS;
        if (d < -(int32_t)(ENC_COUNTS * 0.5f)) d += (int32_t)ENC_COUNTS;

        s_mech_deg += ENC_DIR * ((float)d / ENC_COUNTS) * one_turn_deg;
        s_erpm      = ENC_DIR * ((float)d / ENC_COUNTS) * one_turn_deg
                      * foc->config.pwm_freq * 60.0f / 360.0f;
        s_last_raw  = raw;
    }

    /* 把结果写回 FOC 上下文(multiturn Deg / elec theta), 供控制律使用 */
    foc->angle.elec_acc_deg = s_mech_deg * foc->config.pole_pairs;   /* 机械角→累计电角度 */
    foc->angle.theta        = (float)((int)(s_mech_deg * foc->config.pole_pairs) % 360)
                              * 0.01745329252f;
    foc->angle.omega        = s_erpm * 6.28318531f / 60.0f / foc->config.pwm_freq;
    foc->angle.omega_filt   = foc->angle.omega;
    foc->angle.synced       = 1U;
    foc->hall.state         = 0U;   /* 编码器没有扇区概念 */
}

static float Enc_GetMechDeg(const FOC_Control_t *foc)
{
    return foc->angle.elec_acc_deg / foc->config.pole_pairs;
}

static float Enc_GetElecThetaRad(const FOC_Control_t *foc)
{
    return foc->angle.theta;
}

static float Enc_GetErpm(const FOC_Control_t *foc)
{
    return foc->angle.omega_filt * foc->config.pwm_freq * 60.0f / 6.28318531f;
}

static uint8_t Enc_GetSector(const FOC_Control_t *foc)
{
    (void)foc;
    return 0U;      /* 编码器无扇区 */
}

/* 编码器是绝对量, 不需要"用扇区反解 offset"; 若要修正安装零点, 在这里做 */
static void Enc_CalibrateAtZero(FOC_Control_t *foc, uint8_t sector)
{
    (void)foc;
    (void)sector;
    /* TODO: 记录当前机械角作为 0°, 或把 CFG_ENC_ZERO_DEG 写进配置 */
}

const Sensor_Driver_t g_sensor = {
    "encoder",
    Enc_Poll,
    Enc_GetMechDeg,
    Enc_GetElecThetaRad,
    Enc_GetErpm,
    Enc_GetSector,
    Enc_CalibrateAtZero
};

#else   /* 未选中编码器实现: 本文件保持为空(避免未使用函数的告警) */
typedef int sensor_encoder_unused_t;
#endif

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
