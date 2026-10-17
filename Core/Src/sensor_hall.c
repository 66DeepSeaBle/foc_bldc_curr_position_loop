/**
  ******************************************************************************
  * @file    sensor_hall.c
  * @brief   数字霍尔(3 路 U/V/W)位置传感器实现 —— 把 angle_observer 的
  *          扇区观测算法包装成 sensor.h 的统一接口。
  ******************************************************************************
  *
  *  要点(实机踩过的坑, 别轻易改):
  *   · 边沿检测必须用 **20kHz 轮询**而不是 EXTI: 实测 EXTI 会丢边沿, 丢一次
  *     扇区序号就少算 6(整 90° 机械), 位置读数会突然跳 90°。
  *   · 轮询回调里"非相邻跳变"一律当噪声丢弃(50µs 跨两个扇区 = 30 万°/s,
  *     物理上不存在)。
  *   · 停转时的模型外推由 angle_observer 内部按 POS_MODEL_TAU_S 衰减,
  *     不会出现"读数匀速虚爬"。
  *
  ******************************************************************************
  */
#include "sensor.h"
#include "angle_observer.h"
#include "board_config.h"
#include "bsp.h"

#if (CFG_SENSOR_TYPE == CFG_SENSOR_HALL)

/* 轮询参数(原样沿用 main.c 里的实机调好的值) */
#define HALL_POLL_MIN_DT_TICKS   4.0f       /* 边沿最小间隔 4 拍(200µs) 防抖 */
#define HALL_POLL_MAX_ERPM       3000.0f    /* 速度上限 */
#define HALL_POLL_DT_S           1.5e-5f    /* 50µs 的一拍(含余量) */

/* 每个控制周期: 读三路霍尔电平 → 合成扇区 → 交给观测器做边沿检测/测速 */
static void Hall_Poll(FOC_Control_t *foc)
{
    uint8_t u = 0U, v = 0U, w = 0U;
    uint8_t st = BSP_Hall_ReadSector(&u, &v, &w);

    foc->hall.u = u;
    foc->hall.v = v;
    foc->hall.w = w;

    (void)AngleObserver_PollHall(&foc->angle, &foc->hall, st, foc->tick,
                                 foc->config.pwm_freq,
                                 HALL_POLL_MIN_DT_TICKS,
                                 HALL_POLL_MAX_ERPM,
                                 HALL_POLL_DT_S);
}

static float Hall_GetMechDeg(const FOC_Control_t *foc)
{
    return AngleObserver_GetMechDeg(&foc->angle, foc->config.pole_pairs);
}

static float Hall_GetElecThetaRad(const FOC_Control_t *foc)
{
    return foc->angle.theta;        /* 控制环上一拍算好的值, 只读 */
}

static float Hall_GetErpm(const FOC_Control_t *foc)
{
    return AngleObserver_GetOmegaFiltErpm(&foc->angle, foc->config.pwm_freq);
}

static uint8_t Hall_GetSector(const FOC_Control_t *foc)
{
    return foc->hall.state;
}

/* ALIGN 阶段把转子锁在电角度 0°, 此时读到的霍尔扇区就是"零点", 反解 offset */
static void Hall_CalibrateAtZero(FOC_Control_t *foc, uint8_t sector)
{
    AngleObserver_CalibrateOffsetAtZero(&foc->angle, sector);
}

const Sensor_Driver_t g_sensor = {
    "hall",
    Hall_Poll,
    Hall_GetMechDeg,
    Hall_GetElecThetaRad,
    Hall_GetErpm,
    Hall_GetSector,
    Hall_CalibrateAtZero
};

#else   /* 未选中霍尔实现: 本文件保持为空(避免未使用函数的告警) */
typedef int sensor_hall_unused_t;
#endif

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
