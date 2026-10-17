/**
  ******************************************************************************
  * @file    sensor.h
  * @brief   位置传感器抽象层 —— 现在用数字霍尔, 将来换磁编码器只改
  *          board_config.h 里的 CFG_SENSOR_TYPE, 应用层代码不变。
  ******************************************************************************
  *
  *  为什么把接口定义成"传 FOC_Control_t 指针":
  *    位置/速度的观测状态(扇区号、扇区内进角、边沿时间戳、模型外推)本来
  *    就住在 FOC_Control_t 里, 控制律(位置环/速度环)要逐拍用到它们。
  *    这里不重新发明一套持状态的上下文, 而是把"读传感器/做边沿检测/换算出
  *    机械角与电角度"这几件事抽象成函数指针 —— 换传感器时只要用另一套实现
  *    填同一张表(见 sensor_hall.c / sensor_encoder.c), 上层与算法层都不用动。
  *
  *  实现约定(很重要):
  *    · Poll()  是**有副作用**的, 每个控制周期只能调一次(它推进边沿检测/
  *      测速/模型外推)。只应该由 App_OnCurrentSample() 在 20kHz 里调用。
  *    · 其余 Get*() 都是**只读**的, 随便在哪调(日志、波形、按键处理),
  *      不会改变观测器状态 —— 否则会在一个周期里重复外推, 位置会漂。
  *
  ******************************************************************************
  */
#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "foc_control.h"   /* FOC_Control_t 定义在这里(它内部持有观测器/霍尔状态) */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *name;                                        /* 便于日志/调试识别 */

    /* 20kHz: 采样传感器 + 边沿/测速更新(有副作用, 每周期一次) */
    void    (*Poll)(FOC_Control_t *foc);

    /* 机械位置(多圈, 机械度) */
    float   (*GetMechDeg)(const FOC_Control_t *foc);

    /* 电角度(rad, 0..2π) —— 只读"控制环上一次算出的值", 不重新外推 */
    float   (*GetElecThetaRad)(const FOC_Control_t *foc);

    /* 滤波后的转速(eRPM) */
    float   (*GetErpm)(const FOC_Control_t *foc);

    /* 扇区号(霍尔: 1..6; 编码器实现可返回 0) */
    uint8_t (*GetSector)(const FOC_Control_t *foc);

    /* ALIGN 结束时用当前扇区标定零点(编码器实现可以是空操作) */
    void    (*CalibrateAtZero)(FOC_Control_t *foc, uint8_t sector);
} Sensor_Driver_t;

/* 具体实现由 board_config.h 的 CFG_SENSOR_TYPE 在编译期选择
   (sensor_hall.c 或 sensor_encoder.c 里只会有一个定义 g_sensor)。 */
extern const Sensor_Driver_t g_sensor;

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_H__ */
