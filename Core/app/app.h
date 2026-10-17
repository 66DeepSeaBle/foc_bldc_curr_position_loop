/**
  ******************************************************************************
  * @file    app.h
  * @brief   应用层接口 —— main.c 只调这三个函数。
  ******************************************************************************
  */
#ifndef __APP_H__
#define __APP_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 上电初始化: 填参数 → FOC 初始化 → 启动 PWM/采样 → 零电流标定 →
   状态机进 IDLE → 打帮助 → 5s 手动窗口 → 启动波形输出 */
void App_Init(void);

/* 主循环体(由 main 的 while(1) 反复调用):
   按键 → 低频日志 → (霍尔验证模式) → 主状态日志 → 10ms 延时 */
void App_Loop(void);

/* ---- 中断钩子: 由 bsp 里的 HAL 回调转调, 应用层不直接进中断 ---- */
void App_OnCurrentSample(void);   /* 20kHz: 三相电流到齐 ⇒ 传感器轮询 + 控制环 */
void App_OnWaveTick(void);        /* 1kHz : VOFA 波形一帧 */

#ifdef __cplusplus
}
#endif

#endif /* __APP_H__ */
