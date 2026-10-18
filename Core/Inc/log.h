/**
  * @file    log.h
  * @brief   简易分级日志 (RTT/printf), 级别: info / debug / warn / error
  *
  * 用法(printf 风格, 支持可变参数):
  *     LOGI("...");      LOGD("...");      LOGW("...");      LOGE("...");
  *     LOGD("x=%d\r\n", x);
  *
  * 级别数值越大越严重:
  *     INFO(0) < DEBUG(1) < WARN(2) < ERROR(3)
  * LOG_MIN_LEVEL 为打印门限: 只有 level >= LOG_MIN_LEVEL 的日志才会输出.
  *
  * 当前门限 = LOG_LEVEL_DEBUG  →  只打印 debug / warn / error,
  * INFO 级(原先调试霍尔用的日志)暂不打印. 需要重新查看时, 只需把下面
  * LOG_MIN_LEVEL 改成 LOG_LEVEL_INFO 即可(无需改动任何调用点).
  */
#ifndef LOG_H
#define LOG_H

#include <stdio.h>

/* ==========================================================================
 * 统一日志出口（★ cal25）
 * --------------------------------------------------------------------------
 * 问题: `printf` → `fputc` → `SEGGER_RTT_PutChar` 逐字符写, 一行要 20~120us。
 * 而这些日志里有 20 多处是从 **20kHz 控制中断** 里打的（[POS] CREEP /
 * BLOCKED / STICTION / [ALIGN] / [STARTUP] / [OL] ...）
 * ⇒ 实测 [PROF] max=116us, 而控制周期只有 50us ⇒ **那几拍已经掉拍**。
 *
 * 对策: 所有日志统一走 LOG_Printf()。它自己看 SCB->ICSR 的 VECTACTIVE 位：
 *   · 主循环上下文 ⇒ 直接写 RTT(保持日志顺序)
 *   · 中断上下文   ⇒ 只 vsnprintf 到环形队列, 由主循环的 BSP_Log_DeferFlush()
 *                      写出。中断里开销从 20~120us 降到 2~6us。
 *
 * ✅ 调用点**不需要改** —— LOGD/LOGW 宏内部已经指向 LOG_Printf。
 * ========================================================================== */
int LOG_Printf(const char *fmt, ...);

#define LOG_LEVEL_INFO   0
#define LOG_LEVEL_DEBUG  1
#define LOG_LEVEL_WARN   2
#define LOG_LEVEL_ERROR  3

/* 打印门限(编译期常量): 只输出 >= 该级别的日志 */
#ifndef LOG_MIN_LEVEL
#define LOG_MIN_LEVEL    LOG_LEVEL_DEBUG
#endif

#if (LOG_MIN_LEVEL <= LOG_LEVEL_INFO)
#define LOGI(...)   do { (void)LOG_Printf(__VA_ARGS__); } while (0)
#else
#define LOGI(...)   do { } while (0)
#endif

#if (LOG_MIN_LEVEL <= LOG_LEVEL_DEBUG)
#define LOGD(...)   do { (void)LOG_Printf(__VA_ARGS__); } while (0)
#else
#define LOGD(...)   do { } while (0)
#endif

#if (LOG_MIN_LEVEL <= LOG_LEVEL_WARN)
#define LOGW(...)   do { (void)LOG_Printf(__VA_ARGS__); } while (0)
#else
#define LOGW(...)   do { } while (0)
#endif

#if (LOG_MIN_LEVEL <= LOG_LEVEL_ERROR)
#define LOGE(...)   do { (void)LOG_Printf(__VA_ARGS__); } while (0)
#else
#define LOGE(...)   do { } while (0)
#endif

#endif /* LOG_H */
