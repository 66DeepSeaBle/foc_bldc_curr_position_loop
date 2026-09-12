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

#define LOG_LEVEL_INFO   0
#define LOG_LEVEL_DEBUG  1
#define LOG_LEVEL_WARN   2
#define LOG_LEVEL_ERROR  3

/* 打印门限(编译期常量): 只输出 >= 该级别的日志 */
#ifndef LOG_MIN_LEVEL
#define LOG_MIN_LEVEL    LOG_LEVEL_DEBUG
#endif

#if (LOG_MIN_LEVEL <= LOG_LEVEL_INFO)
#define LOGI(...)   do { printf(__VA_ARGS__); } while (0)
#else
#define LOGI(...)   do { } while (0)
#endif

#if (LOG_MIN_LEVEL <= LOG_LEVEL_DEBUG)
#define LOGD(...)   do { printf(__VA_ARGS__); } while (0)
#else
#define LOGD(...)   do { } while (0)
#endif

#if (LOG_MIN_LEVEL <= LOG_LEVEL_WARN)
#define LOGW(...)   do { printf(__VA_ARGS__); } while (0)
#else
#define LOGW(...)   do { } while (0)
#endif

#if (LOG_MIN_LEVEL <= LOG_LEVEL_ERROR)
#define LOGE(...)   do { printf(__VA_ARGS__); } while (0)
#else
#define LOGE(...)   do { } while (0)
#endif

#endif /* LOG_H */
