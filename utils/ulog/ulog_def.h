#ifndef _ULOG_DEF_H_
#define _ULOG_DEF_H_

#include <stdbool.h>
#include <stddef.h>
#include "ulog.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 日志级别定义 ========================== */
#define LOG_LVL_ASSERT  0
#define LOG_LVL_ERROR   3
#define LOG_LVL_WARNING 4
#define LOG_LVL_INFO    6
#define LOG_LVL_DBG     7

/* ========================== 核心配置 ========================== */
#ifndef ULOG_OUTPUT_LVL
#define ULOG_OUTPUT_LVL LOG_LVL_DBG // 全局输出级别
#endif

#ifndef ULOG_LINE_BUF_SIZE
#define ULOG_LINE_BUF_SIZE 128 // 单条日志最大长度
#endif

#ifndef ULOG_NEWLINE_SIGN
#define ULOG_NEWLINE_SIGN "\r\n" // 换行符
#endif

/* ========================== 输出内容控制（已关闭级别字符） ========================== */
#define ULOG_OUTPUT_TIME        1 // 输出时间戳
#define ULOG_OUTPUT_LEVEL       0 // 输出 E/、W/ 等级别字符
#define ULOG_OUTPUT_TAG         1 // 输出标签
#define ULOG_OUTPUT_THREAD_NAME 1 // 输出线程名

/* ========================== 颜色输出 ========================== */
#define ULOG_USING_COLOR        1
#define CSI_START               "\033["
#define CSI_END                 "\033[0m"

/* 颜色定义 */
#define F_GREEN                 "32m"
#define F_YELLOW                "33m"
#define F_RED                   "31m"
#define F_MAGENTA               "35m"

#ifndef ULOG_COLOR_DEBUG
#define ULOG_COLOR_DEBUG NULL
#endif

#ifndef ULOG_COLOR_INFO
#define ULOG_COLOR_INFO F_GREEN
#endif

#ifndef ULOG_COLOR_WARN
#define ULOG_COLOR_WARN F_YELLOW
#endif

#ifndef ULOG_COLOR_ERROR
#define ULOG_COLOR_ERROR F_RED
#endif

#ifndef ULOG_COLOR_ASSERT
#define ULOG_COLOR_ASSERT F_MAGENTA
#endif

#define LOG_E_LOCK(...)                                                                                                                              \
    do                                                                                                                                               \
    {                                                                                                                                                \
        ulog_output(LOG_LVL_ERROR, LOG_TAG, true, __VA_ARGS__);                                                                                      \
        ulog_flush();                                                                                                                                \
        while (1);                                                                                                                                   \
    } while (0)

/* ========================== 断言 ========================== */
#ifdef ULOG_ASSERT_ENABLE
#define ULOG_ASSERT(EXPR)                                                                                                                            \
    do                                                                                                                                               \
    {                                                                                                                                                \
        if (!(EXPR))                                                                                                                                 \
        {                                                                                                                                            \
            ulog_output(LOG_LVL_ASSERT, LOG_TAG, true, "Assert failed: %s @ %s:%ld", #EXPR, __FUNCTION__, __LINE__);                                 \
            ulog_flush();                                                                                                                            \
            while (1);                                                                                                                               \
        }                                                                                                                                            \
    } while (0)
#else
#define ULOG_ASSERT(EXPR) ((void)0)
#endif

#ifndef ASSERT
#define ASSERT ULOG_ASSERT
#endif

/* ========================== LOG_TAG/LOG_LVL 默认值 ========================== */
#ifndef LOG_TAG
#define LOG_TAG                                                                                                                                      \
    (__has_include("DBG_TAG") ? DBG_TAG : (__has_include("DBG_SECTION_NAME") ? DBG_SECTION_NAME : "NO_TAG"))
#endif

#ifndef LOG_LVL
#define LOG_LVL                                                                                                                                      \
    (__has_include("DBG_LVL") ? DBG_LVL : (__has_include("DBG_LEVEL") ? DBG_LEVEL : LOG_LVL_DBG))
#endif

/* ========================== 日志输出宏 ========================== */
#define _ULOG_ENABLED(lvl)     ((LOG_LVL >= (lvl)) && (ULOG_OUTPUT_LVL >= (lvl)))
#define ulog_d(TAG, ...)       _ULOG_ENABLED(LOG_LVL_DBG) ? ulog_output(LOG_LVL_DBG, TAG, true, __VA_ARGS__) : ((void)0)
#define ulog_i(TAG, ...)       _ULOG_ENABLED(LOG_LVL_INFO) ? ulog_output(LOG_LVL_INFO, TAG, true, __VA_ARGS__) : ((void)0)
#define ulog_w(TAG, ...)       _ULOG_ENABLED(LOG_LVL_WARNING) ? ulog_output(LOG_LVL_WARNING, TAG, true, __VA_ARGS__) : ((void)0)
#define ulog_e(TAG, ...)       _ULOG_ENABLED(LOG_LVL_ERROR) ? ulog_output(LOG_LVL_ERROR, TAG, true, __VA_ARGS__) : ((void)0)
#define ulog_hex(TAG, w, b, s) _ULOG_ENABLED(LOG_LVL_DBG) ? ulog_hexdump(TAG, w, b, s) : ((void)0)

/* 对外统一接口 */
#define LOG_E(...)             ulog_e(LOG_TAG, __VA_ARGS__)
#define LOG_W(...)             ulog_w(LOG_TAG, __VA_ARGS__)
#define LOG_I(...)             ulog_i(LOG_TAG, __VA_ARGS__)
#define LOG_D(...)             ulog_d(LOG_TAG, __VA_ARGS__)
#define LOG_RAW(...)           ulog_raw(__VA_ARGS__)
#define LOG_HEX(n, w, b, s)    ulog_hex(n, w, b, s)

#ifdef __cplusplus
}
#endif

#endif /* _ULOG_DEF_H_ */
