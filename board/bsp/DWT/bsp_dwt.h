/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-08 18:07:13
 * @LastEditors: laladuduqq 17503181697@163.com
 * @LastEditTime: 2026-04-21 13:12:31
 * @FilePath: \mas_threadx\board\bsp\DWT\bsp_dwt.h
 * @Description:
 */
#ifndef _BSP_DWT_H_
#define _BSP_DWT_H_

#include "stdint.h"

#if defined(STM32H723xx)
#include "stm32h723xx.h"

#define BSP_DWT_MEASURE_START()                                                                                                                      \
    do                                                                                                                                               \
    {                                                                                                                                                \
        uint32_t __dwt_start_time = DWT->CYCCNT;

#define BSP_DWT_MEASURE_END(function_name)                                                                                                           \
    uint32_t __dwt_end_time = DWT->CYCCNT;                                                                                                           \
    uint32_t __dwt_cycles   = __dwt_end_time - __dwt_start_time;                                                                                     \
    float    __dwt_us_time  = (float)__dwt_cycles / (480000000 / 1000000.0f);                                                                        \
    LOG_I("[PERF] %s: %lu cycles, %.2f us", function_name, __dwt_cycles, __dwt_us_time);                                                             \
    }                                                                                                                                                \
    while (0)
#elif defined(STM32F407xx)
#include "stm32f407xx.h"

#define BSP_DWT_MEASURE_START()                                                                                                                      \
    do                                                                                                                                               \
    {                                                                                                                                                \
        uint32_t __dwt_start_time = DWT->CYCCNT;

#define BSP_DWT_MEASURE_END(function_name)                                                                                                           \
    uint32_t __dwt_end_time = DWT->CYCCNT;                                                                                                           \
    uint32_t __dwt_cycles   = __dwt_end_time - __dwt_start_time;                                                                                     \
    float    __dwt_us_time  = (float)__dwt_cycles / (168000000 / 1000000.0f);                                                                        \
    LOG_I("[PERF] %s: %lu cycles, %.2f us", function_name, __dwt_cycles, __dwt_us_time);                                                             \
    }                                                                                                                                                \
    while (0)
#elif defined(STM32F105xC)
#include "stm32f105xc.h"

#define BSP_DWT_MEASURE_START()
#define BSP_DWT_MEASURE_END(function_name)
#elif defined(STM32F103xB)
#include "stm32f103xb.h"

#define BSP_DWT_MEASURE_START()
#define BSP_DWT_MEASURE_END(function_name)
#endif

/**
 * @brief 初始化DWT,传入参数为CPU频率,单位MHz
 * @param CPU_Freq_mHz c板为168MHz,A板为180MHz,达妙h7为480MHz
 */
void BSP_DWT_Init(uint32_t CPU_Freq_mHz);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s
 * @param cnt_last 上一次调用的时间戳
 * @return float 时间间隔,单位为秒/s
 */
float BSP_DWT_GetDeltaT(uint32_t *cnt_last);

/**
 * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度
 * @param cnt_last 上一次调用的时间戳
 * @return double 时间间隔,单位为秒/s
 */
double BSP_DWT_GetDeltaT64(uint32_t *cnt_last);

/**
 * @brief 获取当前时间,单位为秒/s,即初始化后的时间
 */
float BSP_DWT_GetTimeline_s(void);

/**
 * @brief 获取当前时间,单位为毫秒/ms,即初始化后的时间
 */
float BSP_DWT_GetTimeline_ms(void);

/**
 * @brief 获取当前时间,单位为微秒/us,即初始化后的时间
 */
uint64_t BSP_DWT_GetTimeline_us(void);

/**
 * @brief DWT延时函数,单位为秒/s
 * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
 * @note 禁止在rtos初始化完成之前或者中断临界区使用HAL_Delay(),tx_thread_sleep()函数,应使用本函数
 * @param Delay 延时时间,单位为秒/s
 */
void BSP_DWT_Delay(float Delay);

#endif // _BSP_DWT_H_
