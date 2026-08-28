/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-04 09:49:49
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-04 10:04:41
 * @FilePath: /mas_embedded_threadx/board/bsp/bsp_def.h
 * @Description:
 */
#ifndef _BSP_DEF_H_
#define _BSP_DEF_H_

#include "tx_api.h"

extern TX_BYTE_POOL tx_app_byte_pool;

/* h7与f4 数据和线程栈等SECTION宏定义 */
#if defined(STM32H723xx)
#define APPS_STACK_SECTION __attribute__((section(".dtcmram")))
#define BUFFER_SECTION     __attribute__((section(".RAM_D1"))) __attribute__((aligned(32)))
#elif defined(STM32F407xx)
#define APPS_STACK_SECTION __attribute__((section(".ccmram")))
#define BUFFER_SECTION     __attribute__((section(".ram"))) __attribute__((aligned(32)))
#elif defined(STM32F105xC) || defined(STM32F103xB)
#define APPS_STACK_SECTION
#define BUFFER_SECTION                   __attribute__((aligned(32)))
#endif

/* H7 Cache 同步 & DMA 地址检查 宏定义 */
#if defined(STM32H723xx)
/**
 * @brief 检查地址是否在 DMA 不可访问区域（DTCM / ITCM）
 * @param addr 待检查的内存地址
 * @return 非零 = DMA 不可访问
 */
#define BSP_IS_DMA_INACCESSIBLE(addr)                                                                                                                \
    ((((uint32_t)(addr) >= 0x20000000UL) && ((uint32_t)(addr) < 0x20020000UL)) ||                                                                    \
     (((uint32_t)(addr) >= 0x00000000UL) && ((uint32_t)(addr) < 0x00010000UL)))
/**
 * @brief 清理 D-Cache（TX DMA 发送前使用，确保 DMA 读到 CPU 写入的最新数据）
 * @param addr 数据起始地址
 * @param len  数据长度（字节）
 */
#define BSP_CACHE_CLEAN(addr, len)                                                                                                                   \
    do                                                                                                                                               \
    {                                                                                                                                                \
        uint32_t _s = (uint32_t)(addr) & ~0x1FUL;                                                                                                    \
        uint32_t _e = ((uint32_t)(addr) + (len) + 31UL) & ~0x1FUL;                                                                                   \
        SCB_CleanDCache_by_Addr((uint32_t *)_s, _e - _s);                                                                                            \
    } while (0)
/**
 * @brief 无效化 D-Cache（RX DMA 接收后使用，确保 CPU 读到 DMA 写入的最新数据）
 * @param addr 数据起始地址
 * @param len  数据长度（字节）
 */
#define BSP_CACHE_INVALIDATE(addr, len)                                                                                                              \
    do                                                                                                                                               \
    {                                                                                                                                                \
        uint32_t _s = (uint32_t)(addr) & ~0x1FUL;                                                                                                    \
        uint32_t _e = ((uint32_t)(addr) + (len) + 31UL) & ~0x1FUL;                                                                                   \
        SCB_InvalidateDCache_by_Addr((uint32_t *)_s, _e - _s);                                                                                       \
    } while (0)

#else

#define BSP_IS_DMA_INACCESSIBLE(addr)   (0)
#define BSP_CACHE_CLEAN(addr, len)      ((void)0)
#define BSP_CACHE_INVALIDATE(addr, len) ((void)0)

#endif 

/* 内存分配和释放宏定义 */
/**
 * @brief 从应用字节池分配内存（带等待超时）
 * @param ptr 内存指针变量（将被设置为分配的地址）
 * @param size 要分配的字节数
 * @param wait_option 等待选项（TX_NO_WAIT 或超时值）
 * @return TX_SUCCESS 成功，其他值表示失败
 */
#define BSP_MEM_ALLOC_WAIT(ptr, size, wait_option)                                                                                                   \
    do                                                                                                                                               \
    {                                                                                                                                                \
        UINT _status;                                                                                                                                \
        _status = tx_byte_allocate(&tx_app_byte_pool, (void **)&(ptr), (size), (wait_option));                                                       \
        if (_status != TX_SUCCESS)                                                                                                                   \
        {                                                                                                                                            \
            (ptr) = NULL;                                                                                                                            \
        }                                                                                                                                            \
    } while (0)

/**
 * @brief 释放内存到应用字节池
 * @param ptr 要释放的内存指针
 * @return TX_SUCCESS 成功，其他值表示失败
 */
#define BSP_MEM_FREE(ptr)                                                                                                                            \
    do                                                                                                                                               \
    {                                                                                                                                                \
        if ((ptr) != NULL)                                                                                                                           \
        {                                                                                                                                            \
            tx_byte_release((void *)(ptr));                                                                                                          \
            (ptr) = NULL;                                                                                                                            \
        }                                                                                                                                            \
    } while (0)

#endif // _BSP_DEF_H_
