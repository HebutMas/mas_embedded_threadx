/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-04 12:11:52
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-05 08:38:28
 * @FilePath: /mas_embedded_threadx/board/bsp/UART/bsp_uart.h
 * @Description:
 */
#ifndef _BSP_UART_H_
#define _BSP_UART_H_

#include <stdint.h>

#if defined(STM32H723xx)
#define BSP_UART_DEVICE_NUM 6
#elif defined(STM32F407xx)
#define BSP_UART_DEVICE_NUM 3
#elif defined(STM32F105xC) || defined(STM32F103xB)
#define BSP_UART_DEVICE_NUM 3
#endif

typedef struct UART_Device UART_Device;

// 模式选择
typedef enum
{
    UART_MODE_BLOCKING,
    UART_MODE_IT,
    UART_MODE_DMA
} UART_Mode;

// 初始化配置结构体
typedef struct
{
    void     *huart;           // UART句柄 (UART_HandleTypeDef*)
    uint8_t  *rx_buf;          // RX缓冲区（2的幂）
    uint32_t  rx_buf_size;     // RX缓冲区总大小
    uint32_t  expected_rx_len; // 期望接收的数据包长度(0为不定长)
    UART_Mode rx_mode;         // 接收模式
    UART_Mode tx_mode;         // 发送模式
} UART_Device_init_config;

/**
 * @brief UART初始化
 * @param config 初始化配置
 * @return UART_Device* 成功返回设备指针，失败返回NULL
 */
UART_Device *BSP_UART_Device_Init(UART_Device_init_config *config);

/**
 * @brief UART发送（阻塞至DMA完成或超时）
 * @param device UART设备
 * @param data   发送数据指针（DMA模式下需保持有效至发送完成）
 * @param len    数据长度
 * @param timeout 超时时间
 * @return >0：实际发送字节数，-1：失败
 */
int BSP_UART_Send(UART_Device *device, uint8_t *data, uint32_t len, uint32_t timeout);

/**
 * @brief UART接收
 * @param device   UART设备
 * @param buf      接收数据缓冲区
 * @param buf_size 接收缓冲区容量 (字节), 实际读取不会超过此值
 * @param rx_len   输出实际接收数据长度 (字节)
 * @param timeout  超时时间
 * @return >0：实际接收字节数，-1：失败/超时
 */
int BSP_UART_Read(UART_Device *device, uint8_t *buf, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout);

#endif // _BSP_UART_H_
