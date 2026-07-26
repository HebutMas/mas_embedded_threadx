/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-04 12:11:52
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-05 08:38:28
 * @FilePath: /mas_embedded_threadx/board/bsp/SPI/bsp_spi.h
 * @Description:
 */
#ifndef _BSP_SPI_H_
#define _BSP_SPI_H_

#include <stdint.h>

#if defined(STM32H723xx)
#define SPI_BUS_NUM 4
#elif defined(STM32F407xx)
#define SPI_BUS_NUM 3
#elif defined(STM32F105xC) || defined(STM32F103xB)
#define SPI_BUS_NUM 2
#endif

typedef struct SPI_Device SPI_Device;

/* 传输模式 */
typedef enum
{
    SPI_MODE_BLOCKING,
    SPI_MODE_IT,
    SPI_MODE_DMA
} SPI_Mode;

/* 初始化配置 */
typedef struct
{
    void    *hspi;    /* SPI HAL 句柄 (SPI_HandleTypeDef *) */
    void    *cs_port; /* 片选 GPIO 端口，NULL = 无 CS */
    uint16_t cs_pin;  /* 片选引脚号，0 = 无 CS */
    SPI_Mode tx_mode; /* 发送模式 */
    SPI_Mode rx_mode; /* 接收模式 */
} SPI_Device_Init_Config;

/**
 * @description:  初始化SPI设备
 * @param {SPI_Device_Init_Config*} config
 * @return {SPI_Device*} 成功返回设备实例指针，失败返回NULL
 */
SPI_Device *BSP_SPI_Device_Init(SPI_Device_Init_Config *config);
/**
 * @description: SPI发送数据
 * @param {SPI_Device*} dev，要操作的SPI设备实例
 * @param {uint8_t*} tx_data，发送数据指针
 * @param {uint16_t} size，数据大小
 * @param {uint32_t} timeout，超时时间
 * @return {uint8_t} 返回操作状态,0表示成功，其他值表示失败
 */
uint8_t BSP_SPI_Transmit(SPI_Device *dev, const uint8_t *tx_data, uint16_t size, uint32_t timeout);
/**
 * @description: SPI接收数据
 * @param {SPI_Device*} dev，要操作的SPI设备实例
 * @param {uint8_t*} rx_data，接收数据指针
 * @param {uint16_t} size，数据大小
 * @param {uint32_t} timeout，超时时间
 * @return {uint8_t} 返回操作状态,0表示成功，其他值表示失败
 */
uint8_t BSP_SPI_Receive(SPI_Device *dev, uint8_t *rx_data, uint16_t size, uint32_t timeout);
/**
 * @description: SPI发送和接受数据
 * @param {SPI_Device*} dev，要操作的SPI设备实例
 * @param {uint8_t*} tx_data，发送数据指针
 * @param {uint8_t*} rx_data，接收数据指针
 * @param {uint16_t} size，数据大小，注意这里是发送和接收同时进行，所以发送和接收的数据量是相等的
 * @param {uint32_t} timeout，超时时间
 * @return {uint8_t} 返回操作状态,0表示成功，其他值表示失败
 */
uint8_t BSP_SPI_TransReceive(SPI_Device *dev, const uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout);
/**
 * @description: SPI连续发送两次数据
 * @param {SPI_Device*} dev，要操作的SPI设备实例
 * @param {const uint8_t*} tx_data1，第一次发送的数据指针
 * @param {uint16_t} size1，第一次发送的数据大小
 * @param {const uint8_t*} tx_data2，第二次发送的数据指针
 * @param {uint16_t} size2，第二次发送的数据大小
 * @param {uint32_t} timeout，超时时间
 * @return {uint8_t} 返回操作状态,0表示成功，其他值表示失败
 */
uint8_t BSP_SPI_TransAndTrans(SPI_Device *dev, const uint8_t *tx_data1, uint16_t size1, const uint8_t *tx_data2, uint16_t size2, uint32_t timeout);
/**
 * @description: SPI连续接收两次数据
 * @param {SPI_Device*} dev，要操作的SPI设备实例
 * @param {uint8_t*} rx_data1，第一次接收的数据指针
 * @param {uint16_t} size1，第一次接收的数据大小
 * @param {uint8_t*} rx_data2，第二次接收的数据指针
 * @param {uint16_t} size2，第二次接收的数据大小
 * @param {uint32_t} timeout，超时时间
 * @return {uint8_t} 返回操作状态,0表示成功，其他值表示失败
 */
uint8_t BSP_SPI_ReceAndRece(SPI_Device *dev, uint8_t *rx_data1, uint16_t size1, uint8_t *rx_data2, uint16_t size2, uint32_t timeout);

#endif /* _BSP_SPI_H_ */
