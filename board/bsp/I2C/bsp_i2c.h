/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-08 10:44:28
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-05 09:00:00
 * @FilePath: /mas_embedded_threadx/board/bsp/I2C/bsp_i2c.h
 * @Description:
 */
#ifndef _BSP_I2C_H_
#define _BSP_I2C_H_

#include <stdint.h>

#if defined(STM32H723xx)
#define I2C_BUS_NUM 4
#elif defined(STM32F407xx)
#define I2C_BUS_NUM 3
#elif defined(STM32F105xC) || defined(STM32F103xB)
#define I2C_BUS_NUM 2
#endif

/* 对外不透明句柄 */
typedef struct I2C_Device I2C_Device;

/* 传输模式 */
typedef enum
{
    I2C_MODE_BLOCKING,
    I2C_MODE_IT,
    I2C_MODE_DMA
} I2C_Mode;

/* 初始化配置 */
typedef struct
{
    void    *hi2c;        /* I2C HAL 句柄 (I2C_HandleTypeDef *) */
    uint16_t dev_address; /* 设备地址（STM32 中为左移 1 位的地址） */
    I2C_Mode tx_mode;     /* 发送模式 */
    I2C_Mode rx_mode;     /* 接收模式 */
} I2C_Device_Init_Config;

/**
 * @description: 初始化 I2C 设备
 * @param {I2C_Device_Init_Config*} config
 * @return {I2C_Device*} 成功返回设备实例指针，失败返回 NULL
 */
I2C_Device *BSP_I2C_Device_Init(I2C_Device_Init_Config *config);

/**
 * @description: I2C 发送数据
 * @param {I2C_Device*} dev，要操作的 I2C 设备实例
 * @param {const uint8_t*} tx_data，发送数据指针
 * @param {uint16_t} size，数据大小
 * @param {uint32_t} timeout，超时时间
 * @return {uint8_t} 返回操作状态，0 表示成功，其他值表示失败
 */
uint8_t BSP_I2C_Transmit(I2C_Device *dev, const uint8_t *tx_data, uint16_t size, uint32_t timeout);

/**
 * @description: I2C 接收数据
 * @param {I2C_Device*} dev，要操作的 I2C 设备实例
 * @param {uint8_t*} rx_data，接收数据指针
 * @param {uint16_t} size，数据大小
 * @param {uint32_t} timeout，超时时间
 * @return {uint8_t} 返回操作状态，0 表示成功，其他值表示失败
 */
uint8_t BSP_I2C_Receive(I2C_Device *dev, uint8_t *rx_data, uint16_t size, uint32_t timeout);

/**
 * @description: I2C 内存写操作（指定寄存器地址后写入数据）
 * @param {I2C_Device*} dev，要操作的 I2C 设备实例
 * @param {uint16_t} mem_address，内存/寄存器地址
 * @param {uint16_t} mem_add_size，地址宽度：I2C_MEMADD_SIZE_8BIT 或 I2C_MEMADD_SIZE_16BIT
 * @param {const uint8_t*} data，数据指针
 * @param {uint16_t} size，数据大小
 * @param {uint32_t} timeout，超时时间
 * @return {uint8_t} 返回操作状态，0 表示成功，其他值表示失败
 */
uint8_t BSP_I2C_Mem_Write(I2C_Device *dev, uint16_t mem_address, uint16_t mem_add_size, const uint8_t *data, uint16_t size, uint32_t timeout);

/**
 * @description: I2C 内存读操作（指定寄存器地址后读取数据）
 * @param {I2C_Device*} dev，要操作的 I2C 设备实例
 * @param {uint16_t} mem_address，内存/寄存器地址
 * @param {uint16_t} mem_add_size，地址宽度：I2C_MEMADD_SIZE_8BIT 或 I2C_MEMADD_SIZE_16BIT
 * @param {uint8_t*} data，数据指针
 * @param {uint16_t} size，数据大小
 * @param {uint32_t} timeout，超时时间
 * @return {uint8_t} 返回操作状态，0 表示成功，其他值表示失败
 */
uint8_t BSP_I2C_Mem_Read(I2C_Device *dev, uint16_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout);

#endif /* _BSP_I2C_H_ */
