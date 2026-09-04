/*
 * @file        vofa.h
 * @brief       VOFA (Visualization of Floats Add-on) 串口波形协议模块
 *              RM2025 → RM2027 移植版
 *              修改:
 *               1. osal_status_t → vofa_err_t (去 OSAL 依赖)
 *               2. 发送/接收帧注册表: 静态数组 → 动态链表 (BSP_MEM_ALLOC_WAIT)
 *               3. BSP_UART 调用适配 RM2027 签名 (Send 带 timeout, Read 缓冲+长度)
 * @note        协议核心 + 模块入口合并于 vofa.c (含 Module_VOFA_Init)
 */
#ifndef _VOFA_H_
#define _VOFA_H_

#include <stdint.h>
#include <stddef.h>
#include "bsp_uart.h"

/* ========== 配置参数 ========== */
#define VOFA_STRING_DATA_LEN        20          // 字符串数据长度(参数名)
#define VOFA_TX_BUFFER_SIZE         64          // 发送缓冲区大小

/* ========== 返回状态 ========== */
typedef enum
{
    VOFA_OK   = 0,
    VOFA_ERR  = 1,
} vofa_err_t;

/* ========== VOFA协议格式定义 ========== */
typedef enum
{
    VOFA_FORMAT_JUSTFLOAT = 0,  // just float格式: 浮点数数组 + 帧尾
    VOFA_FORMAT_FIREWATER = 1   // fire water格式: 字符串前缀 + sprintf格式化浮点数
} VOFA_Format;

/* ========== VOFA帧定义 ========== */
#define VOFA_JUSTFLOAT_TAIL_LEN     4           // just float帧尾长度
#define VOFA_JUSTFLOAT_TAIL         {0x00,0x00,0x80,0x7f}  // just float帧尾
#define VOFA_FIREWATER_MAX_LEN      50          // fire water缓冲区大小
#define VOFA_JUSTFLOAT_MAX_LEN      50          // just float缓冲区大小

/* 接收回调类型 */
typedef void (*VOFA_RxCallback)(void);

/* =============== 模块入口 =============== */

/**
 * @brief 初始化 VOFA 模块 (由 modules/module_init.c 在 MODULE_VOFA 开启时调用)
 *        内部: 初始化 UART6 + 创建本模块接收线程 (模块自建任务范式)
 */
void Module_VOFA_Init(void);

/* =============== API 函数声明 =============== */

/**
 * @brief 初始化VOFA设备
 * @param uart_dev UART设备指针
 * @param format 协议格式 (VOFA_FORMAT_JUSTFLOAT 或 VOFA_FORMAT_FIREWATER)
 * @param firewater_prefix firewater格式的字符串前缀
 * @return vofa_err_t (VOFA_OK/VOFA_ERR)
 */
vofa_err_t VOFA_Init(UART_Device *uart_dev, VOFA_Format format, const char *firewater_prefix);

/**
 * @brief 注册待发送数据 (动态分配节点, 入发送链表)
 * @param name 数据名称字符串（如 "sin"）
 * @param float_ptr 浮点数指针地址
 * @return vofa_err_t
 */
vofa_err_t VOFA_RegisterTx(const char *name, float *float_ptr);

/**
 * @brief 注册接收数据 (动态分配节点, 入接收链表)
 * @param name 数据名称字符串（如 "freq"）
 * @param float_ptr 浮点数指针地址（用于存储解析结果）
 * @param callback 接收完成回调函数（可选, NULL则不调用）
 * @return vofa_err_t
 */
vofa_err_t VOFA_RegisterRx(const char *name, float *float_ptr, VOFA_RxCallback callback);

/**
 * @brief 通用发送函数 - 根据当前格式自动选择发送方式
 * @return vofa_err_t
 */
vofa_err_t VOFA_Send(void);

/**
 * @brief 接收数据处理函数 (由模块接收线程周期调用)
 * 从UART读取接收数据，解析参数名和浮点数，匹配注册项后自动赋值
 */
void VOFA_ReceiveHandler(void);

/**
 * @brief 设置VOFA工作格式
 * @param format VOFA_FORMAT_JUSTFLOAT / VOFA_FORMAT_FIREWATER
 * @return vofa_err_t
 */
vofa_err_t VOFA_SetFormat(VOFA_Format format);

#endif /* _VOFA_H_ */
