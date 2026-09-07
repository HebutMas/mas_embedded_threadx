/*
 * @file        vofa.h
 * @brief       VOFA (Visualization of Floats Add-on) 串口波形协议模块
 *              协议核心 + 模块入口(Module_VOFA_Init 建 RX/TX 双线程)
 *              配置宏见下方顶部 VOFA_UART / VOFA_FORMAT / VOFA_TX_INTERVAL_MS
 */
#ifndef _VOFA_H_
#define _VOFA_H_

#include <stdint.h>
#include <stddef.h>
#include "bsp_uart.h"

/* ================================================================
 *                  █ 模块配置 (最显眼处, 按需覆盖)
 *
 * 默认值可直接改这里; 也可经 apps/<robot>/robot.cmake 的 set()
 * 注入 module_config.h 覆盖(define 同名宏会先于此生效).
 * ================================================================ */

/* 使用的串口句柄 */
#ifndef VOFA_UART
#define VOFA_UART huart6
#endif

/* 协议格式: 0 = JustFloat(二进制), 1 = FireWater(文本) */
#ifndef VOFA_FORMAT
#define VOFA_FORMAT 0
#endif

/* FireWater 文本协议的前缀字符串 */
#ifndef VOFA_FIREWATER_PREFIX
#define VOFA_FIREWATER_PREFIX "vofa:"
#endif

/* TX 发送周期(ms): 模块发送线程每隔该时长自动调用 VOFA_Send() 一次 */
#ifndef VOFA_TX_INTERVAL_MS
#define VOFA_TX_INTERVAL_MS 10
#endif

/* RX/TX 线程栈与优先级 */
#ifndef VOFA_TASK_STACK_SIZE
#define VOFA_TASK_STACK_SIZE 1024
#endif
#ifndef VOFA_TASK_PRIORITY
#define VOFA_TASK_PRIORITY 11
#endif

/* ========== 内部参数 ========== */
#define VOFA_STRING_DATA_LEN        20          // 字符串数据长度(参数名)
#define VOFA_TX_BUFFER_SIZE         64          // 发送缓冲区大小

/* ========== 返回状态 ========== */
typedef enum
{
    VOFA_OK   = 0,
    VOFA_ERR  = 1,
} vofa_err_t;

/* ========== VOFA协议格式定义 (与 VOFA_FORMAT 宏对应) ========== */
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
 *        内部: 初始化 UART + 创建 RX(接收)/TX(发送) 双线程 (模块自建任务范式)
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
 *        模块 TX 线程会按 VOFA_TX_INTERVAL_MS 周期自动调用, 一般无需手动调
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

/* ================================================================
 *                  █ 使用示例
 *
 *   (1) 启用模块: 在 apps/<robot>/robot.cmake 的模块列表中加入 VOFA
 *       set(MODULES_SINGLE OFFLINE VOFA)
 *       (模块默认不启用; Module_VOFA_Init 会在 MODULE_Init 阶段自动执行,
 *        内部完成 UART 初始化 + 收发双线程创建)
 *
 *   (2) 应用层注册收发参数 (在任意 init 中调用, 推荐在 robot_control_init):
 *
 *       #include "vofa.h"
 *       #include "module_offline.h"
 *
 *       static float g_param = 0.0f;      // 可被下行 set_param 更新的变量
 *       static float g_sin   = 0.0f;      // 上行正弦
 *
 *       static void on_isonline(void)     // 收到 isonline → 喂心跳
 *       {
 *           Module_Offline_device_update(g_offline_dev);
 *       }
 *
 *       // robot_control_init() 中:
 *       g_offline_dev = Module_Offline_register(&(Offline_Init_config_t){
 *           .name="vofa_link", .timeout_ms=500, .enable=1});
 *
 *       VOFA_RegisterRx("isonline", &g_param, on_isonline); // 下行→喂狗
 *       VOFA_RegisterRx("set_param", &g_param, NULL);       // 下行→写变量
 *
 *       VOFA_RegisterTx("sin",   &g_sin);   // 上行正弦(线程内自己更新)
 *       VOFA_RegisterTx("param", &g_param); // 上行 param 当前值
 *
 *       // 之后无需手动发送: 模块 TX 线程每 VOFA_TX_INTERVAL_MS 自动发一次
 *
 *   (3) 上位机:
 *       - 串口波特率与 UART 一致, 接 VOFA_UART 对应引脚
 *       - 选择 Just Float 协议(默认) → 看到 "sin"+"param" 两条曲线
 *       - 下行发送 "isonline"  保持连接(喂心跳, 防掉线)
 *       - 下行发送 "set_param:3.14"  → param 曲线变化到 3.14
 * ================================================================ */

#endif /* _VOFA_H_ */
