/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-11 19:45:56
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-10 16:53:41
 * @FilePath: /mas_embedded_threadx/modules/OFFLINE/module_offline.h
 * @Description:
 */
#ifndef _OFFLINE_H_
#define _OFFLINE_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef OFFLINE_WATCHDOG_ENABLE
#define OFFLINE_WATCHDOG_ENABLE 1 /* 启用离线检测看门狗功能 */
#endif
#ifndef OFFLINE_BEEP_ENABLE
#define OFFLINE_BEEP_ENABLE 1 /* 启用离线蜂鸣器功能 */
#endif
#ifndef OFFLINE_TASK_STACK_SIZE
#define OFFLINE_TASK_STACK_SIZE 1024 /* 检测任务栈大小 */
#endif
#ifndef OFFLINE_TASK_PRIORITY
#define OFFLINE_TASK_PRIORITY 6 /* 检测任务优先级 */
#endif

/*  蜂鸣器参数  */
#if defined(STM32H723xx)
#define OFFLINE_BEEP_TUNE_VALUE 100
#define OFFLINE_BEEP_CTRL_VALUE 100
#elif defined(STM32F407xx)
#define OFFLINE_BEEP_TUNE_VALUE 50
#define OFFLINE_BEEP_CTRL_VALUE 100
#endif

#define STATE_ONLINE  0
#define STATE_OFFLINE 1

typedef struct Offline_Device Offline_Device;

/**
 * @brief 设备注册初始化配置
 */
typedef struct
{
    const char *name;       /* 设备名称 (最多 31 字符) */
    uint32_t    timeout_ms; /* 超时时间 (ms) */
    uint8_t     beep_times; /* 蜂鸣次数  */
    uint8_t     enable;     /* 初始启用状态 */
} Offline_Init_config_t;

/**
 * @brief 初始化离线检测模块
 */
int Module_Offline_init(void);

/**
 * @brief 注册离线检测设备
 * @param init 设备初始化配置指针
 * @return 成功返回设备句柄 (Offline_Device *)，失败返回 NULL
 */
Offline_Device *Module_Offline_register(const Offline_Init_config_t *init);

/**
 * @brief 更新设备心跳时间
 */
void Module_Offline_device_update(Offline_Device *dev);

/**
 * @brief 启用设备离线检测
 * @param dev 设备句柄
 */
void Module_Offline_device_enable(Offline_Device *dev);

/**
 * @brief 禁用设备离线检测
 * @param dev 设备句柄
 */
void Module_Offline_device_disable(Offline_Device *dev);

/**
 * @brief 获取设备当前离线状态
 * @param dev 设备句柄
 * @return STATE_ONLINE (0) 或 STATE_OFFLINE (1)，NULL 指针返回 STATE_OFFLINE
 */
uint8_t Module_Offline_get_device_status(Offline_Device *dev);

#endif // _OFFLINE_H_
