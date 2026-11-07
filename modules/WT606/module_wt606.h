/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-01-27 19:55:26
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-21 18:54:24
 * @FilePath: /mas_embedded_threadx/modules/WT606/module_wt606.h
 * @Description: WT606/WT901 IMU 模块 — 串口协议解析
 *
 * 协议格式 (每帧11字节):
 *   0x55 | TYPE | DATA1L | DATA1H | DATA2L | DATA2H | DATA3L | DATA3H | DATA4L | DATA4H | SUM
 *   校验和: SUM = (0x55 + TYPE + 全部8个数据字节) 取低8位, unsigned char
 *   数据转换: DATA = (short)((short)DATAH << 8 | DATAL) — DATAH 必须先强转为 signed short 再移位
 */
#ifndef _MODULE_WT606_H_
#define _MODULE_WT606_H_

#include "bsp_uart.h"
#include "module_offline.h"
#include <stdbool.h>
#include <stdint.h>

// 这里根据需要自定义修改
#ifndef WT606_UART
#define WT606_UART huart1
#endif

#ifndef WT606_TASK_STACK_SIZE
#define WT606_TASK_STACK_SIZE 1024 /* 检测任务栈 (字节) */
#endif
#ifndef WT606_TASK_PRIORITY
#define WT606_TASK_PRIORITY 8 /* 检测任务优先级 */
#endif

#ifndef WT606_OFFLINE_ENABLE
#define WT606_OFFLINE_ENABLE 1 /* 是否开启离线检测 */
#endif

typedef struct
{
    float   acc[3];               /* 加速度 (m/s²)    — 量程 ±16g, g=9.8 */
    float   gyro_dps[3];          /* 角速度 (°/s)     — 量程 ±2000°/s */
    float   gyro_rps[3];          /* 角速度 (rad/s) */
    float   Euler_degree[3];      /* 欧拉角 (°)        — roll / pitch / yaw */
    float   Euler_rad[3];         /* 欧拉角 (rad) */
    float   quat[4];              /* 四元数 [w, x, y, z] */
    float   temperature;          /* 温度 (°C) */
    float   YawTotalAngle_degree; /* yaw 连续总角度 (°) */
    float   YawTotalAngle_rad;    /* yaw 连续总角度 (rad) */
    int32_t YawRoundCount;        /* yaw 过零圈数 */
    float   last_yaw_degree;      /* 上一帧 yaw 角度 (°) */

    uint8_t initialized;

    UART_Device    *uart_dev;
    Offline_Device *offline_dev;
} Module_WT606_Device_t;

int Module_WT606_Init(void);

const Module_WT606_Device_t *Module_WT606_Get(void);

uint8_t Module_WT606_Get_offline_state(void);

#endif // _MODULE_WT606_H_
