/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 17:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/ZDT_STEPMOTOR/motor_zdt.h
 * @Description:
 */
#ifndef _MOTOR_ZDT_H_
#define _MOTOR_ZDT_H_

#include "motor_base.h"
#include "bsp_uart.h"
#include <stdint.h>

/* 控制模式 */
typedef enum
{
    UART_MODE_SPEED    = 0, /* 速度模式: ref = RPM */
    UART_MODE_POSITION = 1, /* 位置模式: ref = 脉冲数 */
} UART_Mode_e;

typedef struct
{
    Motor_Base   base;
    UART_Device *uart_dev;
    uint8_t      addr;          /* 从机地址 (1-255, 0=广播) */
    uint8_t      tx_buf[16];    /* 发送缓冲区 */
    uint8_t      rx_buf[16];    /* 接收缓冲区 */
    uint8_t      enable_state;  /* 使能状态缓存 */
    uint16_t     pulse_per_rev; /* 每圈脉冲数 (默认 3200 = 16细分) */
    uint8_t      accel;         /* 加速度档位 (0-255, 0=无曲线) */
    UART_Mode_e  mode;          /* 控制模式 (速度/位置) */

    float position_rad; /* 实时位置 (rad) */
    float velocity_rpm; /* 实时转速 (RPM) */
} UART_Motor_t;

/**
 * @brief ZDT 步进电机初始化
 * @param config         电机初始化配置 (transport 必须为 MOTOR_TRANSPORT_UART)
 * @param addr           从机地址 (1-255, 0=广播)
 * @param pulse_per_rev  每圈脉冲数 (如 3200 = 16细分)
 * @param accel          加速度档位 (0-255, 0=无加减速曲线)
 * @return UART_Motor_t 指针, 失败返回 NULL
 */
UART_Motor_t *Motor_ZDT_Init(Motor_Init_Config_s *config, uint8_t addr, uint16_t pulse_per_rev, uint8_t accel);

/**
 * @brief ZDT 电机使能 (发送使能命令并开始控制)
 */
void Motor_ZDT_Start(UART_Motor_t *motor);

/**
 * @brief ZDT 电机停止 (发送停止命令并清零使能标志)
 */
void Motor_ZDT_Stop(UART_Motor_t *motor);

/**
 * @brief 切换控制模式
 * @param mode  UART_MODE_SPEED=速度模式, UART_MODE_POSITION=位置模式
 */
void Motor_ZDT_SetMode(UART_Motor_t *motor, UART_Mode_e mode);

#endif /* _MOTOR_ZDT_H_ */
