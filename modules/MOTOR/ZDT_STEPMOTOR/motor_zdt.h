/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 17:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/ZDT_STEPMOTOR/motor_zdt.h
 * @Description: 协议版本：张大头 Emm_V5.0 步进闭环驱动说明书 Rev1.3（2023/11/20）
 */
#ifndef _MOTOR_ZDT_H_
#define _MOTOR_ZDT_H_

#include "motor_base.h"
#include <stdint.h>

/* CAN 位置命令最大拆包数 */
#define ZDT_CAN_MAX_PKT 2

/* 控制模式 */
typedef enum
{
    ZDT_MODE_SPEED    = 0, /* 速度模式: ref = RPM */
    ZDT_MODE_POSITION = 1, /* 位置模式: 相对脉冲增量 */
} ZDT_Mode_e;

typedef struct
{
    Motor_Base base;

    uint8_t    addr;          /* 从机地址*/
    uint8_t    accel;         /* 加速度档位*/
    uint16_t   pulse_per_rev; /* 每圈脉冲数*/
    ZDT_Mode_e mode;          /* 当前控制模式 */
    uint8_t    enable_state;  /* 使能状态缓存 */

    UART_Device *uart_dev;                 /* UART 传输 */
    Can_Device  *can_dev[ZDT_CAN_MAX_PKT]; /* CAN 传输：每个包号一个设备 */

    /* 位置模式增量 */
    int32_t  pending_pulses;
    uint16_t pending_speed_rpm;
    uint8_t  pending_absolute; /* 0=相对, 1=绝对 */
    uint8_t  pending_sync;     /* 0=立即执行, 1=挂起等待多机同步 */

} ZDT_Motor_t;

/**
 * @brief ZDT 步进电机初始化
 * @param config         电机初始化配置；UART 用 transport_config.uart，CAN 用 transport_config.can.hcan
 * @param addr           从机地址 (1-255, 0=广播)
 * @param pulse_per_rev  每圈脉冲数 (如 3200)
 * @param accel          加速度档位 (0-255, 0=无加减速曲线)
 * @return ZDT_Motor_t 指针, 失败返回 NULL
 */
ZDT_Motor_t *Motor_ZDT_Init(Motor_Init_Config_s *config, uint8_t addr, uint16_t pulse_per_rev, uint8_t accel);

/**
 * @brief 使能电机
 */
void Motor_ZDT_Start(ZDT_Motor_t *motor);

/**
 * @brief 失能电机
 */
void Motor_ZDT_Stop(ZDT_Motor_t *motor);

/**
 * @brief 切换控制模式
 */
void Motor_ZDT_SetMode(ZDT_Motor_t *motor, ZDT_Mode_e mode);

/**
 * @brief 位置模式：设置目标脉冲，由 Apply 阶段发送一次
 * @param motor     电机对象
 * @param pulses    相对模式下为相对上一目标的脉冲增量（有符号，负 = 反向）；绝对模式下为目标位置
 * @param speed_rpm 运行速度 (RPM)，0 时使用默认 300
 * @param absolute  0=相对, 1=绝对（V5.0）
 * @param sync      0=立即执行, 1=挂起（需另发多机同步命令触发）
 */
void Motor_ZDT_SendPosition(ZDT_Motor_t *motor, int32_t pulses, uint16_t speed_rpm, uint8_t absolute, uint8_t sync);

/**
 * @brief 清除位置
 * @param motor 电机对象
 */
void Motor_ZDT_ClearPosition(ZDT_Motor_t *motor);

/**
 * @brief 同步触发：发送多机同步命令,先发挂起的位置命令再调用本函数统一启动
 * @param motor 电机对象
 */
void Motor_ZDT_SyncTrigger(ZDT_Motor_t *motor);

#endif /* _MOTOR_ZDT_H_ */
