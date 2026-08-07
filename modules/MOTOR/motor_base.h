/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 17:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/motor_base.h
 * @Description:
 */
#ifndef _MOTOR_BASE_H_
#define _MOTOR_BASE_H_

#include "module_offline.h"
#include "motor_def.h"
#include <stdint.h>

typedef struct Motor_Base Motor_Base;

struct Motor_Base
{
    Motor_Base       *next;
    Motor_Type_e      type;
    Motor_Transport_e transport;

    Motor_Info_s       info;
    Motor_Setting_s    setting;
    Motor_Controller_s controller;
    Motor_Measure_s    measure;
    Offline_Device    *offline_dev;

    void *transport_dev; /* 底层设备句柄 (Can_Device / UART_Device / PWM_Device) */

    /* 输出应用 → 传输层 (协议层实现) */
    void (*Apply)(Motor_Base *motor);
};

#define MOTOR_GET_DERIVED(base_ptr, derived_type) ((derived_type *)(base_ptr))

/**
 * @brief 注册电机
 * @param motor 电机实例指针
 */
void Motor_Register(Motor_Base *motor);

/**
 * @brief 控制计算，统一执行: 离线/禁用 → ZeroState; 开环 → 跳过; 闭环 → 算法计算
 */
void Motor_ControlAll(void);

/**
 * @brief 输出应用
 */
void Motor_ApplyAll(void);
/**
 * @brief 启动电机
 */
void Motor_Start(Motor_Base *motor);
/**
 * @brief 停止电机
 */
void Motor_Stop(Motor_Base *motor);
/**
 * @brief 设置电机参考值
 */
void Motor_SetRef(Motor_Base *motor, float ref);
/**
 * @brief 改变电机反馈源
 */
void Motor_ChangeFeed(Motor_Base *motor, Closeloop_Type_e loop, uint8_t feedback_source);
/**
 * @brief 设置电机外环控制模式
 */
void Motor_OuterLoop(Motor_Base *motor, Closeloop_Type_e outer_loop);
/**
 * @brief 设置电机前馈扭矩
 */
void Motor_SetForwardTorque(Motor_Base *motor, float torque);
/**
 * @brief 设置电机输出扭矩
 */
void Motor_SetOutputTorque(Motor_Base *motor, float torque);

#endif /* _MOTOR_BASE_H_ */
