/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 17:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/DJI/POWER_CONTROL/power_control.h
 * @Description: 
 */
#ifndef _POWER_CONTROL_H_
#define _POWER_CONTROL_H_

#include "motor_base.h"
#include <stdint.h>

#define POWER_CTRL_MAX_MOTORS 8

/* 功率控制角色 */
typedef enum
{
    PC_ROLE_NONE  = 0,
    PC_ROLE_DRIVE = 1, /* 驱动轮 */
    PC_ROLE_STEER = 2, /* 舵向轮 */
} PowerCtrl_Role_e;

/* 损耗模型参数: P = tau*omega + k1*|omega| + k2*tau^2 + k3 */
typedef struct
{
    float k1, k2, k3;
} PowerCtrl_Param_t;

/**
 * @brief 注册电机参与功率控制
 * @param motor Motor_Base 指针
 * @param role  角色 (PC_ROLE_NONE 取消注册)
 * @param param 损耗模型系数
 */
void PowerControl_Register(Motor_Base *motor, PowerCtrl_Role_e role, PowerCtrl_Param_t param);

/**
 * @brief 设置裁判系统功率上限
 */
void PowerControl_SetLimit(float power_limit_w, float buffer_energy_j, uint8_t use_buffer);

/**
 * @brief 执行功率分配
 * @note  在 Motor_ControlAll() 之后、Motor_ApplyAll() 之前调用
 */
void PowerControl_Update(void);

#endif /* _POWER_CONTROL_H_ */
