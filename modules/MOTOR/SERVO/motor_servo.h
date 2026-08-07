/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 17:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/SERVO/motor_servo.h
 * @Description: 
 */
#ifndef _MOTOR_SERVO_H_
#define _MOTOR_SERVO_H_

#include "motor_base.h"
#include "bsp_pwm.h"
#include <stdint.h>

/* Servo_Motor_t — 继承 Motor_Base */
typedef struct
{
    Motor_Base  base;              /* [必须首字段] 公共基类 */
    PWM_Device *pwm_dev;           /* PWM 设备句柄 */
    uint16_t    min_pulse;         /* 最小脉宽 (对应最小角度) */
    uint16_t    max_pulse;         /* 最大脉宽 (对应最大角度)  */
    float       min_angle_deg;     /* 最小角度 (度) */
    float       max_angle_deg;     /* 最大角度 (度) */
} Servo_Motor_t;

/**
 * @brief 舵机初始化
 * @param config      电机初始化配置 (transport 必须为 MOTOR_TRANSPORT_PWM)
 * @param min_pulse   最小角度对应脉宽 (如 500)
 * @param max_pulse   最大角度对应脉宽 (如 2500)
 * @param min_angle   最小角度 (度, 如 0.0)
 * @param max_angle   最大角度 (度, 如 180.0)
 * @return Servo_Motor_t 指针, 失败返回 NULL
 */
Servo_Motor_t *Motor_Servo_Init(Motor_Init_Config_s *config, uint16_t min_pulse, uint16_t max_pulse, float min_angle, float max_angle);

#endif /* _MOTOR_SERVO_H_ */
