/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-01-26 01:14:35
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-18 12:19:41
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/motor_def.h
 * @Description:
 */
#ifndef _MOTOR_DEF_H_
#define _MOTOR_DEF_H_

#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_pwm.h"
#include "lqr.h"
#include "module_offline.h"
#include "pid.h"
#include <stdint.h>

/* 闭环类型 */
typedef enum
{
    OPEN_LOOP            = 0x0, /* 0b0000 - 开环控制       */
    SPEED_LOOP           = 0x2, /* 0b0010 - 速度闭环控制    */
    ANGLE_LOOP           = 0x4, /* 0b0100 - 位置闭环控制    */
    ANGLE_AND_SPEED_LOOP = 0x6, /* 0b0110 - 位置速度双闭环  */
} Closeloop_Type_e;

/* 控制算法类型 */
typedef enum
{
    CONTROL_PID = 0,
    CONTROL_LQR
} Control_Algorithm_Type_e;

/* 传输层类型 */
typedef enum
{
    MOTOR_TRANSPORT_CAN  = 0,
    MOTOR_TRANSPORT_UART = 1,
    MOTOR_TRANSPORT_PWM  = 2,
} Motor_Transport_e;

/* 电机控制设置 (闭环类型, 反转标志, 反馈来源) */
typedef struct
{
    Closeloop_Type_e         loop_type;             /* 闭环类型 */
    Control_Algorithm_Type_e algorithm_type;        /* 控制算法类型 */
    uint8_t                  enableflag;            /* 0=禁用, 1=启用 */
    uint8_t                  motor_reverse_flag;    /* 0=正常反转, 1=反转反转 */
    uint8_t                  feedback_reverse_flag; /* 0=正常反馈, 1=反转反馈 */
    uint8_t                  angle_feedback_source; /* 0=电机反馈, 1=外部反馈 */
    uint8_t                  speed_feedback_source; /* 0=电机反馈, 1=外部反馈 */
} Motor_Setting_s;

/* 电机控制器 */
typedef struct
{
    const float *other_angle_feedback_ptr;
    const float *other_speed_feedback_ptr;

    PIDInstance speed_PID;
    PIDInstance angle_PID;

    LQRInstance lqr;

    float ref;
    float feedforward_torque; /* 前馈力矩 (Nm)，用于补偿 */
    float output;
    float output_torque;
} Motor_Controller_s;

/* 电机类型枚举 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    /* DJI 电机 */
    GM6020_CURRENT,
    GM6020_VOLTAGE,
    M3508,
    M2006,
    /* 达妙电机 */
    DM4310,
    DM6220,
    DM8009,
    DM3507,
    DM3519,
    /* 翎控电机 */
    MG8016,
    /* 舵机 */
    SERVO_GENERIC,
    /* UART 电机 */
    ZDT_STEEP_MOTOR,
} Motor_Type_e;

/* 电机基本信息 */
typedef struct
{
    Motor_Type_e motor_type;
    float        gear_ratio;
    float        torque_constant; /* 减速前扭矩常数 (Nm/A) */
    float        max_torque;      /* 最大力矩 (Nm) */
} Motor_Info_s;

/* 公共测量数据 */
typedef struct
{
    float speed_rad;          /* 角速度 (rad/s) */
    float single_round_angle; /* 单圈角度 (rad) */
    float total_angle;        /* 总角度 (rad)   */
    float torque_nm;          /* 当前力矩 (Nm)  */
} Motor_Measure_s;

/* 控制器初始化配置 */
typedef struct
{
    const float *other_angle_feedback_ptr;
    const float *other_speed_feedback_ptr;

    PID_Init_Config_s speed_PID;
    PID_Init_Config_s angle_PID;

    LQR_Init_Config_s lqr_init;
} Motor_Controller_Init_s;

/*  电机初始化配置
 *  根据 transport 字段选择 transport_config 中的对应成员:
 *    MOTOR_TRANSPORT_CAN  → transport_config.can
 *    MOTOR_TRANSPORT_UART → transport_config.uart
 *    MOTOR_TRANSPORT_PWM  → transport_config.pwm
 */
typedef struct
{
    Motor_Controller_Init_s controller_init_config;
    Motor_Setting_s         setting_init_config;
    Motor_Info_s            motor_init_info;
    Offline_Init_config_t   offline_init_config;

    Motor_Transport_e transport;

    union
    {
        Can_Device_Init_Config_s can;
        UART_Device_init_config  uart;
        PWM_Init_Config          pwm;
    } transport_config;
} Motor_Init_Config_s;

#endif /* _MOTOR_DEF_H_ */
