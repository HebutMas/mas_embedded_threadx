/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 17:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/DAMIAO/motor_damiao.h
 * @Description:
 */
#ifndef _MOTOR_DAMIAO_H_
#define _MOTOR_DAMIAO_H_

#include "motor_base.h"
#include "motor_def.h"
#include <stdint.h>

/* 达妙电机控制模式 */
#define DM_MIT_MODE 0x000
#define DM_POS_MODE 0x100
#define DM_SPD_MODE 0x200
#define DM_PSI_MODE 0x300

/* 达妙电机错误码 */
typedef enum
{
    DM_NO_ERROR               = 0X00,
    OVERVOLTAGE_ERROR         = 0X08,
    UNDERVOLTAGE_ERROR        = 0X09,
    OVERCURRENT_ERROR         = 0X0A,
    MOS_OVERTEMP_ERROR        = 0X0B,
    MOTOR_COIL_OVERTEMP_ERROR = 0X0C,
    COMMUNICATION_LOST_ERROR  = 0X0D,
    OVERLOAD_ERROR            = 0X0E,
} DMMotorError_t;

/* 达妙电机命令 */
typedef enum
{
    DM_CMD_MOTOR_START   = 0xfc,
    DM_CMD_MOTOR_STOP    = 0xfd,
    DM_CMD_ZERO_POSITION = 0xfe,
    DM_CMD_CLEAR_ERROR   = 0xfb,
} DMMotor_Mode_e;

/* 达妙电机参数配置 */
typedef struct
{
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
} DM_Motor_Params_t;

/* 达妙电机测量数据 */
typedef struct
{
    uint8_t        id;                      /* 电机ID */
    uint8_t        state;                   /* 电机状态 */
    float          last_single_round_angle; /* 上一次单圈角度(rad) */
    float          torque;                  /* 输出力矩(Nm) */
    float          T_Mos;                   /* MOS 温度 (℃) */
    float          T_Rotor;                 /* 线圈温度 (℃) */
    int32_t        total_round;             /* 总圈数 */
    DMMotorError_t Error_Code;              /* 错误码 */
} DM_Motor_Measure_s;

/* DM_Motor_t — 继承 Motor_Base */
typedef struct
{
    Motor_Base               base;    /* [必须首字段] 公共基类 */
    DM_Motor_Measure_s       measure; /* 达妙测量数据 */
    uint32_t                 mode_type;
    const DM_Motor_Params_t *params; /* 电机参数配置指针 */
} DM_Motor_t;

/**
 * @brief 发送控制命令 (启动/停止/清零/清除错误)
 */
void Motor_DM_Cmd(DM_Motor_t *motor, DMMotor_Mode_e cmd);

/**
 * @brief 达妙电机初始化
 * @param config       电机初始化配置 (transport 必须为 MOTOR_TRANSPORT_CAN)
 * @param DM_Mode_type 电机工作模式 (DM_MIT_MODE / DM_POS_MODE / ...)
 * @return DM_Motor_t 指针, 失败返回 NULL
 */
DM_Motor_t *Motor_DM_Init(Motor_Init_Config_s *config, uint32_t DM_Mode_type);

#endif /* _MOTOR_DAMIAO_H_ */
