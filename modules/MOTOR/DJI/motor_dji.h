/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 17:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/DJI/motor_dji.h
 * @Description:
 */
#ifndef _MOTOR_DJI_H_
#define _MOTOR_DJI_H_

#include "motor_base.h"
#include <stdint.h>

typedef struct
{
    uint16_t last_ecd;     /* 上一次编码器值 */
    uint16_t ecd;          /* 0-8191 */
    float    speed_rpm;    /* 角速度 (RPM) */
    int16_t  real_current; /* 实际电流 (A) */
    uint8_t  temperature;  /* 温度 (℃) */
    int32_t  total_round;  /* 总圈数 */
} DJI_Measure_s;

/* DJI_Motor_t — 继承 Motor_Base */
typedef struct
{
    Motor_Base    base;         /* [必须首字段] 公共基类 */
    DJI_Measure_s measure;      /* DJI电机数据 */
    uint8_t       sender_group; /* CAN 发送分组 */
    uint8_t       message_num;  /* 组内序号 0-3 */
} DJI_Motor_t;

/**
 * @brief DJI 电机初始化
 * @param config 电机初始化配置 (transport 必须为 MOTOR_TRANSPORT_CAN)
 * @return DJI_Motor_t 指针, 失败返回 NULL
 */
DJI_Motor_t *Motor_DJI_Init(Motor_Init_Config_s *config);

#endif /* _MOTOR_DJI_H_ */
