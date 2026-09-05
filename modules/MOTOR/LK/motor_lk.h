#ifndef _MOTOR_LK_H_
#define _MOTOR_LK_H_

#include "motor_base.h"

/* 翎控电机测量数据 */
typedef struct {
    int8_t   temperature ;            /*温度*/
    int16_t  iq ;                     /*转矩电流（除MS）*/
    uint16_t power;                   /*电机的输出功率值（仅MS）*/
    int16_t  speed;                   /*电机速度(电机轴端)*/
    uint16_t encoder;                 /*编码器位置(电机轴端, 位数依据电机而定)*/
    float    last_single_round_angle; /*上一周期单圈角度(电机轴端, rad)*/
    int32_t  total_round;             /*电机轴端累计圈数: 单圈角每跨 2π 边界 ±1*/
    uint8_t  first_frame;             /*首帧标志: 0=未接收, 1=已接收*/
} LK_Measure_s;

/* LK_Motor_t — 继承 Motor_Base */
typedef struct {
    Motor_Base   base;         // [必须首字段]
    LK_Measure_s measure;      // 扩展测量数据
    uint32_t     mode_type;    // 控制帧模式
} LK_Motor_t;

/* 声明 API */
LK_Motor_t *Motor_LK_Init(Motor_Init_Config_s *config, uint32_t LK_Mode_type);

#endif
