#ifndef _HERO2_DEF_H_
#define _HERO2_DEF_H_

/* 底盘遥控命令 */
typedef struct
{
    float body_target_dx; /* 前后速度命令 m/s, 上推为正, ±2.0 */
    float turn_cmd;       /* 转向设定角命令 (rad), CH4, 满推 ±0.15 (符号实车修正) */
} Chassis_Ctrl_Cmd_t;

#endif
