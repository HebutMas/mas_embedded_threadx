/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-10-20
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-10-20
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\turn_control\turn_control.h
 * @Description: 转向控制 (mode0 单环, 对齐 luntui1 x_observe.c turn_control)
 */

#ifndef _TURN_CONTROL_H_
#define _TURN_CONTROL_H_

/* 转向控制解算输出 */
typedef struct
{
    float delta_t; /* 差分轮矩 (N·m), 左右轮输出各减 delta_t/2 */
} turn_control_t;

void turn_control_init(void);
void turn_control_update(float turn_set_angle); /* dyaw 模块内自取 (同 LQR_calc 拿 obs 的风格) */
const turn_control_t *turn_control_get(void);
void turn_control_reset(void); /* 掉线: 清输出 (对应 luntui1 flag!=0) */

#endif /* _TURN_CONTROL_H_ */
