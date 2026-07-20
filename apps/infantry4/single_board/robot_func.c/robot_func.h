#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_

#include "infantry_def.h"

/**
 * @brief 计算相对于对齐角度的最小旋转角度，返回编码器差值
 * @param getyawangle 当前编码器值 (0 - 8191)
 * @return 归一化后的编码器差值 (范围 -4095 ～ 4095)，符号表示旋转方向
 */
int16_t CalcOffsetAngle(float getyawangle);

/**
 * @brief 根据遥控器输入设置底盘、云台和发射机构的控制命令
 * @param Chassis_Ctrl 底盘控制命令结构体指针
 * @param Shoot_Ctrl 发射机构控制命令结构体指针
 * @param Gimbal_Ctrl 云台控制命令结构体指针
 */
void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl);

#endif // _ROBOT_FUNC_H_