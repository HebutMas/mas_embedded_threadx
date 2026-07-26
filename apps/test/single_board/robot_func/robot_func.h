/*
 * @Description: 机器人通用函数模板 (单板)
 *
 * 包含:
 *   - CalcOffsetAngle(): 计算云台与底盘对齐的编码器差值
 *   - RemoteControlSet(): 遥控器输入 → 底盘/云台/发射控制命令
 *
 * 使用说明:
 *   1. 将 <robot>_def.h 改为你的 def 文件
 *   2. 根据遥控器通道自定义按键映射
 *   3. 遥控器离线时默认停止所有执行器
 */

#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_

#include "test_def.h" 

/**
 * @brief 计算 yaw 编码器与底盘对齐点的最小旋转差值
 * @param getyawangle 当前 yaw 编码器值 (0 ~ 8191)
 * @return 归一化编码器差值 (-4095 ~ 4095)
 */
int16_t CalcOffsetAngle(float getyawangle);

/**
 * @brief 遥控器输入 → 底盘/发射/云台 控制命令
 */
void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl,
                      Shoot_Ctrl_Cmd_t  *Shoot_Ctrl,
                      Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl);

#endif // _ROBOT_FUNC_H_
