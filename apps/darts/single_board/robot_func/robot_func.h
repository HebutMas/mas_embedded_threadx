/*
 * @Description: 飞镖机器人通用函数 (单板)
 *
 * 包含:
 *   - RemoteControlSet(): 遥控器输入 → 云台/发射控制命令
 *
 * 使用说明:
 *   1. 根据遥控器通道自定义按键映射
 *   2. 遥控器离线时默认停止所有执行器
 */

#ifndef _DARTS_ROBOT_FUNC_H_
#define _DARTS_ROBOT_FUNC_H_

#include "darts_def.h"
#include <stdbool.h>

/**
 * @brief 遥控器输入 → 云台/发射控制命令
 * @return 本次读取的遥控器在线状态
 */
bool RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl);

#endif // _DARTS_ROBOT_FUNC_H_
