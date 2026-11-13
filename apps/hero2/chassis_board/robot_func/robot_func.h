#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_

#include "hero2_def.h"

/**
 * @brief 根据遥控器输入设置底盘控制命令
 * @param cmd 底盘控制命令结构体指针
 */
void RemoteControlSet(Chassis_Ctrl_Cmd_t *cmd);

#endif
