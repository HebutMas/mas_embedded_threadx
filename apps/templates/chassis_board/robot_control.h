/*
 * @Description: 底盘板机器人主控入口 (模板)
 *
 * 使用说明:
 *   1. 底盘板负责: 接收云台板指令、底盘控制、裁判系统通信
 *   2. 板间通讯离线时自动进入安全模式 (zero_force)
 */

#ifndef _ROBOT_CONTROL_H_
#define _ROBOT_CONTROL_H_

int robot_control_init(void);

#endif // _ROBOT_CONTROL_H_
