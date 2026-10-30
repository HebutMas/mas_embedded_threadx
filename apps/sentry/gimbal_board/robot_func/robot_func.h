/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-14 16:11:04
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-14 16:11:07
 * @FilePath: /mas_embedded_threadx/apps/sentry/gimbal_board/robot_func/robot_func.h
 * @Description:
 */
#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_

#include "sentry_def.h"
#include "module_vision.h"
#include "module_nav.h"
#include "module_ins.h"
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

/**
 * @brief 自动模式下处理视觉控制；道路对齐模式下屏蔽视觉并停止拨盘
 * @param Shoot_Ctrl 发射机构控制命令结构体指针
 * @param Gimbal_Ctrl 云台控制命令结构体指针
 * @param Ins 姿态指针
 * @param receive_packet 接收到的视觉(自瞄)数据包指针
 */
void gimbal_auto_func(Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl, const Ins_t *Ins, const VisionReceivePacket *receive_packet);

/**
 * @brief 先验证 NAV 命令再切换模式；无效时清零三路速度并退出底盘跟随
 * @note 在 RemoteControlSet 之后、gimbal_auto_func 之前调用，手动/停止模式不接管
 * @param Chassis_Ctrl 底盘控制命令结构体指针
 * @param Gimbal_Ctrl 云台控制命令结构体指针
 * @param nav_packet 接收到的导航数据包指针
 */
void nav_auto_func(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl, const NavReceivePacket *nav_packet);

#endif // _ROBOT_FUNC_H_
