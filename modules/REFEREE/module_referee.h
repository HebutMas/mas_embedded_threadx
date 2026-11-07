/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 15:49:51
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-23 12:03:01
 * @FilePath: /mas_embedded_threadx/modules/REFEREE/module_referee.h
 * @Description:
 */
#ifndef _MODULE_REFEREE_H_
#define _MODULE_REFEREE_H_

#include <stdint.h>

// 这里根据需要自定义修改
#ifndef REFEREE_UART
#define REFEREE_UART huart1
#endif

#ifndef REFEREE_TASK_STACK_SIZE
#define REFEREE_TASK_STACK_SIZE 1024
#endif
#ifndef REFEREE_TASK_PRIORITY
#define REFEREE_TASK_PRIORITY 10
#endif

#ifndef REFEREE_OFFLINE_ENABLE
#define REFEREE_OFFLINE_ENABLE 1 /* 离线检测开启 */
#endif

/**
 * @brief 初始化裁判系统模块
 */
int Module_Referee_Init(void);
/**
 * @brief 发送 0x0301 机器人交互帧（选手端 UI/机间通信）
 * @param sub_cmd_id  子内容ID（如 UI_CMD_DRAW_1）
 * @param payload     用户数据指针
 * @param payload_len 用户数据长度（最大112字节）
 * @param timeout     超时时间
 * @note sender_id 根据裁判系统下发的 robot_id 自动确定
 *       recv_id: sentry/dart/radar/outpost/base → SERVER_ID(0x8080)，其他 → 自身 client ID
 */
void Module_Referee_Send_Interaction(uint16_t sub_cmd_id, const void *payload, uint16_t payload_len, uint32_t timeout);
/**
 * @brief 获取裁判系统模块命令数据
 * @param cmd_id 命令码
 * @return uint8_t* 命令数据指针
 */
uint8_t *Module_Referee_Get_cmd_data(uint16_t cmd_id);
/**
 * @brief 获取裁判系统模块离线状态
 * @return uint8_t 离线状态
 */
uint8_t Module_Referee_Get_offline_state(void);

#endif // _MODULE_REFEREE_H_
