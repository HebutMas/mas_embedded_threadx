/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-07 10:45:15
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-21 16:00:00
 * @FilePath: /mas_embedded_threadx/board/bsp/CAN/bsp_can_task.h
 * @Description: CAN 收发任务
 */
#ifndef _BSP_CAN_TASK_H_
#define _BSP_CAN_TASK_H_

#include "tx_api.h"

extern TX_SEMAPHORE g_can_rx_sem;
extern TX_SEMAPHORE g_can_tx_sem;

/**
 * @brief 初始化 CAN 收发后台任务
 * @note  必须在 tx_application_define() 中调用, 且在 BSP_CAN_Device_Init 之前
 */
int BSP_CAN_TaskInit(void);

#endif /* _BSP_CAN_TASK_H_ */
