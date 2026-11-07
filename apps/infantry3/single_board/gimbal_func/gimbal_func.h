/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-14 20:56:35
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-23 16:25:13
 * @FilePath: /mas_embedded_threadx/apps/infantry3/single_board/gimbal_func/gimbal_func.h
 * @Description:
 */
#ifndef _GIMBAL_FUNC_H_
#define _GIMBAL_FUNC_H_

#include "infantry_def.h"

int gimbal_init(void);


uint8_t gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd);

#endif // _GIMBAL_FUNC_H_