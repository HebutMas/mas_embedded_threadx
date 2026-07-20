/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-17 10:31:31
 * @LastEditors: zhishang06 2494841771@qq.com
 * @LastEditTime: 2026-07-16 10:06:56
 * @FilePath: /mas_embedded_threadx/apps/sentry/chassis_board/chassis/chassis_func.h
 * @Description:
 */
#ifndef _CHASSIS_FUNC_H_
#define _CHASSIS_FUNC_H_

#include "infantry_def.h"


void chassis_init(void);

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd);

#endif // _CHASSIS_FUNC_H_