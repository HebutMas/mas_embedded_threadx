/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-17 10:31:31
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-18 09:43:36
 * @FilePath: /mas_embedded_threadx/apps/sentry/chassis_board/chassis/chassis_func.h
 * @Description:
 */
#ifndef _CHASSIS_FUNC_H_
#define _CHASSIS_FUNC_H_

#include "sentry_def.h"

int chassis_init(void);

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd);

#endif // _CHASSIS_FUNC_H_