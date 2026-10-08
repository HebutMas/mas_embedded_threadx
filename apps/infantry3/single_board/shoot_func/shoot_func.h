/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-14 14:33:13
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-23 16:24:38
 * @FilePath: /mas_embedded_threadx/apps/infantry3/single_board/shoot_func/shoot_func.h
 * @Description:
 */
#ifndef _SHOOT_FUNC_H_
#define _SHOOT_FUNC_H_

#include "infantry_def.h"

int shoot_init(void);

void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd);

#endif // _SHOOT_FUNC_H_