/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-14 14:33:13
 * @LastEditors: zhishang06 2494841771@qq.com
 * @LastEditTime: 2026-07-19 21:30:49
 * @FilePath: /mas_embedded_threadx/apps/infantry3/single_board/shoot_func/shoot_func.h
 * @Description:
 */
#ifndef _SHOOT_FUNC_H_
#define _SHOOT_FUNC_H_

#include "infantry_def.h"

void shoot_init(void);

void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd);

#endif // _SHOOT_FUNC_H_