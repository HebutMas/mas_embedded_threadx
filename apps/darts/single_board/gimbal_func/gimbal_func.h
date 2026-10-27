/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-09-15 20:36:42
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-09-15 20:45:21
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\gimbal_func\gimbal_func.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Description: 云台功能模板
 *
 * 使用说明:
 *   1. gimbal_init(): 初始化云台电机/编码器/PID
 *   2. gimbal_func(): 每周期云台控制逻辑
 */

#ifndef _GIMBAL_FUNC_H_
#define _GIMBAL_FUNC_H_

#include "darts_def.h"   /* TODO: 改为你的 def 文件 */

void gimbal_init(void);

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd);

#endif // _GIMBAL_FUNC_H_
