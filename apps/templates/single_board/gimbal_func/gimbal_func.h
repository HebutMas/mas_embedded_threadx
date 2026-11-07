/*
 * @Description: 云台功能模板
 *
 * 使用说明:
 *   1. gimbal_init(): 初始化云台电机/编码器/PID
 *   2. gimbal_func(): 每周期云台控制逻辑
 */

#ifndef _GIMBAL_FUNC_H_
#define _GIMBAL_FUNC_H_

#include "<robot>_def.h" /* TODO: 改为你的 def 文件 */

int gimbal_init(void);

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd);

#endif // _GIMBAL_FUNC_H_
