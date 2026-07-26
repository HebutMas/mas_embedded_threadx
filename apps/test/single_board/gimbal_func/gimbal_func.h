/*
 * @Description: 云台功能 — F103 测试用
 *   仅初始化达妙电机 DM4310，func 留空
 */

#ifndef _GIMBAL_FUNC_H_
#define _GIMBAL_FUNC_H_

#include "test_def.h"

void gimbal_init(void);

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd);

#endif // _GIMBAL_FUNC_H_
