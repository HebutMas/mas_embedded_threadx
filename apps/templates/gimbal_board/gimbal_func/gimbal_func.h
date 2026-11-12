/*
 * @Description: 云台功能模板 (云台板)
 *
 * 与单板版本的接口相同
 */

#ifndef _GIMBAL_FUNC_H_
#define _GIMBAL_FUNC_H_

#include "<robot>_def.h" /* TODO: 改为你的 def 文件 */

int gimbal_init(void);


uint8_t gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd);

#endif // _GIMBAL_FUNC_H_
