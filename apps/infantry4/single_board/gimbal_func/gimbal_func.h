#ifndef _GIMBAL_FUNC_H_
#define _GIMBAL_FUNC_H_

#include "infantry_def.h"

void gimbal_init(void);

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd);

#endif // _GIMBAL_FUNC_H_