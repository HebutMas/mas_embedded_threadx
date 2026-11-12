/*
 * @Description: 底盘功能模板 (底盘板)
 */

#ifndef _CHASSIS_FUNC_H_
#define _CHASSIS_FUNC_H_

#include "<robot>_def.h" /* TODO: 改为你的 def 文件 */

int chassis_init(void);

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd);

#endif // _CHASSIS_FUNC_H_
