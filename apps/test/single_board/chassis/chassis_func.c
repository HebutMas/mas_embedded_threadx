/*
 * @Description: 底盘功能 — F103 测试用空实现
 */

#include "chassis_func.h"

void chassis_init(void)
{
    /* 暂不初始化底盘电机 */
}

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd)
{
    (void)chassis_cmd;
    /* 暂不执行底盘控制 */
}
