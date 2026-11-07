/*
 * @Description: 底盘功能实现模板 (底盘板)
 *
 * TODO: 实现底盘控制逻辑
 */

#include "chassis_func.h"

int chassis_init(void)
{
    /* TODO: 初始化底盘电机 */
    return 0;
}

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd)
{
    (void)chassis_cmd;

    /* TODO: 实现底盘控制
     *   - 根据 chassis_cmd->vx / vy / wz 计算电机速度
     *   - 根据 chassis_cmd->chassis_mode 切换控制模式
     *   - 跟随模式: 使用 chassis_cmd->offset_angle 做 yaw 闭环
     */
}
