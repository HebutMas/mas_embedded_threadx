/*
 * @Description: 底盘功能实现模板
 *
 * TODO: 实现底盘控制逻辑
 *   1. 初始化底盘电机
 *   2. 根据 chassis_cmd (vx/vy/wz) 计算逆运动学 → 电机 CAN 命令
 *   3. 底盘跟随云台模式需要 chassis_cmd->offset_angle
 */

#include "chassis_func.h"

int chassis_init(void) { /* TODO: 初始化底盘电机 */ return 0; }

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd)
{
    (void)chassis_cmd;

    /* TODO: 实现底盘控制
     *   - 根据 chassis_cmd->vx / vy / wz 计算电机速度
     *   - 根据 chassis_cmd->chassis_mode 切换控制模式
     *   - 跟随模式: 使用 chassis_cmd->offset_angle 做 yaw 闭环
     */
}
