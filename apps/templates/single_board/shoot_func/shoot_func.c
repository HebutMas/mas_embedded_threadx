/*
 * @Description: 发射机构功能实现模板
 *
 * TODO: 实现发射逻辑
 *   1. 初始化摩擦轮电机、拨盘电机
 *   2. 根据 shoot_cmd 控制摩擦轮开关和拨盘动作
 */

#include "shoot_func.h"

int shoot_init(void)
{
    /* TODO: 初始化发射电机 */
    return 0;
}

void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd)
{
    (void)shoot_cmd;

    /* TODO: 实现发射控制
     *   - shoot_cmd->friction_mode: 控制摩擦轮 on/off
     *   - shoot_cmd->shoot_mode:    控制发射使能
     *   - shoot_cmd->load_mode:     控制拨盘 (单发/连发/反转/停止)
     */
}
