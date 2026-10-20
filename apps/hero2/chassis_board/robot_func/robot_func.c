/*
 * @Description: 遥控器业务解析: 通道值 → 底盘控制命令
 *               (SBUS 协议解析在 REMOTE 模块层完成, 此处只做映射)
 */
#include "robot_func.h"
#include "module_remote.h"
#include <stddef.h>

void RemoteControlSet(Chassis_Ctrl_Cmd_t *cmd)
{
    if (cmd == NULL) return;

    /* 右摇杆: CH2 上下 → 前后速度 (上推为正; 偏移/死区已在 REMOTE 模块解码层处理) */
    cmd->body_target_dx = (float)Module_Remote_get_channel(2)
                        / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS) * 2.0f;
    cmd->turn_cmd       = (float)Module_Remote_get_channel(1)
                        / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS) * 0.07f;
    /* 满行程 ≈ ±0.07rad ≈ ±4°; 49.19×0.07≈3.4N·m 恰在 ±3.5 限幅内 */
}
