/*
 * @Description: 云台功能实现模板
 *
 * TODO: 实现你的云台控制逻辑
 *   1. 初始化 gimbal yaw/pitch 电机 (CAN)
 *   2. 每周期根据 gimbal_cmd 计算 PID 输出
 *   3. 回传当前 yaw 编码器值
 */

#include "gimbal_func.h"

int gimbal_init(void)
{
    /* TODO: 初始化云台电机 */
    return 0;
}

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd)
{
    (void)gimbal_cmd;
    (void)yaw_ecd;

    /* TODO: 实现云台控制
     *   - 根据 gimbal_cmd->yaw / gimbal_cmd->pitch 计算目标位置
     *   - 根据 gimbal_cmd->gimbal_mode 切换控制模式
     *   - 读取 yaw 编码器 → *yaw_ecd
     *   - 调用电机 CAN 发送命令
     */
}
