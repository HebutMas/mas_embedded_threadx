/*
 * @Description: 机器人通用函数实现模板 (单板)
 *
 * 使用说明:
 *   1. RemoteControlSet() 中的通道映射和按键逻辑需要按实际遥控器配置修改
 *   2. 参考 sentry/gimbal_board/robot_func/robot_func.c 了解完整实现
 */

#include "robot_func.h"
#include "module_remote.h"
#include <stdint.h>
#include <string.h>

int16_t CalcOffsetAngle(float getyawangle)
{
    float offset_ecd;

    const float ECD_MAX  = 8191.0f;
    const float ECD_HALF = 4095.5f;

    offset_ecd = getyawangle - (float)YAW_CHASSIS_ALIGN_ECD;

    while (offset_ecd > ECD_HALF) offset_ecd -= ECD_MAX;
    while (offset_ecd < -ECD_HALF) offset_ecd += ECD_MAX;

    return (int16_t)offset_ecd;
}

void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl,
                      Shoot_Ctrl_Cmd_t  *Shoot_Ctrl,
                      Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl)
{
    if (!Chassis_Ctrl || !Shoot_Ctrl || !Gimbal_Ctrl) return;

    uint8_t state = Module_Remote_get_offline_status();

    if (state & 0x01) /* RC 在线 */
    {
        /* ── 底盘控制 ──
         * CH1: vy, CH2: vx, CH4: wz
         * 摇杆值 → 速度比例 (-1.0 ~ +1.0) */
        Chassis_Ctrl->vx = (float)Module_Remote_get_channel(2) / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);
        Chassis_Ctrl->vy = (float)Module_Remote_get_channel(1) / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);
        Chassis_Ctrl->wz = (float)Module_Remote_get_channel(4) / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);

        /* TODO: 根据遥控器拨杆自定义底盘模式切换
         * 示例 (sentry 拨杆映射):
         *   CH8 上: chassis_follow_gimbal_yaw
         *   CH8 中: chassis_rotate
         *   CH8 下: chassis_rotate_reverse
         */
        Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;

        /* ── 云台控制 ──
         * CH3: pitch, CH4: yaw 微调 */
        Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;
        Gimbal_Ctrl->yaw        -= 0.001f * (float)Module_Remote_get_channel(4);
        Gimbal_Ctrl->pitch      += 0.001f * (float)Module_Remote_get_channel(3);

        /* TODO: 根据拨杆切换云台模式
         * 示例: CH6 上=手动, CH6 中=自动, CH6 下=停止
         */

        /* ── 发射控制 ──
         * TODO: 根据拨杆自定义发射逻辑
         * 示例: CH5 上=关闭, CH5 中=待机, CH5 下=开启摩擦轮+发射
         *       CH7 控制连发模式
         */
        Shoot_Ctrl->shoot_mode    = shoot_off;
        Shoot_Ctrl->friction_mode = friction_off;
        Shoot_Ctrl->load_mode     = load_stop;
    }
    else /* RC 离线 → 停止所有执行器 */
    {
        Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
        Chassis_Ctrl->chassis_mode = chassis_zero_force;
        Shoot_Ctrl->shoot_mode     = shoot_off;
        Shoot_Ctrl->friction_mode  = friction_off;
        Shoot_Ctrl->load_mode      = load_stop;
        memset(Chassis_Ctrl, 0, sizeof(Chassis_Ctrl_Cmd_t));
    }
}
