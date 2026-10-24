/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-10-06 14:12:54
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-10-09 16:28:03
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\robot_func\robot_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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

bool RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl)
{
    if (!Chassis_Ctrl || !Shoot_Ctrl || !Gimbal_Ctrl) return false;

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
         * CH3: pitch, CH4: yaw。摇杆压过阈值 → 定速往该方向走, 回中即停;
         * 0.002 = 控制周期 2ms (robot_control.c 的 tx_thread_sleep(2)) */
        Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;

        int16_t ch_yaw   = Module_Remote_get_channel(2);
        int16_t ch_pitch = Module_Remote_get_channel(3);

        if (ch_yaw > GIMBAL_STICK_TH)
            Gimbal_Ctrl->yaw += GIMBAL_YAW_RATE_DPS * 0.002f;
        else if (ch_yaw < -GIMBAL_STICK_TH)
            Gimbal_Ctrl->yaw -= GIMBAL_YAW_RATE_DPS * 0.002f;

        if (ch_pitch > GIMBAL_STICK_TH)
            Gimbal_Ctrl->pitch += GIMBAL_PITCH_RATE_DPS * 0.002f;
        else if (ch_pitch < -GIMBAL_STICK_TH)
            Gimbal_Ctrl->pitch -= GIMBAL_PITCH_RATE_DPS * 0.002f;

        /* TODO: 根据拨杆切换云台模式
         * 示例: CH6 上=手动, CH6 中=自动, CH6 下=停止
         */

        /* ── 发射控制 ──
         * SW1 (CH5): 下拨 = 打一发 (shoot_func 里按下拨边沿触发) */
        if (Module_Remote_get_channel(5) == SBUS_CHX_DOWN)
            Shoot_Ctrl->shoot_mode = shoot_on;
        else
            Shoot_Ctrl->shoot_mode = shoot_off;

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
    return (state & 0x01U) != 0U;
}
