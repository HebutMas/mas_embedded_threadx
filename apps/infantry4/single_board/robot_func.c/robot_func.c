/*
 * @Author: zhishang06 2494841771@qq.com
 * @Date: 2026-07-17 19:16:56
 * @LastEditors: zhishang06 2494841771@qq.com
 * @LastEditTime: 2026-07-17 19:22:15
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\robot_func.c\robot_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_func.h"
#include "module_remote.h"
#include <stdint.h>
#include <string.h>
#include "infantry_def.h"
#include "user_lib.h"

int16_t CalcOffsetAngle(float getyawangle)
{
    float offset_ecd;

    const float ECD_MAX  = 8191.0f;
    const float ECD_HALF = 4095.5f;

    offset_ecd = getyawangle - YAW_CHASSIS_ALIGN_ECD;

    while (offset_ecd > ECD_HALF) offset_ecd -= ECD_MAX;
    while (offset_ecd < -ECD_HALF) offset_ecd += ECD_MAX;

    return (int16_t)offset_ecd;
}

void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl)
{
    if (!Chassis_Ctrl || !Shoot_Ctrl || !Gimbal_Ctrl) return;

    uint8_t state = Module_Remote_get_offline_status();

    if (state & 0x01)
    {
        Chassis_Ctrl->vx = (float)Module_Remote_get_channel(1) / (float)(DT7_CH_VALUE_MAX - DT7_CH_VALUE_MIN);
        Chassis_Ctrl->vy = -(float)Module_Remote_get_channel(2) / (float)(DT7_CH_VALUE_MAX - DT7_CH_VALUE_MIN);

        dt7_custom_t *dt7_custom = Module_Remote_get_dt7_custom();
        if (dt7_custom != NULL)
        {
            if (dt7_custom->sw2 == DT7_SW_UP)
            {
                Chassis_Ctrl->chassis_mode = chassis_rotate;
            }
            else if (dt7_custom->sw2 == DT7_SW_MID)
            {
                Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
            }
            else if (dt7_custom->sw2 == DT7_SW_DOWN)
            {
                Chassis_Ctrl->chassis_mode = chassis_rotate_reverse;
            }

            if (dt7_custom->sw1 == DT7_SW_MID)
            {
                Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;
                Gimbal_Ctrl->yaw -= 0.001f * (float)(Module_Remote_get_channel(3));
                Gimbal_Ctrl->pitch += 0.001f * (float)(Module_Remote_get_channel(4));
                VAL_LIMIT(Gimbal_Ctrl->pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
            }
            else if (dt7_custom->sw1 == DT7_SW_DOWN)
            {
                Chassis_Ctrl->chassis_mode = chassis_zero_force;
                Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
                Shoot_Ctrl->shoot_mode     = shoot_off;
                Shoot_Ctrl->friction_mode  = friction_off;
                Shoot_Ctrl->load_mode      = load_stop;
            }

            if (dt7_custom->sw1 == DT7_SW_UP)
            {
                Shoot_Ctrl->shoot_mode    = shoot_on;
                Shoot_Ctrl->friction_mode = friction_on;
                Shoot_Ctrl->load_mode     = load_stop;
            }
            else if (dt7_custom->sw1 == DT7_SW_MID)
            {
                Shoot_Ctrl->shoot_mode    = shoot_on;
                Shoot_Ctrl->friction_mode = friction_off;

                if (dt7_custom->wheel == 0)
                {
                    Shoot_Ctrl->load_mode = load_stop;
                }
                else if (dt7_custom->wheel > 0)
                {
                    Shoot_Ctrl->load_mode = load_reverse;
                }
                else if (dt7_custom->wheel < 0)
                {
                    Shoot_Ctrl->load_mode = load_burstfire;
                }
            }
        }
        else
        {
            Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
            Chassis_Ctrl->chassis_mode = chassis_zero_force;
            Shoot_Ctrl->shoot_mode     = shoot_off;
            Shoot_Ctrl->friction_mode  = friction_off;
            Shoot_Ctrl->load_mode      = load_stop;
            memset(Chassis_Ctrl, 0, sizeof(*Chassis_Ctrl));
        }
    }
    else
    {
        Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
        Chassis_Ctrl->chassis_mode = chassis_zero_force;
        Shoot_Ctrl->shoot_mode     = shoot_off;
        Shoot_Ctrl->friction_mode  = friction_off;
        Shoot_Ctrl->load_mode      = load_stop;
        memset(Chassis_Ctrl, 0, sizeof(*Chassis_Ctrl));
    }
}