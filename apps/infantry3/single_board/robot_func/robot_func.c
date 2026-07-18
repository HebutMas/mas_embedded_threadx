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

    // 归一化到最小旋转角度对应的编码器差值 ([-ECD_HALF, ECD_HALF])
    while (offset_ecd > ECD_HALF) offset_ecd -= ECD_MAX;
    while (offset_ecd < -ECD_HALF) offset_ecd += ECD_MAX;

    return (int16_t)offset_ecd;
}

void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl)
{
    if (!Chassis_Ctrl || !Shoot_Ctrl || !Gimbal_Ctrl) return;

    uint8_t state = Module_Remote_get_offline_status();

    /* RC 在线 */
    if (state & 0x01)
    {
        /* 摇杆 → 速度比例 (-1.0 ~ +1.0) */
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

            /* 云台控制部分 */
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

            // 发射机构控制部分
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
