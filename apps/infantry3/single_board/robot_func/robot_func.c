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
        float ch_scale = 1.0f / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);

        /* 右摇杆 → 底盘 vx/vy */
        Chassis_Ctrl->vx = (float)Module_Remote_get_channel(2) * ch_scale;  // 上推为正
        Chassis_Ctrl->vy = -(float)Module_Remote_get_channel(1) * ch_scale;  // 右推为正

        /* SW4 (CH8) — 底盘模式
         * 中档 = rotate(小陀螺): 固定 0.2转/s 自旋 + 全向移动
         * 挡位优先级低于 SW2(CH6) 失控保护 */
        {
            int16_t ch8 = Module_Remote_get_channel(8);
            if (ch8 == SBUS_CHX_UP)
                Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
            else if (ch8 == SBUS_CHX_BIAS)
                Chassis_Ctrl->chassis_mode = chassis_rotate;
            else if (ch8 == SBUS_CHX_DOWN)
                Chassis_Ctrl->chassis_mode = chassis_rotate;
        }

        /* SW2 (CH6) — 云台模式 (保留) */
        {
            int16_t ch6 = Module_Remote_get_channel(6);
            if (ch6 == SBUS_CHX_UP)
            {
                Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;
                /* 左摇杆 → yaw/pitch 微调 */
                Gimbal_Ctrl->yaw   -= 0.001f * (float)Module_Remote_get_channel(4);  // 左摇杆左右
                Gimbal_Ctrl->pitch += 0.001f * (float)Module_Remote_get_channel(3);  // 左摇杆上下
                VAL_LIMIT(Gimbal_Ctrl->pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
            }
            else if (ch6 == SBUS_CHX_BIAS)
            {
                Gimbal_Ctrl->gimbal_mode = gimbal_auto_mode;
                /* 不改变底盘模式，保持 SW4 设置 */
            }
            else if (ch6 == SBUS_CHX_DOWN)
            {
                Chassis_Ctrl->chassis_mode = chassis_zero_force;
                Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
                Shoot_Ctrl->shoot_mode     = shoot_off;
                Shoot_Ctrl->friction_mode  = friction_off;
                Shoot_Ctrl->load_mode      = load_stop;
            }
        }

        /* SW1 (CH5) — 摩擦轮开关 / SW3 (CH7) — 拨弹模式 */
        {
            int16_t ch5 = Module_Remote_get_channel(5);
            int16_t ch7 = Module_Remote_get_channel(7);

            if (ch5 == SBUS_CHX_UP)
            {
                Shoot_Ctrl->shoot_mode    = shoot_off;
                Shoot_Ctrl->friction_mode = friction_off;
                Shoot_Ctrl->load_mode     = load_stop;
            }
            else if (ch5 == SBUS_CHX_BIAS)
            {
                Shoot_Ctrl->shoot_mode    = shoot_on;
                Shoot_Ctrl->friction_mode = friction_off;
                Shoot_Ctrl->load_mode     = load_stop;
            }
            else if (ch5 == SBUS_CHX_DOWN)
            {
                Shoot_Ctrl->shoot_mode    = shoot_on;
                Shoot_Ctrl->friction_mode = friction_on;

                if (ch7 == SBUS_CHX_UP)
                    Shoot_Ctrl->load_mode = load_1_bullet;
                else if (ch7 == SBUS_CHX_BIAS)
                    Shoot_Ctrl->load_mode = load_stop;
                else if (ch7 == SBUS_CHX_DOWN)
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