#include "robot_func.h"
#include "module_remote.h"
#include "module_vision.h"
#include "module_offline.h"
#include "module_ins.h"
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "sentry_def.h"
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
        /* 摇杆 → 云台系速度 (m/s), 与导航速度同单位
         * SBUS 通道值: 中位 1024, 上 240, 下 1807 → 零偏后 -784 ~ +783
         * 归一化到 -1.0 ~ +1.0 后乘满杆速度 */
        Chassis_Ctrl->vx = (float)Module_Remote_get_channel(2) / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS) * CHASSIS_MAX_SPEED_MPS;
        Chassis_Ctrl->vy = (float)Module_Remote_get_channel(1) / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS) * CHASSIS_MAX_SPEED_MPS;

        int16_t ch8 = Module_Remote_get_channel(8);
        if (ch8 == SBUS_CHX_UP)
            Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
        else if (ch8 == SBUS_CHX_BIAS)
            Chassis_Ctrl->chassis_mode = chassis_rotate;
        else if (ch8 == SBUS_CHX_DOWN)
            Chassis_Ctrl->chassis_mode = chassis_rotate_reverse;

        int16_t ch6 = Module_Remote_get_channel(6);
        if (ch6 == SBUS_CHX_UP)
        {
            Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;
            Gimbal_Ctrl->yaw -= 0.001f * (float)Module_Remote_get_channel(4);
            Gimbal_Ctrl->pitch += 0.001f * (float)Module_Remote_get_channel(3);
            VAL_LIMIT(Gimbal_Ctrl->pitch, SMALL_YAW_PITCH_MIN_ANGLE, SMALL_YAW_PITCH_MAX_ANGLE);
        }
        else if (ch6 == SBUS_CHX_BIAS)
        {
            Gimbal_Ctrl->gimbal_mode = gimbal_auto_mode;
        }
        else if (ch6 == SBUS_CHX_DOWN)
        {
            Chassis_Ctrl->chassis_mode = chassis_zero_force;
            Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
            Shoot_Ctrl->shoot_mode     = shoot_off;
            Shoot_Ctrl->friction_mode  = friction_off;
            Shoot_Ctrl->load_mode      = load_stop;
        }

        // 发射机构控制部分
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

void gimbal_auto_func(Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl, const Ins_t *Ins, const VisionReceivePacket *receive_packet)
{
    if (Shoot_Ctrl == NULL || Gimbal_Ctrl == NULL)
    {
        return;
    }
    if (Gimbal_Ctrl->gimbal_mode == gimbal_gyro_nav_mode)
    {
        Shoot_Ctrl->load_mode = load_stop;
        return;
    }
    if (Gimbal_Ctrl->gimbal_mode != gimbal_auto_mode)
    {
        return;
    }
    // 检查minipc离线状态
    if (Module_Vision_Get_offline_state() == STATE_OFFLINE)
    {
        // 离线停止搜索
        Gimbal_Ctrl->auto_search = 1;
        Shoot_Ctrl->load_mode    = load_stop;
        return;
    }
    if (receive_packet != NULL && SMALL_YAW_PITCH_MIN_ANGLE * DEGREE_2_RAD < -receive_packet->target_pitch &&
        -receive_packet->target_pitch < SMALL_YAW_PITCH_MAX_ANGLE * DEGREE_2_RAD)
    {
        // 进入跟踪模式
        Gimbal_Ctrl->auto_search = 0;
        Gimbal_Ctrl->pitch       = -receive_packet->target_pitch;
        Gimbal_Ctrl->yaw         = receive_packet->target_yaw + Ins->YawRoundCount * 360.0f * DEGREE_2_RAD;
        if (receive_packet->fire_advice == 1)
        {
            Shoot_Ctrl->load_mode = load_burstfire;
        }
        else
        {
            Shoot_Ctrl->load_mode = load_stop;
        }
    }
    else
    {
        // 进入搜索模式
        Gimbal_Ctrl->auto_search = 1;
        Shoot_Ctrl->load_mode    = load_stop;
    }
}

void nav_auto_func(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl, const NavReceivePacket *nav_packet)
{
    if (Chassis_Ctrl == NULL || Gimbal_Ctrl == NULL)
    {
        return;
    }
    if (Gimbal_Ctrl->gimbal_mode == gimbal_gyro_mode || Gimbal_Ctrl->gimbal_mode == gimbal_zero_force)
    {
        return;
    }

    /* 先验证完整命令，避免无效平移或角度仍开启云台/底盘跟随。 */
    if (Module_Nav_Get_offline_state() != STATE_ONLINE || nav_packet == NULL || !isfinite(nav_packet->vx) || !isfinite(nav_packet->vy) ||
        !isfinite(nav_packet->small_yaw_ref) || fabsf(nav_packet->small_yaw_ref) > 180.0f || nav_packet->follow_gimbal > 1)
    {
        Chassis_Ctrl->vx           = 0.0f;
        Chassis_Ctrl->vy           = 0.0f;
        Chassis_Ctrl->wz           = 0.0f;
        Chassis_Ctrl->chassis_mode = chassis_automode;
        Gimbal_Ctrl->gimbal_mode   = gimbal_auto_mode;
        return;
    }

    if (nav_packet->follow_gimbal == 1)
    {
        Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
        Gimbal_Ctrl->gimbal_mode   = gimbal_gyro_nav_mode;
        Gimbal_Ctrl->yaw           = nav_packet->small_yaw_ref;
        Gimbal_Ctrl->pitch         = 0;
    }
    else
    {
        Chassis_Ctrl->chassis_mode = chassis_automode;
        Gimbal_Ctrl->gimbal_mode   = gimbal_auto_mode;
    }

    Chassis_Ctrl->vx = nav_packet->vx;
    Chassis_Ctrl->vy = -nav_packet->vy;
    Chassis_Ctrl->wz = 2.0f;
}
