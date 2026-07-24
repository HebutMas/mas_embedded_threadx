#include "robot_func.h"
#include "module_remote.h"
#include <stdint.h>
#include <string.h>
#include "infantry_def.h"
#include "user_lib.h"
//#include <stdlib.h>  // 添加头文件

#define SBUS_SW_UP     240
#define SBUS_SW_DOWN   1807
#define SBUS_SW_MID    1024
//#define RC_DEADZONE 50  // 死区阈值

int16_t ch1_buf, ch2_buf, ch3_buf, ch4_buf, sw1_buf, sw2_buf, sw3_buf, sw4_buf;
uint8_t remote_state_buf;

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
    uint8_t state = Module_Remote_get_offline_status();

    if (!(state & 0x01))
    {
        if (Gimbal_Ctrl) {
            Gimbal_Ctrl->gimbal_mode = gimbal_zero_force;
        }
        if (Chassis_Ctrl) {
            Chassis_Ctrl->chassis_mode = chassis_zero_force;
            memset(Chassis_Ctrl, 0, sizeof(*Chassis_Ctrl));
        }
        if (Shoot_Ctrl) {
            Shoot_Ctrl->shoot_mode    = shoot_off;
            Shoot_Ctrl->friction_mode = friction_off;
            Shoot_Ctrl->load_mode     = load_stop;
        }
        return;
    }

    ch1_buf = Module_Remote_get_channel(1);
    ch2_buf = Module_Remote_get_channel(2);
    ch3_buf = Module_Remote_get_channel(3);
    ch4_buf = Module_Remote_get_channel(4);
    sw1_buf = Module_Remote_get_channel(5);
    sw2_buf = Module_Remote_get_channel(6);
    sw4_buf = Module_Remote_get_channel(8);
    remote_state_buf = state;


    
    if (Chassis_Ctrl) {
        Chassis_Ctrl->vx = -(float)Module_Remote_get_channel(1) / 660.0f;
        Chassis_Ctrl->vy = -(float)Module_Remote_get_channel(2) / 660.0f;


        // 限幅保护
        VAL_LIMIT(Chassis_Ctrl->vx, -1.0f, 1.0f);
        VAL_LIMIT(Chassis_Ctrl->vy, -1.0f, 1.0f);

        int16_t sw2 = Module_Remote_get_channel(6);
        if (sw2 == SBUS_SW_UP) {
            Chassis_Ctrl->chassis_mode = chassis_zero_force;// 底盘停止模式
        } else if (sw2 == SBUS_SW_DOWN) {
            Chassis_Ctrl->chassis_mode = chassis_rotate_reverse;// 旋转反向
        } else if (sw2 == SBUS_SW_MID) {
            Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;// 跟随陀螺仪角度
        }
    }

    if (Gimbal_Ctrl) {
        int16_t sw1 = Module_Remote_get_channel(5);// 云台模式开关



     
        if (sw1 == SBUS_SW_UP) {
            Gimbal_Ctrl->gimbal_mode = gimbal_zero_force;
        } else if (sw1 == SBUS_SW_MID) {
            Gimbal_Ctrl->gimbal_mode = gimbal_gyro_mode;
            Gimbal_Ctrl->yaw -= 0.0003f * (float)(Module_Remote_get_channel(4));
            Gimbal_Ctrl->pitch += 0.001f * (float)(Module_Remote_get_channel(3));
          //  VAL_LIMIT(Gimbal_Ctrl->yaw, -180.0f, 180.0f);
            VAL_LIMIT(Gimbal_Ctrl->pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
        } else {
            Gimbal_Ctrl->gimbal_mode = gimbal_zero_force;
        }
    }

    if (Shoot_Ctrl) {
        int16_t sw1 = Module_Remote_get_channel(5);
        int16_t sw4 = Module_Remote_get_channel(8);

        if (sw1 == SBUS_SW_MID) {
            Shoot_Ctrl->shoot_mode    = shoot_on;
            Shoot_Ctrl->friction_mode = friction_off;
            Shoot_Ctrl->load_mode     = load_stop;
        } else if (sw1 == SBUS_SW_DOWN) {
            Shoot_Ctrl->shoot_mode    = shoot_on;
            Shoot_Ctrl->friction_mode = friction_on;

            if (sw4 == SBUS_SW_UP) {
                Shoot_Ctrl->load_mode =  load_stop;// 停止拨盘
            } else if (sw4 == SBUS_SW_MID) {
                Shoot_Ctrl->load_mode = load_1_bullet;// 单发
            } else if (sw4 == SBUS_SW_DOWN) {
                Shoot_Ctrl->load_mode = load_burstfire;// 连发
            }
        } else {
            Shoot_Ctrl->shoot_mode    = shoot_off;
            Shoot_Ctrl->friction_mode = friction_off;
            Shoot_Ctrl->load_mode     = load_stop;
        }
    }
}