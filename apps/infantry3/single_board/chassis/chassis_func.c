#include "chassis_func.h"
#include "module_offline.h"
#include "motor_def.h"
#include "motor_dji.h"
#include "power_control.h"
#include "user_lib.h"
#include <stdint.h>
#include "chassis_type.h"

#define LOG_TAG "app_chassis"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

static DJI_Motor_t                *chassis_motors[4];
static float                       chassis_vx, chassis_vy, chassis_wz; // 将云台系的速度投影到底盘
static PIDInstance                 chassis_follow_pid;
static const Chassis_Diff_Config_s chassis_diff_config = {.decele_ratio = 16.0f, .wheel_base_x = 0.5, .wheel_base_y = 0.3, .wheel_radius = 0.075};

void chassis_init(void)
{
    PID_Init_Config_s config = {
        .MaxOut = 5, .IntegralLimit = 0.01, .DeadBand = 10, .Kp = 0.1, .Ki = 0, .Kd = 0.001, .Improve = 0x01}; // enable integratiaon limit
    PIDInit(&chassis_follow_pid, &config);

    Motor_Init_Config_s chassis_motor_config = {
        .offline_init_config =
            {
                .timeout_ms = 100, // 超时时间
                .enable     = 1,   // 是否启用离线管理
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config =
            {
                .can.hcan = BSP_CAN_HANDLE1,
            },
        .controller_init_config = {.lqr_init =
                                       {
                                           .K         = {0.008f},
                                           .state_dim = 1,
                                       }},
        .setting_init_config =
            {
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = SPEED_LOOP,
                .feedback_reverse_flag = 0,
                .algorithm_type        = CONTROL_LQR,
            },
        .motor_init_info = {.motor_type = M3508, .gear_ratio = 16, .max_torque = 5.32, .torque_constant = 0.016},
    };

    PowerCtrl_Param_t power_config = {.k1 = 0.132, .k2 = 3.47, .k3 = 1};

    chassis_motor_config.transport_config.can.tx_id             = 1;
    chassis_motor_config.setting_init_config.motor_reverse_flag = 0;
    chassis_motor_config.offline_init_config.name               = "m3508_1";
    chassis_motor_config.offline_init_config.beep_times         = 1;
    chassis_motors[0]                                           = Motor_DJI_Init(&chassis_motor_config);
    if (chassis_motors[0] == NULL)
    {
        LOG_E("chassis motor[0] init failed");
        return;
    }
    PowerControl_Register(&chassis_motors[0]->base, PC_ROLE_DRIVE, power_config);

    chassis_motor_config.transport_config.can.tx_id             = 2;
    chassis_motor_config.setting_init_config.motor_reverse_flag = 0;
    chassis_motor_config.offline_init_config.name               = "m3508_2";
    chassis_motor_config.offline_init_config.beep_times         = 2;
    chassis_motors[1]                                           = Motor_DJI_Init(&chassis_motor_config);
    if (chassis_motors[1] == NULL)
    {
        LOG_E("chassis motor[1] init failed");
        return;
    }
    PowerControl_Register(&chassis_motors[1]->base, PC_ROLE_DRIVE, power_config);

    chassis_motor_config.transport_config.can.tx_id             = 3;
    chassis_motor_config.setting_init_config.motor_reverse_flag = 1;
    chassis_motor_config.offline_init_config.name               = "m3508_3";
    chassis_motor_config.offline_init_config.beep_times         = 3;
    chassis_motors[2]                                           = Motor_DJI_Init(&chassis_motor_config);
    if (chassis_motors[2] == NULL)
    {
        LOG_E("chassis motor[2] init failed");
        return;
    }
    PowerControl_Register(&chassis_motors[2]->base, PC_ROLE_DRIVE, power_config);

    chassis_motor_config.transport_config.can.tx_id             = 4;
    chassis_motor_config.setting_init_config.motor_reverse_flag = 1;
    chassis_motor_config.offline_init_config.name               = "m3508_4";
    chassis_motor_config.offline_init_config.beep_times         = 4;
    chassis_motors[3]                                           = Motor_DJI_Init(&chassis_motor_config);
    if (chassis_motors[3] == NULL)
    {
        LOG_E("chassis motor[3] init failed");
        return;
    }
    PowerControl_Register(&chassis_motors[3]->base, PC_ROLE_DRIVE, power_config);

    PowerControl_SetLimit(120, 60, 0);

    LOG_I("Chassis initialized");
}

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd)
{
    if (chassis_cmd != NULL)
    {
        if (!Module_Offline_get_device_status(chassis_motors[0]->base.offline_dev) &&
            !Module_Offline_get_device_status(chassis_motors[1]->base.offline_dev) &&
            !Module_Offline_get_device_status(chassis_motors[2]->base.offline_dev) &&
            !Module_Offline_get_device_status(chassis_motors[3]->base.offline_dev))
        {
            if (chassis_cmd->chassis_mode == chassis_zero_force)
            {
                Motor_Stop((Motor_Base *)chassis_motors[0]);
                Motor_Stop((Motor_Base *)chassis_motors[1]);
                Motor_Stop((Motor_Base *)chassis_motors[2]);
                Motor_Stop((Motor_Base *)chassis_motors[3]);
            }
            else
            {
                Motor_Start((Motor_Base *)chassis_motors[0]);
                Motor_Start((Motor_Base *)chassis_motors[1]);
                Motor_Start((Motor_Base *)chassis_motors[2]);
                Motor_Start((Motor_Base *)chassis_motors[3]);
            }

            // 根据控制模式设定旋转速度
            switch (chassis_cmd->chassis_mode)
            {
            case chassis_rotate_reverse: // 自旋反转,同时保持全向机动
                chassis_wz = -8;
                break;
            case chassis_follow_gimbal_yaw: // 跟随云台
                PIDCalculate(&chassis_follow_pid, chassis_cmd->offset_angle, 0);
                chassis_wz = chassis_follow_pid.Output;
                break;
            case chassis_rotate: // 自旋,同时保持全向机动
                chassis_wz = 3;
                break;
            default:
                break;
            }

            // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
            // 底盘逆时针旋转为角度正方向;云台命令的方向以云台指向的方向为x,采用右手系
            static float sin_theta, cos_theta;
            float        total_angle_rad = chassis_cmd->offset_angle * DEGREE_2_RAD;
            cos_theta                    = arm_cos_f32(total_angle_rad);
            sin_theta                    = arm_sin_f32(total_angle_rad);
            chassis_vx                   = chassis_cmd->vx * cos_theta - chassis_cmd->vy * sin_theta;
            chassis_vy                   = chassis_cmd->vx * sin_theta + chassis_cmd->vy * cos_theta;

            Chassis_Mecanum_Calc(chassis_motors, &chassis_diff_config, chassis_vx, chassis_vy, chassis_wz);
        }
        else
        {
            Motor_Stop((Motor_Base *)chassis_motors[0]);
            Motor_Stop((Motor_Base *)chassis_motors[1]);
            Motor_Stop((Motor_Base *)chassis_motors[2]);
            Motor_Stop((Motor_Base *)chassis_motors[3]);
        }
    }
}
