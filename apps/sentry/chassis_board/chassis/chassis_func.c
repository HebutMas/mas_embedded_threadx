#include "chassis_func.h"
#include "module_referee.h"
#include "referee_protocol.h"
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

static DJI_Motor_t                  *chassis_motors[8];
static float                         chassis_vx, chassis_vy, chassis_wz; // 将云台系的速度投影到底盘
static PIDInstance                   chassis_follow_pid;
static const Chassis_Swerve_Config_s chassis_swerve_config = {
    .align_rad      = {4458 * 0.000767f, 1010 * 0.000767f, 7177 * 0.000767f, 3756 * 0.000767f}, // lf lb rb rf(rad)
    .decele_ratio   = 16.0f,
    .radius_wheel_m = 0.12f,
    .wheel_r        = 0.5f,
};

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
                .can.hcan = BSP_CAN_HANDLE2,
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

    Motor_Init_Config_s gm6020_motor_config = {
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
                                           .K         = {2.23f, 0.2f},
                                           .state_dim = 2,
                                       }},
        .setting_init_config =
            {
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = ANGLE_LOOP,
                .feedback_reverse_flag = 0,
                .algorithm_type        = CONTROL_LQR,
            },
        .motor_init_info = {.motor_type = GM6020_CURRENT, .gear_ratio = 1, .max_torque = 2.23, .torque_constant = 0.741},
    };

    PowerCtrl_Param_t gm6020_power_config = {.k1 = 0.005, .k2 = 12.98, .k3 = 1};

    gm6020_motor_config.transport_config.can.tx_id             = 1;
    gm6020_motor_config.setting_init_config.motor_reverse_flag = 0;
    gm6020_motor_config.offline_init_config.name               = "gm6020_1";
    gm6020_motor_config.offline_init_config.beep_times         = 5;
    chassis_motors[4]                                          = Motor_DJI_Init(&gm6020_motor_config);
    if (chassis_motors[4] == NULL)
    {
        LOG_E("chassis motor[4] init failed");
        return;
    }
    PowerControl_Register(&chassis_motors[4]->base, PC_ROLE_STEER, gm6020_power_config);

    gm6020_motor_config.transport_config.can.tx_id             = 2;
    gm6020_motor_config.setting_init_config.motor_reverse_flag = 0;
    gm6020_motor_config.offline_init_config.name               = "gm6020_2";
    gm6020_motor_config.offline_init_config.beep_times         = 6;
    chassis_motors[5]                                          = Motor_DJI_Init(&gm6020_motor_config);
    if (chassis_motors[5] == NULL)
    {
        LOG_E("chassis motor[5] init failed");
        return;
    }
    PowerControl_Register(&chassis_motors[5]->base, PC_ROLE_STEER, gm6020_power_config);

    gm6020_motor_config.transport_config.can.tx_id             = 3;
    gm6020_motor_config.setting_init_config.motor_reverse_flag = 0;
    gm6020_motor_config.offline_init_config.name               = "gm6020_3";
    gm6020_motor_config.offline_init_config.beep_times         = 7;
    chassis_motors[6]                                          = Motor_DJI_Init(&gm6020_motor_config);
    if (chassis_motors[6] == NULL)
    {
        LOG_E("chassis motor[6] init failed");
        return;
    }
    PowerControl_Register(&chassis_motors[6]->base, PC_ROLE_STEER, gm6020_power_config);

    gm6020_motor_config.transport_config.can.tx_id             = 4;
    gm6020_motor_config.setting_init_config.motor_reverse_flag = 0;
    gm6020_motor_config.offline_init_config.name               = "gm6020_4";
    gm6020_motor_config.offline_init_config.beep_times         = 8;
    chassis_motors[7]                                          = Motor_DJI_Init(&gm6020_motor_config);
    if (chassis_motors[7] == NULL)
    {
        LOG_E("chassis motor[7] init failed");
        return;
    }
    PowerControl_Register(&chassis_motors[7]->base, PC_ROLE_STEER, gm6020_power_config);

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
            !Module_Offline_get_device_status(chassis_motors[3]->base.offline_dev) &&
            !Module_Offline_get_device_status(chassis_motors[4]->base.offline_dev) &&
            !Module_Offline_get_device_status(chassis_motors[5]->base.offline_dev) &&
            !Module_Offline_get_device_status(chassis_motors[6]->base.offline_dev) &&
            !Module_Offline_get_device_status(chassis_motors[7]->base.offline_dev))
        {
            if (chassis_cmd->chassis_mode == chassis_zero_force)
            {
                Motor_Stop((Motor_Base *)chassis_motors[0]);
                Motor_Stop((Motor_Base *)chassis_motors[1]);
                Motor_Stop((Motor_Base *)chassis_motors[2]);
                Motor_Stop((Motor_Base *)chassis_motors[3]);
                Motor_Stop((Motor_Base *)chassis_motors[4]);
                Motor_Stop((Motor_Base *)chassis_motors[5]);
                Motor_Stop((Motor_Base *)chassis_motors[6]);
                Motor_Stop((Motor_Base *)chassis_motors[7]);
            }
            else
            {
                Motor_Start((Motor_Base *)chassis_motors[0]);
                Motor_Start((Motor_Base *)chassis_motors[1]);
                Motor_Start((Motor_Base *)chassis_motors[2]);
                Motor_Start((Motor_Base *)chassis_motors[3]);
                Motor_Start((Motor_Base *)chassis_motors[4]);
                Motor_Start((Motor_Base *)chassis_motors[5]);
                Motor_Start((Motor_Base *)chassis_motors[6]);
                Motor_Start((Motor_Base *)chassis_motors[7]);
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
            case chassis_automode:
            {
                const game_status_t *game_status = (game_status_t *)Module_Referee_Get_cmd_data(CMD_ID_GAME_STATUS);
                if (game_status != NULL && game_status->type_progress.game_progress == 4) // 比赛中
                {
                }
                else
                {
                    chassis_vx = 0;
                    chassis_vy = 0;
                    chassis_wz = 0;
                }
                break;
            }
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

            Chassis_Swerve_Calc(chassis_motors, &chassis_swerve_config, chassis_vx, chassis_vy, chassis_wz);
        }
        else
        {
            Motor_Stop((Motor_Base *)chassis_motors[0]);
            Motor_Stop((Motor_Base *)chassis_motors[1]);
            Motor_Stop((Motor_Base *)chassis_motors[2]);
            Motor_Stop((Motor_Base *)chassis_motors[3]);
            Motor_Stop((Motor_Base *)chassis_motors[4]);
            Motor_Stop((Motor_Base *)chassis_motors[5]);
            Motor_Stop((Motor_Base *)chassis_motors[6]);
            Motor_Stop((Motor_Base *)chassis_motors[7]);
        }
    }
}
