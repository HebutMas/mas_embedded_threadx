#include "shoot_func.h"
#include "module_offline.h"
#include "motor_base.h"
#include "motor_def.h"
#include "motor_dji.h"
#include "user_lib.h"

#define LOG_TAG "app_shoot"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

/* 摩擦轮/拨盘用 DJI 子类指针, 走基类 API 时显式 (Motor_Base *) 转换 */
DJI_Motor_t *friction_l = NULL;
DJI_Motor_t *friction_r = NULL;
DJI_Motor_t *loader     = NULL; // 拨盘电机

/* 摩擦轮/拨弹目标 ref: 宏值已是"输出轴端" rad/s, 与基类 measure 同单位, 可直接作 Motor_SetRef 期望。*/
#define FRICTION_WHEEL_REF 37.5f
#define LOADER_SINGLE_REF  11.7f
#define LOADER_BURST_REF   23.3f

void shoot_init(void)
{
    Motor_Init_Config_s friction_config = {
        .offline_init_config =
            {
                .name       = "3508_1", // 设备名称
                .timeout_ms = 100,      // 超时时间
                .beep_times = 5,        // 蜂鸣次数
                .enable     = 1,        // 是否启用离线管理
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan = BSP_CAN_HANDLE1,
            },
        .controller_init_config = {.lqr_init =
                                       {
                                           .K         = {0.0011f},
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
        .motor_init_info = {.motor_type = M3508, .gear_ratio = 19, .max_torque = 6, .torque_constant = 0.3f},
    };
    // 左摩擦轮
    friction_config.transport_config.can.tx_id = 1;
    friction_l                                 = Motor_DJI_Init(&friction_config);
    if (friction_l == NULL)
    {
        LOG_E("friction_l init failed");
        return;
    }
    // 右摩擦轮
    friction_config.transport_config.can.tx_id     = 2; // 右摩擦轮,改txid和方向就行
    friction_config.offline_init_config.name       = "3508_2";
    friction_config.offline_init_config.beep_times = 6;
    friction_r                                     = Motor_DJI_Init(&friction_config);
    if (friction_r == NULL)
    {
        LOG_E("friction_r init failed");
        return;
    }

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .offline_init_config =
            {
                .name       = "m2006", // 设备名称
                .timeout_ms = 100,     // 超时时间
                .beep_times = 7,       // 蜂鸣次数
                .enable     = 1,       // 是否启用离线管理
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 3,
            },
        .controller_init_config = {.lqr_init =
                                       {
                                           .K         = {0.005f}, // 0.0317
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
        .motor_init_info =
            {
                .motor_type      = M2006,
                .gear_ratio      = 36,
                .max_torque      = 1.8f,
                .torque_constant = 0.18f,
            },
    };
    loader = Motor_DJI_Init(&loader_config);
    if (loader == NULL)
    {
        LOG_E("loader init failed");
        return;
    }
}
void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd)
{
    if (!friction_l || !friction_r || !loader) return;

    if (shoot_cmd != NULL)
    {
        // 从cmd获取控制数据
        if (!Module_Offline_get_device_status(friction_l->base.offline_dev) && !Module_Offline_get_device_status(friction_r->base.offline_dev) &&
            !Module_Offline_get_device_status(loader->base.offline_dev))
        {
            if (shoot_cmd->shoot_mode == shoot_on)
            {
                Motor_Start((Motor_Base *)friction_l);
                Motor_Start((Motor_Base *)friction_r);
                Motor_Start((Motor_Base *)loader);
                // 确定是否开启摩擦轮
                if (shoot_cmd->friction_mode == friction_on)
                {
                    // 根据收到的弹速设置摩擦轮电机参考值(宏值已为输出轴端, 需实测后填入)
                    Motor_SetRef((Motor_Base *)friction_l, FRICTION_WHEEL_REF); // 左摩擦轮
                    Motor_SetRef((Motor_Base *)friction_r, -FRICTION_WHEEL_REF);
                    switch (shoot_cmd->load_mode)
                    {
                    // 停止拨盘
                    case load_stop:
                        Motor_SetRef((Motor_Base *)loader, 0);
                        break;
                    case load_1_bullet:
                        Motor_SetRef((Motor_Base *)loader, -LOADER_SINGLE_REF); // 单发拨弹
                        break;
                    case load_burstfire:
                        Motor_SetRef((Motor_Base *)loader, -LOADER_BURST_REF); // 连发拨弹
                        break;
                    default:
                        break;
                    }
                }
                else // 关闭摩擦轮
                {
                    Motor_SetRef((Motor_Base *)friction_l, 0);
                    Motor_SetRef((Motor_Base *)friction_r, 0);
                    Motor_SetRef((Motor_Base *)loader, 0);
                }
            }
            else
            {
                Motor_Stop((Motor_Base *)friction_l);
                Motor_Stop((Motor_Base *)friction_r);
                Motor_Stop((Motor_Base *)loader);
            }
        }
        else
        {
            Motor_Stop((Motor_Base *)friction_l);
            Motor_Stop((Motor_Base *)friction_r);
            Motor_Stop((Motor_Base *)loader);
        }
    }
    else
    {
        Motor_Stop((Motor_Base *)friction_l);
        Motor_Stop((Motor_Base *)friction_r);
        Motor_Stop((Motor_Base *)loader);
    }
}
