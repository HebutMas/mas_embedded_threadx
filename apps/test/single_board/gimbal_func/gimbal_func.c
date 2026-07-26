/*
 * @Author: AdoreElysia w2006825@qq.com
 * @Date: 2026-07-20 19:36:04
 * @LastEditors: AdoreElysia w2006825@qq.com
 * @LastEditTime: 2026-07-21 14:38:13
 * @FilePath: \mas_embedded_threadx-f103\apps\test\single_board\gimbal_func\gimbal_func.c
 * @Description: 
 */
/*
 * @Description: 云台功能 — F103 测试用
 *   gimbal_init(): 初始化达妙电机 DM4310, MIT 模式
 *   gimbal_func(): 留空 (不使用标准 zero_force/gyro_mode/auto_mode)
 */

#include "gimbal_func.h"
#include "module_motor.h"
#include "motor_damiao.h"

#define LOG_TAG "app_gimbal"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static DM_Motor_t *yaw_motor = NULL;

void gimbal_init(void)
{
    Motor_Init_Config_s cfg = {
        .offline_init_config =
            {
                .name       = "dm4310",
                .timeout_ms = 100,
                .beep_times = 1,
                .enable     = 1,
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 0x01,
                .rx_id = 0xf1,
            },
        .controller_init_config =
            {
                .speed_PID =
                    {
                        .Kp = 0.0f, .Ki = 0.0f, .Kd = 0.0f,
                        .MaxOut = 10.0f, .DeadBand = 0.0f,
                        .IntegralLimit = 3.0f,
                        .Improve = PID_Integral_Limit,
                    },
                .angle_PID = {0},
            },
        .setting_init_config =
            {
                .algorithm_type        = CONTROL_PID,
                .feedback_reverse_flag = 0,
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = SPEED_LOOP,
            },
        .motor_init_info = {.motor_type = DM4310, .gear_ratio = 1, .max_torque = 10.0f, .torque_constant = 0.093f},
    };

    yaw_motor = Motor_DM_Init(&cfg, DM_MIT_MODE);
    if (yaw_motor != NULL)
    {
        Motor_DM_Start(yaw_motor);
        Motor_DM_SetRef(yaw_motor, 0.0f);
        LOG_I("gimbal yaw DM4310 initialized (CAN1 ID=0x01)");
    }
    else
    {
        LOG_E("gimbal yaw DM4310 init FAILED");
    }
}

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd)
{
    (void)gimbal_cmd;
    (void)yaw_ecd;
    /* 暂不使用标准云台控制模式，留空 */
}
