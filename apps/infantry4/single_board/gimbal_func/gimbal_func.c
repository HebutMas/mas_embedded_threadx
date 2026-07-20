/*
 * @Author: zhishang06 2494841771@qq.com
 * @Date: 2026-07-18 11:35:16
 * @LastEditors: zhishang06 2494841771@qq.com
 * @LastEditTime: 2026-07-19 15:29:22
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\gimbal_func\gimbal_func.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "gimbal_func.h"
#include "motor_dji.h"
#include "user_lib.h"
#include "module_bmi088.h"
#include "motor_damiao.h"
#include "module_ins.h"
#include "user_lib.h"

static DJI_Motor_t *yaw_motor = NULL;
static DM_Motor_t  *pitch_motor = NULL; // pitch电机指针

const static Bmi088_device_t *bmi088_dev = NULL;
const static Ins_t *ins = NULL;
void gimbal_init(void)
{
    ins = Module_INS_get();
    bmi088_dev = Module_BMI088_get_device();
    if (bmi088_dev == NULL)
    {
       
        return;
    }
    Motor_Init_Config_s yaw_config = {
        .offline_init_config =
            {
                .name       = "6020",
                .timeout_ms = 100,
                .beep_times = 3,
                .enable     = 1,
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 1,
            },
        .controller_init_config = {.other_angle_feedback_ptr = &ins->YawTotalAngle_rad,
                                    .other_speed_feedback_ptr = &bmi088_dev->gyro[2],
                                   .lqr_init =
                                       {
                                           .K         = {5.47f, 0.56f},
                                           .state_dim = 2,
                                       }},

            .setting_init_config =
            {
                .algorithm_type        = CONTROL_LQR,
                .feedback_reverse_flag = 0,// 0: 正向反馈, 1: 反向反馈
                .angle_feedback_source = 1,// 0: 陀螺仪反馈, 1: 角度反馈
                .speed_feedback_source = 1,// 0: 速度反馈, 1: 角度反馈
                .loop_type             = ANGLE_LOOP,// 0: 速度环, 1: 角度环
            },
        .motor_init_info = {.motor_type = GM6020_CURRENT, .gear_ratio = 1, .max_torque = 2.223f, .torque_constant = 0.741f}};
    yaw_motor = Motor_DJI_Init(&yaw_config);

// PITCH
    Motor_Init_Config_s pitch_config = {.offline_init_config =
                                            {
                                                .name       = "dm4310",
                                                .timeout_ms = 100,
                                                .beep_times = 4,
                                                .enable     = 1,
                                            },
                                        .transport = MOTOR_TRANSPORT_CAN,
                                        .transport_config.can =
                                            {
                                                .hcan  = BSP_CAN_HANDLE2,
                                                .tx_id = 0X01,
                                                .rx_id = 0Xf1,
                                            },
                                        .controller_init_config =
                                            {
                                                .other_angle_feedback_ptr = &ins->euler_rad[1],
                                                .other_speed_feedback_ptr = &bmi088_dev->gyro[0], // c板的pitch轴角速度，根据实际选择对应角速度
                                                .lqr_init =
                                                    {
                                                        .K         = {5.4f,0.6f}, // 28.7312f,2.5974f
                                                        .state_dim = 2, 
                                                    },
                                            },
                                        .setting_init_config =
                                            {
                                                .algorithm_type        = CONTROL_LQR,
                                                .feedback_reverse_flag = 1,
                                                .angle_feedback_source = 1,
                                                .speed_feedback_source = 1,
                                                .loop_type             = ANGLE_LOOP,
                                            },
                                        .motor_init_info = {.motor_type = DM4310, .gear_ratio = 10, .max_torque = 10, .torque_constant = 0.093f}};
                                       pitch_motor  = Motor_DM_Init(&pitch_config, DM_MIT_MODE);
    
}

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd)
{
    if (gimbal_cmd == NULL || yaw_motor == NULL)
        return;

    if (!Module_Offline_get_device_status(yaw_motor->base.offline_dev))
    {
        if (gimbal_cmd->gimbal_mode == gimbal_gyro_mode)
        {
            Motor_DJI_Start(yaw_motor);
            Motor_DM_Start(pitch_motor);
            Motor_DJI_SetRef(yaw_motor, gimbal_cmd->yaw * DEGREE_2_RAD);
             Motor_DM_SetRef(pitch_motor, gimbal_cmd->pitch * DEGREE_2_RAD);
        }
        else
        {
            Motor_DJI_Stop(yaw_motor);
             Motor_DM_Stop(pitch_motor);
        }
    }
    else
    {
        Motor_DJI_Stop(yaw_motor);
        Motor_DM_Stop(pitch_motor);
    }

    if (yaw_ecd != NULL)
    {
        *yaw_ecd = yaw_motor->measure.ecd;
    }
}