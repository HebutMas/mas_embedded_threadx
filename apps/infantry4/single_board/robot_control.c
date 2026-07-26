/*
 * @Author: zhishang06 2494841771@qq.com
 * @Date: 2026-07-18 11:56:47
 * @LastEditors: zhishang06 2494841771@qq.com
 * @LastEditTime: 2026-07-26 09:48:26
 * @FilePath: \mas_embedded_threadx\apps\infantry4\single_board\robot_control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_control.h"
#include "referee_protocol.h"
#include "module_referee.h"
#include "module_ins.h"
#include "module_vision.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "infantry_def.h"
#include "user_lib.h"
#include "gimbal_func.h"
#include "shoot_func.h"
#include "robot_func.h"
#include "chassis_func.h"

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

// 虚拟串口数据结构体
static ReceivePacket *receive_packet = NULL;
static SendPacket     send_packet;
// 姿态角数据
const static Ins_t *ins = NULL;

static Gimbal_Ctrl_Cmd_t gimbal_cmd = {0};
static uint16_t         yaw_ecd = 0;
static Shoot_Ctrl_Cmd_t shoot_cmd = {0};
static Chassis_Ctrl_Cmd_t chassis_cmd = {0};


static void robot_control_task(ULONG thread_input)
{
    while (1)
    {
        /* 遥控器控制输入 */
        RemoteControlSet(&chassis_cmd, &shoot_cmd, &gimbal_cmd);

        /* 虚拟串口 */
        const robot_status_t *robot_status = (robot_status_t *)Module_Referee_Get_cmd_data(CMD_ID_ROBOT_STATUS);
        if (robot_status)
        {
            if (robot_status->robot_id >= 100)
            {
                send_packet.mode = 0; // 识别红方
            }
            else
            {
                send_packet.mode = 1; // 识别蓝方
            }
        }
        else
        {
            send_packet.mode = 0; // 默认识别红方
        }
        send_packet.q[0] = ins->q[0];
        send_packet.q[1] = ins->q[1];
        send_packet.q[2] = ins->q[2];
        send_packet.q[3] = ins->q[3];
        Module_Vision_Send(&send_packet, TX_NO_WAIT);

        receive_packet = Module_Vision_Receive();

        /* 视觉自动跟踪 - 仅在遥控器选择自动模式时生效 */
        if (gimbal_cmd.gimbal_mode == gimbal_auto_mode && receive_packet != NULL && receive_packet->found)
        {
            gimbal_cmd.yaw   = receive_packet->target_yaw * RAD_2_DEGREE;
            gimbal_cmd.pitch = receive_packet->target_pitch * RAD_2_DEGREE;
            if (receive_packet->fire_advice)
            {
                shoot_cmd.shoot_mode = shoot_on;
                shoot_cmd.friction_mode = friction_on;
            }
        }

        /* 云台控制 */
        gimbal_func(&gimbal_cmd, &yaw_ecd);

        /* 发射机构控制 */
        shoot_func(&shoot_cmd);
         /* 底盘控制 */
        chassis_cmd.offset_angle = CalcOffsetAngle(yaw_ecd) * 360.0f / 8191.0f;
        chassis_cmd.vx           = chassis_cmd.vx * CHASSIS_MAX_SPEED_MPS;
        chassis_cmd.vy           = chassis_cmd.vy * CHASSIS_MAX_SPEED_MPS;
        chassis_cmd.wz           = chassis_cmd.wz * CHASSIS_MAX_SPEED_MPS;
        chassis_func(&chassis_cmd);

        tx_thread_sleep(2);
    }
}

void robot_control_init(void)
{
    UINT status;

     ins = Module_INS_get();
    gimbal_init();
    shoot_init();
     chassis_init();
    status = tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 15, 15,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_task failed!");
        return;
    }

    LOG_I("robot_control init success!");
}