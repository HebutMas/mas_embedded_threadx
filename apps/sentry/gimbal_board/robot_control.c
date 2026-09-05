#include "robot_control.h"
#include "module_boardcomm.h"
#include "module_ins.h"
#include "module_vision.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "sentry_def.h"
#include "gimbal_func.h"
#include "shoot_func.h"
#include "robot_func.h"
#include "user_lib.h"
#include <math.h>

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
// 云台与发射机构命令
static Gimbal_Ctrl_Cmd_t gimbal_cmd;
static Shoot_Ctrl_Cmd_t  shoot_cmd;
// 接收云台数据
static uint16_t yaw_ecd;
// 板间通讯部分
static Chassis_Ctrl_Cmd_t        chassis_cmd;
static GimbalToChassis_cmd_t     chassis_send_cmd;
static ChassisToGimbal_referee_t chassis_upload_data;

/*
 * Navigation and RC velocities are expressed in the gimbal frame.  The
 * chassis board must receive chassis-frame velocities, so do this rotation
 * once here while the yaw encoder sample is available.
 */
static void project_velocity_to_chassis(float gimbal_vx, float gimbal_vy, uint16_t yaw_ecd_value, float *chassis_vx, float *chassis_vy)
{
    const float offset_angle = (float)CalcOffsetAngle(yaw_ecd_value) * 360.0f / 8191.0f;
    const float theta        = offset_angle * DEGREE_2_RAD;
    const float cos_theta    = cosf(theta);
    const float sin_theta    = sinf(theta);

    *chassis_vx = gimbal_vx * cos_theta + gimbal_vy * sin_theta;
    *chassis_vy = -gimbal_vx * sin_theta + gimbal_vy * cos_theta;
}

static void robot_control_task(ULONG thread_input)
{
    while (1)
    {

        /* 遥控器控制输入 */
        RemoteControlSet(&chassis_cmd, &shoot_cmd, &gimbal_cmd);

        /* 虚拟串口 */
        send_packet.mode = 1 - chassis_upload_data.robot_color;
        send_packet.q[0] = ins->q[0];
        send_packet.q[1] = ins->q[1];
        send_packet.q[2] = ins->q[2];
        send_packet.q[3] = ins->q[3];
        Module_Vision_Send(&send_packet, TX_NO_WAIT);
        receive_packet = Module_Vision_Receive();
        /* 自动模式 */
        gimbal_auto_func(&chassis_cmd, &shoot_cmd, &gimbal_cmd, ins, receive_packet);
        /* 云台控制 */
        gimbal_func(&gimbal_cmd, &yaw_ecd);
        /* 发射机构控制 */
        shoot_func(&shoot_cmd);

        /* 板间通讯 */
        float chassis_vx;
        float chassis_vy;
        project_velocity_to_chassis(chassis_cmd.vx, chassis_cmd.vy, yaw_ecd, &chassis_vx, &chassis_vy);
        VAL_LIMIT(chassis_vx, -CHASSIS_MAX_SPEED_MPS, CHASSIS_MAX_SPEED_MPS);
        VAL_LIMIT(chassis_vy, -CHASSIS_MAX_SPEED_MPS, CHASSIS_MAX_SPEED_MPS);
        // 底盘系 m/s -> 板间 mm/s，底盘端按 1:1 恢复
        chassis_send_cmd.vx           = (int16_t)lroundf(chassis_vx * 1000.0f);
        chassis_send_cmd.vy           = (int16_t)lroundf(chassis_vy * 1000.0f);
        chassis_send_cmd.wz           = (int8_t)(chassis_cmd.wz * 10.0f);
        chassis_send_cmd.offset_angle = CalcOffsetAngle(yaw_ecd);
        chassis_send_cmd.chassis_mode = chassis_cmd.chassis_mode;
        Module_BoardComm_Send((uint8_t *)&chassis_send_cmd, sizeof(GimbalToChassis_cmd_t));

        tx_thread_sleep(2);
    }
}

void robot_control_init(void)
{
    UINT status;

    /* 获取ins指针 */
    ins = Module_INS_get();
    if (ins == NULL)
    {
        LOG_E("ins is null");
        return;
    }

    /* 云台初始化 */
    gimbal_init();
    /* 发射机构初始化 */
    shoot_init();

    /* 板间通讯注册 */
    Module_BoardComm_RegisterRxBuffer(&chassis_upload_data, sizeof(ChassisToGimbal_referee_t));

    status = tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 30, 30,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_task failed!");
        return;
    }

    LOG_I("robot_control init success!");
}
