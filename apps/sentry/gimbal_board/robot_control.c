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
        gimbal_auto_func(&chassis_cmd,&shoot_cmd,&gimbal_cmd,ins,receive_packet);
        /* 云台控制 */
        gimbal_func(&gimbal_cmd, &yaw_ecd);
        /* 发射机构控制 */
        shoot_func(&shoot_cmd);

        /* 板间通讯 */
        // 速度比例 (-1.0~+1.0) → 板间 int8 (-10~+10)
        chassis_send_cmd.vx           = (int8_t)(chassis_cmd.vx * 10.0f);
        chassis_send_cmd.vy           = (int8_t)(chassis_cmd.vy * 10.0f);
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
