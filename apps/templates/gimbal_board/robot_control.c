/*
 * @Description: 云台板主控任务 (模板)
 *
 * 使用说明:
 *   1. 将 <robot>_def.h 改为你的 def 文件
 *   2. 云台板: 遥控器 → 云台+发射控制 → 板间通讯 → 底盘板
 *   3. 同时接收底盘板的裁判系统数据用于视觉通信
 *   4. 板间通讯结构体在 <robot>_def.h 中定义 (取消 #if 0 以启用)
 */

#include "robot_control.h"
#include "module_boardcomm.h"
#include "module_ins.h"
#include "module_vision.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "<robot>_def.h" /* TODO: 改为 <你的机器人>_def.h */
#include "gimbal_func.h"
#include "shoot_func.h"
#include "robot_func.h"

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

static ReceivePacket *receive_packet = NULL;
static SendPacket     send_packet;
const static Ins_t   *ins = NULL;

static Gimbal_Ctrl_Cmd_t gimbal_cmd;
static Shoot_Ctrl_Cmd_t  shoot_cmd;
static uint16_t          yaw_ecd;

/* ── 板间通讯 ── */
static Chassis_Ctrl_Cmd_t        chassis_cmd;
static GimbalToChassis_cmd_t     chassis_send_cmd;
static ChassisToGimbal_referee_t chassis_upload_data;

static void robot_control_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        Module_BoardComm_Receive(&chassis_upload_data, sizeof(ChassisToGimbal_referee_t));

        /* ── 遥控器控制输入 ── */
        RemoteControlSet(&chassis_cmd, &shoot_cmd, &gimbal_cmd);

        /* ── 虚拟串口 (视觉通信) ──
         * 使用底盘板传回的裁判系统数据判断红蓝方 */
        send_packet.mode  = 1 - chassis_upload_data.robot_color;
        send_packet.q[0] = ins->q[0];
        send_packet.q[1] = ins->q[1];
        send_packet.q[2] = ins->q[2];
        send_packet.q[3] = ins->q[3];
        Module_Vision_Send(&send_packet, TX_NO_WAIT);
        receive_packet = Module_Vision_Receive();

        /* ── 云台控制 ── */
        const uint8_t yaw_ecd_valid = gimbal_func(&gimbal_cmd, &yaw_ecd);

        /* ── 发射机构控制 ── */
        shoot_func(&shoot_cmd);

        /* ── 板间通讯: 云台板 → 底盘板 ──
         * 速度比例 (-1.0~+1.0) → int8 (-10~+10) */
        chassis_send_cmd.vx = (int8_t)(chassis_cmd.vx * 10.0f);
        chassis_send_cmd.vy = (int8_t)(chassis_cmd.vy * 10.0f);
        chassis_send_cmd.wz = (int8_t)(chassis_cmd.wz * 10.0f);
        chassis_send_cmd.offset_angle = yaw_ecd_valid ? CalcOffsetAngle((float)yaw_ecd) : 0;
        chassis_send_cmd.chassis_mode = yaw_ecd_valid ? chassis_cmd.chassis_mode : chassis_zero_force;
        Module_BoardComm_Send((uint8_t *)&chassis_send_cmd, sizeof(GimbalToChassis_cmd_t));

        tx_thread_sleep(2);
    }
}

int robot_control_init(void)
{
    UINT status;

    /* 获取 INS 姿态数据指针 */
    ins = Module_INS_get();
    if (ins == NULL)
    {
        LOG_E("ins is null");
        return -1;
    }

    /* ── 子系统初始化 ── */
    if (gimbal_init() != 0 || shoot_init() != 0)
    {
        return -1;
    }

    status = tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 30, 30,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_task failed!");
        return -1;
    }

    LOG_I("robot_control init success!");

    return 0;
}
