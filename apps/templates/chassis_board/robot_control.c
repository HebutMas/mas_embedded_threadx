/*
 * @Description: 底盘板主控任务 (模板)
 *
 * 使用说明:
 *   1. 将 <robot>_def.h 改为你的 def 文件
 *   2. 底盘板: 接收云台板指令 → 底盘控制 → 回传裁判系统数据
 *   3. 板间通讯离线时自动停止底盘
 */

#include "robot_control.h"
#include "chassis_func.h"
#include "module_boardcomm.h"
#include "module_offline.h"
#include "robot_func.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "<robot>_def.h" /* TODO: 改为 <你的机器人>_def.h */

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

/* ── 底盘命令 ── */
static Chassis_Ctrl_Cmd_t chassis_cmd;

/* ── 板间通讯 ── */
static GimbalToChassis_cmd_t     chassis_recv_cmd;
static ChassisToGimbal_referee_t chassis_send_data;

static void robot_control_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        /* ── 板间通讯接收 ──
         * 离线时进入安全模式 (所有执行器 zero_force) */
        if (Module_BoardComm_Get_Offline_State() == STATE_OFFLINE)
        {
            chassis_cmd.vx           = 0;
            chassis_cmd.vy           = 0;
            chassis_cmd.wz           = 0;
            chassis_cmd.offset_angle = 0;
            chassis_cmd.chassis_mode = chassis_zero_force;
        }
        else
        {
            /* 板间 int8 (-10~+10) → 实际速度 (m/s) */
            chassis_cmd.vx = (float)chassis_recv_cmd.vx / 10.0f * CHASSIS_MAX_SPEED_MPS;
            chassis_cmd.vy = (float)chassis_recv_cmd.vy / 10.0f * CHASSIS_MAX_SPEED_MPS;
            chassis_cmd.wz = (float)chassis_recv_cmd.wz / 10.0f * CHASSIS_MAX_SPEED_MPS;

            /* 编码器差值 → 角度 (度) */
            chassis_cmd.offset_angle = (float)chassis_recv_cmd.offset_angle * 360.0f / 8191.0f;
            chassis_cmd.chassis_mode = chassis_recv_cmd.chassis_mode;
        }

        /* ── 裁判系统数据 → 云台板 ── */
        handle_referee_data(&chassis_send_data);
        Module_BoardComm_Send((uint8_t *)&chassis_send_data, sizeof(ChassisToGimbal_referee_t));

        /* ── 底盘控制 ── */
        chassis_func(&chassis_cmd);

        tx_thread_sleep(2);
    }
}

int robot_control_init(void)
{
    UINT status;

    if (chassis_init() != 0)
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
