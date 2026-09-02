#include "robot_control.h"
#include "chassis_func.h"
#include "module_boardcomm.h"
#include "module_offline.h"
#include "robot_func.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "sentry_def.h"

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];


// 底盘命令
static Chassis_Ctrl_Cmd_t chassis_cmd;
// 板间通讯部分
static GimbalToChassis_cmd_t     chassis_recv_cmd;
static ChassisToGimbal_referee_t chassis_send_data;

static void robot_control_task(ULONG thread_input)
{
    while (1)
    {
        /* 板间通讯 */
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
            // 板间底盘系 mm/s -> m/s；线速度已经在云台板完成坐标转换
            chassis_cmd.vx           = (float)chassis_recv_cmd.vx / 1000.0f;
            chassis_cmd.vy           = (float)chassis_recv_cmd.vy / 1000.0f;
            chassis_cmd.wz           = (float)chassis_recv_cmd.wz / 10.0f * CHASSIS_MAX_SPEED_MPS;
            // 编码器差值 → 角度: ecd / 8191 * 360
            chassis_cmd.offset_angle = (float)chassis_recv_cmd.offset_angle * 360.0f / 8191.0f;
            chassis_cmd.chassis_mode = chassis_recv_cmd.chassis_mode;
        }

        handle_referee_data(&chassis_send_data);
        Module_BoardComm_Send((uint8_t *)&chassis_send_data, sizeof(ChassisToGimbal_referee_t));

        /* 底盘控制 */
        chassis_func(&chassis_cmd);

        tx_thread_sleep(2);
    }
}

void robot_control_init(void)
{
    UINT status;

    /* 底盘初始化 */
    chassis_init();

    /* 板间通讯注册 */
    Module_BoardComm_RegisterRxBuffer(&chassis_recv_cmd, sizeof(GimbalToChassis_cmd_t));

    status = tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 30, 30,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_task failed!");
        return;
    }

    LOG_I("robot_control init success!");
}
