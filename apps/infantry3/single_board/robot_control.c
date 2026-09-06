#include "robot_control.h"
#include "module_ins.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "infantry_def.h"
#include "gimbal_func.h"
#include "shoot_func.h"
#include "chassis_func.h"
#include "robot_func.h"

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

// 姿态角数据
const static Ins_t *ins = NULL;
// 云台,发射机构,底盘命令
static Gimbal_Ctrl_Cmd_t  gimbal_cmd;
static Shoot_Ctrl_Cmd_t   shoot_cmd;
static Chassis_Ctrl_Cmd_t chassis_cmd;
// 接收云台数据
static uint16_t yaw_ecd;
// 板间通讯部分

static void robot_control_task(ULONG thread_input)
{
    while (1)
    {
        /* 遥控器控制输入 */
        RemoteControlSet(&chassis_cmd, &shoot_cmd, &gimbal_cmd);

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

    /* 获取ins指针 */
    ins = Module_INS_get();
    if (ins == NULL)
    {
        LOG_E("ins is null");
        return;
    }

    gimbal_init();
    shoot_init();
    chassis_init();

    status = tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 30, 30,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_task failed!");
        return;
    }

    LOG_I("robot_control init success!");
}
