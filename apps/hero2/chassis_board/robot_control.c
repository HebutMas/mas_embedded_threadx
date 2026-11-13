/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-24 18:16:58
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-08-28 21:00:51
 * @FilePath: \mas_embedded_threadx\apps\hero2\chassis_board\robot_control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-23 13:12:06
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-07-24 10:25:26
 * @FilePath: \mas_embedded_threadx\apps\infantrytest\single_board\robot_control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "robot_control.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "chassis_func.h"
#include "robot_func.h"
#include "user_lib.h"
#include "module_offline.h"


static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];
static Chassis_Ctrl_Cmd_t chassis_cmd;

static void robot_control_task(ULONG thread_input)
{
    while (1)
    {
        RemoteControlSet(&chassis_cmd);
        chassis_func(&chassis_cmd);
        tx_thread_sleep(2);
    }
}

void robot_control_init(void)
{
    

    chassis_init();

    tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 30, 30,
                              TX_NO_TIME_SLICE, TX_AUTO_START);

}