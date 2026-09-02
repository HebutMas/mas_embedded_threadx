#include "module_motor.h"
#include "power_control.h"
#include "tx_api.h"
#include "bsp_def.h"

#define LOG_TAG "Module_Motor"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  motor_thread;
APPS_STACK_SECTION static uint8_t motor_thread_stack[MOTOR_TASK_STACK_SIZE];

static void motor_task_entry(ULONG thread_input)
{
    while (1)
    {
        /* 控制计算 */
        Motor_ControlAll();
        /* 功率限制 */
        PowerControl_Update();
        /* 输出应用 */
        Motor_ApplyAll();

        tx_thread_sleep(2);
    }
}

void Module_Motor_Init(void)
{
    UINT ret = tx_thread_create(&motor_thread, "motor", motor_task_entry, 0, motor_thread_stack, MOTOR_TASK_STACK_SIZE, MOTOR_TASK_PRIORITY,
                                MOTOR_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (ret != TX_SUCCESS)
    {
        LOG_E("Failed to create motor thread");
    }

    LOG_I("Motor module initialized");
}
