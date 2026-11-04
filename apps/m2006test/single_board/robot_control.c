/*
 * @file        robot_control.c
 * @brief       m2006test 兵种 — M2006 拨盘电机测试
 *
 * 用途: 验证 2006 电机与过弹盘机械结构
 *  - 上电后拨盘以连发转速 -8000 RPM 持续旋转 (与步兵3/哨兵拨盘同约定)
 *  - 电机离线时停止, 恢复后自动继续
 *  - 电机配置照抄 infantry3 shoot_func 的拨盘 (CAN1, tx_id 7, 36:1)
 */
#include "robot_control.h"
#include "module_offline.h"
#include "motor_def.h"
#include "motor_dji.h"
#include "user_lib.h"
#include "tx_api.h"
#include "bsp_def.h"

#define LOG_TAG "m2006_test"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static DJI_Motor_t *loader = NULL; /* 拨盘电机 */

/* ========== 主循环任务 ========== */
static TX_THREAD                  g_dial_thread;
APPS_STACK_SECTION static uint8_t g_dial_stack[1024];

static void dial_task_entry(ULONG arg)
{
    (void)arg;

    /* 上电即转: 使能 + 连发档参考转速 */
    Motor_Start((Motor_Base *)loader);
    Motor_SetRef((Motor_Base *)loader, -8000.0f * RPM_2_RAD_PER_SEC);

    while (1)
    {
        /* 离线检测: 掉线停转, 恢复后重新使能并给参考 */
        if (Module_Offline_get_device_status(loader->base.offline_dev))
        {
            Motor_Stop((Motor_Base *)loader);
        }
        else
        {
            Motor_Start((Motor_Base *)loader);
            Motor_SetRef((Motor_Base *)loader, -8000.0f * RPM_2_RAD_PER_SEC);
        }

        tx_thread_sleep(2);
    }
}

/* ========== 入口 ========== */

void robot_control_init(void)
{
    /* 拨盘电机 (照抄 infantry3 shoot_func 的 loader 配置) */
    Motor_Init_Config_s loader_config = {
        .offline_init_config =
            {
                .name       = "m2006", // 设备名称
                .timeout_ms = 100,     // 超时时间
                .beep_times = 7,       // 蜂鸣次数
                .enable     = 1,       // 是否启用离线管理
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 7,
            },
        .controller_init_config = {.lqr_init =
                                       {
                                           .K         = {0.005f}, // 0.0317
                                           .state_dim = 1,
                                       }},
        .setting_init_config =
            {
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = SPEED_LOOP,
                .feedback_reverse_flag = 0,
                .algorithm_type        = CONTROL_LQR,
            },
        .motor_init_info =
            {
                .motor_type      = M2006,
                .gear_ratio      = 36,
                .max_torque      = 1.8f,
                .torque_constant = 0.005f,
            },
    };
    loader = Motor_DJI_Init(&loader_config);
    if (loader == NULL)
    {
        LOG_E("loader init failed");
        return;
    }

    /* 主循环任务 (模块注册需在 MODULE_Init 之后, 此处即可) */
    UINT status = tx_thread_create(&g_dial_thread, "dial_loop", dial_task_entry, 0,
                                   g_dial_stack, sizeof(g_dial_stack),
                                   8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("dial thread create failed (0x%02x)", status);
        return;
    }

    LOG_I("m2006test init done");
}
