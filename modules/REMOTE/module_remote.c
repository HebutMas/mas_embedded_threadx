
#include "module_remote.h"

#include "bsp_def.h"
#include "module_offline.h"
#include "tx_api.h"

#include <string.h>

#include "sbus.h"
#include "dt7.h"
#include "vt02.h"
#include "vt03.h"

#define LOG_TAG "module_remote"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/*  模块内部变量 */
static Remote_Data_t              g_remote_data;                               /* 统一数据 (各子模块解码直接写入) */
static volatile bool              g_initialized;                               /* 模块初始化完成标志 */
static TX_THREAD                  g_remote_ctrl_task;                          /* 遥控器解码任务 */
static TX_THREAD                  g_vt_task;                                   /* 图传解码任务 */
APPS_STACK_SECTION static uint8_t g_remote_ctrl_stack[REMOTE_TASK_STACK_SIZE]; /* 遥控器任务栈 */
APPS_STACK_SECTION static uint8_t g_vt_task_stack[REMOTE_TASK_STACK_SIZE];     /* 图传任务栈 */
static Offline_Device            *g_rc_offline = NULL;                         /* 遥控器离线设备句柄 */
static Offline_Device            *g_vt_offline = NULL;                         /* 图传离线设备句柄 */

/**
 * @brief 遥控器解码任务
 */
static void remote_ctrl_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
#if (REMOTE_SOURCE == 1)
        remote_sbus_decode(&g_remote_data, g_rc_offline);
#elif (REMOTE_SOURCE == 2)
        remote_dt7_decode(&g_remote_data, g_rc_offline);
#else
        tx_thread_sleep(100);
#endif
    }
}

/**
 * @brief 图传解码任务
 */
static void vt_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
#if (REMOTE_VT_SOURCE == 1)
        remote_vt02_decode(&g_remote_data, g_vt_offline);
#elif (REMOTE_VT_SOURCE == 2)
        remote_vt03_decode(&g_remote_data, g_vt_offline);
#else
        tx_thread_sleep(100);
#endif
    }
}

int Module_Remote_init(void)
{
    if (g_initialized) return 0;

    memset(&g_remote_data, 0, sizeof(g_remote_data));

    /* ── 初始化遥控器子模块 ── */
#if (REMOTE_SOURCE == 1)
    if (remote_sbus_init(&g_rc_offline) != TX_SUCCESS)
    {
        LOG_E("SBUS init failed");
        return -1;
    }
#elif (REMOTE_SOURCE == 2)
    if (remote_dt7_init(&g_rc_offline) != TX_SUCCESS)
    {
        LOG_E("DT7 init failed");
        return -1;
    }
#endif

    /* ── 初始化图传子模块 ── */
#if (REMOTE_VT_SOURCE == 1)
    if (remote_vt02_init(&g_vt_offline) != TX_SUCCESS)
    {
        LOG_E("VT02 init failed");
        return -1;
    }
#elif (REMOTE_VT_SOURCE == 2)
    if (remote_vt03_init(&g_vt_offline) != TX_SUCCESS)
    {
        LOG_E("VT03 init failed");
        return -1;
    }
#endif

    /* 遥控器解码任务  */
    UINT status = tx_thread_create(&g_remote_ctrl_task, "Remote Ctrl", remote_ctrl_task_entry, 0, g_remote_ctrl_stack, REMOTE_TASK_STACK_SIZE,
                                   REMOTE_TASK_PRIORITY, REMOTE_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("Failed to create remote ctrl task");
        return -1;
    }

    /* 图传解码任务 */
    status = tx_thread_create(&g_vt_task, "Remote VT", vt_task_entry, 0, g_vt_task_stack, REMOTE_TASK_STACK_SIZE, REMOTE_TASK_VT_PRIORITY,
                              REMOTE_TASK_VT_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("Failed to create VT task");
        return -1;
    }

    g_initialized = true;
    LOG_I("Remote module initialized (RC=%d, VT=%d)", REMOTE_SOURCE, REMOTE_VT_SOURCE);

    return 0;
}

Remote_Data_t *Module_Remote_get_data(void)
{
    if (!g_initialized) return NULL;
    return &g_remote_data;
}

int16_t Module_Remote_get_channel(uint8_t ch)
{
    if (!g_initialized || ch < 1 || ch > 16) return 0;
    return g_remote_data.channels[ch - 1];
}

mouse_state_t *Module_Remote_get_mouse(void)
{
    if (!g_initialized) return NULL;
    return &g_remote_data.mouse;
}

keyboard_state_t *Module_Remote_get_keyboard(void)
{
    if (!g_initialized) return NULL;
    return &g_remote_data.keyboard;
}

dt7_custom_t *Module_Remote_get_dt7_custom(void)
{
    if (!g_initialized) return NULL;
    return &g_remote_data.dt7;
}

vt03_custom_t *Module_Remote_get_vt03_custom(void)
{
    if (!g_initialized) return NULL;
    return &g_remote_data.vt03;
}

uint8_t Module_Remote_get_offline_status(void)
{
    uint8_t rc_status = Module_Offline_get_device_status(g_rc_offline);
    uint8_t vt_status = Module_Offline_get_device_status(g_vt_offline);

    uint8_t result = 0;
    if (rc_status == STATE_ONLINE) result |= 0x01; /* bit0: 遥控器在线 */
    if (vt_status == STATE_ONLINE) result |= 0x02; /* bit1: 图传在线   */
    return result;
}
