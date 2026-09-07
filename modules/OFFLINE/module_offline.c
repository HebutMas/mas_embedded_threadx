#include "module_offline.h"

#include "tx_api.h"
#include "bsp_def.h"
#include "bsp_dwt.h"
#include "bsp_beep.h"
#include "bsp_led.h"

#include <string.h>
#if OFFLINE_WATCHDOG_ENABLE
#include "iwdg.h"
#endif

#define LOG_TAG "Offline"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

struct Offline_Device
{
    Offline_Device *next;       /* 指向下一个设备指针 */
    char            name[32];   /* 设备名称  */
    uint32_t        timeout_ms; /* 超时时间  */
    uint8_t         beep_times; /* 蜂鸣次数*/
    bool            is_offline; /* 当前离线状态*/
    uint32_t        last_time;  /* 上次心跳时间戳 */
    uint8_t         enable;     /* 检测开关 */
};

// 模块内部变量

static Offline_Device            *g_device_list;                         /* 设备链表头 */
static volatile bool              g_silent_error;                        /* 静音报警标志  */
static bool                       g_initialized;                         /* 模块初始化完成标志*/
static TX_THREAD                  g_task_thread;                         /* 检测任务句柄 */
APPS_STACK_SECTION static uint8_t g_task_stack[OFFLINE_TASK_STACK_SIZE]; /* 检测任务栈*/

/* 检测任务 */
static void offline_detect_task_entry(ULONG arg)
{
    (void)arg;

    /* 蜂鸣/LED 状态机变量 */
    static uint32_t beep_period_start_time; /* 当前 2s 大周期起始时刻 */
    static uint32_t beep_cycle_start_time;  /* 当前闪烁子周期起始时刻 */
    static uint8_t  remaining_beep_cycles;  /* 当前大周期内剩余闪烁次数 */

    while (1)
    {
#if OFFLINE_WATCHDOG_ENABLE
#if defined(STM32H723xx)
        HAL_IWDG_Refresh(&hiwdg1);
#elif defined(STM32F407xx)
        HAL_IWDG_Refresh(&hiwdg);
#endif
#endif
        if (!g_initialized) break;

        /* 遍历设备链表, 聚合离线状态 */
        uint32_t now                = (uint32_t)BSP_DWT_GetTimeline_ms();
        bool     has_severe_fault   = false;
        bool     has_silent_fault   = false;
        uint8_t  request_beep_times = 0;

        for (Offline_Device *dev = g_device_list; dev; dev = dev->next)
        {
            if (!dev->enable) continue;

            uint32_t elapsed = now - dev->last_time;
            if (elapsed > dev->timeout_ms)
            {
                dev->is_offline = true;

                if (dev->beep_times > 0)
                {
                    has_severe_fault = true;
                    if (request_beep_times == 0 || dev->beep_times < request_beep_times) request_beep_times = dev->beep_times;
                }
                else
                {
                    has_silent_fault = true;
                }
            }
            else
            {
                dev->is_offline = false;
            }
        }

        uint8_t current_beep_times; /* 本周期蜂鸣次数 */

        if (has_severe_fault)
        {
            g_silent_error     = false;
            current_beep_times = request_beep_times;
        }
        else if (has_silent_fault)
        {
            g_silent_error     = true;
            current_beep_times = 0;
        }
        else
        {
            g_silent_error     = false;
            current_beep_times = 0;
        }

        /* 蜂鸣/LED  */

        if (g_silent_error)
        {
            /* 红灯常亮, 蜂鸣关闭, 重置状态机 */
            BSP_BEEP_Set(0, 0);
            BSP_LED_Show(LED_Red);
            remaining_beep_cycles  = 0;
            beep_period_start_time = now;
        }
        else
        {
            /* 检查是否进入新的大周期 */
            if (now - beep_period_start_time > 2000UL)
            {
                remaining_beep_cycles  = current_beep_times;
                beep_period_start_time = now;
                beep_cycle_start_time  = now;
            }

            if (remaining_beep_cycles > 0)
            {
                /* 子周期 0–100ms: 红灯亮 + 蜂鸣响 */
                if (now - beep_cycle_start_time < 100UL)
                {
#if OFFLINE_BEEP_ENABLE
                    BSP_BEEP_Set(OFFLINE_BEEP_TUNE_VALUE, OFFLINE_BEEP_CTRL_VALUE);
#else
                    BSP_BEEP_Set(0, 0);
#endif
                    BSP_LED_Show(LED_Red);
                }
                /* 子周期 100–200ms: 黑灯 + 蜂鸣停 */
                else if (now - beep_cycle_start_time < 200UL)
                {
                    BSP_BEEP_Set(0, 0);
                    BSP_LED_Show(LED_Black);
                }
                /* 子周期 >200ms: 完成一次闪烁, 递减计数, 进入下一次 */
                else
                {
                    remaining_beep_cycles--;
                    beep_cycle_start_time = now;
                }
            }
            else
            {
                /* 当前大周期内闪烁已完成 (或无故障) */
                BSP_BEEP_Set(0, 0);
                if (current_beep_times > 0)
                {
                    BSP_LED_Show(LED_Black);
                }
                else
                {
                    BSP_LED_Show(LED_Green);
                }
            }
        }

        tx_thread_sleep(10);
    }
}

void Module_Offline_init(void)
{
    if (g_initialized) return;

    g_device_list  = NULL;
    g_silent_error = false;

    BSP_BEEP_Start();
    BSP_LED_Show(LED_Green);

    if (tx_thread_create(&g_task_thread, "Offline Detect", offline_detect_task_entry, 0, g_task_stack, OFFLINE_TASK_STACK_SIZE, OFFLINE_TASK_PRIORITY,
                         OFFLINE_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("Failed to create offline detect task");
        return;
    }

#if OFFLINE_WATCHDOG_ENABLE
#if defined(STM32H723xx)
    MX_IWDG1_Init();
    __HAL_DBGMCU_FREEZE_IWDG1();
#elif defined(STM32F407xx)
    MX_IWDG_Init();
    __HAL_DBGMCU_FREEZE_IWDG();
#endif
#endif

    g_initialized = true;
    LOG_I("Offline module initialized");
}

Offline_Device *Module_Offline_register(const Offline_Init_config_t *init)
{
    if (init == NULL || !g_initialized)
    {
        LOG_E("Register failed: init=%p initialized=%d", (void *)init, g_initialized);
        return NULL;
    }

    Offline_Device *dev = NULL;
    BSP_MEM_ALLOC_WAIT(dev, sizeof(Offline_Device), TX_NO_WAIT);
    if (dev == NULL)
    {
        LOG_E("Register [%s] failed: out of memory", init->name ? init->name : "?");
        return NULL;
    }

    /* 初始化节点 */
    memset(dev, 0, sizeof(Offline_Device));
    if (init->name)
    {
        strncpy(dev->name, init->name, sizeof(dev->name) - 1);
        dev->name[sizeof(dev->name) - 1] = '\0';
    }
    dev->timeout_ms = init->timeout_ms;
    dev->beep_times = init->beep_times;
    dev->enable     = init->enable;
    dev->is_offline = false;
    dev->last_time  = (uint32_t)BSP_DWT_GetTimeline_ms(); /* 初始心跳 = 当前时间 */

    /* 入链 */
    dev->next     = g_device_list;
    g_device_list = dev;

    LOG_I("Device [%s] registered (timeout=%lu ms, beep=%u)", dev->name, (unsigned long)dev->timeout_ms, dev->beep_times);
    return dev;
}

void Module_Offline_device_update(Offline_Device *dev)
{
    if (dev == NULL) return;
    dev->last_time = (uint32_t)BSP_DWT_GetTimeline_ms();
}

void Module_Offline_device_enable(Offline_Device *dev)
{
    if (dev == NULL) return;
    dev->enable = 1;
}

void Module_Offline_device_disable(Offline_Device *dev)
{
    if (dev == NULL) return;
    dev->enable = 0;
}

uint8_t Module_Offline_get_device_status(Offline_Device *dev)
{
    if (dev == NULL) return STATE_OFFLINE;
    return dev->is_offline ? STATE_OFFLINE : STATE_ONLINE;
}