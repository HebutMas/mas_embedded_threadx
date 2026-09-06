/*
 * @file        robot_control.c
 * @brief       test 兵种 — 新模块接入测试骨架
 *
 * ────────────────────────────────────────────────────────────────
 *  新模块开发工作流:
 *    ① 编写新模块 (modules/<MOD>/), 加入三步注册
 *    ② 在 config.cmake 切 ROBOT=test, 在本文件 主循环区 写临时测试代码
 *       (调用/注册新模块 API, 观察日志/串口/波形等验证)
 *    ③ 测试成功 → 删除本文件主循环区测试代码 (恢复空循环)
 *    ④ 把已验证的用法整理进 新模块 .h 的使用示例注释
 *    ⑤ 提交新模块 (test 兵种作为骨架保留, 不再依赖主循环测试)
 * ────────────────────────────────────────────────────────────────
 */
#include "robot_control.h"
#include "tx_api.h"
#include "bsp_def.h"

#define LOG_TAG "test_robot"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* ========== 主循环任务 ========== */
static TX_THREAD                  g_test_loop_thread;
APPS_STACK_SECTION static uint8_t g_test_loop_stack[1024];

static void test_loop_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        /* ===== 临时测试区 (工作流②): 测试新模块 API, 成功后删除 ===== */
        /* 示例: 调用/注册待测模块 API, 周期性打印或触发验证          */
        /* ============================================================ */

        tx_thread_sleep(10);
    }
}

/* ========== 入口 ========== */

void robot_control_init(void)
{
    /* 主循环任务 (模块注册需在 MODULE_Init 之后, 此处即可) */
    UINT status = tx_thread_create(&g_test_loop_thread, "test_loop", test_loop_task_entry, 0,
                                   g_test_loop_stack, sizeof(g_test_loop_stack),
                                   8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("test loop thread create failed (0x%02x)", status);
        return;
    }

    LOG_I("test robot init done");
}