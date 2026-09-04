/*
 * @file        robot_control.c
 * @brief       test 兵种 — VOFA 链路测试
 *
 * 测试内容:
 *   1. 注册 "isonline" 接收消息 → 收到即更新掉线检测 (Module_Offline_device_update)
 *      验证: VOFA 下行 → 上位机心跳喂狗链路
 *   2. 写一个 param 参数, 获得其指针 → RegisterRx("set_param",&param)
 *      验证: VOFA 下行 set_param → 直接更新 param 变量
 *   3. 不断发送 正弦波 + param 当前值 (RegisterTx + 周期 VOFA_Send)
 *      验证: VOFA 上行上报链路
 *
 * 依赖: MODULE_VOFA 已在模块层初始化(UART6 + 接收线程), 见 Module_VOFA_Init
 *      本文件仅做 Register + 发送线程
 */
#include "robot_control.h"
#include "tx_api.h"
#include "vofa.h"
#include "module_offline.h"
#include "bsp_def.h"
#include <math.h>

#define LOG_TAG "test_vofa"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* ---- 测试变量 ---- */

/* 掉线检测: 模拟一条 "上位机连接" 链路, isonline 消息喂它 */
static Offline_Device *g_vofa_link_offline = NULL;

/* param: 供下行 set_param 更新的目标变量 */
static float g_test_param = 0.0f;

/* 上行参数 */
static float g_sin_val  = 0.0f;

/* 发送线程 */
static TX_THREAD                  g_tx_thread;
APPS_STACK_SECTION static uint8_t g_tx_stack[1024];

/* ---- 回调: 收到 isonline 消息 → 更新掉线检测 ---- */
static void vofa_isonline_callback(void)
{
    if (g_vofa_link_offline != NULL)
    {
        Module_Offline_device_update(g_vofa_link_offline);
    }
}

/* ---- 发送线程: 周期发送 正弦波 + param ---- */
static void tx_thread_entry(ULONG arg)
{
    (void)arg;
    float angle = 0.0f;
    const float freq_step = 0.05f;   /* 正弦步进 */
    const float TWO_PI = 6.2831853f;

    while (1)
    {
        /* 正弦波 */
        g_sin_val = sinf(angle);
        angle += freq_step;
        if (angle > TWO_PI) angle -= TWO_PI;

        /* 一次性发送: 顺序 = RegisterTx 注册顺序 */
        VOFA_Send();

        tx_thread_sleep(10);   /* 10ms 周期 → 100Hz 上报 */
    }
}

/* ---- 入口 ---- */
void robot_control_init(void)
{
    /* 1. 注册 "vofa_link" 掉线检测设备 (超时 500ms) */
    Offline_Init_config_t offline_cfg = {
        .name       = "vofa_link",
        .timeout_ms = 500,
        .beep_times = 0,
        .enable     = 1,
    };
    g_vofa_link_offline = Module_Offline_register(&offline_cfg);
    if (g_vofa_link_offline == NULL)
    {
        LOG_E("vofa_link offline register failed");
        return;
    }

    /* 2. 注册接收消息 */
    /*   isonline: 收到 → 回调里喂心跳 (测试1) */
    VOFA_RegisterRx("isonline", &g_test_param, vofa_isonline_callback);
    /*   set_param: 收到 → 直接写 g_test_param (测试2) */
    VOFA_RegisterRx("set_param", &g_test_param, NULL);

    /* 3. 注册上行参数 */
    VOFA_RegisterTx("sin", &g_sin_val);      /* 正弦波 (测试3) */
    VOFA_RegisterTx("param", &g_test_param); /* param 指针当前值 (测试3) */

    /* 4. 创建发送线程 */
    UINT status = tx_thread_create(&g_tx_thread, "vofa_tx", tx_thread_entry, 0,
                                   g_tx_stack, sizeof(g_tx_stack),
                                   10, 10, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("vofa tx thread create failed (0x%02x)", status);
        return;
    }

    LOG_I("test robot init done (vofa link test)");
}
