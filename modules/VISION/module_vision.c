/*
 * @Author: sji733055-glitch sji733055@gmail.com
 * @Date: 2026-07-01 19:24:17
 * @LastEditors: sji733055-glitch sji733055@gmail.com
 * @LastEditTime: 2026-07-15 08:59:45
 * @FilePath: \mas_embedded_threadx\modules\VISION\module_vision.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "module_vision.h"
#include "bsp_def.h"
#include "usbd_cdc_acm_user.h"
#include <string.h>
#include "tx_api.h"
#include "module_offline.h"
#include "crc_rm.h"

#define LOG_TAG "module_vision"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 双缓冲区 */
static ReceivePacket              rx_packet[2]; // 双缓冲区
static volatile uint32_t          rx_idx;       // 正在写的槽
static ReceivePacket              rx_snapshot;  // 接收快照
static Offline_Device            *offline_dev = NULL;
static TX_THREAD                  vision_thread;
APPS_STACK_SECTION static uint8_t vision_thread_stack[VISION_TASK_STACK_SIZE];

/**
 * @brief 视觉任务 — 阻塞读取 USB 数据, 扫描定长帧并校验 CRC16
 */
static void vision_thread_entry(ULONG arg)
{
    uint8_t  buf[64];
    uint32_t rx_len;

    while (1)
    {
        int ret = cdc_acm_recv(buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER);
        if (ret <= 0)
        {
            tx_thread_sleep(1);
            continue;
        }

        for (uint32_t i = 0; i + VISION_FRAME_RX_SIZE <= rx_len; i++)
        {
            if (buf[i] == VISION_RX_HEADER && Verify_CRC16_Check_Sum(&buf[i], VISION_FRAME_RX_SIZE))
            {
                rx_packet[rx_idx] = *((const ReceivePacket *)&buf[i + 1]);
                rx_idx ^= 1; 
                Module_Offline_device_update(offline_dev);
                break;
            }
        }
    }
}

/* 对外函数 */

int Module_Vision_Init(void)
{
    Offline_Init_config_t offlineconfig = {.name = "minipc", .beep_times = 10, .enable = VISION_OFFLINE_ENABLE, .timeout_ms = 100};
    offline_dev                         = Module_Offline_register(&offlineconfig);
    if (offline_dev == NULL)
    {
        LOG_E("offline device register error");
        return -1;
    }

    UINT status = tx_thread_create(&vision_thread, "vision_thread", vision_thread_entry, 0, vision_thread_stack, VISION_TASK_STACK_SIZE,
                                   VISION_TASK_PRIORITY, VISION_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("thread create failed");
        return -1;
    }

    LOG_I("Vision module initialized");

    return 0;
}

void Module_Vision_Send(const SendPacket *packet, uint32_t timeout)
{
    uint8_t frame[VISION_FRAME_TX_SIZE];

    frame[0] = VISION_TX_HEADER;
    memcpy(&frame[1], packet, sizeof(SendPacket));
    Append_CRC16_Check_Sum(frame, sizeof(frame));
    cdc_acm_send(frame, sizeof(frame), timeout);
}

ReceivePacket *Module_Vision_Receive(void)
{
    rx_snapshot = rx_packet[rx_idx ^ 1u];
    return &rx_snapshot;
}

uint8_t Module_Vision_Get_offline_state(void)
{
    if (offline_dev == NULL)
    {
        return STATE_OFFLINE; // 如果离线设备未初始化，默认返回离线状态
    }
    return Module_Offline_get_device_status(offline_dev);
}
