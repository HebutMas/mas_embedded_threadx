/*
 * @Author: Lancelot jishiyu20060116@163.com
 * @Date: 2026-10-10 16:08:31
 * @LastEditors: Lancelot jishiyu20060116@163.com
 * @LastEditTime: 2026-10-10 16:32:08
 * @FilePath: \mas_embedded_threadx\modules\VISION\module_vision.c
 * @Description:
 */
#include "module_vision.h"
#include "bsp_def.h"
#include "usbd_cdc_acm_user.h"
#include "tx_api.h"
#include "module_offline.h"
#include "crc_rm.h"
#include <string.h>

#define LOG_TAG "module_vision"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define VISION_CDC_PORT      0
#define VISION_TX_HEADER     0x5A
#define VISION_RX_HEADER     0xA5
#define VISION_FRAME_TX_SIZE (1 + sizeof(VisionSendPacket) + 2)
#define VISION_FRAME_RX_SIZE (1 + sizeof(VisionReceivePacket) + 2)

/* 自瞄双缓冲区 */
static VisionReceivePacket receive_packet[2];
static volatile uint32_t   receive_idx;
static VisionReceivePacket receive_snapshot;

static Offline_Device            *offline_dev;
static TX_THREAD                  vision_thread;
APPS_STACK_SECTION static uint8_t vision_thread_stack[VISION_TASK_STACK_SIZE];

static void vision_thread_entry(ULONG arg)
{
    (void)arg;
    uint8_t  buf[64];
    uint32_t rx_len;

    while (1)
    {
        if (cdc_acm_recv(VISION_CDC_PORT, buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER) <= 0)
        {
            tx_thread_sleep(1);
            continue;
        }

        /* ponytail: 每次读取仅取首个完整帧；跨次半帧需增加流式重组。 */
        for (uint32_t i = 0; i + VISION_FRAME_RX_SIZE <= rx_len; i++)
        {
            if (buf[i] == VISION_RX_HEADER && Verify_CRC16_Check_Sum(&buf[i], VISION_FRAME_RX_SIZE))
            {
                memcpy(&receive_packet[receive_idx], &buf[i + 1], sizeof(VisionReceivePacket));
                receive_idx ^= 1;
                Module_Offline_device_update(offline_dev);
                break;
            }
        }

        tx_thread_relinquish();
    }
}

int Module_Vision_Init(void)
{
    Offline_Init_config_t offlineconfig = {.name = "minipc_vision", .beep_times = 10, .enable = VISION_OFFLINE_ENABLE, .timeout_ms = 100};
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

void Module_Vision_Send(const VisionSendPacket *packet, uint32_t timeout)
{
    uint8_t frame[VISION_FRAME_TX_SIZE];
    frame[0] = VISION_TX_HEADER;
    memcpy(&frame[1], packet, sizeof(VisionSendPacket));
    Append_CRC16_Check_Sum(frame, sizeof(frame));
    cdc_acm_send(VISION_CDC_PORT, frame, sizeof(frame), timeout);
}

VisionReceivePacket *Module_Vision_Receive(void)
{
    receive_snapshot = receive_packet[receive_idx ^ 1u];
    return &receive_snapshot;
}

uint8_t Module_Vision_Get_offline_state(void) { return Module_Offline_get_device_status(offline_dev); }
