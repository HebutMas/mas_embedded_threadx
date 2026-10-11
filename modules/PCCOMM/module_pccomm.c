/*
 * @Author: sji733055-glitch sji733055@gmail.com
 * @Date: 2026-07-01 19:24:17
 * @LastEditors: sji733055-glitch sji733055@gmail.com
 * @LastEditTime: 2026-07-15 08:59:45
 * @FilePath: \mas_embedded_threadx\modules\PCCOMM\module_pccomm.c
 * @Description:
 */
#include "module_pccomm.h"
#include "bsp_def.h"
#include "usbd_cdc_acm_user.h"
#include <string.h>
#include "tx_api.h"
#include "module_offline.h"
#include "crc_rm.h"

#define LOG_TAG "module_pccomm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 自瞄双缓冲区 */
static ReceivePacket     aim_packet[2];
static volatile uint32_t aim_idx;
static ReceivePacket     aim_snapshot;
/* 导航双缓冲区 */
static NavPacket         nav_packet[2];
static volatile uint32_t nav_idx;
static NavPacket         nav_snapshot;

static Offline_Device            *offline_dev = NULL;
static TX_THREAD                  pccomm_thread;
APPS_STACK_SECTION static uint8_t pccomm_thread_stack[PCCOMM_TASK_STACK_SIZE];

static void pccomm_thread_entry(ULONG arg)
{
    uint8_t  buf[64];
    uint32_t rx_len;

    while (1)
    {
        int got = 0;

        /* 轮询所有虚拟串口 */
        for (uint8_t port = 0; port < CDC_ACM_PORT_NUM; port++)
        {
            if (cdc_acm_recv(port, buf, sizeof(buf), &rx_len, TX_NO_WAIT) <= 0)
            {
                continue;
            }
            got = 1;

            if (port == PCCOMM_AIM_PORT)
            {
                for (uint32_t i = 0; i + PCCOMM_AIM_FRAME_RX_SIZE <= rx_len; i++)
                {
                    if (buf[i] == PCCOMM_AIM_RX_HEADER && Verify_CRC16_Check_Sum(&buf[i], PCCOMM_AIM_FRAME_RX_SIZE))
                    {
                        aim_packet[aim_idx] = *((const ReceivePacket *)&buf[i + 1]);
                        aim_idx ^= 1;
                        Module_Offline_device_update(offline_dev);
                        break;
                    }
                }
            }
            else if (port == PCCOMM_NAV_PORT)
            {
                for (uint32_t i = 0; i + PCCOMM_NAV_FRAME_RX_SIZE <= rx_len; i++)
                {
                    if (buf[i] == PCCOMM_NAV_RX_HEADER && Verify_CRC16_Check_Sum(&buf[i], PCCOMM_NAV_FRAME_RX_SIZE))
                    {
                        nav_packet[nav_idx] = *((const NavPacket *)&buf[i + 1]);
                        nav_idx ^= 1;
                        break;
                    }
                }
            }
        }

        if (!got)
        {
            tx_thread_sleep(1);
        }
    }
}

int Module_PCComm_Init(void)
{
    Offline_Init_config_t offlineconfig = {.name = "minipc", .beep_times = 10, .enable = PCCOMM_OFFLINE_ENABLE, .timeout_ms = 100};
    offline_dev                         = Module_Offline_register(&offlineconfig);
    if (offline_dev == NULL)
    {
        LOG_E("offline device register error");
        return -1;
    }

    UINT status = tx_thread_create(&pccomm_thread, "pccomm_thread", pccomm_thread_entry, 0, pccomm_thread_stack, PCCOMM_TASK_STACK_SIZE,
                                   PCCOMM_TASK_PRIORITY, PCCOMM_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("thread create failed");
        return -1;
    }

    LOG_I("PCComm module initialized");

    return 0;
}

void Module_PCComm_Send(const SendPacket *packet, uint32_t timeout)
{
    uint8_t frame[PCCOMM_AIM_FRAME_TX_SIZE];

    frame[0] = PCCOMM_AIM_TX_HEADER;
    memcpy(&frame[1], packet, sizeof(SendPacket));
    Append_CRC16_Check_Sum(frame, sizeof(frame));
    cdc_acm_send(PCCOMM_AIM_PORT, frame, sizeof(frame), timeout);
}

void Module_PCComm_Send_Nav(const NavSendPacket *packet, uint32_t timeout)
{
    uint8_t frame[PCCOMM_NAV_FRAME_TX_SIZE];

    frame[0] = PCCOMM_NAV_TX_HEADER;
    memcpy(&frame[1], packet, sizeof(NavSendPacket));
    Append_CRC16_Check_Sum(frame, sizeof(frame));
    cdc_acm_send(PCCOMM_NAV_PORT, frame, sizeof(frame), timeout);
}

ReceivePacket *Module_PCComm_Receive(void)
{
    aim_snapshot = aim_packet[aim_idx ^ 1u];
    return &aim_snapshot;
}

NavPacket *Module_PCComm_Receive_Nav(void)
{
    nav_snapshot = nav_packet[nav_idx ^ 1u];
    return &nav_snapshot;
}

uint8_t Module_PCComm_Get_offline_state(void)
{
    if (offline_dev == NULL)
    {
        return STATE_OFFLINE; // 如果离线设备未初始化，默认返回离线状态
    }
    return Module_Offline_get_device_status(offline_dev);
}
