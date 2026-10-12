#include "module_nav.h"
#include "bsp_def.h"
#include "usbd_cdc_acm_user.h"
#include "tx_api.h"
#include "module_offline.h"
#include "crc_rm.h"
#include <string.h>

#define LOG_TAG "module_nav"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define NAV_CDC_PORT      1
#define NAV_TX_HEADER     0x5B
#define NAV_RX_HEADER     0xB5
#define NAV_FRAME_TX_SIZE (1 + sizeof(NavSendPacket) + 2)
#define NAV_FRAME_RX_SIZE (1 + sizeof(NavReceivePacket) + 2)

/* 导航双缓冲区 */
static NavReceivePacket  nav_packet[2];
static volatile uint32_t nav_idx;
static NavReceivePacket  nav_snapshot;

static Offline_Device            *offline_dev;
static TX_THREAD                  nav_thread;
APPS_STACK_SECTION static uint8_t nav_thread_stack[NAV_TASK_STACK_SIZE];

static void nav_thread_entry(ULONG arg)
{
    (void)arg;
    uint8_t  buf[64];
    uint32_t rx_len;

    while (1)
    {
        if (cdc_acm_recv(NAV_CDC_PORT, buf, sizeof(buf), &rx_len, TX_NO_WAIT) <= 0)
        {
            tx_thread_sleep(1);
            continue;
        }

        /* ponytail: 每次读取仅取首个完整帧；跨次半帧需增加流式重组。 */
        for (uint32_t i = 0; i + NAV_FRAME_RX_SIZE <= rx_len; i++)
        {
            if (buf[i] == NAV_RX_HEADER && Verify_CRC16_Check_Sum(&buf[i], NAV_FRAME_RX_SIZE))
            {
                memcpy(&nav_packet[nav_idx], &buf[i + 1], sizeof(NavReceivePacket));
                nav_idx ^= 1;
                Module_Offline_device_update(offline_dev);
                break;
            }
        }

        tx_thread_relinquish();
    }
}

int Module_Nav_Init(void)
{
    if (NAV_CDC_PORT >= CDC_ACM_PORT_NUM)
    {
        LOG_I("Nav disabled: CDC1 unavailable");
        return 0;
    }

    Offline_Init_config_t offlineconfig = {.name = "minipc_nav", .beep_times = 10, .enable = NAV_OFFLINE_ENABLE, .timeout_ms = 100};
    offline_dev                         = Module_Offline_register(&offlineconfig);
    if (offline_dev == NULL)
    {
        LOG_E("offline device register error");
        return -1;
    }

    UINT status = tx_thread_create(&nav_thread, "nav_thread", nav_thread_entry, 0, nav_thread_stack, NAV_TASK_STACK_SIZE, NAV_TASK_PRIORITY,
                                   NAV_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("thread create failed");
        return -1;
    }

    LOG_I("Nav module initialized");
    return 0;
}

void Module_Nav_Send(const NavSendPacket *packet, uint32_t timeout)
{
    uint8_t frame[NAV_FRAME_TX_SIZE];
    frame[0] = NAV_TX_HEADER;
    memcpy(&frame[1], packet, sizeof(NavSendPacket));
    Append_CRC16_Check_Sum(frame, sizeof(frame));
    cdc_acm_send(NAV_CDC_PORT, frame, sizeof(frame), timeout);
}

NavReceivePacket *Module_Nav_Receive(void)
{
    nav_snapshot = nav_packet[nav_idx ^ 1u];
    return &nav_snapshot;
}

uint8_t Module_Nav_Get_offline_state(void) { return Module_Offline_get_device_status(offline_dev); }
