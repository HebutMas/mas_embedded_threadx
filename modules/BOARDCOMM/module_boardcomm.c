#include "module_boardcomm.h"
#include "bsp_can.h"
#include "module_offline.h"
#include <string.h>

#define LOG_TAG "BoardComm"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static Can_Device            *boardcomm_dev         = NULL;
static Offline_Device        *boardcomm_offline_dev = NULL;
static BoardComm_RxCallback_t app_rx_callback       = NULL;

/* 双缓冲区 */
static uint8_t           rx_frame[2][BOARDCOMM_FRAME_MAX];
static uint8_t           rx_frame_len;
static volatile uint32_t rx_idx;

#if !SINGLE_BOARD

/* BSP CAN 内部回调 → 转发给 app 注册的回调 / 写入双缓冲区 */
static void boardcomm_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    (void)dev;

    if (app_rx_callback != NULL)
    {
        app_rx_callback(data, len);
        Module_Offline_device_update(boardcomm_offline_dev);
        return;
    }

    if (len == 0 || len > BOARDCOMM_FRAME_MAX)
    {
        return;
    }

    memcpy(rx_frame[rx_idx], data, len);
    rx_frame_len = len;
    rx_idx ^= 1;
    Module_Offline_device_update(boardcomm_offline_dev);
}
#endif

int Module_BoardComm_Init(void)
{
#if SINGLE_BOARD
    LOG_I("BoardComm skipped (single board)");
#endif

#if CHASSIS_BOARD
    Can_Device_Init_Config_s cfg = {
        .hcan        = BOARDCOMM_CAN,
        .tx_id       = BOARDCOMM_CHASSIS_ID,
        .rx_id       = BOARDCOMM_GIMBAL_ID,
        .rx_callback = boardcomm_rx_callback,
    };
    boardcomm_dev = BSP_CAN_Device_Init(&cfg);
#endif

#if GIMBAL_BOARD
    Can_Device_Init_Config_s cfg = {
        .hcan        = BOARDCOMM_CAN,
        .tx_id       = BOARDCOMM_GIMBAL_ID,
        .rx_id       = BOARDCOMM_CHASSIS_ID,
        .rx_callback = boardcomm_rx_callback,
    };
    boardcomm_dev = BSP_CAN_Device_Init(&cfg);
#endif

    if (boardcomm_dev == NULL)
    {
        LOG_E("Failed to init can device");
        return -1;
    }

    Offline_Init_config_t offlineconfig = {
        .name       = "boardcomm",
        .beep_times = 10,
        .enable     = BOARDCOMM_OFFLINE_ENABLE,
        .timeout_ms = 100,
    };
    boardcomm_offline_dev = Module_Offline_register(&offlineconfig);
    if (boardcomm_offline_dev == NULL)
    {
        LOG_E("offline device register error");
        return -1;
    }

    LOG_I("BoardComm initialized");

    return 0;
}

void Module_BoardComm_Send(uint8_t *data, uint8_t len)
{
    if (data == NULL || len == 0 || len > BOARDCOMM_FRAME_MAX)
    {
        LOG_E("send dropped: data=%p len=%u", (void *)data, len);
        return;
    }
    if (boardcomm_dev == NULL)
    {
        LOG_E("send dropped: can device not inited");
        return;
    }
    BSP_CAN_Send(boardcomm_dev, data, len);
}

uint8_t Module_BoardComm_Receive(void *dst, uint8_t len)
{
    if (dst == NULL || len == 0 || len > BOARDCOMM_FRAME_MAX || len != rx_frame_len)
    {
        return 0;
    }

    memcpy(dst, rx_frame[rx_idx ^ 1u], len);
    return 1;
}

uint8_t Module_BoardComm_Get_Offline_State(void)
{
    if (boardcomm_offline_dev == NULL)
    {
        return STATE_OFFLINE;
    }
    return Module_Offline_get_device_status(boardcomm_offline_dev);
}

void Module_BoardComm_RegisterRx(BoardComm_RxCallback_t callback) { app_rx_callback = callback; }
