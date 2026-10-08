#include "module_supercap.h"
#include "bsp_can.h"
#include "string.h"

#define LOG_TAG "module_supercap"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static Module_SuperCap_t module_supercap;

static void can_supercap_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    if (len == sizeof(SuperCap_Receive_t))
    {
        memcpy(&module_supercap.receive_data, data, len);
        Module_Offline_device_update(module_supercap.offline_dev);
    }
}

int Module_SuperCap_Init(void)
{
    Can_Device_Init_Config_s board_com_init = {
        .hcan        = SUPERCAP_CAN,
        .rx_id       = 0X051,
        .tx_id       = 0X061,
        .rx_callback = can_supercap_callback,
        .user_arg    = (void *)0,
    };
    module_supercap.device = BSP_CAN_Device_Init(&board_com_init);
    if (module_supercap.device == NULL)
    {
        LOG_E("can device init failed");
        return -1;
    }
    Offline_Init_config_t offline_manage_init = {.name = "supercap", .timeout_ms = 500, .beep_times = 10, .enable = SUPERCAP_OFFLINE_ENABLE};
    module_supercap.offline_dev               = Module_Offline_register(&offline_manage_init);
    if (module_supercap.offline_dev == NULL)
    {
        LOG_E("offline device init failed");
        return -1;
    }

    module_supercap.initialized = 1;

    LOG_I("module_supercap init finished");

    return 0;
}

void Module_SuperCap_Send(const SuperCap_Send_t *data)
{
    if (module_supercap.initialized && data != NULL)
    {
        uint8_t buf[8];
        memcpy(buf, data, sizeof(SuperCap_Send_t));
        BSP_CAN_Send(module_supercap.device, buf, sizeof(SuperCap_Send_t));
    }
}

const Module_SuperCap_t *Module_SuperCap_Get(void)
{
    if (module_supercap.initialized)
    {
        return &module_supercap;
    }
    return NULL;
}

uint8_t Module_SuperCap_Get_offline_state(void)
{
    if (module_supercap.offline_dev == NULL)
    {
        return STATE_OFFLINE; // 如果离线设备未初始化，默认返回离线状态
    }
    return Module_Offline_get_device_status(module_supercap.offline_dev);
}
