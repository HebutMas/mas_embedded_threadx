#include "app_init.h"
#include "tx_api.h"

#define LOG_LVL LOG_LVL_INFO
#define LOG_TAG "APP_Init"
#include "ulog_def.h"

#include "robot_control.h"

void APP_Init(void)
{
    if (robot_control_init() != 0)
    {
        LOG_E_LOCK("APP_Init failed");
    }

    LOG_I("APP init finished");
}