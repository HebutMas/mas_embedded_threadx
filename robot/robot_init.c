/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2026-04-13 18:08:51
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-14 09:50:17
 * @FilePath: /mas_embedded_threadx/robot/robot_init.c
 * @Description:
 */

#include "robot_init.h"

#include "tx_api.h"
#include "utils_init.h"
#include "bsp_init.h"
#include "module_init.h"
#include "bsp_def.h"
#include "app_init.h"

#define LOG_LVL LOG_LVL_INFO
#define LOG_TAG "Robot_Init"
#include "ulog_def.h"

#if defined(STM32F103xB)
#define TX_APP_MEM_POOL_SIZE (4 * 1024)   // 4KB F103: 仅20KB RAM
#elif defined(STM32F105xC)
#define TX_APP_MEM_POOL_SIZE (12 * 1024)  // 12KB F105: 64KB RAM (含CMSIS-DSP开销)
#else
#define TX_APP_MEM_POOL_SIZE (20 * 1024)  // 20KB F407/H723: 充足RAM
#endif
static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE];
TX_BYTE_POOL tx_app_byte_pool;

void Robot_Init(void)
{
    if (tx_byte_pool_create(&tx_app_byte_pool, "Tx App memory pool", tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
    {
        LOG_E("Failed to create byte pool");
        while (1)
        {
        };
    }

    UTILS_Init();
    BSP_Init();
    MODULE_Init();
    APP_Init();

    LOG_I("robot init finished");
}
