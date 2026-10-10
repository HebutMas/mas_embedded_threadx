/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-10 16:06:48
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-11 16:25:44
 * @FilePath: /mas_embedded_threadx/modules/module_init.c
 * @Description:
 */
#include "module_init.h"

#if MODULE_OFFLINE
#include "module_offline.h"
#endif
#if MODULE_REMOTE
#include "module_remote.h"
#endif
#if MODULE_BMI088
#include "module_bmi088.h"
#endif
#if MODULE_INS
#include "module_ins.h"
#endif
#if MODULE_REFEREE
#include "module_referee.h"
#endif
#if MODULE_WT606
#include "module_wt606.h"
#endif
#if MODULE_SUPERCAP
#include "module_supercap.h"
#endif
#if MODULE_MOTOR
#include "module_motor.h"
#endif
#if MODULE_PCCOMM
#include "module_pccomm.h"
#endif
#if MODULE_BOARDCOMM
#include "module_boardcomm.h"
#endif
#if MODULE_LORA
#include "module_lora.h"
#endif
#if MODULE_VOFA
#include "vofa.h"
#endif
#if MODULE_NRF24L01
#include "module_nrf24l01.h"
#endif

#define LOG_LVL LOG_LVL_INFO
#define LOG_TAG "Robot_Init"
#include "ulog_def.h"

void MODULE_Init(void)
{
    bool ok = true;

#if MODULE_OFFLINE
    ok &= Module_Offline_init() == 0;
#endif
#if MODULE_REMOTE
    ok &= Module_Remote_init() == 0;
#endif
#if MODULE_BMI088
    ok &= Module_BMI088_init() == 0;
#endif
#if MODULE_INS
    ok &= Module_INS_Init() == 0;
#endif
#if MODULE_REFEREE
    ok &= Module_Referee_Init() == 0;
#endif
#if MODULE_WT606
    ok &= Module_WT606_Init() == 0;
#endif
#if MODULE_SUPERCAP
    ok &= Module_SuperCap_Init() == 0;
#endif
#if MODULE_MOTOR
    ok &= Module_Motor_Init() == 0;
#endif
#if MODULE_PCCOMM
    ok &= Module_PCComm_Init() == 0;
#endif
#if MODULE_BOARDCOMM
    ok &= Module_BoardComm_Init() == 0;
#endif
#if MODULE_LORA
    ok &= Module_Lora_Init() == 0;
#endif
#if MODULE_VOFA
    Module_VOFA_Init(); // 无失败路径
#endif
#if MODULE_NRF24L01
    ok &= Module_NRF24L01_Init() == 0;
#endif

    if (!ok)
    {
        LOG_E_LOCK("MODULE_Init failed");
    }

    LOG_I("Modules init finished");
}
