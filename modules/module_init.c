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
#if MODULE_VISION
#include "module_vision.h"
#endif
#if MODULE_BOARDCOMM
#include "module_boardcomm.h"
#endif
#if MODULE_VOFA
#include "vofa.h"
#endif


#define LOG_LVL LOG_LVL_INFO
#define LOG_TAG "Robot_Init"
#include "ulog_def.h"

void MODULE_Init(void)
{
#if MODULE_OFFLINE
    Module_Offline_init();
#endif
#if MODULE_REMOTE
    Module_Remote_init();
#endif
#if MODULE_BMI088
    Module_BMI088_init();
#endif
#if MODULE_INS
    Module_INS_Init();
#endif
#if MODULE_REFEREE
    Module_Referee_Init();
#endif
#if MODULE_WT606
    Module_WT606_Init();
#endif
#if MODULE_SUPERCAP
    Module_SuperCap_Init();
#endif
#if MODULE_MOTOR
    Module_Motor_Init();
#endif
#if MODULE_VISION
    Module_Vision_Init();
#endif
#if MODULE_BOARDCOMM
    Module_BoardComm_Init();
#endif
#if MODULE_VOFA
    Module_VOFA_Init();
#endif

    LOG_I("Modules init finished");
}
