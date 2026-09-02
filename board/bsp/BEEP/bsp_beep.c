#include "bsp_beep.h"
#include "bsp_pwm.h"
#include "tim.h"
#include <stdint.h>
#include <string.h>

#define LOG_TAG "bsp_beep"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

// 蜂鸣器设备实例
static PWM_Device *beep_pwm_dev = NULL;

void BSP_BEEP_Init()
{
#if defined(STM32H723xx)
    PWM_Init_Config pwm_config = {.htim = &htim12, .Channel = TIM_CHANNEL_2, .dutyx10 = 0, .Mode = PWM_MODE_BLOCKING};
#elif defined(STM32F407xx)
    PWM_Init_Config pwm_config = {.htim = &htim4, .Channel = TIM_CHANNEL_3, .dutyx10 = 0, .Mode = PWM_MODE_BLOCKING};
#elif defined(STM32F105xC) || defined(STM32F103xB)
    PWM_Init_Config pwm_config = {.htim = NULL, .Channel = 0, .dutyx10 = 0, .Mode = PWM_MODE_BLOCKING};
#endif

    beep_pwm_dev = BSP_PWM_Device_Init(&pwm_config);
    if (beep_pwm_dev == NULL)
    {
        return;
    }

    // 默认关闭蜂鸣器
    BSP_BEEP_Stop();

    LOG_I("BEEP init success");
}

void BSP_BEEP_Start() { BSP_PWM_Start(beep_pwm_dev, NULL, 0); }

void BSP_BEEP_Stop() { BSP_PWM_Stop(beep_pwm_dev); }

void BSP_BEEP_Set(uint16_t psc, uint16_t pwm)
{
    if (beep_pwm_dev == NULL)
    {
        return;
    }

    BSP_PWM_SetPSC(beep_pwm_dev, psc);
    BSP_PWM_SetPulse(beep_pwm_dev, pwm);
}
