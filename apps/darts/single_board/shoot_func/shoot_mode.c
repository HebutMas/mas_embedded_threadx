/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-09-20 13:00:00
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-10-09 17:20:15
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\shoot_func\shoot_mode.c
 * @Description:
 */
#include "shoot_mode.h"
#include "darts_def.h"

#include "bsp_dwt.h"
#include "main.h"
#include "tx_api.h"

#include <math.h>
#include <stddef.h>

#define LOG_TAG "app_shoot_mode"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 限位开关接线: PB12 (EXTI12), 常闭串联, 任意开关断开产生上升沿。
 * 板级未配置该引脚与中断向量, 因此在 app 层完成 GPIO/EXTI/NVIC 初始化,
 * 并定义 EXTI15_10_IRQHandler 覆盖启动文件的弱定义 (仅接管 EXTI 线 10~15)。 */

static uint64_t delay_start_us;
static bool     delay_active;
static const float dm_position_tolerance_rad = 0.030f;  /* 拨盘 DM4310, rad (≈2°), 精度不够改这里 */
static volatile bool limit_switch_event;
static volatile bool limit_switch_locked;
static void         *limit_switch_port;
static uint16_t      limit_switch_pin;
static void          (*limit_switch_action)(void);
static ULONG         limit_switch_lock_start_tick;
static const ULONG   limit_switch_lock_ticks = 3 * TX_TIMER_TICKS_PER_SECOND;

static void shoot_mode_limit_switch_callback(void)
{
    if (limit_switch_locked || limit_switch_event) return;

    limit_switch_event = true;
    limit_switch_action();
}

bool shoot_mode_3508_is_position_arrived(const DJI_Motor_t *motor, float target_angle_rad)
{
    if (motor == NULL) return false;

    float error_mm = fabsf(motor->base.measure.total_angle - target_angle_rad) * FRICTION_PULLEY_RADIUS_MM / motor->base.info.gear_ratio;
    return error_mm <= FRICTION_POSITION_TOLERANCE_MM;
}

bool shoot_mode_3508_is_stopped(const DJI_Motor_t *motor)
{
    if (motor == NULL) return false;

    float speed_mm_s = fabsf(motor->base.measure.speed_rad) * FRICTION_PULLEY_RADIUS_MM / motor->base.info.gear_ratio;
    return speed_mm_s < FRICTION_SPEED_TOLERANCE_MM_S;
}

bool shoot_mode_dm_is_arrived(const DM_Motor_t *motor, float target_angle_rad)
{
    if (motor == NULL) return false;

    return fabsf(motor->base.measure.total_angle - target_angle_rad) <= dm_position_tolerance_rad &&
           fabsf(motor->base.measure.speed_rad) < 0.2f;
}

bool shoot_mode_delay_ms(uint32_t delay_ms)
{
    uint64_t now_us = BSP_DWT_GetTimeline_us();

    if (!delay_active)
    {
        delay_start_us = now_us;
        delay_active   = true;
    }

    if (now_us - delay_start_us < (uint64_t)delay_ms * 1000ULL) return false;

    delay_active = false;
    return true;
}

bool shoot_mode_limit_switch_init(void *port, uint16_t pin, void (*action)(void))
{
    if (port == NULL || action == NULL || (pin & 0xFC00U) == 0U)
    {
        LOG_E("limit switch init failed: need EXTI line 10~15, pin mask 0x%04X", (unsigned int)pin);
        return false;
    }

    GPIO_InitTypeDef gpio_init = {
        .Pin  = pin,
        .Mode = GPIO_MODE_IT_RISING,
        .Pull = GPIO_PULLUP,
    };
    HAL_GPIO_Init((GPIO_TypeDef *)port, &gpio_init);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    limit_switch_port   = port;
    limit_switch_pin    = pin;
    limit_switch_action = action;
    limit_switch_event  = false;
    limit_switch_locked = false;
    return true;
}

void EXTI15_10_IRQHandler(void)
{
    if (limit_switch_pin != 0U && __HAL_GPIO_EXTI_GET_IT(limit_switch_pin) != 0U)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(limit_switch_pin);
        shoot_mode_limit_switch_callback();
    }
}

bool shoot_mode_limit_switch_take_event(void)
{
    ULONG now_tick = tx_time_get();

    if (limit_switch_event)
    {
        limit_switch_event           = false;
        limit_switch_locked          = true;
        limit_switch_lock_start_tick = now_tick;
        delay_active                 = false;
        return true;
    }

    if (limit_switch_locked)
    {
        if ((ULONG)(now_tick - limit_switch_lock_start_tick) < limit_switch_lock_ticks) return false;
        if (HAL_GPIO_ReadPin((GPIO_TypeDef *)limit_switch_port, limit_switch_pin) != GPIO_PIN_RESET) return false;

        limit_switch_locked = false;
    }

    return false;
}
