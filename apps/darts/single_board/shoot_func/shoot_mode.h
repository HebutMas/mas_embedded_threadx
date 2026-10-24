/*
 * @Author: abbhhh 804433588@qq.com
 * @Date: 2026-09-20 13:00:00
 * @LastEditors: abbhhh 804433588@qq.com
 * @LastEditTime: 2026-09-20 13:00:00
 * @FilePath: \mas_embedded_threadx\apps\darts\single_board\shoot_func\shoot_mode.h
 * @Description:
 */
#ifndef DARTS_SHOOT_MODE_H
#define DARTS_SHOOT_MODE_H

#include "motor_damiao.h"
#include "motor_dji.h"

#include <stdbool.h>
#include <stdint.h>

bool shoot_mode_3508_is_position_arrived(const DJI_Motor_t *motor, float target_angle_rad);
bool shoot_mode_3508_is_stopped(const DJI_Motor_t *motor);
bool shoot_mode_dm_is_arrived(const DM_Motor_t *motor, float target_angle_rad);
bool shoot_mode_delay_ms(uint32_t delay_ms);
bool shoot_mode_limit_switch_init(void *port, uint16_t pin, void (*action)(void));
bool shoot_mode_limit_switch_take_event(void);

#endif /* DARTS_SHOOT_MODE_H */
