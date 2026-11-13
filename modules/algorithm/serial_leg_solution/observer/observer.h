/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-26 12:10:31
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-07-27 17:55:22
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\observer\observer.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _OBSERVER_H_
#define _OBSERVER_H_

#include "motor_damiao.h"
#include "motor_dji.h"
#include "solution_def.h"

void observer_init(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[]);
void observer_update(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[]);
const observer *observer_get(void);
void observer_reset_kf_x(void);   /* 掉线轻重置, 只清位移积分器 */

#endif