/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-08-07 10:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/motor_algorithm.h
 * @Description:
 */
#ifndef _MOTOR_ALGORITHM_H_
#define _MOTOR_ALGORITHM_H_

#include "motor_base.h"

/**
 * @brief 通用控制计算
 * @return 扭矩 (Nm)
 */
float Motor_CalculateTorque(Motor_Base *motor);

#endif /* _MOTOR_ALGORITHM_H_ */
