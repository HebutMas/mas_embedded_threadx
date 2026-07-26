/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-11 17:00:00
 * @FilePath: /mas_embedded_threadx/modules/MOTOR/module_motor.h
 * @Description:
 */
#ifndef _MODULE_MOTOR_H_
#define _MODULE_MOTOR_H_

#include "motor_def.h"
#include "motor_base.h"

#include "motor_dji.h"
#include "motor_damiao.h"
#include "motor_servo.h"
#include "motor_zdt.h"
#include "motor_lk.h"
#ifndef MOTOR_TASK_STACK_SIZE
#define MOTOR_TASK_STACK_SIZE 1024 /* 检测任务栈 (字节) */
#endif
#ifndef MOTOR_TASK_PRIORITY
#define MOTOR_TASK_PRIORITY 12 /* 检测任务优先级 */
#endif

/**
 * @brief 初始化电机模块
 */
void Module_Motor_Init(void);

#endif /* _MODULE_MOTOR_H_ */
