/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-09-15 18:33:44
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-09-15 18:34:58
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\gas_spring\gas_spring.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef _GAS_SPRING_H_
#define _GAS_SPRING_H_

#include "solution_def.h"
#include "user_lib.h"

#define Fs              350.0f
#define GAS_SPRING_FS_LIMIT 3.0f  
#define curtine_theta2  12.26f/180.0f*PI
#define curtine_theta1  73.74f/180.0f*PI
#define L1              0.21f
#define L2              0.25f
#define L3              0.2021f
#define L6              0.0503f

#define GAS_SPRING_SIN_EPS  1e-4f
#define GAS_SPRING_L4_EPS   1e-4f

void gas_spring_init(void);
void gas_spring_calc_left(void);
void gas_spring_calc_right(void);
void gas_spring_calc(void);
const GasSpring *gas_spring_get(void);

#endif