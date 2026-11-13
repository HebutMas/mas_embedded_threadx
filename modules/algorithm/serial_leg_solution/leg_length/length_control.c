/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-26 20:23:17
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-09-23 20:30:35
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\leg_length\length_control.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "length_control.h"
#include "solution_def.h"
#include "kinematics.h"
#include "pid.h"
#include "arm_math.h"
#include "car_state.h"

#define LENGTH_COS_EPS 0.1f    // |θ|>84° 防除零, 对应目标腿长上限 1.55m

static Length lenth_ctrl;
static PIDInstance pid_left;
static PIDInstance pid_right;

void length_init()
{
    lenth_ctrl.target_length_L = 0.155f;
    lenth_ctrl.target_length_R = 0.155f;

    PID_Init_Config_s pd_config = {
        .Kp       = 400.0f,  // N/m
        .Ki       = 0.0f,
        .Kd       = 0.0f,
        .MaxOut   = 50.0f,   // N
        .DeadBand = 0.0f,
        .Improve  = PID_Derivative_On_Measurement,
    };

    PIDInit(&pid_left,  &pd_config);
    PIDInit(&pid_right, &pd_config);
}

void length_control ()
{
    const fk *fk_left = kinematics_get_left();
    const fk *fk_right = kinematics_get_right();

    if (car_state_get() == STATE_BALANCE)
    {
        // 目标腿长 = 站立高度 0.155 / cos(θ), 随摆角更新
        float cos_l = arm_cos_f32(fk_left->theta);
        float cos_r = arm_cos_f32(fk_right->theta);
        if (cos_l < LENGTH_COS_EPS) cos_l = LENGTH_COS_EPS;
        if (cos_r < LENGTH_COS_EPS) cos_r = LENGTH_COS_EPS;
        lenth_ctrl.target_length_L = 0.155f / cos_l;
        lenth_ctrl.target_length_R = 0.155f / cos_r;
    }
    else
    {
        /* 起立: 固定目标腿长 0.14 */
        lenth_ctrl.target_length_L = 0.14f;
        lenth_ctrl.target_length_R = 0.14f;
    }

    lenth_ctrl.F_L_pid = PIDCalculate(&pid_left,  fk_left->L_0,  lenth_ctrl.target_length_L);
    lenth_ctrl.F_R_pid = PIDCalculate(&pid_right, fk_right->L_0, lenth_ctrl.target_length_R);
}

const Length *Length_get()
{
    return &lenth_ctrl;
}
