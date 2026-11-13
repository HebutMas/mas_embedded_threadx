/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-27 13:36:19
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-07-27 16:53:34
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\leg_coor\leg_coor.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "leg_coor.h"
#include "kinematics.h"
#include "pid.h"

static coordination leg_coor_ctrl;
static PIDInstance  leg_coor_pid;

void leg_coor_init(void)
{
    PID_Init_Config_s config = {
        .Kp            = 100.0f,                   /* N·m/rad */
        .Ki            = 0.5f,                     /* 0.001/周期折算为 SI (÷dt) */
        .Kd            = 0.016f,                   /* 8/周期折算为 SI (×dt), 误差微分 */
        .MaxOut        = 50.0f,                    /* N·m */
        .DeadBand      = 0.0f,                     /* rad */
        .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit,  /* 梯形积分 + 积分限幅(须勾选才生效) */
        .IntegralLimit = 10.0f,                    /* N·m */
    };
    PIDInit(&leg_coor_pid, &config);
}

void leg_coor_control(void)
{
    const fk *fk_l = kinematics_get_left();
    const fk *fk_r = kinematics_get_right();

    float out = PIDCalculate(&leg_coor_pid, fk_l->theta, fk_r->theta);

    leg_coor_ctrl.Tp_L_coor =  out;
    leg_coor_ctrl.Tp_R_coor = -out;
}

const coordination *leg_coor_get(void)
{
    return &leg_coor_ctrl;
}
