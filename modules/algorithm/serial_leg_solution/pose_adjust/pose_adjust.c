/*
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\pose_adjust\pose_adjust.c
 * @Description: 调姿摆腿控制 (对齐 luntui1 stand_up_dec 的 mode 3 摆腿)
 */
#include "pose_adjust.h"
#include "solution_def.h"
#include "kinematics.h"
#include "car_state.h"
#include "pid.h"

/* 到位窗口用 car_state.h 的 CAR_POSE_THETA_LO/HI, 与状态机判据同源 */

/* 摆腿: 腿角速度伺服, 目标 dphi_0 = -1.6 rad/s (luntui1 swing_angle PID) */
#define SWING_TARGET (-1.6f)
#define SWING_F      8.0f /* 摆腿支撑力 (N, 对齐 luntui1 mode 3, VMC 内再减气弹簧) */

static PoseAdjust  pose_ctrl;
static PIDInstance pid_L;
static PIDInstance pid_R;

void pose_adjust_init(void)
{
    PID_Init_Config_s config = {
        .Kp            = 20.0f,  /* luntui1 Kp=20 */
        .Ki            = 25.0f,  /* luntui1 Ki=0.05/周期 → SI: 0.05/0.002 */
        .Kd            = 0.0f,
        .MaxOut        = 20.0f,  /* N·m, 对齐 luntui1 out_limit=20 */
        .DeadBand      = 0.0f,
        .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit,
        .IntegralLimit = 0.5f,   /* N·m, 对齐 luntui1 ki_limit=0.5 */
    };
    PIDInit(&pid_L, &config);
    PIDInit(&pid_R, &config);
}

void pose_adjust_control(void)
{
    const fk *fk_l = kinematics_get_left();
    const fk *fk_r = kinematics_get_right();
    if (fk_l == NULL || fk_r == NULL)
        return;

    if (car_state_get() != STATE_POSE_ADJUST)
    {
        /* 非调姿态: 输出清零, PID 冻结 (积分限幅 0.5 兜底, 重入时残留可忽略) */
        pose_ctrl.Tp_L = 0.0f;
        pose_ctrl.Tp_R = 0.0f;
        pose_ctrl.F_L  = 0.0f;
        pose_ctrl.F_R  = 0.0f;
        return;
    }

    /* 每腿独立判断 (luntui1 stand_up_dec 的 per-leg 逻辑) */
    if (fk_l->theta > CAR_POSE_THETA_LO && fk_l->theta <= CAR_POSE_THETA_HI)
    {
        pose_ctrl.Tp_L = 0.0f; /* 已到位: 清零等待, 放松待着 */
        pose_ctrl.F_L  = 0.0f;
    }
    else
    {
        /* ⚠ 符号链: mas Tp = +pid_out (luntui1 为 -pid_out, 两边 VMC 加号/减号约定相反) */
        pose_ctrl.Tp_L = PIDCalculate(&pid_L, fk_l->dphi_0, SWING_TARGET);
        pose_ctrl.F_L  = SWING_F;
    }

    if (fk_r->theta > CAR_POSE_THETA_LO && fk_r->theta <= CAR_POSE_THETA_HI)
    {
        pose_ctrl.Tp_R = 0.0f;
        pose_ctrl.F_R  = 0.0f;
    }
    else
    {
        pose_ctrl.Tp_R = PIDCalculate(&pid_R, fk_r->dphi_0, SWING_TARGET);
        pose_ctrl.F_R  = SWING_F;
    }
}

const PoseAdjust *pose_adjust_get(void)
{
    return &pose_ctrl;
}
