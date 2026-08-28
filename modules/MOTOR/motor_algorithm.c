#include "motor_algorithm.h"
#include "user_lib.h"

/* 控制算法实现*/
static float motor_calc_pid(Motor_Base *m)
{
    float pid_measure, pid_ref;

    pid_ref = m->controller.ref;
    if (m->setting.motor_reverse_flag == 1) pid_ref *= -1;

    /* 位置环 */
    if (m->setting.loop_type & ANGLE_LOOP)
    {
        pid_measure = (m->setting.angle_feedback_source == 1 && m->controller.other_angle_feedback_ptr) ? *m->controller.other_angle_feedback_ptr
                                                                                                        : m->measure.total_angle;

        if (m->setting.feedback_reverse_flag == 1) pid_measure *= -1;
        pid_ref = PIDCalculate(&m->controller.angle_PID, pid_measure, pid_ref);
    }

    /* 速度环 */
    if (m->setting.loop_type & SPEED_LOOP)
    {
        pid_measure = (m->setting.speed_feedback_source == 1 && m->controller.other_speed_feedback_ptr) ? *m->controller.other_speed_feedback_ptr
                                                                                                        : m->measure.speed_rad;

        if (m->setting.feedback_reverse_flag == 1) pid_measure *= -1;
        pid_ref = PIDCalculate(&m->controller.speed_PID, pid_measure, pid_ref);
    }

    return pid_ref;
}

static float motor_calc_lqr(Motor_Base *m)
{
    float ref, rad_angle, rad_speed;

    ref = m->controller.ref;
    if (m->setting.motor_reverse_flag == 1) ref *= -1;

    rad_angle = (m->setting.angle_feedback_source == 1 && m->controller.other_angle_feedback_ptr) ? *m->controller.other_angle_feedback_ptr
                                                                                                  : m->measure.total_angle;
    if (m->setting.feedback_reverse_flag == 1) rad_angle *= -1;

    rad_speed = (m->setting.speed_feedback_source == 1 && m->controller.other_speed_feedback_ptr) ? *m->controller.other_speed_feedback_ptr
                                                                                                  : m->measure.speed_rad;
    if (m->setting.feedback_reverse_flag == 1) rad_speed *= -1;

    return LQRCalculate(&m->controller.lqr, rad_angle, rad_speed, ref);
}

/* 算法分派 */
float Motor_CalculateTorque(Motor_Base *m)
{
    float torque = 0;

    switch (m->setting.algorithm_type)
    {
    case CONTROL_PID:
        torque = motor_calc_pid(m);
        break;
    case CONTROL_LQR:
        torque = motor_calc_lqr(m);
        break;
    default:
        break;
    }

    torque += m->controller.feedforward_torque;
    VAL_LIMIT(torque, -m->info.max_torque, m->info.max_torque);
    return torque;
}
