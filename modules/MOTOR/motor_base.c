#include "motor_base.h"
#include "motor_algorithm.h"

static Motor_Base *g_motor_list = NULL;

void Motor_Register(Motor_Base *motor)
{
    if (motor == NULL) return;
    motor->next  = g_motor_list;
    g_motor_list = motor;
}

void Motor_ControlAll(void)
{
    for (Motor_Base *motor = g_motor_list; motor; motor = motor->next)
    {
        uint8_t offline = (motor->offline_dev != NULL && Module_Offline_get_device_status(motor->offline_dev) == STATE_OFFLINE);

        if (offline || motor->setting.enableflag == 0)
        {
            motor->controller.output           = 0;
            motor->controller.output_torque    = 0;
            motor->controller.speed_PID.Output = 0;
            motor->controller.speed_PID.Iout   = 0;
            motor->controller.angle_PID.Output = 0;
            motor->controller.angle_PID.Iout   = 0;
            continue;
        }

        /* 开环电机: 无控制计算 (Apply 阶段直通 ref) */
        if (motor->setting.loop_type == OPEN_LOOP) continue;

        motor->controller.output_torque = Motor_CalculateTorque(motor);
    }
}

void Motor_ApplyAll(void)
{
    for (Motor_Base *motor = g_motor_list; motor; motor = motor->next)
    {
        if (motor->Apply != NULL) motor->Apply(motor);
    }
}

void Motor_Start(Motor_Base *m)
{
    if (m == NULL) return;
    m->setting.enableflag = 1;
}

void Motor_Stop(Motor_Base *m)
{
    if (m == NULL) return;
    m->setting.enableflag = 0;
}

void Motor_SetRef(Motor_Base *m, float ref)
{
    if (m == NULL) return;
    m->controller.ref = ref;
}

void Motor_ChangeFeed(Motor_Base *m, Closeloop_Type_e loop, uint8_t feedback_source)
{
    if (m == NULL) return;
    if (loop == ANGLE_LOOP)
        m->setting.angle_feedback_source = feedback_source;
    else if (loop == SPEED_LOOP)
        m->setting.speed_feedback_source = feedback_source;
}

void Motor_OuterLoop(Motor_Base *m, Closeloop_Type_e outer_loop)
{
    if (m == NULL) return;
    m->setting.loop_type = outer_loop;
}

void Motor_SetForwardTorque(Motor_Base *m, float torque)
{
    if (m == NULL) return;
    m->controller.feedforward_torque = torque;
}

void Motor_SetOutputTorque(Motor_Base *m, float torque)
{
    if (m == NULL) return;
    m->controller.output_torque = torque;
}
