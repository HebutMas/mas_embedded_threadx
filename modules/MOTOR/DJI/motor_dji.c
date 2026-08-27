#include "motor_dji.h"
#include "bsp_can.h"
#include "bsp_def.h"
#include "module_offline.h"
#include "user_lib.h"
#include <stdint.h>
#include <string.h>

#define LOG_TAG "motor_dji"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

#define ECD_ANGLE_COEF_DJI     0.043945f /* (360/8192) */
#define ECD_ANGLE_COEF_DJI_RAD 0.000767f /* (2.0f * PI / 8192.0f) */

/* DJI 电机发送分组数量 */
#if defined(STM32H723xx)
#define MOTOR_SENDER_SIZE 15
#elif defined(STM32F407xx)
#define MOTOR_SENDER_SIZE 10
#elif defined(STM32F105xC)
#define MOTOR_SENDER_SIZE 10
#elif defined(STM32F103xB)
#define MOTOR_SENDER_SIZE 5
#endif

/*
 *  DJI CAN 发送缓冲区 (多电机共用一帧 CAN 报文)
 *  C610(M2006)/C620(M3508): 0x1ff, 0x200
 *  GM6020:                  0x1ff, 0x2ff, 0x1fe, 0x2fe
 */
/* clang-format off */
static BSP_CanMsg_t sender_assignment[MOTOR_SENDER_SIZE] = {
    [0]  = { .hcan = BSP_CAN_HANDLE1, .id = 0x1ff, .len = 8, .data = {0} },
    [1]  = { .hcan = BSP_CAN_HANDLE1, .id = 0x200, .len = 8, .data = {0} },
    [2]  = { .hcan = BSP_CAN_HANDLE1, .id = 0x2ff, .len = 8, .data = {0} },
    [3]  = { .hcan = BSP_CAN_HANDLE1, .id = 0x1fe, .len = 8, .data = {0} },
    [4]  = { .hcan = BSP_CAN_HANDLE1, .id = 0x2fe, .len = 8, .data = {0} },
#if defined(STM32F407xx) || defined(STM32H723xx) || defined(STM32F105xC)
    [5]  = { .hcan = BSP_CAN_HANDLE2, .id = 0x1ff, .len = 8, .data = {0} },
    [6]  = { .hcan = BSP_CAN_HANDLE2, .id = 0x200, .len = 8, .data = {0} },
    [7]  = { .hcan = BSP_CAN_HANDLE2, .id = 0x2ff, .len = 8, .data = {0} },
    [8]  = { .hcan = BSP_CAN_HANDLE2, .id = 0x1fe, .len = 8, .data = {0} },
    [9]  = { .hcan = BSP_CAN_HANDLE2, .id = 0x2fe, .len = 8, .data = {0} },
#endif
#if defined(STM32H723xx)
    [10] = { .hcan = BSP_CAN_HANDLE3, .id = 0x1ff, .len = 8, .data = {0} },
    [11] = { .hcan = BSP_CAN_HANDLE3, .id = 0x200, .len = 8, .data = {0} },
    [12] = { .hcan = BSP_CAN_HANDLE3, .id = 0x2ff, .len = 8, .data = {0} },
    [13] = { .hcan = BSP_CAN_HANDLE3, .id = 0x1fe, .len = 8, .data = {0} },
    [14] = { .hcan = BSP_CAN_HANDLE3, .id = 0x2fe, .len = 8, .data = {0} },
#endif
};
/* clang-format on */

static uint8_t sender_enable_flag[MOTOR_SENDER_SIZE] = {0};

/* CAN 接收回调 */
static void dji_can_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    if (len < 8 || !dev->user_arg) return;
    DJI_Motor_t *motor = (DJI_Motor_t *)dev->user_arg;

    motor->measure.last_ecd                = motor->measure.ecd;
    motor->measure.ecd                     = ((uint16_t)data[0] << 8) | data[1];
    motor->base.measure.single_round_angle = ECD_ANGLE_COEF_DJI_RAD * (float)motor->measure.ecd;
    motor->measure.speed_rpm               = (int16_t)(data[2] << 8 | data[3]);
    motor->base.measure.speed_rad          = motor->measure.speed_rpm * RPM_2_RAD_PER_SEC;
    motor->measure.real_current            = (int16_t)(data[4] << 8 | data[5]);
    motor->measure.temperature             = data[6];

    /* 多圈角度计算 */
    int16_t delta_ecd = motor->measure.ecd - motor->measure.last_ecd;
    if (delta_ecd > 4096)
        motor->measure.total_round--;
    else if (delta_ecd < -4096)
        motor->measure.total_round++;

    motor->base.measure.total_angle = (float)motor->measure.total_round * (2.0f * PI) + motor->base.measure.single_round_angle;

    motor->base.measure.torque_nm = motor->base.controller.output_torque;

    /* 更新在线状态 */
    Module_Offline_device_update(motor->base.offline_dev);
}

/* PID 控制计算  */
static float CalculatePIDOutput(DJI_Motor_t *motor)
{
    float pid_measure, pid_ref;

    pid_ref = motor->base.controller.ref;
    if (motor->base.setting.motor_reverse_flag == 1) pid_ref *= -1;

    /* 位置环 */
    if (motor->base.setting.loop_type & ANGLE_LOOP)
    {
        pid_measure = (motor->base.setting.angle_feedback_source == 1 && motor->base.controller.other_angle_feedback_ptr)
                          ? *motor->base.controller.other_angle_feedback_ptr
                          : motor->base.measure.total_angle;

        if (motor->base.setting.feedback_reverse_flag == 1) pid_measure *= -1;
        pid_ref = PIDCalculate(&motor->base.controller.angle_PID, pid_measure, pid_ref);
    }

    /* 速度环 */
    if (motor->base.setting.loop_type & SPEED_LOOP)
    {
        pid_measure = (motor->base.setting.speed_feedback_source == 1 && motor->base.controller.other_speed_feedback_ptr)
                          ? *motor->base.controller.other_speed_feedback_ptr
                          : motor->base.measure.speed_rad;

        if (motor->base.setting.feedback_reverse_flag == 1) pid_measure *= -1;
        pid_ref = PIDCalculate(&motor->base.controller.speed_PID, pid_measure, pid_ref);
    }

    return pid_ref;
}

/* LQR 控制计算 */
static float CalculateLQROutput(DJI_Motor_t *motor)
{
    float ref, rad_angle, rad_speed;

    ref = motor->base.controller.ref;
    if (motor->base.setting.motor_reverse_flag == 1) ref *= -1;

    rad_angle = (motor->base.setting.angle_feedback_source == 1 && motor->base.controller.other_angle_feedback_ptr)
                    ? *motor->base.controller.other_angle_feedback_ptr
                    : motor->base.measure.total_angle;
    if (motor->base.setting.feedback_reverse_flag == 1) rad_angle *= -1;

    rad_speed = (motor->base.setting.speed_feedback_source == 1 && motor->base.controller.other_speed_feedback_ptr)
                    ? *motor->base.controller.other_speed_feedback_ptr
                    : motor->base.measure.speed_rad;
    if (motor->base.setting.feedback_reverse_flag == 1) rad_speed *= -1;

    return LQRCalculate(&motor->base.controller.lqr, rad_angle, rad_speed, ref);
}

/* dji控制函数 */
static void dji_control(Motor_Base *base)
{
    DJI_Motor_t *motor = MOTOR_GET_DERIVED(base, DJI_Motor_t);

    /* 离线或禁用: 清零 CAN 发送缓冲区 */
    if (Module_Offline_get_device_status(base->offline_dev) == STATE_OFFLINE || base->setting.enableflag == 0)
    {
        uint8_t group                              = motor->sender_group;
        uint8_t num                                = motor->message_num;
        sender_assignment[group].data[2 * num]     = 0;
        sender_assignment[group].data[2 * num + 1] = 0;
        base->controller.output                    = 0;
        base->controller.output_torque             = 0;
        base->controller.speed_PID.Output          = 0;
        base->controller.speed_PID.Iout            = 0;
        base->controller.angle_PID.Output          = 0;
        base->controller.angle_PID.Iout            = 0;
        return;
    }

    float torque = 0;

    /* PID/LQR 计算 */
    switch (base->setting.algorithm_type)
    {
    case CONTROL_PID:
        torque = CalculatePIDOutput(motor);
        break;
    case CONTROL_LQR:
        torque = CalculateLQROutput(motor);
        break;
    default:
        break;
    }

    torque = torque + motor->base.controller.feedforward_torque;
    VAL_LIMIT(torque, -motor->base.info.max_torque, motor->base.info.max_torque);
    motor->base.controller.output_torque = torque;

    /* 扭矩 → 电流 → 整数值 (CAN 报文) */
    switch (motor->base.info.motor_type)
    {
    case GM6020_CURRENT:
        motor->base.controller.output =
            currentToInteger(-3.0f, 3.0f, -16384, 16384, torque / (motor->base.info.torque_constant * motor->base.info.gear_ratio));
        break;
    case M3508:
        motor->base.controller.output =
            currentToInteger(-20.0f, 20.0f, -16384, 16384, torque / (motor->base.info.torque_constant * motor->base.info.gear_ratio));
        break;
    case M2006:
        motor->base.controller.output =
            currentToInteger(-10.0f, 10.0f, -10000, 10000, torque / (motor->base.info.torque_constant * motor->base.info.gear_ratio));
        break;
    default:
        motor->base.controller.output = 0;
        break;
    }

    /* 填充 CAN 发送缓冲区 (不实际发送) */
    uint8_t group                              = motor->sender_group;
    uint8_t num                                = motor->message_num;
    int16_t out                                = (int16_t)base->controller.output;
    sender_assignment[group].data[2 * num]     = (uint8_t)(out >> 8);
    sender_assignment[group].data[2 * num + 1] = (uint8_t)(out & 0x00ff);
}

/* CAN 发送分组计算 */
static UINT MotorSenderGrouping(DJI_Motor_t *motor, Can_Device_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1;
    uint8_t motor_send_num;
    uint8_t motor_grouping;
    UINT    status = TX_SUCCESS;

    switch (motor->base.info.motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            if (BSP_CAN_IS_HANDLE1(config->hcan))
                motor_grouping = 1;
            else if (BSP_CAN_IS_HANDLE2(config->hcan))
                motor_grouping = 6;
            else if (BSP_CAN_IS_HANDLE3(config->hcan))
                motor_grouping = 11;
            else
            {
                status = TX_DELETED;
                break;
            }
        }
        else
        {
            motor_send_num = motor_id - 4;
            if (BSP_CAN_IS_HANDLE1(config->hcan))
                motor_grouping = 0;
            else if (BSP_CAN_IS_HANDLE2(config->hcan))
                motor_grouping = 5;
            else if (BSP_CAN_IS_HANDLE3(config->hcan))
                motor_grouping = 10;
            else
            {
                status = TX_DELETED;
                break;
            }
        }
        config->rx_id = 0x200 + motor_id + 1;
        break;

    case GM6020_CURRENT:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            if (BSP_CAN_IS_HANDLE1(config->hcan))
                motor_grouping = 3;
            else if (BSP_CAN_IS_HANDLE2(config->hcan))
                motor_grouping = 8;
            else if (BSP_CAN_IS_HANDLE3(config->hcan))
                motor_grouping = 13;
            else
            {
                status = TX_DELETED;
                break;
            }
        }
        else
        {
            motor_send_num = motor_id - 4;
            if (BSP_CAN_IS_HANDLE1(config->hcan))
                motor_grouping = 4;
            else if (BSP_CAN_IS_HANDLE2(config->hcan))
                motor_grouping = 9;
            else if (BSP_CAN_IS_HANDLE3(config->hcan))
                motor_grouping = 14;
            else
            {
                status = TX_DELETED;
                break;
            }
        }
        config->rx_id = 0x204 + motor_id + 1;
        break;

    case GM6020_VOLTAGE:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            if (BSP_CAN_IS_HANDLE1(config->hcan))
                motor_grouping = 0;
            else if (BSP_CAN_IS_HANDLE2(config->hcan))
                motor_grouping = 5;
            else if (BSP_CAN_IS_HANDLE3(config->hcan))
                motor_grouping = 10;
            else
            {
                status = TX_DELETED;
                break;
            }
        }
        else
        {
            motor_send_num = motor_id - 4;
            if (BSP_CAN_IS_HANDLE1(config->hcan))
                motor_grouping = 2;
            else if (BSP_CAN_IS_HANDLE2(config->hcan))
                motor_grouping = 7;
            else if (BSP_CAN_IS_HANDLE3(config->hcan))
                motor_grouping = 12;
            else
            {
                status = TX_DELETED;
                break;
            }
        }
        config->rx_id = 0x204 + motor_id + 1;
        break;

    default:
        status = TX_DELETED;
        break;
    }

    if (status != TX_SUCCESS) return status;

    sender_enable_flag[motor_grouping] = 1;
    motor->message_num                 = motor_send_num;
    motor->sender_group                = motor_grouping;

    return TX_SUCCESS;
}

/* 对外函数 */
DJI_Motor_t *Motor_DJI_Init(Motor_Init_Config_s *config)
{
    DJI_Motor_t *motor = NULL;
    BSP_MEM_ALLOC_WAIT(motor, sizeof(DJI_Motor_t), TX_NO_WAIT);
    if (motor == NULL)
    {
        LOG_E("Failed to allocate memory for DJI motor");
        return NULL;
    }
    memset(motor, 0, sizeof(DJI_Motor_t));

    /* 初始化基类字段 */
    motor->base.type      = config->motor_init_info.motor_type;
    motor->base.transport = MOTOR_TRANSPORT_CAN;
    motor->base.info      = config->motor_init_info;
    motor->base.setting   = config->setting_init_config;

    /* 电机分组 */
    if (MotorSenderGrouping(motor, &config->transport_config.can) != TX_SUCCESS)
    {
        LOG_E("Motor Sender Grouping Failed!");
        BSP_MEM_FREE(motor);
        return NULL;
    }

    /* 注册 CAN 设备 */
    Can_Device *can_dev = BSP_CAN_Device_Init(&config->transport_config.can);
    if (can_dev == NULL)
    {
        LOG_E("Failed to initialize CAN device for DJI motor");
        BSP_MEM_FREE(motor);
        return NULL;
    }
    motor->base.transport_dev = can_dev;

    /* 设置 CAN 接收回调 */
    can_dev->rx_callback = dji_can_rx_callback;
    can_dev->user_arg    = motor;

    /* 初始化控制器 */
    if (motor->base.setting.algorithm_type == CONTROL_PID)
    {
        PIDInit(&motor->base.controller.speed_PID, &config->controller_init_config.speed_PID);
        PIDInit(&motor->base.controller.angle_PID, &config->controller_init_config.angle_PID);
    }
    else if (motor->base.setting.algorithm_type == CONTROL_LQR)
    {
        LQRInit(&motor->base.controller.lqr, &config->controller_init_config.lqr_init);
    }
    motor->base.controller.other_angle_feedback_ptr = config->controller_init_config.other_angle_feedback_ptr;
    motor->base.controller.other_speed_feedback_ptr = config->controller_init_config.other_speed_feedback_ptr;

    /* 离线检测 */
    motor->base.offline_dev = Module_Offline_register(&config->offline_init_config);

    /* 注册到全局链表 */
    motor->base.ControlAndSend = dji_control;
    Motor_Register(&motor->base);

    LOG_I("DJI motor initialized (type=%d, tx_id=%d, rx_id=%d)", motor->base.info.motor_type, config->transport_config.can.tx_id,
          config->transport_config.can.rx_id);

    return motor;
}

void Motor_DJI_Stop(DJI_Motor_t *motor) { motor->base.setting.enableflag = 0; }
void Motor_DJI_Start(DJI_Motor_t *motor) { motor->base.setting.enableflag = 1; }

void Motor_DJI_ChangeFeed(DJI_Motor_t *motor, Closeloop_Type_e loop, uint8_t feedback_source)
{
    if (loop == ANGLE_LOOP)
        motor->base.setting.angle_feedback_source = feedback_source;
    else if (loop == SPEED_LOOP)
        motor->base.setting.speed_feedback_source = feedback_source;
}

void Motor_DJI_OuterLoop(DJI_Motor_t *motor, Closeloop_Type_e outer_loop) { motor->base.setting.loop_type = outer_loop; }

void Motor_DJI_SetRef(DJI_Motor_t *motor, float ref) { motor->base.controller.ref = ref; }

void Motor_DJI_Flush(void)
{
    for (size_t i = 0; i < MOTOR_SENDER_SIZE; ++i)
    {
        if (sender_enable_flag[i])
        {
            BSP_CAN_SendMessage(&sender_assignment[i]);
        }
    }
}

void Motor_DJI_SetForwardTorque(DJI_Motor_t *motor, float torque) { motor->base.controller.feedforward_torque = torque; }
