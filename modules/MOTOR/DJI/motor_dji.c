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

static uint8_t sender_motor_count[MOTOR_SENDER_SIZE] = {0}; /* 组内电机数 (Init 时累加) */
static uint8_t sender_write_count[MOTOR_SENDER_SIZE] = {0}; /* 本周期已写入数 */

/* 写入完成后计数, 组内最后一个电机负责发送整帧 */
static void dji_batch_send(DJI_Motor_t *motor)
{
    uint8_t g = motor->sender_group;

    sender_write_count[g]++;
    if (sender_write_count[g] >= sender_motor_count[g])
    {
        sender_write_count[g] = 0;
        BSP_CAN_SendMessage(&sender_assignment[g]);
    }
}

/* CAN 接收回调 */
static void dji_can_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    if (len < 8 || !dev->user_arg) return;
    DJI_Motor_t *motor = (DJI_Motor_t *)dev->user_arg;

    motor->measure.last_ecd     = motor->measure.ecd;
    motor->measure.ecd          = ((uint16_t)data[0] << 8) | data[1];
    motor->measure.speed_rpm    = (int16_t)(data[2] << 8 | data[3]);
    motor->measure.real_current = (int16_t)(data[4] << 8 | data[5]);
    motor->measure.temperature  = data[6];

    /* 编码器/报文在电机轴上: 按电机轴端算单圈/转速/整圈, 末了统一折算到输出轴 */
    const float gear_ratio       = (motor->base.info.gear_ratio > 0.0f) ? motor->base.info.gear_ratio : 1.0f;
    float       single_motor_rad = ECD_ANGLE_COEF_DJI_RAD * (float)motor->measure.ecd;
    float       speed_motor_rad  = motor->measure.speed_rpm * RPM_2_RAD_PER_SEC;

    /* 多圈计数: 单圈角跨 2π 边界时 total_round 增减 (沿用原始 ecd 阈值判断) */
    int16_t delta_ecd = motor->measure.ecd - motor->measure.last_ecd;
    if (delta_ecd > 4096)
        motor->measure.total_round--;
    else if (delta_ecd < -4096)
        motor->measure.total_round++;

    /* 电机轴累计角 = 整圈×2π + 当前单圈, 统一折算到输出轴 */
    float motor_total_rad           = (float)motor->measure.total_round * (2.0f * PI) + single_motor_rad;
    motor->base.measure.total_angle = motor_total_rad / gear_ratio;
    motor->base.measure.speed_rad   = speed_motor_rad / gear_ratio;

    /* 单圈角度(输出轴 0~2π): 去掉输出轴累计角的整圈部分, 每输出轴一整圈回绕一次 */
    float output_single_rad = motor->base.measure.total_angle * (1.0f / (2.0f * PI));
    output_single_rad = (output_single_rad - (float)(int32_t)output_single_rad) * (2.0f * PI);
    if (output_single_rad < 0.0f)
    {
        output_single_rad += (2.0f * PI);
    }
    motor->base.measure.single_round_angle = output_single_rad;

    /* 力矩: 镜像自下发命令, 本就为输出端扭矩 */
    motor->base.measure.torque_nm = motor->base.controller.output_torque;

    /* 更新在线状态 */
    Module_Offline_device_update(motor->base.offline_dev);
}

static void dji_apply(Motor_Base *base)
{
    DJI_Motor_t *motor   = MOTOR_GET_DERIVED(base, DJI_Motor_t);
    uint8_t      offline = (base->offline_dev != NULL && Module_Offline_get_device_status(base->offline_dev) == STATE_OFFLINE);

    if (offline || base->setting.enableflag == 0)
    {
        sender_assignment[motor->sender_group].data[2 * motor->message_num]     = 0;
        sender_assignment[motor->sender_group].data[2 * motor->message_num + 1] = 0;
        dji_batch_send(motor);
        return;
    }

    float torque = base->controller.output_torque;

    /* 扭矩 → 电流 → 整数值 (CAN 报文)
     * 轴端契约: output_torque / torque_constant 均为输出轴端。
     * 电流(A) = τ_out(输出轴 Nm) / Kt(输出轴 Nm/A) */
    switch (base->info.motor_type)
    {
    case GM6020_CURRENT:
        base->controller.output = currentToInteger(-3.0f, 3.0f, -16384, 16384, torque / base->info.torque_constant);
        break;
    case M3508:
        base->controller.output = currentToInteger(-20.0f, 20.0f, -16384, 16384, torque / base->info.torque_constant);
        break;
    case M2006:
        base->controller.output = currentToInteger(-10.0f, 10.0f, -10000, 10000, torque / base->info.torque_constant);
        break;
    default:
        base->controller.output = 0;
        break;
    }

    int16_t out                                                             = (int16_t)base->controller.output;
    sender_assignment[motor->sender_group].data[2 * motor->message_num]     = (uint8_t)(out >> 8);
    sender_assignment[motor->sender_group].data[2 * motor->message_num + 1] = (uint8_t)(out & 0x00ff);
    dji_batch_send(motor);
}

/* CAN 发送分组计算 (同时设置 config->rx_id) */
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

    motor->sender_group = motor_grouping;
    motor->message_num  = motor_send_num;

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

    /* 电机分组 (同时计算 rx_id) */
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

    /* 组内电机计数 (用于最后一个写入者发送整帧) */
    sender_motor_count[motor->sender_group]++;

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

    /* 绑定两阶段调度, 注册到全局链表 */
    motor->base.Apply = dji_apply;
    Motor_Register(&motor->base);

    LOG_I("DJI motor initialized (type=%d, tx_id=%d, rx_id=%d)", motor->base.info.motor_type, config->transport_config.can.tx_id,
          config->transport_config.can.rx_id);

    return motor;
}
