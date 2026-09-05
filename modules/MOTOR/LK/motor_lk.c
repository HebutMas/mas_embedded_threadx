#include "motor_lk.h"
#include "motor_lk_def.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_def.h"
#include "module_offline.h"
#include "user_lib.h"


#define LOG_TAG "motor_lk"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"
/* 根据电机型号查编码器最大值 */
static uint16_t lk_get_encoder_max(Motor_Type_e type)
{
    switch (type)
    {
    case MG8016:
        return LK_ENCODER_MAX_16BIT;
    /* 后续在此添加其他型号映射 */
    default:
        return LK_ENCODER_MAX_16BIT;
    }
}
/* 翎控电机基本命令发布 */
static void Motor_LK_Cmd(LK_Motor_t *motor, uint8_t cmd)
{
    if (motor == NULL) return;
    Can_Device *can_dev = (Can_Device *)motor->base.transport_dev;
    if (can_dev == NULL || can_dev->_bus == NULL) return;

    BSP_CanMsg_t     msg;
    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;

    msg.hcan    = bus->hcan;
    msg.id      = can_dev->tx_id;
    msg.len     = LK_CAN_DLC;
    msg.data[0] = cmd;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    BSP_CAN_SendMessage(&msg);
    BSP_DWT_Delay(0.0002f); /* 防止can总线拥挤 */
}
/* 读取电机状态2 */
static void Motor_LK_ReadStatus2(LK_Motor_t *motor)
{
    Motor_LK_Cmd(motor, LK_CMD_READ_STATUS2);
}

/* 清除错误标志 */
static void Motor_LK_ClearError(LK_Motor_t *motor)
{
    Motor_LK_Cmd(motor, LK_CMD_CLEAR_ERROR);
}

/* 电机关闭 */
static void Motor_LK_MotorOff(LK_Motor_t *motor)
{
    Motor_LK_Cmd(motor, LK_CMD_MOTOR_OFF);
}

/* 电机运行 */
static void Motor_LK_MotorRun(LK_Motor_t *motor)
{
    Motor_LK_Cmd(motor, LK_CMD_MOTOR_RUN);
}

/* 电机停止 */
static void Motor_LK_MotorStop(LK_Motor_t *motor)
{
    Motor_LK_Cmd(motor, LK_CMD_MOTOR_STOP);
}

/* 翎控电机接收回调 */
static void lk_can_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    if (len < 8 || !dev->user_arg) return;

    LK_Motor_t *motor = (LK_Motor_t *)dev->user_arg;
    uint8_t     cmd   = data[0];
    /* 状态2 回复帧解析 (读取状态2 + 所有闭环控制命令均回复此格式（因为只用扭矩闭环，因此只加它）) */
    if (cmd == LK_CMD_READ_STATUS2 || cmd == LK_CMD_TORQUE_CLOSED_LOOP)
    {
        /* 1. 解析原始数据 */
        motor->measure.temperature = (int8_t)data[1];
        motor->measure.iq          = (int16_t)((data[3] << 8) | data[2]);
        motor->measure.power       = motor->measure.iq; /* MS 电机此处为输出功率，字段复用 */
        motor->measure.speed       = (int16_t)((data[5] << 8) | data[4]);
        motor->measure.encoder     = (uint16_t)((data[7] << 8) | data[6]);

        /* 2. 计算基类测量值 — 输出轴端 (减速后)。编码器/转速均在电机轴上,
         *    按 gear_ratio 统一折算到输出轴, 使 base.measure 与上层轴端契约一致 */
        const float gear_ratio = (motor->base.info.gear_ratio > 0.0f) ? motor->base.info.gear_ratio : 1.0f;

        /* 速度: 电机轴 dps → rad/s → 输出轴 */
        motor->base.measure.speed_rad = motor->measure.speed * DEGREE_2_RAD / gear_ratio;

        /* 电机轴端单圈角: 编码器原始读数 (0~2π) */
        uint16_t encoder_max            = lk_get_encoder_max(motor->base.info.motor_type);
        float    single_motor_angle_rad = ((float)motor->measure.encoder / (float)(encoder_max + 1u))
                                          * (2.0f * PI);

        /* 整圈计数: 单圈角跨 2π 边界时 total_round ±1 (首帧只记录基准, 不判跨圈) */
        if (!motor->measure.first_frame)
        {
            motor->measure.first_frame = 1;
        }
        else
        {
            float diff = single_motor_angle_rad - motor->measure.last_single_round_angle;
            if (diff < -PI)
            {
                motor->measure.total_round++;
            }
            else if (diff > PI)
            {
                motor->measure.total_round--;
            }
        }
        motor->measure.last_single_round_angle = single_motor_angle_rad;

        /* 电机轴累计角 = 整圈×2π + 当前单圈, 折算到输出轴。 */
        motor->base.measure.total_angle = ((float)motor->measure.total_round * (2.0f * PI) + single_motor_angle_rad) / gear_ratio;

        /* 单圈角度(输出轴 0~2π): 去掉输出轴累计角的整圈部分, 每输出轴一整圈回绕一次。 */
        float output_single_rad = motor->base.measure.total_angle * (1.0f / (2.0f * PI));
        output_single_rad = (output_single_rad - (float)(int32_t)output_single_rad) * (2.0f * PI);
        if (output_single_rad < 0.0f)
        {
            output_single_rad += (2.0f * PI);
        }
        motor->base.measure.single_round_angle = output_single_rad;

        /* 力矩: iq → 电流 → 输出轴扭矩 (仅 MF/MG, MS 无意义); torque_constant 为输出轴扭矩常数 */
        if (motor->base.info.motor_type == MG8016)
        {
            motor->base.measure.torque_nm = (int16_t)motor->measure.iq
                                            * LK_MG_TORQUE_CURRENT_RES
                                            * motor->base.info.torque_constant;
        }
        else
        {
            motor->base.measure.torque_nm = 0.0f;
        }

        Module_Offline_device_update(motor->base.offline_dev);
    }
}
/* 转矩闭环控制 — 仅 MF/MH/MG */
static void Motor_LK_TorqueCtrl(LK_Motor_t *motor, int16_t iq)
{
    if (motor == NULL) return;
    Can_Device *can_dev = (Can_Device *)motor->base.transport_dev;
    if (can_dev == NULL || can_dev->_bus == NULL) return;

    VAL_LIMIT(iq, LK_TORQUE_IQ_MIN, LK_TORQUE_IQ_MAX);

    BSP_CanMsg_t     msg;
    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;

    msg.hcan    = bus->hcan;
    msg.id      = can_dev->tx_id;
    msg.len     = LK_CAN_DLC;
    msg.data[0] = LK_CMD_TORQUE_CLOSED_LOOP;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = (uint8_t)(iq & 0xFF);
    msg.data[5] = (uint8_t)((iq >> 8) & 0xFF);
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    BSP_CAN_SendMessage(&msg);
    BSP_DWT_Delay(0.0002f); /* 200us间隔防止can总线出错 */
}
/**
 * @brief 翎控电机阶段2: 输出应用 (每周期调用)
 * @note  轴端契约: output_torque / torque_constant / base.measure 均为输出轴端 (减速后)。
 *         数据流: output_torque(输出轴 Nm) → ÷Kt(输出轴 Nm/A) → 电流(A) → 分辨率 → iq → CAN 发送
 */
static void lk_apply(Motor_Base *base)
{
    LK_Motor_t *motor = MOTOR_GET_DERIVED(base, LK_Motor_t);
    uint8_t     offline = (base->offline_dev != NULL &&
                           Module_Offline_get_device_status(base->offline_dev) == STATE_OFFLINE);

    /* 离线或禁用: 发零扭矩 */
    if (offline || base->setting.enableflag == 0)
    {
        switch (motor->mode_type)
        {
        case LK_CMD_TORQUE_CLOSED_LOOP:
            Motor_LK_TorqueCtrl(motor, 0);
            break;
        default:
            break;
        }
        return;
    }

    /* 扭矩(Nm) → iqControl → 按模式发送
     * 输出扭矩 ÷ Kt(输出轴 Nm/A) = 电机电流(A)
     * MG: ±2048 ↔ ±33A,  MF: ±2048 ↔ ±16.5A */
    int16_t iq = 0;
    switch (base->info.motor_type)
    {
    case MG8016:
    {
        /* output_torque 为输出轴端扭矩; torque_constant 为输出轴端扭矩常数 */
        float motor_current = base->controller.output_torque / base->info.torque_constant;
        iq = (int16_t)(motor_current / LK_MG_TORQUE_CURRENT_RES);
        break;
    }
    /* 后续在此添加 MF / MH / MS 映射 */
    default:
        break;
    }
    VAL_LIMIT(iq, LK_TORQUE_IQ_MIN, LK_TORQUE_IQ_MAX);

    switch (motor->mode_type)
    {
    case LK_CMD_TORQUE_CLOSED_LOOP:
        Motor_LK_TorqueCtrl(motor, iq);
        break;
    /* 后续在此添加其他模式: 速度/位置闭环 */
    default:
        break;
    }
}
/* 翎控电机初始化 */
LK_Motor_t *Motor_LK_Init(Motor_Init_Config_s *config, uint32_t LK_Mode_type)
{
    LK_Motor_t *motor = NULL;
    BSP_MEM_ALLOC_WAIT(motor, sizeof(LK_Motor_t), TX_NO_WAIT);
    if (motor == NULL)
    {
        LOG_E("Failed to allocate memory for LK motor");
        return NULL;
    }
    memset(motor, 0, sizeof(LK_Motor_t));

    /* 初始化基类字段 */
    motor->base.type      = config->motor_init_info.motor_type;
    motor->base.transport = MOTOR_TRANSPORT_CAN;
    motor->base.info      = config->motor_init_info;
    motor->base.setting   = config->setting_init_config;
    motor->mode_type      = LK_Mode_type;

    /* 注册 CAN 设备 */
    Can_Device *can_dev = BSP_CAN_Device_Init(&config->transport_config.can);
    if (can_dev == NULL)
    {
        LOG_E("Failed to initialize CAN device for LK motor");
        BSP_MEM_FREE(motor);
        return NULL;
    }
    motor->base.transport_dev = can_dev;

    /* 设置 CAN 接收回调 */
    can_dev->rx_callback = lk_can_rx_callback;
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

    //上电后默认就是使能,开启状态
    Motor_LK_ClearError(motor);
    Motor_LK_MotorRun(motor);
    /* 注册到全局链表 */
    motor->base.Apply   = lk_apply;
    Motor_Register(&motor->base);

    LOG_I("LK motor initialized (type=%d, mode=0x%03X)", motor->base.info.motor_type, LK_Mode_type);

    return motor;
}
