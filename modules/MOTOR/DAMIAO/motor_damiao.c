#include "motor_damiao.h"
#include "bsp_can.h"
#include "bsp_dwt.h"
#include "bsp_def.h"
#include "module_offline.h"
#include "user_lib.h"
#include <stdint.h>
#include <string.h>

#define LOG_TAG "motor_dm"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

/* 达妙电机参数配置表 */
static const DM_Motor_Params_t dm_motor_params[] = {
    {-12.5f, 12.5f, -30.0f, 30.0f, 0.0f, 500.0f, 0.0f, 5.0f, -10.0f, 10.0f}, /* DM4310 */
};

static const DM_Motor_Params_t *dm_get_params(Motor_Type_e type)
{
    switch (type)
    {
    case DM4310:
        return &dm_motor_params[0];
    /* 后续在此添加其他型号映射 */
    default:
        return &dm_motor_params[0];
    }
}

/*  CAN 接收回调 */
static void dm_can_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    if (len < 8 || !dev->user_arg) return;
    DM_Motor_t              *motor = (DM_Motor_t *)dev->user_arg;
    const DM_Motor_Params_t *param = motor->params;

    uint8_t error_code        = (data[0] >> 4) & 0x0F;
    motor->measure.id         = data[0] & 0x0F;
    motor->measure.Error_Code = (error_code >= 0x08 && error_code <= 0x0E) ? (DMMotorError_t)error_code : DM_NO_ERROR;

    uint16_t tmp;
    tmp                                    = (uint16_t)((data[1] << 8) | data[2]);
    motor->base.measure.single_round_angle = uint_to_float(tmp, param->p_min, param->p_max, 16);
    tmp                                    = (uint16_t)((data[3] << 4) | data[4] >> 4);
    motor->base.measure.speed_rad          = uint_to_float(tmp, param->v_min, param->v_max, 12);
    tmp                                    = (uint16_t)(((data[4] & 0x0f) << 8) | data[5]);
    motor->measure.torque                  = uint_to_float(tmp, param->t_min, param->t_max, 12);
    motor->measure.T_Mos                   = (float)data[6];
    motor->measure.T_Rotor                 = (float)data[7];

    float current_angle = motor->base.measure.single_round_angle;
    float diff          = current_angle - motor->measure.last_single_round_angle;
    float range         = param->p_max - param->p_min;
    float half_range    = range * 0.5f;
    if (diff < -half_range)
    {
        diff += range;
        motor->measure.total_round++;
    }
    else if (diff > half_range)
    {
        diff -= range;
        motor->measure.total_round--;
    }

    motor->base.measure.total_angle += diff;
    motor->measure.last_single_round_angle = current_angle;
    motor->base.measure.torque_nm          = motor->measure.torque;

    Module_Offline_device_update(motor->base.offline_dev);
}

/*  MIT 模式控制帧 */
static void mit_ctrl(DM_Motor_t *motor, float pos, float vel, float kp, float kd, float torq)
{
    Can_Device              *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t             msg;
    const DM_Motor_Params_t *param = motor->params;

    /* 从 Can_Device 获取 CAN 句柄 */
    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan             = bus->hcan;
    msg.id               = can_dev->tx_id + DM_MIT_MODE;
    msg.len              = 8;

    uint16_t pos_tmp = float_to_uint(pos, param->p_min, param->p_max, 16);
    uint16_t vel_tmp = float_to_uint(vel, param->v_min, param->v_max, 12);
    uint16_t kp_tmp  = float_to_uint(kp, param->kp_min, param->kp_max, 12);
    uint16_t kd_tmp  = float_to_uint(kd, param->kd_min, param->kd_max, 12);
    uint16_t tor_tmp = float_to_uint(torq, param->t_min, param->t_max, 12);

    msg.data[0] = (pos_tmp >> 8);
    msg.data[1] = pos_tmp;
    msg.data[2] = (vel_tmp >> 4);
    msg.data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    msg.data[4] = kp_tmp;
    msg.data[5] = (kd_tmp >> 4);
    msg.data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    msg.data[7] = tor_tmp;

    BSP_CAN_SendMessage(&msg);
}

/* 位置-速度模式控制帧 */
static void pos_speed_ctrl(DM_Motor_t *motor, float pos_degree, float vel)
{
    Can_Device              *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t             msg;
    const DM_Motor_Params_t *param = motor->params;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan             = bus->hcan;
    msg.id               = can_dev->tx_id + DM_POS_MODE;
    msg.len              = 8;

    float    pos_rad = deg_to_rad(pos_degree);
    uint8_t *pbuf    = (uint8_t *)&pos_rad;
    uint8_t *vbuf    = (uint8_t *)&vel;

    VAL_LIMIT(pos_rad, param->p_min, param->p_max);
    msg.data[0] = pbuf[0];
    msg.data[1] = pbuf[1];
    msg.data[2] = pbuf[2];
    msg.data[3] = pbuf[3];
    msg.data[4] = vbuf[0];
    msg.data[5] = vbuf[1];
    msg.data[6] = vbuf[2];
    msg.data[7] = vbuf[3];

    BSP_CAN_SendMessage(&msg);
}

/* 速度模式控制帧 */
static void speed_ctrl(DM_Motor_t *motor, float vel)
{
    Can_Device  *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t msg;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan             = bus->hcan;
    msg.id               = can_dev->tx_id + DM_SPD_MODE;
    msg.len              = 4;

    uint8_t *vbuf = (uint8_t *)&vel;
    msg.data[0]   = vbuf[0];
    msg.data[1]   = vbuf[1];
    msg.data[2]   = vbuf[2];
    msg.data[3]   = vbuf[3];

    BSP_CAN_SendMessage(&msg);
}

/* PSI 模式控制帧 (位置-速度-电流) */
static void psi_ctrl(DM_Motor_t *motor, float pos, float vel, float current)
{
    Can_Device  *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t msg;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan             = bus->hcan;
    msg.id               = can_dev->tx_id + DM_PSI_MODE;
    msg.len              = 8;

    uint16_t u16_vel = vel * 100;
    uint16_t u16_cur = current * 10000;

    uint8_t *pbuf = (uint8_t *)&pos;
    uint8_t *vbuf = (uint8_t *)&u16_vel;
    uint8_t *ibuf = (uint8_t *)&u16_cur;

    msg.data[0] = pbuf[0];
    msg.data[1] = pbuf[1];
    msg.data[2] = pbuf[2];
    msg.data[3] = pbuf[3];
    msg.data[4] = vbuf[0];
    msg.data[5] = vbuf[1];
    msg.data[6] = ibuf[0];
    msg.data[7] = ibuf[1];

    BSP_CAN_SendMessage(&msg);
}

static void dm_apply(Motor_Base *base)
{
    DM_Motor_t *motor   = MOTOR_GET_DERIVED(base, DM_Motor_t);
    uint8_t     offline = (base->offline_dev != NULL && Module_Offline_get_device_status(base->offline_dev) == STATE_OFFLINE);

    if (offline || base->setting.enableflag == 0)
    {
        switch (motor->mode_type)
        {
        case DM_MIT_MODE:
            mit_ctrl(motor, 0, 0, 0, 0, 0);
            break;
        case DM_POS_MODE:
            pos_speed_ctrl(motor, 0, 0);
            break;
        case DM_SPD_MODE:
            speed_ctrl(motor, 0);
            break;
        case DM_PSI_MODE:
            psi_ctrl(motor, 0, 0, 0);
            break;
        default:
            break;
        }
        return;
    }

    switch (motor->mode_type)
    {
    case DM_MIT_MODE:
        mit_ctrl(motor, 0, 0, 0, 0, base->controller.output_torque);
        break;
    case DM_POS_MODE:
        pos_speed_ctrl(motor, base->controller.ref, PI);
        break;
    case DM_SPD_MODE:
        speed_ctrl(motor, base->controller.ref);
        break;
    case DM_PSI_MODE:
        psi_ctrl(motor, 0, 0, base->controller.ref);
        break;
    default:
        break;
    }
    // BSP_DWT_Delay(0.0002f); /* 每完成一次发送，就延迟200us */
}

/* 对外函数 */
void Motor_DM_Cmd(DM_Motor_t *motor, DMMotor_Mode_e cmd)
{
    Can_Device  *can_dev = (Can_Device *)motor->base.transport_dev;
    BSP_CanMsg_t msg;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)can_dev->_bus;
    msg.hcan             = bus->hcan;
    msg.id               = can_dev->tx_id + motor->mode_type;
    msg.len              = 8;

    msg.data[0] = 0xff;
    msg.data[1] = 0xff;
    msg.data[2] = 0xff;
    msg.data[3] = 0xff;
    msg.data[4] = 0xff;
    msg.data[5] = 0xff;
    msg.data[6] = 0xff;
    msg.data[7] = (uint8_t)cmd;

    BSP_CAN_SendMessage(&msg);
    BSP_DWT_Delay(0.0002f); /* 200us 间隔 */
}

DM_Motor_t *Motor_DM_Init(Motor_Init_Config_s *config, uint32_t DM_Mode_type)
{
    DM_Motor_t *motor = NULL;
    BSP_MEM_ALLOC_WAIT(motor, sizeof(DM_Motor_t), TX_NO_WAIT);
    if (motor == NULL)
    {
        LOG_E("Failed to allocate memory for DM motor");
        return NULL;
    }
    memset(motor, 0, sizeof(DM_Motor_t));

    /* 初始化基类字段 */
    motor->base.type      = config->motor_init_info.motor_type;
    motor->base.transport = MOTOR_TRANSPORT_CAN;
    motor->base.info      = config->motor_init_info;
    motor->base.setting   = config->setting_init_config;
    motor->mode_type      = DM_Mode_type;
    motor->params         = dm_get_params(config->motor_init_info.motor_type);

    /* Master ID (rx_id) 必须大于 CAN ID (tx_id) 且各不相同 */
    {
        uint32_t can_id    = config->transport_config.can.tx_id;
        uint32_t master_id = config->transport_config.can.rx_id;
        if (master_id <= can_id)
        {
            LOG_E("DM motor init failed: Master ID (rx_id=0x%03X) must be greater than CAN ID (tx_id=0x%03X)", master_id, can_id);
            BSP_MEM_FREE(motor);
            return NULL;
        }
    }

    /* 注册 CAN 设备 */
    Can_Device *can_dev = BSP_CAN_Device_Init(&config->transport_config.can);
    if (can_dev == NULL)
    {
        LOG_E("Failed to initialize CAN device for DM motor");
        BSP_MEM_FREE(motor);
        return NULL;
    }
    motor->base.transport_dev = can_dev;

    /* 设置 CAN 接收回调 */
    can_dev->rx_callback = dm_can_rx_callback;
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

    /* 清除错误 + 使能电机 */
    motor->measure.Error_Code = DM_NO_ERROR;
    Motor_DM_Cmd(motor, DM_CMD_CLEAR_ERROR);
    Motor_DM_Cmd(motor, DM_CMD_MOTOR_START);

    /* 注册到全局链表 */
    motor->base.Apply   = dm_apply;
    Motor_Register(&motor->base);

    LOG_I("DM motor initialized (type=%d, mode=0x%03X)", motor->base.info.motor_type, DM_Mode_type);

    return motor;
}
