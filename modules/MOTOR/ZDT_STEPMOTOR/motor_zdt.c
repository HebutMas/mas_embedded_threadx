#include "motor_zdt.h"
#include "bsp_def.h"
#include <string.h>

#define LOG_TAG "motor_uart"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 命令字节 */
#define CMD_ENABLE   0xF3
#define CMD_SPEED    0xF6
#define CMD_POSITION 0xFD
#define CMD_STOP     0xFE
#define CMD_CLR_POS  0x0A
#define CHECKSUM     0x6B

static void zdt_enable(UART_Motor_t *m, uint8_t state)
{
    m->tx_buf[0] = m->addr;
    m->tx_buf[1] = CMD_ENABLE;
    m->tx_buf[2] = 0xAB;
    m->tx_buf[3] = state;
    m->tx_buf[4] = 0x00; /* 不同步 */
    m->tx_buf[5] = CHECKSUM;
    BSP_UART_Send(m->uart_dev, m->tx_buf, 6, TX_WAIT_FOREVER);
}

static void zdt_stop(UART_Motor_t *m)
{
    m->tx_buf[0] = m->addr;
    m->tx_buf[1] = CMD_STOP;
    m->tx_buf[2] = 0x98;
    m->tx_buf[3] = 0x00;
    m->tx_buf[4] = CHECKSUM;
    BSP_UART_Send(m->uart_dev, m->tx_buf, 5, TX_WAIT_FOREVER);
}

static void zdt_speed_cmd(UART_Motor_t *m, float rpm)
{
    uint8_t  dir = (rpm >= 0) ? 0x00 : 0x01; /* 0=CW, 1=CCW */
    uint16_t spd = (uint16_t)((rpm >= 0) ? rpm : -rpm);

    m->tx_buf[0] = m->addr;
    m->tx_buf[1] = CMD_SPEED;
    m->tx_buf[2] = dir;
    m->tx_buf[3] = (uint8_t)(spd >> 8);
    m->tx_buf[4] = (uint8_t)(spd & 0xFF);
    m->tx_buf[5] = m->accel;
    m->tx_buf[6] = 0x00; /* 不同步 */
    m->tx_buf[7] = CHECKSUM;
    BSP_UART_Send(m->uart_dev, m->tx_buf, 8, TX_WAIT_FOREVER);
}

static void zdt_position_cmd(UART_Motor_t *m, int32_t pulses, uint16_t speed_rpm)
{
    uint8_t  dir        = (pulses >= 0) ? 0x00 : 0x01;
    uint32_t abs_pulses = (uint32_t)((pulses >= 0) ? pulses : -pulses);

    m->tx_buf[0]  = m->addr;
    m->tx_buf[1]  = CMD_POSITION;
    m->tx_buf[2]  = dir;
    m->tx_buf[3]  = (uint8_t)(speed_rpm >> 8);
    m->tx_buf[4]  = (uint8_t)(speed_rpm & 0xFF);
    m->tx_buf[5]  = m->accel;
    m->tx_buf[6]  = (uint8_t)(abs_pulses >> 24);
    m->tx_buf[7]  = (uint8_t)(abs_pulses >> 16);
    m->tx_buf[8]  = (uint8_t)(abs_pulses >> 8);
    m->tx_buf[9]  = (uint8_t)(abs_pulses & 0xFF);
    m->tx_buf[10] = 0x00; /* 相对位置模式 */
    m->tx_buf[11] = 0x00; /* 不同步 */
    m->tx_buf[12] = CHECKSUM;
    BSP_UART_Send(m->uart_dev, m->tx_buf, 13, TX_WAIT_FOREVER);
}

/* 开环直通: 无控制计算阶段, loop_type 恒为 OPEN_LOOP 由基类统一跳过 */
static void zdt_apply(Motor_Base *base)
{
    UART_Motor_t *m = MOTOR_GET_DERIVED(base, UART_Motor_t);

    if (!base->setting.enableflag)
    {
        if (m->enable_state != 0)
        {
            zdt_enable(m, 0);
            m->enable_state = 0;
        }
        return;
    }

    /* 确保电机已使能 */
    if (m->enable_state != 1)
    {
        zdt_enable(m, 1);
        m->enable_state = 1;
    }

    if (m->mode == UART_MODE_POSITION)
    {
        /* 位置模式: ref = 脉冲数 */
        int32_t  pulses    = (int32_t)base->controller.ref;
        uint16_t speed_rpm = (uint16_t)base->info.max_torque; /* max_torque 复用为位置模式速度 */
        if (speed_rpm == 0) speed_rpm = 300;
        zdt_position_cmd(m, pulses, speed_rpm);
    }
    else
    {
        /* 速度模式: ref = RPM */
        float rpm = base->controller.ref;
        zdt_speed_cmd(m, rpm);
    }
}

/* 对外函数 */

UART_Motor_t *Motor_ZDT_Init(Motor_Init_Config_s *config, uint8_t addr, uint16_t pulse_per_rev, uint8_t accel)
{
    UART_Motor_t *motor = NULL;
    BSP_MEM_ALLOC_WAIT(motor, sizeof(UART_Motor_t), TX_NO_WAIT);
    if (!motor)
    {
        LOG_E("alloc failed");
        return NULL;
    }
    memset(motor, 0, sizeof(UART_Motor_t));

    motor->base.type           = ZDT_STEEP_MOTOR;
    motor->base.transport      = MOTOR_TRANSPORT_UART;
    motor->base.info           = config->motor_init_info;
    motor->base.setting        = config->setting_init_config;
    motor->base.setting.loop_type = OPEN_LOOP; /* 开环直通: 基类跳过控制计算 */
    motor->addr                = addr;
    motor->pulse_per_rev       = pulse_per_rev;
    motor->accel               = accel;
    motor->mode                = UART_MODE_SPEED;

    UART_Device *uart = BSP_UART_Device_Init(&config->transport_config.uart);
    if (!uart)
    {
        LOG_E("UART init failed");
        BSP_MEM_FREE(motor);
        return NULL;
    }
    motor->uart_dev           = uart;
    motor->base.transport_dev = uart;

    //motor->base.offline_dev = Module_Offline_register(&config->offline_init_config);

    /* 绑定调度,注册到全局链表 */
    motor->base.Apply = zdt_apply;
    Motor_Register(&motor->base);

    LOG_I("zdt stepper init (addr=%d, ppr=%d, accel=%d)", addr, pulse_per_rev, accel);
    return motor;
}

void Motor_ZDT_Start(UART_Motor_t *motor)
{
    motor->base.setting.enableflag = 1;
    motor->enable_state            = 0xFF; /* 强制刷新使能命令 */
}

void Motor_ZDT_Stop(UART_Motor_t *motor)
{
    motor->base.setting.enableflag = 0;
    /* 发送停止命令 */
    zdt_stop(motor);
    motor->enable_state = 0xFF;
}

void Motor_ZDT_SetMode(UART_Motor_t *motor, UART_Mode_e mode)
{
    motor->mode = mode;
}
