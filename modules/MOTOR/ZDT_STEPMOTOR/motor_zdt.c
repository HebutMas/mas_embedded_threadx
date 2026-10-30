#include "motor_zdt.h"
#include "bsp_def.h"
#include <string.h>

#define LOG_TAG "motor_zdt"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 命令字节 */
#define CMD_ENABLE            0xF3
#define CMD_SPEED             0xF6
#define CMD_POSITION          0xFD
#define CMD_STOP              0xFE
#define CMD_CLR_POS           0x0A
#define CHECKSUM              0x6B

#define ZDT_DEFAULT_SPEED_RPM 300

/* 统一发送：UART 加地址一帧直发；CAN 去地址后按 8 字节拆包，每包首字节重复功能码，校验随末包 */
static void zdt_send(ZDT_Motor_t *m, const uint8_t *payload, uint8_t len)
{
    if (m->base.transport == MOTOR_TRANSPORT_CAN)
    {
        uint8_t buf[8];
        uint8_t off = 1; /* payload[0] 为功能码，每包重复 */
        uint8_t pkt = 0;

        do
        {
            uint8_t n = 0;
            while (n < 7u && (uint8_t)(off + n) < len) n++;

            buf[0] = payload[0];
            memcpy(buf + 1, payload + off, n);
            if (pkt < ZDT_CAN_MAX_PKT && m->can_dev[pkt] != NULL) BSP_CAN_Send(m->can_dev[pkt], buf, (uint8_t)(n + 1u));

            off = (uint8_t)(off + n);
            pkt++;
        } while (off < len);
    }
    else
    {
        uint8_t buf[16];
        if (m->uart_dev == NULL || (uint16_t)len + 1u > sizeof(buf)) return;

        buf[0] = m->addr;
        memcpy(buf + 1, payload, len);
        BSP_UART_Send(m->uart_dev, buf, (uint16_t)(len + 1), TX_WAIT_FOREVER);
    }
}

static void zdt_enable(ZDT_Motor_t *m, uint8_t state)
{
    const uint8_t payload[] = {CMD_ENABLE, 0xAB, state, 0x00, CHECKSUM};
    zdt_send(m, payload, (uint8_t)sizeof(payload));
}

static void zdt_stop(ZDT_Motor_t *m)
{
    const uint8_t payload[] = {CMD_STOP, 0x98, 0x00, CHECKSUM};
    zdt_send(m, payload, (uint8_t)sizeof(payload));
}

static void zdt_speed_cmd(ZDT_Motor_t *m, float rpm)
{
    uint8_t        dir       = (rpm >= 0) ? 0x00 : 0x01;
    const uint16_t spd       = (uint16_t)((rpm >= 0) ? rpm : -rpm);
    const uint8_t  payload[] = {CMD_SPEED, dir, (uint8_t)(spd >> 8), (uint8_t)(spd & 0xFF), m->accel, 0x00, CHECKSUM};
    zdt_send(m, payload, (uint8_t)sizeof(payload));
}

static void zdt_position_cmd(ZDT_Motor_t *m, int32_t pulses, uint16_t speed_rpm, uint8_t absolute, uint8_t sync)
{
    const uint8_t  dir   = (pulses >= 0) ? 0x00 : 0x01;
    const uint32_t abs_p = (uint32_t)((pulses >= 0) ? pulses : -pulses);
    /* [9] 相对/绝对: 0=相对上一目标位置, 1=绝对位置
     * [10] 多机同步: 0=立即执行, 1=挂起等待 0xFF 0x66 触发 */
    const uint8_t payload[] = {CMD_POSITION,
                               dir,
                               (uint8_t)(speed_rpm >> 8),
                               (uint8_t)(speed_rpm & 0xFF),
                               m->accel,
                               (uint8_t)(abs_p >> 24),
                               (uint8_t)(abs_p >> 16),
                               (uint8_t)(abs_p >> 8),
                               (uint8_t)(abs_p & 0xFF),
                               absolute ? 0x01 : 0x00,
                               sync ? 0x01 : 0x00,
                               CHECKSUM};
    zdt_send(m, payload, (uint8_t)sizeof(payload));
}

static void zdt_apply(Motor_Base *base)
{
    ZDT_Motor_t *m = MOTOR_GET_DERIVED(base, ZDT_Motor_t);

    if (!base->setting.enableflag)
    {
        if (m->enable_state != 0)
        {
            zdt_enable(m, 0);
            m->enable_state = 0;
        }
        return;
    }

    if (m->enable_state != 1)
    {
        zdt_enable(m, 1);
        m->enable_state = 1;
    }

    if (m->mode == ZDT_MODE_POSITION)
    {
        if (m->pending_pulses != 0)
        {
            zdt_position_cmd(m, m->pending_pulses, m->pending_speed_rpm, m->pending_absolute, m->pending_sync);
            m->pending_pulses = 0;
        }
    }
    else
    {
        zdt_speed_cmd(m, base->controller.ref);
    }
}

/* 对外函数  */

ZDT_Motor_t *Motor_ZDT_Init(Motor_Init_Config_s *config, uint8_t addr, uint16_t pulse_per_rev, uint8_t accel)
{
    if (config == NULL) return NULL;

    ZDT_Motor_t *motor = NULL;
    BSP_MEM_ALLOC_WAIT(motor, sizeof(ZDT_Motor_t), TX_NO_WAIT);
    if (motor == NULL)
    {
        LOG_E("alloc failed");
        return NULL;
    }
    memset(motor, 0, sizeof(ZDT_Motor_t));

    motor->base.type              = ZDT_STEEP_MOTOR;
    motor->base.transport         = config->transport;
    motor->base.info              = config->motor_init_info;
    motor->base.setting           = config->setting_init_config;
    motor->base.setting.loop_type = OPEN_LOOP; /* 开环直通: 基类跳过控制计算 */
    motor->base.offline_dev       = NULL;      /* 步进默认在线, 不做离线检测 */

    motor->addr          = addr;
    motor->pulse_per_rev = pulse_per_rev;
    motor->accel         = accel;
    motor->mode          = ZDT_MODE_SPEED;

    if (config->transport == MOTOR_TRANSPORT_CAN)
    {
        /* 每个包号一个设备：tx_id = rx_id = 扩展帧 (addr<<8)|包号 */
        for (uint8_t i = 0; i < ZDT_CAN_MAX_PKT; i++)
        {
            Can_Device_Init_Config_s cfg = {
                .hcan        = config->transport_config.can.hcan,
                .tx_id       = BSP_CAN_EXT_ID(((uint32_t)addr << 8) | (uint32_t)i),
                .rx_id       = BSP_CAN_EXT_ID(((uint32_t)addr << 8) | (uint32_t)i),
                .rx_callback = NULL,
                .user_arg    = motor,
            };
            motor->can_dev[i] = BSP_CAN_Device_Init(&cfg);
            if (motor->can_dev[i] == NULL)
            {
                LOG_E("CAN device init failed (pkt %u)", i);
                BSP_MEM_FREE(motor);
                return NULL;
            }
        }
        motor->base.transport_dev = motor->can_dev[0];
    }
    else
    {
        UART_Device *uart = BSP_UART_Device_Init(&config->transport_config.uart);
        if (uart == NULL)
        {
            LOG_E("UART init failed");
            BSP_MEM_FREE(motor);
            return NULL;
        }
        motor->uart_dev           = uart;
        motor->base.transport_dev = uart;
    }

    motor->base.Apply = zdt_apply;
    Motor_Register(&motor->base);

    LOG_I("zdt init (transport=%d, addr=%u, ppr=%u, accel=%u)", (int)config->transport, addr, pulse_per_rev, accel);
    return motor;
}

void Motor_ZDT_Start(ZDT_Motor_t *motor)
{
    if (motor == NULL) return;
    motor->base.setting.enableflag = 1;
    motor->enable_state            = 0xFF; /* 强制下一周期刷新使能命令 */
}

void Motor_ZDT_Stop(ZDT_Motor_t *motor)
{
    if (motor == NULL) return;
    motor->base.setting.enableflag = 0;
    zdt_stop(motor);
    motor->enable_state = 0xFF;
}

void Motor_ZDT_SetMode(ZDT_Motor_t *motor, ZDT_Mode_e mode)
{
    if (motor == NULL) return;
    motor->mode = mode;
}

void Motor_ZDT_SendPosition(ZDT_Motor_t *motor, int32_t pulses, uint16_t speed_rpm, uint8_t absolute, uint8_t sync)
{
    if (motor == NULL || pulses == 0) return;
    motor->pending_pulses    = pulses;
    motor->pending_speed_rpm = (speed_rpm == 0) ? ZDT_DEFAULT_SPEED_RPM : speed_rpm;
    motor->pending_absolute  = absolute ? 1u : 0u;
    motor->pending_sync      = sync ? 1u : 0u;
}

void Motor_ZDT_ClearPosition(ZDT_Motor_t *motor)
{
    if (motor == NULL) return;
    const uint8_t payload[] = {CMD_CLR_POS, 0x6D, CHECKSUM}; /* 0A 6D 6B */
    zdt_send(motor, payload, (uint8_t)sizeof(payload));
}

void Motor_ZDT_SyncTrigger(ZDT_Motor_t *motor)
{
    if (motor == NULL) return;
    const uint8_t payload[] = {0xFF, 0x66, CHECKSUM}; /* FF 66 6B */

    if (motor->base.transport == MOTOR_TRANSPORT_CAN)
    {
        if (motor->can_dev[0] == NULL) return;
        CAN_Bus_Manager *bus = (CAN_Bus_Manager *)motor->can_dev[0]->_bus;
        if (bus == NULL) return;
        BSP_CanMsg_t msg = {.hcan = bus->hcan, .id = BSP_CAN_EXT_ID(0), .len = (uint8_t)sizeof(payload)};
        memcpy(msg.data, payload, sizeof(payload));
        BSP_CAN_SendMessage(&msg); /* 广播地址 0 */
    }
    else if (motor->uart_dev != NULL)
    {
        uint8_t buf[4] = {0x00, payload[0], payload[1], payload[2]}; /* 00 FF 66 6B */
        BSP_UART_Send(motor->uart_dev, buf, 4, TX_WAIT_FOREVER);
    }
}


