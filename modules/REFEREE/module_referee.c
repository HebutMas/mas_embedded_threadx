#include "module_referee.h"
#include "bsp_def.h"
#include "bsp_dwt.h"
#include "module_offline.h"
#include "crc_rm.h"
#include <string.h>
#include "bsp_uart.h"
#include "module_offline.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "usart.h"

#define LOG_TAG "module_referee"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 交互数据发送间隔：官方要求 ≤30Hz，这里限制到 10Hz（100ms = 100,000us） */
#define INTERACTION_SEND_INTERVAL_US 100000ULL

// 裁判系统模块结构体
typedef struct
{
    uint8_t         initialized;
    UART_Device    *uart_dev;
    Offline_Device *offline_dev;

    // 接收数据-- 根据需要添加和删除
    game_status_t      game_status;
    robot_hp_data_t    robot_hp;
    robot_status_t     robot_status;
    power_heat_data_t  power_heat;
    buff_t             buff;
    shoot_data_t       shoot_data;
    allowed_bullet_t   allowed_bullet;
    sentry_auto_sync_t sentry_sync;

    // 帧序号（发送用，协议帧头 seq 为 1 字节）
    uint8_t tx_seq;

    // 交互数据发送频率控制（10Hz，记录上次发送的 us 时间戳）
    uint64_t last_interaction_us;
} Module_Referee_s;

static Module_Referee_s           module_referee;
static TX_THREAD                  referee_thread;
APPS_STACK_SECTION static uint8_t referee_thread_stack[REFEREE_TASK_STACK_SIZE];
BUFFER_SECTION static uint8_t     rx_buffer[1024];
BUFFER_SECTION static uint8_t     tx_buffer[256];

static void referee_task_entry(ULONG arg)
{
    while (1)
    {
        if (!module_referee.initialized)
        {
            tx_thread_sleep(100);
            return;
        }
        static uint8_t rx_buf[512];
        uint32_t       rx_len = 0;

        BSP_UART_Read(module_referee.uart_dev, rx_buf, sizeof(rx_buf), &rx_len, TX_WAIT_FOREVER);
        if (rx_len == 0) return;

        uint16_t       idx     = 0;
        const uint16_t MIN_LEN = FRAME_HEADER_LEN + CMD_ID_LEN + FRAME_TAIL_LEN;

        while (idx < rx_len)
        {
            const uint8_t *p   = rx_buf + idx;
            uint16_t       rem = rx_len - idx;

            // 最短帧长检查，不足则退出
            if (rem < MIN_LEN) break;

            // SOF 对齐：非帧头则跳过该字节
            if (p[0] != REFEREE_SOF)
            {
                idx++;
                continue;
            }

            // 取 data_length，并做合法性检查
            uint16_t data_length = (uint16_t)(p[1] | (p[2] << 8));
            if (data_length > 118u)
            {
                idx++;
                continue;
            }

            // 帧完整性检查：数据不足则等待下次
            uint16_t frame_len = FRAME_HEADER_LEN + CMD_ID_LEN + data_length + FRAME_TAIL_LEN;
            if (rem < frame_len) break;

            // CRC8 校验帧头（前4字节）
            if (Get_CRC8_Check_Sum((uint8_t *)p, 4, 0xFF) != p[4])
            {
                idx++;
                continue;
            }

            // CRC16 校验整包
            uint16_t crc16_recv = (uint16_t)(p[frame_len - 2] | (p[frame_len - 1] << 8));
            if (Get_CRC16_Check_Sum((uint8_t *)p, frame_len - 2u, 0xFFFF) != crc16_recv)
            {
                idx++;
                continue;
            }

            // 解析命令码与数据
            uint16_t       cmd_id  = (uint16_t)(p[5] | (p[6] << 8));
            const uint8_t *payload = p + FRAME_HEADER_LEN + CMD_ID_LEN;

            switch (cmd_id)
            {
            case CMD_ID_GAME_STATUS:
                memcpy(&module_referee.game_status, payload, sizeof(game_status_t));
                break;
            case CMD_ID_ROBOT_HP:
                memcpy(&module_referee.robot_hp, payload, sizeof(robot_hp_data_t));
                break;
            case CMD_ID_ROBOT_STATUS:
                memcpy(&module_referee.robot_status, payload, sizeof(robot_status_t));
                break;
            case CMD_ID_POWER_HEAT_DATA:
                memcpy(&module_referee.power_heat, payload, sizeof(power_heat_data_t));
                break;
            case CMD_ID_BUFF_DATA:
                memcpy(&module_referee.buff, payload, sizeof(buff_t));
                break;
            case CMD_ID_SHOOT_DATA:
                memcpy(&module_referee.shoot_data, payload, sizeof(shoot_data_t));
                break;
            case CMD_ID_ALLOWED_BULLET:
                memcpy(&module_referee.allowed_bullet, payload, sizeof(allowed_bullet_t));
                break;
            case CMD_ID_SENTRY_AUTO_SYNC:
                memcpy(&module_referee.sentry_sync, payload, sizeof(sentry_auto_sync_t));
                break;
            default:
                break;
            }

            Module_Offline_device_update(module_referee.offline_dev);

            idx += frame_len;
        }
    }
}

int Module_Referee_Init(void)
{
    UART_Device_init_config config = {
        .huart = &REFEREE_UART, .expected_rx_len = 0, .rx_buf = rx_buffer, .rx_buf_size = 1024, .tx_mode = UART_MODE_DMA, .rx_mode = UART_MODE_DMA};
    module_referee.uart_dev = BSP_UART_Device_Init(&config);
    if (module_referee.uart_dev == NULL)
    {
        LOG_E("uart device init error");
        return -1;
    }
    Offline_Init_config_t offlineconfig = {.name = "referee", .beep_times = 10, .enable = REFEREE_OFFLINE_ENABLE, .timeout_ms = 100};
    module_referee.offline_dev          = Module_Offline_register(&offlineconfig);
    if (module_referee.offline_dev == NULL)
    {
        LOG_E("offline device register error");
        return -1;
    }

    /* 创建裁判系统数据接收线程 */
    UINT status = tx_thread_create(&referee_thread, "referee_thread", referee_task_entry, 0, referee_thread_stack, REFEREE_TASK_STACK_SIZE,
                                   REFEREE_TASK_PRIORITY, REFEREE_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("thread create error: %u", status);
        return -1;
    }

    module_referee.initialized = 1;

    LOG_I("module referee initialized");

    return 0;
}

/* 根据 robot_id 获取自身 client ID */
static uint16_t get_sender_client_id(void)
{
    uint8_t robot_id = module_referee.robot_status.robot_id;

    /* robot_id == 0 表示尚未收到裁判系统数据 */
    if (robot_id == 0) return 0;

    switch (robot_id)
    {
    case ROBOT_ID_RED_HERO:
        return CLIENT_ID_RED_HERO;
    case ROBOT_ID_RED_ENGINEER:
        return CLIENT_ID_RED_ENGINEER;
    case ROBOT_ID_RED_INFANTRY_3:
        return CLIENT_ID_RED_INFANTRY_3;
    case ROBOT_ID_RED_INFANTRY_4:
        return CLIENT_ID_RED_INFANTRY_4;
    case ROBOT_ID_RED_INFANTRY_5:
        return CLIENT_ID_RED_INFANTRY_5;
    case ROBOT_ID_RED_AIR:
        return CLIENT_ID_RED_AIR;
    case ROBOT_ID_BLUE_HERO:
        return CLIENT_ID_BLUE_HERO;
    case ROBOT_ID_BLUE_ENGINEER:
        return CLIENT_ID_BLUE_ENGINEER;
    case ROBOT_ID_BLUE_INFANTRY_3:
        return CLIENT_ID_BLUE_INFANTRY_3;
    case ROBOT_ID_BLUE_INFANTRY_4:
        return CLIENT_ID_BLUE_INFANTRY_4;
    case ROBOT_ID_BLUE_INFANTRY_5:
        return CLIENT_ID_BLUE_INFANTRY_5;
    case ROBOT_ID_BLUE_AIR:
        return CLIENT_ID_BLUE_AIR;
    default:
        return SERVER_ID; /* sentry/dart/radar/outpost/base 无 client ID */
    }
}

void Module_Referee_Send_Interaction(uint16_t sub_cmd_id, const void *payload, uint16_t payload_len, uint32_t timeout)
{
    if (module_referee.initialized == 0 || payload == NULL || payload_len == 0 || payload_len > 112u) return;

    /* 发送频率限制（10Hz）：未达到发送间隔则直接返回 */
    {
        uint64_t now_us  = BSP_DWT_GetTimeline_us();
        uint64_t elapsed = now_us - module_referee.last_interaction_us;
        if (elapsed < INTERACTION_SEND_INTERVAL_US) return;
        module_referee.last_interaction_us = now_us;
    }

    /* 自动确定 sender / recv */
    uint16_t sender_id = module_referee.robot_status.robot_id;
    if (sender_id == 0) return; /* 尚未收到 robot_id */
    uint16_t recv_id = get_sender_client_id();

    // 交互数据段长度 = 6字节头（sub_cmd_id+sender+recv）+ payload
    const uint16_t data_len = 6u + payload_len;
    uint16_t       offset   = 0;

    // 帧头
    tx_buffer[offset++] = REFEREE_SOF;
    tx_buffer[offset++] = (uint8_t)(data_len & 0xFF);
    tx_buffer[offset++] = (uint8_t)(data_len >> 8);
    tx_buffer[offset++] = module_referee.tx_seq++;
    tx_buffer[offset++] = Get_CRC8_Check_Sum(tx_buffer, 4, 0xFF); // CRC8

    // 命令码 0x0301
    tx_buffer[offset++] = (uint8_t)(CMD_ID_ROBOT_INTERACTION & 0xFF);
    tx_buffer[offset++] = (uint8_t)(CMD_ID_ROBOT_INTERACTION >> 8);

    // 交互帧头：sub_cmd_id + sender_id + recv_id
    tx_buffer[offset++] = (uint8_t)(sub_cmd_id & 0xFF);
    tx_buffer[offset++] = (uint8_t)(sub_cmd_id >> 8);
    tx_buffer[offset++] = (uint8_t)(sender_id & 0xFF);
    tx_buffer[offset++] = (uint8_t)(sender_id >> 8);
    tx_buffer[offset++] = (uint8_t)(recv_id & 0xFF);
    tx_buffer[offset++] = (uint8_t)(recv_id >> 8);

    // 用户数据
    memcpy(&tx_buffer[offset], payload, payload_len);
    offset += payload_len;

    // CRC16
    uint16_t crc16      = Get_CRC16_Check_Sum(tx_buffer, offset, 0xFFFF);
    tx_buffer[offset++] = (uint8_t)(crc16 & 0xFF);
    tx_buffer[offset++] = (uint8_t)(crc16 >> 8);

    BSP_UART_Send(module_referee.uart_dev, tx_buffer, offset, timeout);
}

uint8_t *Module_Referee_Get_cmd_data(uint16_t cmd_id)
{
    if (module_referee.initialized == 0)
    {
        return NULL;
    }
    switch (cmd_id)
    {
    case CMD_ID_GAME_STATUS:
        return (uint8_t *)&module_referee.game_status;
    case CMD_ID_ROBOT_HP:
        return (uint8_t *)&module_referee.robot_hp;
    case CMD_ID_ROBOT_STATUS:
        return (uint8_t *)&module_referee.robot_status;
    case CMD_ID_POWER_HEAT_DATA:
        return (uint8_t *)&module_referee.power_heat;
    case CMD_ID_BUFF_DATA:
        return (uint8_t *)&module_referee.buff;
    case CMD_ID_SHOOT_DATA:
        return (uint8_t *)&module_referee.shoot_data;
    case CMD_ID_ALLOWED_BULLET:
        return (uint8_t *)&module_referee.allowed_bullet;
    case CMD_ID_SENTRY_AUTO_SYNC:
        return (uint8_t *)&module_referee.sentry_sync;
    default:
        return NULL;
    }
}

uint8_t Module_Referee_Get_offline_state(void)
{
    if (module_referee.offline_dev == NULL)
    {
        return STATE_OFFLINE; // 如果离线设备未初始化，默认返回离线状态
    }
    return Module_Offline_get_device_status(module_referee.offline_dev);
}
