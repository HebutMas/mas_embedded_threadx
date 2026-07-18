#include "sbus.h"
#include "bsp_def.h"
#include "bsp_uart.h"
#include "usart.h"
#include "module_offline.h"
#include "module_remote.h"
#include <stdint.h>
#include <stdlib.h>

#define LOG_TAG "remote_sbus"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

#if defined(REMOTE_SOURCE) && (REMOTE_SOURCE == 1)

#define SBUS_FRAME_SIZE 25
#define SBUS_HEADER     0x0F
#define SBUS_TAIL       0x00

BUFFER_SECTION static uint8_t sbus_rx_buf[64];
static UART_Device           *sbus_uart;
static bool                   sbus_initialized;

int8_t remote_sbus_init(Offline_Device **out_offline)
{
    REMOTE_UART.Init.BaudRate = 100000;
    if (HAL_UART_Init(&REMOTE_UART) != HAL_OK)
    {
        LOG_E("uart init error");
        return -1;
    }

    Offline_Init_config_t offline_cfg = {
        .name       = "sbus",
        .timeout_ms = 50,
        .beep_times = 0,
        .enable     = REMOTE_OFFLINE_ENABLE,
    };
    Offline_Device *offline = Module_Offline_register(&offline_cfg);
    if (offline == NULL)
    {
        LOG_E("offline register error");
        return -1;
    }
    if (out_offline) *out_offline = offline;

    UART_Device_init_config uart_cfg = {
        .huart           = &REMOTE_UART,
        .expected_rx_len = SBUS_FRAME_SIZE,
        .rx_buf          = sbus_rx_buf,
        .rx_buf_size     = 64,
        .rx_mode         = UART_MODE_DMA,
        .tx_mode         = UART_MODE_BLOCKING,
    };
    sbus_uart = BSP_UART_Device_Init(&uart_cfg);
    if (sbus_uart == NULL)
    {
        LOG_E("uart device init error");
        return -1;
    }

    sbus_initialized = true;
    LOG_I("sbus init success");
    return 0;
}

void remote_sbus_decode(Remote_Data_t *data, Offline_Device *offline)
{
    if (!sbus_initialized || !data) return;

    static uint8_t buf[64];
    uint32_t       rx_len;
    if (BSP_UART_Read(sbus_uart, buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER) <= 0) return;

    for (uint32_t i = 0; i + SBUS_FRAME_SIZE - 1 < rx_len; i++)
    {
        /* 校验帧头 / 帧尾 / 长度 */
        if (buf[i] != SBUS_HEADER) continue;
        if (buf[i + SBUS_FRAME_SIZE - 1] != SBUS_TAIL) continue;
        /* 通道解码 (ch1-4 做零偏 + 死区) */
        int16_t ch1 = ((int16_t)buf[i + 1] >> 0 | ((int16_t)buf[i + 2] << 8)) & 0x07FF;
        int16_t ch2 = ((int16_t)buf[i + 2] >> 3 | ((int16_t)buf[i + 3] << 5)) & 0x07FF;
        int16_t ch3 = ((int16_t)buf[i + 3] >> 6 | ((int16_t)buf[i + 4] << 2) | (int16_t)buf[i + 5] << 10) & 0x07FF;
        int16_t ch4 = ((int16_t)buf[i + 5] >> 1 | ((int16_t)buf[i + 6] << 7)) & 0x07FF;

        if (ch1 < SBUS_CHX_UP || ch1 > SBUS_CHX_DOWN) ch1 = SBUS_CHX_BIAS;
        if (ch2 < SBUS_CHX_UP || ch2 > SBUS_CHX_DOWN) ch2 = SBUS_CHX_BIAS;
        if (ch3 < SBUS_CHX_UP || ch3 > SBUS_CHX_DOWN) ch3 = SBUS_CHX_BIAS;
        if (ch4 < SBUS_CHX_UP || ch4 > SBUS_CHX_DOWN) ch4 = SBUS_CHX_BIAS;

        ch1 -= SBUS_CHX_BIAS;
        ch2 -= SBUS_CHX_BIAS;
        ch3 -= SBUS_CHX_BIAS;
        ch4 -= SBUS_CHX_BIAS;

        if (abs(ch1) <= REMOTE_DEAD_ZONE) ch1 = 0;
        if (abs(ch2) <= REMOTE_DEAD_ZONE) ch2 = 0;
        if (abs(ch3) <= REMOTE_DEAD_ZONE) ch3 = 0;
        if (abs(ch4) <= REMOTE_DEAD_ZONE) ch4 = 0;

        data->channels[0] = ch1;
        data->channels[1] = ch2;
        data->channels[2] = ch3;
        data->channels[3] = ch4;
        /* 扩展通道 (ch5-16, 原始值) */
        data->channels[4]  = (int16_t)(((int16_t)buf[i + 6] >> 4 | ((int16_t)buf[i + 7] << 4)) & 0x07FF);
        data->channels[5]  = (int16_t)(((int16_t)buf[i + 7] >> 7 | ((int16_t)buf[i + 8] << 1) | (int16_t)buf[i + 9] << 9) & 0x07FF);
        data->channels[6]  = (int16_t)(((int16_t)buf[i + 9] >> 2 | ((int16_t)buf[i + 10] << 6)) & 0x07FF);
        data->channels[7]  = (int16_t)(((int16_t)buf[i + 10] >> 5 | ((int16_t)buf[i + 11] << 3)) & 0x07FF);
        data->channels[8]  = (int16_t)(((int16_t)buf[i + 12] << 0 | ((int16_t)buf[i + 13] << 8)) & 0x07FF);
        data->channels[9]  = (int16_t)(((int16_t)buf[i + 13] >> 3 | ((int16_t)buf[i + 14] << 5)) & 0x07FF);
        data->channels[10] = (int16_t)(((int16_t)buf[i + 14] >> 6 | ((int16_t)buf[i + 15] << 2) | (int16_t)buf[i + 16] << 10) & 0x07FF);
        data->channels[11] = (int16_t)(((int16_t)buf[i + 16] >> 1 | ((int16_t)buf[i + 17] << 7)) & 0x07FF);
        data->channels[12] = (int16_t)(((int16_t)buf[i + 17] >> 4 | ((int16_t)buf[i + 18] << 4)) & 0x07FF);
        data->channels[13] = (int16_t)(((int16_t)buf[i + 18] >> 7 | ((int16_t)buf[i + 19] << 1) | (int16_t)buf[i + 20] << 9) & 0x07FF);
        data->channels[14] = (int16_t)(((int16_t)buf[i + 20] >> 2 | ((int16_t)buf[i + 21] << 6)) & 0x07FF);
        data->channels[15] = (int16_t)(((int16_t)buf[i + 21] >> 5 | ((int16_t)buf[i + 22] << 3)) & 0x07FF);

        if (buf[i + 23] == 0x00)
        {
            Module_Offline_device_update(offline);
        }
    }
}

#else

int8_t remote_sbus_init(Offline_Device **out_offline)
{
    (void)out_offline;
    return -1;
}
void remote_sbus_decode(Remote_Data_t *data, Offline_Device *offline)
{
    (void)data;
    (void)offline;
}

#endif
