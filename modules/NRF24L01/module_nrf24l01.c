/**
 * @file    module_nrf24l01.c
 * @brief   nRF24L01 2.4GHz 无线模块实现
 *
 *  架构:
 *    - 底层: BSP SPI 驱动(硬件SPI2, 线程安全+互斥锁)
 *    - 寄存器层: nRF24L01 SPI指令封装(读/写寄存器, 读/写载荷, 清FIFO)
 *    - 协议层: Enhanced ShockBurst(自动ACK+重传) + 动态包长(DPL)
 *    - 框架层: 能力注册(名字+指针+类型), 自动序列化/反序列化
 *    - 线程层: 单线程处理定时发送 + IRQ唤醒接收
 *    - 系统集成: OFFLINE 心跳
 *
 *  收发流程:
 *    发送: 线程到点 → 遍历注册列表序列化 → 写TX FIFO → CE脉冲触发发送
 *          → 硬件自动ACK+重传 → TX_DS/MAX_RT中断
 *    接收: 模块常驻RX模式 → 收到数据 → IRQ低电平 → EXTI中断置信号量
 *          → 线程被唤醒 → 读STATUS → 读RX载荷 → 反序列化写回变量 → OFFLINE心跳
 */
#include "module_nrf24l01.h"
#include "nrf24l01_reg.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_def.h"
#include "module_offline.h"
#include "spi.h"
#include "gpio.h"
#include "tx_api.h"
#include <string.h>

#define LOG_TAG "NRF24L01"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* ================= 类型大小表 ================= */
static const uint8_t kTypeSize[NRF24L01_TYPE_COUNT] = {
    1, /* INT8   */
    1, /* UINT8  */
    2, /* INT16  */
    2, /* UINT16 */
    4, /* INT32  */
    4, /* UINT32 */
    4, /* FLOAT  */
    8, /* DOUBLE */
};

/* ================= 注册项(结构体成员从大到小排列) ================= */
typedef struct
{
    const char         *name;     /* 4B 名称指针 */
    void               *data_ptr; /* 4B 数据指针 */
    NRF24L01_DataType_e type;     /* 4B 枚举 */
    uint16_t            offset;   /* 2B 数据域偏移 */
    uint8_t             name_len; /* 1B 名称长度 */
    uint8_t             size;     /* 1B 数据字节数 */
} NRF_Cap_t;                      /* sizeof=16 */

/* ================= 全局上下文(成员从大到小) ================= */
typedef struct
{
    NRF_Cap_t       caps[NRF24L01_MAX_CAPS]; /* 注册列表 */
    SPI_Device     *spi_dev;                 /* SPI设备句柄 */
    Offline_Device *offline_dev;             /* OFFLINE设备句柄 */
    TX_THREAD       thread;                  /* 线程控制块 */
    TX_SEMAPHORE    irq_sem;                 /* IRQ信号量 */
    uint8_t         thread_stack[NRF24L01_TASK_STACK_SIZE];
    uint8_t         tx_payload[NRF24L01_PAYLOAD_MAX]; /* 发送缓冲区 */
    uint8_t         rx_payload[NRF24L01_PAYLOAD_MAX]; /* 接收缓冲区 */
    uint8_t         address[5];                       /* 通信地址(5字节) */
    uint32_t        last_tx_tick;                     /* 上次发送时间 */
    uint16_t        total_size;                       /* 注册数据总字节数 */
    uint8_t         cap_count;                        /* 已注册数量 */
    uint8_t         initialized;                      /* 初始化标志 */
} NRF_Ctx_t;

static NRF_Ctx_t g_nrf;

/* 通信地址(收发两端必须一致) */
static const uint8_t kDefaultAddress[5] = {0x11, 0x22, 0x33, 0x44, 0x55};

/* ================= 前向声明 ================= */
static void nrf_thread_entry(ULONG arg);
static void nrf_ce_high(void);
static void nrf_ce_low(void);
static void nrf_serialize(uint8_t *dst, const void *src, NRF24L01_DataType_e type);
static void nrf_deserialize(const uint8_t *src, void *dst, NRF24L01_DataType_e type);

/* ================= 底层: CE 控制 ================= */
static void nrf_ce_high(void) { HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_SET); }

static void nrf_ce_low(void) { HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET); }

/* ================= 寄存器层: SPI 指令封装 ================= */

/**
 * @brief 读寄存器(1字节)
 */
static uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t tx[2] = {NRF24L01_R_REGISTER | reg, NRF24L01_NOP};
    uint8_t rx[2] = {0};
    BSP_SPI_TransReceive(g_nrf.spi_dev, tx, rx, 2, 100);
    return rx[1];
}

/**
 * @brief 写寄存器(1字节)
 */
static void nrf_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {NRF24L01_W_REGISTER | reg, value};
    uint8_t rx[2];
    BSP_SPI_TransReceive(g_nrf.spi_dev, tx, rx, 2, 100);
}

/**
 * @brief 写寄存器并回读校验, 不一致则重试(最多3次), 保证关键寄存器可靠写入
 * @return 0=最终一致, -1=重试后仍不一致
 */
static int8_t nrf_write_reg_checked(uint8_t reg, uint8_t value)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        nrf_write_reg(reg, value);
        if (nrf_read_reg(reg) == value)
        {
            return 0;
        }
    }
    LOG_E("Reg 0x%02X write verify failed: want=0x%02X got=0x%02X", reg, value, nrf_read_reg(reg));
    return -1;
}

/**
 * @brief 写寄存器(多字节, 如地址)
 */
static void nrf_write_regs(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t tx[1 + NRF24L01_PAYLOAD_MAX];
    uint8_t rx[1 + NRF24L01_PAYLOAD_MAX];
    tx[0] = NRF24L01_W_REGISTER | reg;
    memcpy(tx + 1, buf, len);
    BSP_SPI_TransReceive(g_nrf.spi_dev, tx, rx, (uint16_t)(1 + len), 100);
}

/**
 * @brief 读 RX 有效载荷
 */
static uint8_t nrf_read_rx_payload(uint8_t *buf, uint8_t len)
{
    uint8_t tx[1 + NRF24L01_PAYLOAD_MAX];
    uint8_t rx[1 + NRF24L01_PAYLOAD_MAX];
    tx[0] = NRF24L01_R_RX_PAYLOAD;
    memset(tx + 1, NRF24L01_NOP, len);
    BSP_SPI_TransReceive(g_nrf.spi_dev, tx, rx, (uint16_t)(1 + len), 100);
    memcpy(buf, rx + 1, len);
    return len;
}

/**
 * @brief 写 TX 有效载荷
 */
static void nrf_write_tx_payload(const uint8_t *buf, uint8_t len)
{
    uint8_t tx[1 + NRF24L01_PAYLOAD_MAX];
    uint8_t rx[1 + NRF24L01_PAYLOAD_MAX];
    tx[0] = NRF24L01_W_TX_PAYLOAD;
    memcpy(tx + 1, buf, len);
    BSP_SPI_TransReceive(g_nrf.spi_dev, tx, rx, (uint16_t)(1 + len), 100);
}

/**
 * @brief 发送单字节指令(清FIFO等)
 */
static void nrf_send_cmd(uint8_t cmd)
{
    uint8_t tx = cmd;
    uint8_t rx;
    BSP_SPI_TransReceive(g_nrf.spi_dev, &tx, &rx, 1, 100);
}

/**
 * @brief 激活FEATURE寄存器(nRF24L01+必需, 否则DPL/ACK载荷等功能不可用)
 */
static void nrf_activate_feature(void)
{
    uint8_t tx[2] = {NRF24L01_ACTIVATE, NRF24L01_ACTIVATE_DATA};
    uint8_t rx[2];
    BSP_SPI_TransReceive(g_nrf.spi_dev, tx, rx, 2, 100);
}

/**
 * @brief 读状态寄存器
 */
static uint8_t nrf_read_status(void)
{
    uint8_t tx = NRF24L01_NOP;
    uint8_t rx;
    BSP_SPI_TransReceive(g_nrf.spi_dev, &tx, &rx, 1, 100);
    return rx;
}

/* ================= 模式切换 ================= */

static void nrf_set_rx_mode(void)
{
    nrf_ce_low();
    uint8_t cfg = nrf_read_reg(NRF24L01_CONFIG);
    /* 显式保证CRC(2字节)+上电+RX, 防止读改写过程中丢失CRCO位导致收发CRC长度不一致 */
    cfg |= NRF24L01_CONFIG_EN_CRC | NRF24L01_CONFIG_CRCO | NRF24L01_CONFIG_PWR_UP | NRF24L01_CONFIG_PRIM_RX;
    nrf_write_reg_checked(NRF24L01_CONFIG, cfg);
    nrf_ce_high();
}

/* ================= 序列化/反序列化(小端) ================= */

static void nrf_serialize(uint8_t *dst, const void *src, NRF24L01_DataType_e type)
{
    switch (type)
    {
    case NRF24L01_TYPE_INT8:
    case NRF24L01_TYPE_UINT8:
        dst[0] = *(const uint8_t *)src;
        break;
    case NRF24L01_TYPE_INT16:
    case NRF24L01_TYPE_UINT16:
    {
        uint16_t v = *(const uint16_t *)src;
        dst[0]     = (uint8_t)(v & 0xFF);
        dst[1]     = (uint8_t)((v >> 8) & 0xFF);
        break;
    }
    case NRF24L01_TYPE_INT32:
    case NRF24L01_TYPE_UINT32:
    case NRF24L01_TYPE_FLOAT:
    {
        uint32_t v;
        if (type == NRF24L01_TYPE_FLOAT)
        {
            float f = *(const float *)src;
            memcpy(&v, &f, 4);
        }
        else
        {
            v = *(const uint32_t *)src;
        }
        dst[0] = (uint8_t)(v & 0xFF);
        dst[1] = (uint8_t)((v >> 8) & 0xFF);
        dst[2] = (uint8_t)((v >> 16) & 0xFF);
        dst[3] = (uint8_t)((v >> 24) & 0xFF);
        break;
    }
    case NRF24L01_TYPE_DOUBLE:
    {
        uint64_t v;
        double   d = *(const double *)src;
        memcpy(&v, &d, 8);
        for (uint8_t i = 0; i < 8; i++) dst[i] = (uint8_t)((v >> (i * 8)) & 0xFF);
        break;
    }
    default:
        break;
    }
}

static void nrf_deserialize(const uint8_t *src, void *dst, NRF24L01_DataType_e type)
{
    switch (type)
    {
    case NRF24L01_TYPE_INT8:
    case NRF24L01_TYPE_UINT8:
        *(uint8_t *)dst = src[0];
        break;
    case NRF24L01_TYPE_INT16:
    case NRF24L01_TYPE_UINT16:
    {
        uint16_t v       = (uint16_t)src[0] | ((uint16_t)src[1] << 8);
        *(uint16_t *)dst = v;
        break;
    }
    case NRF24L01_TYPE_INT32:
    case NRF24L01_TYPE_UINT32:
    case NRF24L01_TYPE_FLOAT:
    {
        uint32_t v = (uint32_t)src[0] | ((uint32_t)src[1] << 8) | ((uint32_t)src[2] << 16) | ((uint32_t)src[3] << 24);
        if (type == NRF24L01_TYPE_FLOAT)
        {
            float f;
            memcpy(&f, &v, 4);
            *(float *)dst = f;
        }
        else
        {
            *(uint32_t *)dst = v;
        }
        break;
    }
    case NRF24L01_TYPE_DOUBLE:
    {
        uint64_t v = 0;
        for (uint8_t i = 0; i < 8; i++) v |= ((uint64_t)src[i]) << (i * 8);
        double d;
        memcpy(&d, &v, 8);
        *(double *)dst = d;
        break;
    }
    default:
        break;
    }
}

/* ================= 发送 ================= */

void Module_NRF24L01_TriggerTx(void)
{
    if (!g_nrf.initialized || g_nrf.cap_count == 0) return;

    /* 遍历注册列表, 序列化到发送缓冲区 */
    for (uint8_t i = 0; i < g_nrf.cap_count; i++)
    {
        NRF_Cap_t *c = &g_nrf.caps[i];
        nrf_serialize(&g_nrf.tx_payload[c->offset], c->data_ptr, c->type);
    }

    /* 正确发送时序: 先CE低退出RX模式 → 写TX FIFO → CE高脉冲触发 → 切回RX */

    /* 1. CE拉低, 退出RX模式进入Standby-I */
    nrf_ce_low();

    /* 2. 切换到TX模式(PWR_UP=1, PRIM_RX=0), 同时保证2字节CRC配置不丢 */
    uint8_t cfg = nrf_read_reg(NRF24L01_CONFIG);
    cfg |= NRF24L01_CONFIG_EN_CRC | NRF24L01_CONFIG_CRCO | NRF24L01_CONFIG_PWR_UP;
    cfg &= ~NRF24L01_CONFIG_PRIM_RX;
    nrf_write_reg(NRF24L01_CONFIG, cfg);

    /* 3. 清TX FIFO并写载荷(必须在Standby/TX模式下写, RX模式下不生效) */
    nrf_send_cmd(NRF24L01_FLUSH_TX);
    nrf_write_tx_payload(g_nrf.tx_payload, (uint8_t)g_nrf.total_size);

    /* 4. CE高脉冲>10us触发发送(2Mbps下32字节约200us, 用1ms保险) */
    nrf_ce_high();
    tx_thread_sleep(1);
    nrf_ce_low();

    /* 5. 检查发送结果: TX_DS=发送成功(收到ACK), MAX_RT=重传耗尽(对端没收到) */
    {
        uint8_t         tx_status = nrf_read_reg(NRF24L01_STATUS);
        static uint16_t s_tx_ok = 0, s_tx_fail = 0;
        if (tx_status & NRF24L01_STATUS_MAX_RT)
        {
            s_tx_fail++;
            /* 每50次失败打印一次, 避免刷屏 */
            if ((s_tx_fail % 50) == 1) LOG_W("TX failed(no ACK): ok=%u fail=%u, check receiver/channel/address", s_tx_ok, s_tx_fail);
            nrf_send_cmd(NRF24L01_FLUSH_TX);
        }
        else if (tx_status & NRF24L01_STATUS_TX_DS)
        {
            s_tx_ok++;
        }
        nrf_write_reg(NRF24L01_STATUS, NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);
    }

    /* 6. 切回RX模式常驻接收, 保证2字节CRC配置不丢 */
    cfg = nrf_read_reg(NRF24L01_CONFIG);
    cfg |= NRF24L01_CONFIG_EN_CRC | NRF24L01_CONFIG_CRCO | NRF24L01_CONFIG_PWR_UP | NRF24L01_CONFIG_PRIM_RX;
    nrf_write_reg(NRF24L01_CONFIG, cfg);
    nrf_ce_high();
}

/* ================= 接收处理(线程中调用) ================= */

static void nrf_handle_rx(void)
{
    /* 循环处理, 最多3轮, 防止TX_DS/RX_DR竞态导致丢中断(IRQ低电平+下降沿触发) */
    for (uint8_t round = 0; round < 3; round++)
    {
        uint8_t status  = nrf_read_status();
        uint8_t pending = status & (NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);
        if (!pending) break; /* 无中断标志, 退出 */

        if (status & NRF24L01_STATUS_RX_DR)
        {
            /* 读动态包长 */
            uint8_t plen = nrf_read_reg(NRF24L01_R_RX_PL_WID);
            if (plen > 0 && plen <= NRF24L01_PAYLOAD_MAX)
            {
                /* 读载荷 */
                nrf_read_rx_payload(g_nrf.rx_payload, plen);

                /* 长度匹配才解码 */
                if (plen == g_nrf.total_size)
                {
                    for (uint8_t i = 0; i < g_nrf.cap_count; i++)
                    {
                        NRF_Cap_t *c = &g_nrf.caps[i];
                        if ((uint16_t)(c->offset + c->size) <= plen)
                        {
                            nrf_deserialize(&g_nrf.rx_payload[c->offset], c->data_ptr, c->type);
                        }
                    }
                    /* OFFLINE 心跳更新 */
                    if (g_nrf.offline_dev)
                    {
                        Module_Offline_device_update(g_nrf.offline_dev);
                    }
                }
            }
            /* 清RX_DR中断标志并清空RX FIFO */
            nrf_write_reg(NRF24L01_STATUS, NRF24L01_STATUS_RX_DR);
            nrf_send_cmd(NRF24L01_FLUSH_RX);
        }

        if (status & NRF24L01_STATUS_MAX_RT)
        {
            /* 达到最大重传次数, 必须清TX FIFO, 否则后续发送卡死 */
            nrf_send_cmd(NRF24L01_FLUSH_TX);
        }

        /* 清TX_DS和MAX_RT标志 */
        if (status & (NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT))
        {
            nrf_write_reg(NRF24L01_STATUS, NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);
        }
    }
}

/* ================= 线程 ================= */

static void nrf_thread_entry(ULONG arg)
{
    (void)arg;
    g_nrf.last_tx_tick = tx_time_get();

    while (1)
    {
        /* 等待IRQ信号量, 超时=发送间隔; 无论是否超时都检查接收(轮询兜底, 不依赖IRQ) */
        tx_semaphore_get(&g_nrf.irq_sem, NRF24L01_TX_INTERVAL_MS);
        nrf_handle_rx();

#if NRF24L01_TX_ENABLE
        /* 到点发送(纯接收端不发送, 避免空中冲突) */
        ULONG now = tx_time_get();
        if ((now - g_nrf.last_tx_tick) >= NRF24L01_TX_INTERVAL_MS)
        {
            g_nrf.last_tx_tick = now;
            Module_NRF24L01_TriggerTx();
        }
#endif
    }
}

/* ================= EXTI 中断回调(通过BSP注册) ================= */

static void nrf_irq_callback(void)
{
    /* 信号量未创建前不响应, 防止初始化阶段IRQ毛刺导致异常 */
    if (!g_nrf.initialized) return;
    /* nRF24L01 IRQ低电平有效, 置信号量唤醒线程 */
    tx_semaphore_put(&g_nrf.irq_sem);
}

/* ================= 初始化 ================= */

int Module_NRF24L01_Init(void)
{
    if (g_nrf.initialized) return 0;

    memset(&g_nrf, 0, sizeof(g_nrf));
    memcpy(g_nrf.address, kDefaultAddress, 5);

    /* ---- 1. 配置SPI: 强制模式0(CPOL=0/CPHA=0, nRF24L01要求), 并按板设波特率(≤10MHz) ----
     * 分频 NRF24L01_SPI_PRESCALER 按芯片在 .h 选择: F103=36M/4=9M, F407=42M/8=5.25M */
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = NRF24L01_SPI_PRESCALER;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        LOG_E("SPI re-init failed");
        return -1;
    }

    /* ---- 2. CE 引脚初始化(推挽输出) ---- */
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
#if defined(STM32F407xx)
    __HAL_RCC_GPIOF_CLK_ENABLE(); /* dji_c: CE/IRQ 在 GPIOF (CubeMX 已使能, 这里再保证一次) */
#endif
    gpio.Pin   = NRF24L01_CE_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NRF24L01_CE_PORT, &gpio);
    nrf_ce_low();

    /* ---- 3. IRQ 引脚初始化(下降沿触发外部中断) ----
     * 注意: 对应板的 stm32fXxx_it.c 中需有 EXTIx_IRQHandler 调用 HAL_GPIO_EXTI_IRQHandler
     *       (CubeMX 中把 IRQ 脚配为 External Interrupt / Falling edge 即自动生成)
     *       NVIC使能放到信号量创建之后, 防止中断提前触发访问未初始化信号量
     */
    gpio.Pin  = NRF24L01_IRQ_PIN;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(NRF24L01_IRQ_PORT, &gpio);
    HAL_NVIC_SetPriority(NRF24L01_EXTI_IRQn, 5, 0); /* 中断线按芯片在 .h 选择 */

    /* 通过BSP注册EXTI回调(避免与其他模块的HAL_GPIO_EXTI_Callback冲突) */
    BSP_GPIO_EXTI_Register(NRF24L01_IRQ_PIN, nrf_irq_callback);

    /* ---- 4. BSP SPI 设备初始化(CSN由BSP管理) ---- */
    SPI_Device_Init_Config spi_cfg = {0};
    spi_cfg.hspi                   = &hspi2;
    spi_cfg.cs_port                = NRF24L01_CSN_PORT;
    spi_cfg.cs_pin                 = NRF24L01_CSN_PIN;
    spi_cfg.tx_mode                = SPI_MODE_BLOCKING;
    spi_cfg.rx_mode                = SPI_MODE_BLOCKING;
    g_nrf.spi_dev                  = BSP_SPI_Device_Init(&spi_cfg);
    if (g_nrf.spi_dev == NULL)
    {
        LOG_E("BSP SPI device init failed");
        return -2;
    }

    /* ---- 5. nRF24L01 寄存器配置 ---- */
    tx_thread_sleep(100); /* 等待模块上电稳定(系统tick=1ms) */

    nrf_write_reg_checked(NRF24L01_CONFIG, NRF24L01_CONFIG_EN_CRC |     /* 使能CRC */
                                               NRF24L01_CONFIG_CRCO |   /* CRC=2字节(收发两端必须一致) */
                                               NRF24L01_CONFIG_PWR_UP); /* 上电(从掉电到待机需1.5ms, 提前稳定) */

    nrf_write_reg_checked(NRF24L01_EN_AA, 0x01);                     /* 通道0自动应答 */
    nrf_write_reg_checked(NRF24L01_EN_RXADDR, 0x01);                 /* 使能通道0 */
    nrf_write_reg_checked(NRF24L01_SETUP_AW, NRF24L01_ADDR_WIDTH_5); /* 5字节地址 */

    /* 自动重传: 间隔250us*(DELAY+1), 次数COUNT */
    uint8_t retr = (NRF24L01_RETR_DELAY << 4) | (NRF24L01_RETR_COUNT & 0x0F);
    nrf_write_reg_checked(NRF24L01_SETUP_RETR, retr);

    nrf_write_reg_checked(NRF24L01_RF_CH, NRF24L01_RF_CHANNEL);

    /* 射频设置: 速率+功率 */
    uint8_t rf_setup = 0;
    if (NRF24L01_RF_DATARATE == 2) rf_setup |= (1 << 3); /* 2Mbps */
    /* 1Mbps时RF_DR位=0 */
    rf_setup |= ((NRF24L01_RF_POWER & 0x03) << 1); /* 发射功率 */
    nrf_write_reg_checked(NRF24L01_RF_SETUP, rf_setup);

    /* 动态包长(DPL)配置 — 必须先ACTIVATE才能写FEATURE寄存器 */
    nrf_activate_feature();
    nrf_write_reg_checked(NRF24L01_FEATURE, NRF24L01_FEATURE_EN_DPL |         /* 使能动态包长 */
                                                NRF24L01_FEATURE_EN_ACK_PAY); /* 使能应答载荷 */
    nrf_write_reg_checked(NRF24L01_DYNPD, 0x01);                              /* 通道0动态包长 */

    /* 地址配置(收发两端必须一致) */
    nrf_write_regs(NRF24L01_TX_ADDR, g_nrf.address, 5);
    nrf_write_regs(NRF24L01_RX_ADDR_P0, g_nrf.address, 5);

    /* 清FIFO和中断标志 */
    nrf_send_cmd(NRF24L01_FLUSH_TX);
    nrf_send_cmd(NRF24L01_FLUSH_RX);
    nrf_write_reg(NRF24L01_STATUS, NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);

    /* 进入RX模式(常驻接收) */
    nrf_set_rx_mode();

    /* ---- 5.5 寄存器回读自检, 确认SPI通信和配置真正生效 ---- */
    {
        uint8_t rd_cfg     = nrf_read_reg(NRF24L01_CONFIG);
        uint8_t rd_enaa    = nrf_read_reg(NRF24L01_EN_AA);
        uint8_t rd_aw      = nrf_read_reg(NRF24L01_SETUP_AW);
        uint8_t rd_rfch    = nrf_read_reg(NRF24L01_RF_CH);
        uint8_t rd_feature = nrf_read_reg(NRF24L01_FEATURE);
        uint8_t rd_dynpd   = nrf_read_reg(NRF24L01_DYNPD);
        LOG_I("Reg check: CONFIG=0x%02X EN_AA=0x%02X AW=0x%02X RF_CH=%d FEATURE=0x%02X DYNPD=0x%02X", rd_cfg, rd_enaa, rd_aw, rd_rfch, rd_feature,
              rd_dynpd);
        /* FEATURE 必须是 0x06(EN_DPL|EN_ACK_PAY), 为0说明ACTIVATE失败 */
        if (rd_feature == 0x00)
        {
            LOG_E("FEATURE=0, ACTIVATE failed! DPL not enabled. Check SPI wiring.");
        }
    }

    /* ---- 6. OFFLINE 集成 ---- */
    Offline_Init_config_t offline_cfg = {0};
    offline_cfg.name                  = "nrf24l01";
    offline_cfg.timeout_ms            = NRF24L01_OFFLINE_TIMEOUT_MS;
    offline_cfg.beep_times            = 3;
    offline_cfg.enable                = 1;
    g_nrf.offline_dev                 = Module_Offline_register(&offline_cfg);

    /* ---- 7. 创建IRQ信号量和线程 ---- */
    tx_semaphore_create(&g_nrf.irq_sem, "nrf_irq", 0);

    /* 信号量就绪后再使能EXTI中断, 防止中断提前触发访问未初始化信号量 */
    HAL_NVIC_EnableIRQ(NRF24L01_EXTI_IRQn);

    UINT ret = tx_thread_create(&g_nrf.thread, "nrf24l01", nrf_thread_entry, 0, g_nrf.thread_stack, NRF24L01_TASK_STACK_SIZE, NRF24L01_TASK_PRIORITY,
                                NRF24L01_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (ret != TX_SUCCESS)
    {
        LOG_E("Thread create failed: %d", ret);
        return -3;
    }

    g_nrf.initialized = 1;
    LOG_I("NRF24L01 initialized: ch=%d rate=%dMbps power=%ddBm", NRF24L01_RF_CHANNEL, NRF24L01_RF_DATARATE,
          NRF24L01_RF_POWER == 0 ? 0 : -6 * NRF24L01_RF_POWER);
    return 0;
}

/* ================= 注册 ================= */

int8_t Module_NRF24L01_Register(const char *name, void *data_ptr, NRF24L01_DataType_e type)
{
    if (data_ptr == NULL || type >= NRF24L01_TYPE_COUNT) return -1;
    if (g_nrf.cap_count >= NRF24L01_MAX_CAPS) return -1;

    uint8_t sz = kTypeSize[type];
    if ((uint16_t)(g_nrf.total_size + sz) > NRF24L01_PAYLOAD_MAX)
    {
        LOG_W("Register '%s' failed: total %d > %d bytes", name ? name : "?", g_nrf.total_size + sz, NRF24L01_PAYLOAD_MAX);
        return -1;
    }

    NRF_Cap_t *c = &g_nrf.caps[g_nrf.cap_count];
    c->name      = name;
    c->name_len  = name ? (uint8_t)strlen(name) : 0;
    c->data_ptr  = data_ptr;
    c->type      = type;
    c->size      = sz;
    c->offset    = g_nrf.total_size;

    g_nrf.total_size += sz;
    LOG_I("Registered '%s' type=%d size=%d offset=%d (total=%d)", name ? name : "?", type, sz, c->offset, g_nrf.total_size);
    return (int8_t)g_nrf.cap_count++;
}

/* ================= 状态查询 ================= */

uint8_t Module_NRF24L01_GetStatus(void)
{
    if (g_nrf.offline_dev) return Module_Offline_get_device_status(g_nrf.offline_dev);
    return 1; /* 无OFFLINE设备时默认离线 */
}
