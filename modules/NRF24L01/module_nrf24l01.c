/**
 * @file    module_nrf24l01.c
 * @brief   nRF24L01 2.4GHz 无线模块实现
 *
 *  架构:
 *    - 底层: BSP SPI 驱动(硬件SPI2, DMA 模式, 线程安全+互斥锁)
 *    - 寄存器层: nRF24L01 SPI指令封装(读/写寄存器, 读/写载荷, 清FIFO)
 *    - 协议层: Enhanced ShockBurst(自动ACK+重传) + 动态包长(DPL)
 *    - 框架层: 能力注册(指针+尺寸), 收发按 size 逐字节按位拷贝(小端)
 *    - 线程层: 发送/接收各一个线程, 由 NRF24L01_TX_ENABLE / NRF24L01_RX_ENABLE 选择注册
 *    - 系统集成: OFFLINE 心跳
 *
 *  收发流程:
 *    发送: 线程到点 → 按注册尺寸拷入发送缓冲区 → 写TX FIFO → CE脉冲触发发送
 *          → 硬件自动ACK+重传 → TX_DS/MAX_RT中断
 *    接收: 模块常驻RX模式 → 收到数据 → IRQ低电平 → EXTI中断置信号量
 *          → 线程被唤醒 → 读STATUS → 读RX载荷 → 按尺寸写回变量 → OFFLINE心跳
 */
#include "module_nrf24l01.h"
#include "nrf24l01_reg.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_def.h" /* APPS_STACK_SECTION */
#include "module_offline.h"
#include "spi.h"
#include "tx_api.h"
#include <string.h>

#define LOG_TAG "NRF24L01"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 收发角色: 必须且只能选一个 —— nRF24L01 半双工, 同板同时收发没有意义; 不选角色则模块什么都不做 */
#if NRF24L01_TX_ENABLE && NRF24L01_RX_ENABLE
#error "NRF24L01: NRF24L01_TX_ENABLE and NRF24L01_RX_ENABLE are mutually exclusive, pick exactly one role"
#elif !NRF24L01_TX_ENABLE && !NRF24L01_RX_ENABLE
#error "NRF24L01: no role selected, set NRF24L01_TX_ENABLE or NRF24L01_RX_ENABLE to 1"
#endif

/* BSP_SPI_TransReceive 超时(tick, 1 tick = 1ms) */
#define NRF24L01_SPI_TIMEOUT 100

/* 等发送完成(TX_DS/MAX_RT)的最长轮询时间(ms); 正常约0.2ms, 重传耗尽约1.5ms */
#define NRF24L01_TX_WAIT_MS  3

/* 接收线程等 IRQ 的兜底超时(ms): 正常由 IRQ 唤醒, 只用于防漏边沿 */
#define NRF24L01_RX_POLL_MS  50

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

/* ================= 注册项 ================= */
typedef struct
{
    void    *data_ptr; /* 数据指针 */
    uint16_t offset;   /* 数据域偏移 */
    uint8_t  size;     /* 数据字节数 */
} NRF_Cap_t;

/* ================= 全局上下文(不含线程/栈/缓冲) ================= */
typedef struct
{
    NRF_Cap_t       caps[NRF24L01_MAX_CAPS]; /* 注册列表 */
    SPI_Device     *spi_dev;                 /* SPI设备句柄 */
    Offline_Device *offline_dev;             /* OFFLINE设备句柄 */
    uint16_t        total_size;              /* 注册数据总字节数 */
    uint8_t         cap_count;               /* 已注册数量 */
    uint8_t         initialized;             /* 初始化标志 */
} NRF_Ctx_t;

static NRF_Ctx_t g_nrf;

/* SPI 帧缓冲: [0]=命令, [1..]=载荷。
 * DMA 下 TX/RX 是两条独立通道并发跑, 不能再像阻塞模式那样共用一块(会互相覆盖);
 * BUFFER_SECTION 保证 DMA 可访问 + 32 字节对齐(与 WT606/REFEREE 同做法) */
BUFFER_SECTION static uint8_t g_nrf_tx[1 + NRF24L01_PAYLOAD_MAX];
BUFFER_SECTION static uint8_t g_nrf_rx[1 + NRF24L01_PAYLOAD_MAX];

/* 模块内部变量: 线程/栈/信号量按其它模块(INS/REMOTE/WT606)的写法放在文件作用域
 * (栈需要 APPS_STACK_SECTION, 不能放进结构体) */
static TX_THREAD                  g_nrf_thread;
APPS_STACK_SECTION static uint8_t g_nrf_stack[NRF24L01_TASK_STACK_SIZE];
#if NRF24L01_RX_ENABLE
static TX_SEMAPHORE g_nrf_irq_sem; /* IRQ → 接收线程 */
#endif

/* 通信地址(收发两端必须一致; 可在 robot.cmake 里用 NRF24L01_ADDR 覆盖) */
static const uint8_t kAddress[5] = {NRF24L01_ADDR};

/* ================= 底层: CE 控制(宏实现, 省去简单函数的调用/栈开销) ================= */
#define nrf_ce_high() HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_SET)
#define nrf_ce_low()  HAL_GPIO_WritePin(NRF24L01_CE_PORT, NRF24L01_CE_PIN, GPIO_PIN_RESET)

/* ================= 寄存器层: SPI 指令封装 ================= */

/* 一次传输: 发 g_nrf_tx, 收 g_nrf_rx; 宏实现, 不再多一层函数调用 */
#define nrf_spi(len)  BSP_SPI_TransReceive(g_nrf.spi_dev, g_nrf_tx, g_nrf_rx, (uint16_t)(len), NRF24L01_SPI_TIMEOUT)

/* 发命令帧: [0]=命令, [1..]=datalen 字节数据(数据由调用方先填好) */
static inline void nrf_cmd(uint8_t cmd, uint8_t datalen)
{
    g_nrf_tx[0] = cmd;
    nrf_spi(1 + datalen);
}

/**
 * @brief 读寄存器(1字节)
 */
static inline uint8_t nrf_read_reg(uint8_t reg)
{
    g_nrf_tx[1] = NRF24L01_NOP; /* 时钟信号, 内容不关心 */
    nrf_cmd(NRF24L01_R_REGISTER | reg, 1);
    return g_nrf_rx[1];
}

/**
 * @brief 写寄存器(1字节)
 */
static inline void nrf_write_reg(uint8_t reg, uint8_t value)
{
    g_nrf_tx[1] = value;
    nrf_cmd(NRF24L01_W_REGISTER | reg, 1);
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
static inline void nrf_write_regs(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    memcpy(&g_nrf_tx[1], buf, len);
    nrf_cmd(NRF24L01_W_REGISTER | reg, len);
}

#if NRF24L01_RX_ENABLE
/**
 * @brief 读 RX 有效载荷到 g_nrf_rx[1..]
 */
static inline void nrf_read_rx_payload(uint8_t len)
{
    memset(&g_nrf_tx[1], NRF24L01_NOP, len); /* 纯占位, 只为产生时钟 */
    nrf_cmd(NRF24L01_R_RX_PAYLOAD, len);
}
#endif /* NRF24L01_RX_ENABLE */

#if NRF24L01_TX_ENABLE
/**
 * @brief 发送 g_nrf_tx[1..] 中的TX载荷(调用方已组包)
 */
static inline void nrf_write_tx_payload(uint8_t len) { nrf_cmd(NRF24L01_W_TX_PAYLOAD, len); }
#endif /* NRF24L01_TX_ENABLE */

/**
 * @brief 发送单字节命令(清FIFO等)
 */
static inline void nrf_send_cmd(uint8_t cmd) { nrf_cmd(cmd, 0); }

/**
 * @brief 激活FEATURE寄存器(nRF24L01+必需, 否则DPL/ACK载荷等功能不可用)
 */
static inline void nrf_activate_feature(void)
{
    g_nrf_tx[1] = NRF24L01_ACTIVATE_DATA;
    nrf_cmd(NRF24L01_ACTIVATE, 1);
}

/* 以下状态/模式操作只有收发线程会用到(两个角色都关时不必编译) */
#if NRF24L01_TX_ENABLE || NRF24L01_RX_ENABLE

/**
 * @brief 读状态寄存器
 */
static inline uint8_t nrf_read_status(void)
{
    nrf_cmd(NRF24L01_NOP, 0);
    return g_nrf_rx[0]; /* 发命令时同步返回的状态字节 */
}

/* ================= 模式切换 ================= */

/**
 * @brief 读改写 CONFIG: 置位 set / 清零 clr, 并始终保证 CRC(2字节)+上电
 * @note  显式置 CRCO 防止读改写过程中丢失 CRC 宽度位, 导致收发两端 CRC 不一致
 */
static void nrf_config_update(uint8_t set, uint8_t clr)
{
    uint8_t cfg = nrf_read_reg(NRF24L01_CONFIG);
    cfg |= (uint8_t)(NRF24L01_CONFIG_EN_CRC | NRF24L01_CONFIG_CRCO | NRF24L01_CONFIG_PWR_UP | set);
    cfg &= (uint8_t)~clr;
    nrf_write_reg_checked(NRF24L01_CONFIG, cfg);
}

#if NRF24L01_RX_ENABLE
/* 进入RX模式常驻接收: CE低 → 置PRIM_RX → CE高 */
static void nrf_set_rx_mode(void)
{
    nrf_ce_low();
    nrf_config_update(NRF24L01_CONFIG_PRIM_RX, 0);
    nrf_ce_high();
}
#endif /* NRF24L01_RX_ENABLE */

#endif /* NRF24L01_TX_ENABLE || NRF24L01_RX_ENABLE */

/* ================= 发送(仅 NRF24L01_TX_ENABLE=1 时编译) ================= */
#if NRF24L01_TX_ENABLE

/* 按注册表把各变量"当前值"拷入 g_nrf_tx[1..](小端) */
static void nrf_gather_tx(void)
{
    for (uint8_t i = 0; i < g_nrf.cap_count; i++)
    {
        NRF_Cap_t *c = &g_nrf.caps[i];
        memcpy(&g_nrf_tx[1 + c->offset], c->data_ptr, c->size);
    }
}

/* 进入TX模式: CE低退出RX, 清PRIM_RX */
static void nrf_enter_tx(void)
{
    nrf_ce_low();
    nrf_config_update(0, NRF24L01_CONFIG_PRIM_RX);
}

/* 等发送结束: 轮询 STATUS 直到 TX_DS/MAX_RT, 返回最后读到的 STATUS
 * @note  超时也返回"最后读到的状态"而不是硬编 MAX_RT —— 线程被抢占时标志可能已置位,
 *        硬编会把成功报成失败(低优先级下"通一会断"的根因) */
static uint8_t nrf_wait_tx_done(void)
{
    ULONG start = tx_time_get();

    for (;;)
    {
        uint8_t status = nrf_read_status();
        if (status & (NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT)) return status;
        if ((tx_time_get() - start) >= NRF24L01_TX_WAIT_MS) return status;
    }
}

/* 发送结果统计: 只有收到 TX_DS(ACK) 才算成功; MAX_RT 与超时都算失败 */
static void nrf_check_tx_result(uint8_t tx_status)
{
    static uint16_t s_tx_ok = 0, s_tx_fail = 0;

    if (tx_status & NRF24L01_STATUS_TX_DS)
    {
        s_tx_ok++;
        /* 收到 ACK 说明链路通 → 喂心跳, 让"只发不收"的板也能显示在线 */
        if (g_nrf.offline_dev) Module_Offline_device_update(g_nrf.offline_dev);
    }
    else
    {
        s_tx_fail++;
        /* 每50次失败打印一次, 避免刷屏 */
        if ((s_tx_fail % 50) == 1) LOG_W("TX failed(no ACK): ok=%u fail=%u, check receiver/channel/address", s_tx_ok, s_tx_fail);
        nrf_send_cmd(NRF24L01_FLUSH_TX);
    }
    nrf_write_reg(NRF24L01_STATUS, NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);
}

/* 组包+发送一次(仅线程内部调用) */
static void nrf_trigger_tx(void)
{
    if (!g_nrf.initialized || g_nrf.cap_count == 0) return;

    nrf_gather_tx(); /* 1. 读变量当前值 → g_nrf_tx[1..] */

    nrf_enter_tx(); /* 2. 切TX模式(CE低退出RX) */
    nrf_send_cmd(NRF24L01_FLUSH_TX);
    nrf_write_tx_payload((uint8_t)g_nrf.total_size);

    nrf_ce_high();                          /* 3. CE脉冲触发发送(ESB, 保持高直至完成) */
    uint8_t tx_status = nrf_wait_tx_done(); /* 4. 等 TX_DS/MAX_RT, 不再死等1ms */
    nrf_ce_low();

    nrf_check_tx_result(tx_status); /* 5. 统计+清标志 */
}

#endif /* NRF24L01_TX_ENABLE */

/* ================= 接收处理(仅 NRF24L01_RX_ENABLE=1 时编译) ================= */
#if NRF24L01_RX_ENABLE

static void nrf_handle_rx(void)
{
    /* RX FIFO 最多缓存3包, 循环取空; TX 结果已在 TriggerTx 内处理, 这里只管接收 */
    for (uint8_t round = 0; round < 3; round++)
    {
        if (!(nrf_read_status() & NRF24L01_STATUS_RX_DR)) break;

        /* 读动态包长 */
        uint8_t plen = nrf_read_reg(NRF24L01_R_RX_PL_WID);
        if (plen > 0 && plen <= NRF24L01_PAYLOAD_MAX)
        {
            /* 读载荷到 g_nrf_rx[1..] */
            nrf_read_rx_payload(plen);

            /* 长度匹配才解码 */
            if (plen == g_nrf.total_size)
            {
                for (uint8_t i = 0; i < g_nrf.cap_count; i++)
                {
                    NRF_Cap_t *c = &g_nrf.caps[i];
                    memcpy(c->data_ptr, &g_nrf_rx[1 + c->offset], c->size);
                }
                /* OFFLINE 心跳更新 */
                if (g_nrf.offline_dev)
                {
                    Module_Offline_device_update(g_nrf.offline_dev);
                }
            }
        }
        /* 无论包长是否合法都要清RX_DR并清空RX FIFO */
        nrf_write_reg(NRF24L01_STATUS, NRF24L01_STATUS_RX_DR);
        nrf_send_cmd(NRF24L01_FLUSH_RX);
    }
}

#endif /* NRF24L01_RX_ENABLE */

/* ================= 线程 ================= */

/* 收发互斥, 只编译当前角色的线程入口 */
#if NRF24L01_RX_ENABLE
/* 接收: IRQ(或兜底超时)唤醒 → 只做接收 */
static void nrf_thread_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        tx_semaphore_get(&g_nrf_irq_sem, NRF24L01_RX_POLL_MS);
        nrf_handle_rx();
    }
}
#elif NRF24L01_TX_ENABLE
/* 发送: 定时唤醒 → 只做发送 */
static void nrf_thread_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        tx_thread_sleep(NRF24L01_TX_INTERVAL_MS);
        nrf_trigger_tx();
    }
}
#endif

/* ================= EXTI 中断回调(通过BSP注册) ================= */

#if NRF24L01_RX_ENABLE
static void nrf_irq_callback(void)
{
    /* 信号量未创建前不响应, 防止初始化阶段IRQ毛刺导致异常 */
    if (!g_nrf.initialized) return;
    /* nRF24L01 IRQ低电平有效, 置信号量唤醒线程 */
    tx_semaphore_put(&g_nrf_irq_sem);
}
#endif /* NRF24L01_RX_ENABLE */

/* ================= 初始化 ================= */

void Module_NRF24L01_Init(void)
{
    if (g_nrf.initialized) return;

    /* 静态 g_nrf 已在 .bss 清零, 无需 memset(且 memset 会清掉已注册项) */

    /* CE/CSN/IRQ 引脚由板级 CubeMX 配置; 模块只强制 SPI 参数, 并注册 BSP 设备/EXTI 回调 */

    /* ---- 1. SPI 参数强制 + BSP SPI 设备注册 ----
     * 强制模式0(CPOL=0/CPHA=0, nRF24L01要求)并按芯片设分频(SCK ≤10MHz);
     * 同 REMOTE/WT606 强制 UART 波特率: HAL_SPI_Init 会经 MSP 一并配好 SPI 引脚 */
    NRF24L01_SPI.Init.CLKPolarity       = SPI_POLARITY_LOW;
    NRF24L01_SPI.Init.CLKPhase          = SPI_PHASE_1EDGE;
    NRF24L01_SPI.Init.NSS               = SPI_NSS_SOFT;
    NRF24L01_SPI.Init.BaudRatePrescaler = NRF24L01_SPI_PRESCALER;
    if (HAL_SPI_Init(&NRF24L01_SPI) != HAL_OK)
    {
        LOG_E("SPI re-init failed");
        return;
    }

    /* 自检: 模块用 DMA 传输, hdmatx/hdmarx 必须已由板级 CubeMX 的 MSP 链接(_HAL_LINKDMA);
     * 没配的话 HAL_SPI_TransmitReceive_DMA 会直接返回 HAL_ERROR, 表现为所有寄存器读写全失败 */
    if (NRF24L01_SPI.hdmatx == NULL || NRF24L01_SPI.hdmarx == NULL)
    {
        LOG_E("SPI2 DMA not configured: enable SPI2_RX/SPI2_TX DMA in CubeMX (see README)");
        return;
    }

    /* CSN 由 BSP 管理 */
    SPI_Device_Init_Config spi_cfg = {0};
    spi_cfg.hspi                   = &NRF24L01_SPI;
    spi_cfg.cs_port                = NRF24L01_CSN_PORT;
    spi_cfg.cs_pin                 = NRF24L01_CSN_PIN;
    spi_cfg.tx_mode                = SPI_MODE_DMA;
    spi_cfg.rx_mode                = SPI_MODE_DMA;
    g_nrf.spi_dev                  = BSP_SPI_Device_Init(&spi_cfg);
    if (g_nrf.spi_dev == NULL)
    {
        LOG_E("BSP SPI device init failed");
        return;
    }

    /* ---- 2. 注册 EXTI 回调(避免与其他模块的 HAL_GPIO_EXTI_Callback 冲突; 引脚/中断由板级配置) ---- */
#if NRF24L01_RX_ENABLE
    BSP_GPIO_EXTI_Register(NRF24L01_IRQ_PIN, nrf_irq_callback);
#endif

    nrf_ce_low(); /* 确保 CE 初始为低(寄存器配置需处于待机模式) */

    /* ---- 3. nRF24L01 寄存器配置 ---- */
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
    nrf_write_regs(NRF24L01_TX_ADDR, kAddress, 5);
    nrf_write_regs(NRF24L01_RX_ADDR_P0, kAddress, 5);

    /* 清FIFO和中断标志 */
    nrf_send_cmd(NRF24L01_FLUSH_TX);
    nrf_send_cmd(NRF24L01_FLUSH_RX);
    nrf_write_reg(NRF24L01_STATUS, NRF24L01_STATUS_RX_DR | NRF24L01_STATUS_TX_DS | NRF24L01_STATUS_MAX_RT);

    /* 进入RX模式(常驻接收); 仅发送角色不进入RX, 保持待机-I */
#if NRF24L01_RX_ENABLE
    nrf_set_rx_mode();
#endif

    /* ---- 3.5 自检: FEATURE=0 说明 ACTIVATE 失败(DPL/ACK载荷不可用) ---- */
    if (nrf_read_reg(NRF24L01_FEATURE) == 0x00)
    {
        LOG_E("FEATURE=0, ACTIVATE failed! DPL not enabled. Check SPI wiring.");
    }

    /* ---- 4. OFFLINE 集成 ---- */
    Offline_Init_config_t offline_cfg = {0};
    offline_cfg.name                  = "nrf24l01";
    offline_cfg.timeout_ms            = 100;
    offline_cfg.beep_times            = 3;
    offline_cfg.enable                = NRF24L01_OFFLINE_ENABLE;
    g_nrf.offline_dev                 = Module_Offline_register(&offline_cfg);

    /* ---- 5. 创建信号量/线程 ---- */
#if NRF24L01_RX_ENABLE
    tx_semaphore_create(&g_nrf_irq_sem, "nrf_irq", 0);
#endif

    if (tx_thread_create(&g_nrf_thread, "nrf24l01", nrf_thread_entry, 0, g_nrf_stack, NRF24L01_TASK_STACK_SIZE, NRF24L01_TASK_PRIORITY,
                         NRF24L01_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("Thread create failed");
        return;
    }

    g_nrf.initialized = 1;
    LOG_I("NRF24L01 initialized: ch=%d rate=%dMbps power=%ddBm tx=%d rx=%d", NRF24L01_RF_CHANNEL, NRF24L01_RF_DATARATE,
          NRF24L01_RF_POWER == 0 ? 0 : -6 * NRF24L01_RF_POWER, NRF24L01_TX_ENABLE, NRF24L01_RX_ENABLE);
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
    c->data_ptr  = data_ptr;
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