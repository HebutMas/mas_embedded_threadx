#include "bsp_spi.h"
#include "bsp_def.h"
#include "spi.h"
#ifdef HAL_SPI_MODULE_ENABLED
#include "tx_api.h"
#include <string.h>

#define LOG_TAG "bsp_spi"
#define LOG_LVL LOG_LVL_WARNING
#include "ulog_def.h"

/* ── 事件标志位 ── */
#define BSP_SPI_EVENT_TX   ((ULONG)0x01)
#define BSP_SPI_EVENT_RX   ((ULONG)0x02)
#define BSP_SPI_EVENT_TXRX ((ULONG)0x04)

/* SPI 设备 */
typedef struct SPI_Device
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    SPI_Mode           tx_mode;
    SPI_Mode           rx_mode;
    SPI_Device        *next; /* 指向下一个设备指针 */
} SPI_Device;

/* ── SPI 总线管理器 ── */
typedef struct
{
    SPI_HandleTypeDef   *hspi;         /* HAL 句柄 */
    SPI_Device          *devices_list; /* 设备链表 */
    TX_EVENT_FLAGS_GROUP event_flags;  /* spi总线事件组 */
    TX_MUTEX             bus_mutex;    /* 总线互斥锁 */
    uint8_t              initialized;  /* 是否已初始化 */
} SPI_Bus;

static SPI_Bus spi_bus_table[SPI_BUS_NUM];

/* 内部辅助函数 */

/**
 * @brief 根据 HAL 句柄查找已初始化的总线
 */
static SPI_Bus *find_bus(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL) return NULL;
    for (int i = 0; i < SPI_BUS_NUM; i++)
    {
        if (spi_bus_table[i].initialized && spi_bus_table[i].hspi == hspi)
        {
            return &spi_bus_table[i];
        }
    }
    return NULL;
}

static void spi_cs_low(SPI_Device *dev)
{
    if (dev->cs_port != NULL && dev->cs_pin != 0)
    {
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    }
}

static void spi_cs_high(SPI_Device *dev)
{
    if (dev->cs_port != NULL && dev->cs_pin != 0)
    {
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    }
}

/**
 * @brief 传输前处理：获取总线锁 → 检查状态 → 拉低 CS
 * @return NULL = 失败
 */
static SPI_Bus *transfer_begin(SPI_Device *dev, uint32_t timeout)
{
    if (dev == NULL) return NULL;

    SPI_Bus *bus = find_bus(dev->hspi);
    if (bus == NULL)
    {
        LOG_E("Bus not found for hspi=%p", (void *)dev->hspi);
        return NULL;
    }

    /* 仅在任务上下文中获取互斥锁 */
    if (tx_thread_identify() != NULL)
    {
        if (tx_mutex_get(&bus->bus_mutex, timeout) != TX_SUCCESS)
        {
            LOG_E("Failed to acquire bus mutex");
            return NULL;
        }
    }

    /* 检查 SPI 外设就绪 */
    if (HAL_SPI_GetState(dev->hspi) != HAL_SPI_STATE_READY)
    {
        LOG_E("SPI not ready");
        if (tx_thread_identify() != NULL) tx_mutex_put(&bus->bus_mutex);
        return NULL;
    }

    /* 清除残留事件标志（防止上一次异常退出的传输干扰本次等待） */
    ULONG dummy;
    tx_event_flags_get(&bus->event_flags, BSP_SPI_EVENT_TX | BSP_SPI_EVENT_RX | BSP_SPI_EVENT_TXRX, TX_OR_CLEAR, &dummy, TX_NO_WAIT);

    spi_cs_low(dev);
    return bus;
}

/**
 * @brief 传输后：拉高 CS → 释放总线锁
 */
static void transfer_end(SPI_Bus *bus, SPI_Device *dev)
{
    spi_cs_high(dev);
    if (tx_thread_identify() != NULL && bus != NULL)
    {
        tx_mutex_put(&bus->bus_mutex);
    }
}

/**
 * @brief 等待事件标志（IT / DMA 模式）
 */
static int wait_event(SPI_Bus *bus, ULONG event_flag, uint32_t timeout)
{
    ULONG actual = 0;
    UINT  status = tx_event_flags_get(&bus->event_flags, event_flag, TX_OR_CLEAR, &actual, timeout);
    return (status == TX_SUCCESS) ? 0 : 1;
}

/**
 * @brief 通用传输（单次）
 */
static uint8_t transfer_once(SPI_Device *dev, const uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout, int is_tx, int is_rx)
{
    HAL_StatusTypeDef hal_ret = HAL_OK;
    SPI_Bus          *bus     = find_bus(dev->hspi);
    SPI_Mode          mode;
    ULONG             evt = 0;

    if (bus == NULL)
    {
        LOG_E("Bus not found in transfer_once");
        return 1;
    }

    /* 非任务上下文强制走 BLOCKING，保证同步完成 */
    if (tx_thread_identify() == NULL)
    {
        mode = SPI_MODE_BLOCKING;
    }
    else
    {
        mode = is_tx ? dev->tx_mode : dev->rx_mode;
    }

    if (is_tx && is_rx)
        evt = BSP_SPI_EVENT_TXRX;
    else if (is_tx)
        evt = BSP_SPI_EVENT_TX;
    else
        evt = BSP_SPI_EVENT_RX;

    switch (mode)
    {
    case SPI_MODE_BLOCKING:
    {
        unsigned int tmo = (timeout == 0) ? HAL_MAX_DELAY : timeout;
        if (is_tx && is_rx)
            hal_ret = HAL_SPI_TransmitReceive(dev->hspi, (uint8_t *)tx_data, rx_data, size, tmo);
        else if (is_tx)
            hal_ret = HAL_SPI_Transmit(dev->hspi, (uint8_t *)tx_data, size, tmo);
        else
            hal_ret = HAL_SPI_Receive(dev->hspi, rx_data, size, tmo);
        return (hal_ret == HAL_OK) ? 0 : 1;
    }
    case SPI_MODE_IT:
        if (is_tx && is_rx)
            hal_ret = HAL_SPI_TransmitReceive_IT(dev->hspi, (uint8_t *)tx_data, rx_data, size);
        else if (is_tx)
            hal_ret = HAL_SPI_Transmit_IT(dev->hspi, (uint8_t *)tx_data, size);
        else
            hal_ret = HAL_SPI_Receive_IT(dev->hspi, rx_data, size);

        if (hal_ret != HAL_OK) return 1;
        return wait_event(bus, evt, timeout);

    case SPI_MODE_DMA:
        /* DMA 地址检查 */
        if (is_tx && BSP_IS_DMA_INACCESSIBLE(tx_data)) return 1;
        if (is_rx && BSP_IS_DMA_INACCESSIBLE(rx_data)) return 1;

        /* TX Cache Clean */
        if (is_tx) BSP_CACHE_CLEAN(tx_data, size);

        if (is_tx && is_rx)
            hal_ret = HAL_SPI_TransmitReceive_DMA(dev->hspi, (uint8_t *)tx_data, rx_data, size);
        else if (is_tx)
            hal_ret = HAL_SPI_Transmit_DMA(dev->hspi, (uint8_t *)tx_data, size);
        else
            hal_ret = HAL_SPI_Receive_DMA(dev->hspi, rx_data, size);

        if (hal_ret != HAL_OK) return 1;
        if (wait_event(bus, evt, timeout) != 0) return 1;

        /* RX Cache Invalidate */
        if (is_rx) BSP_CACHE_INVALIDATE(rx_data, size);
        return 0;

    default:
        return 1;
    }
}

/* 对外函数 */

SPI_Device *BSP_SPI_Device_Init(SPI_Device_Init_Config *config)
{
    if (config == NULL || config->hspi == NULL)
    {
        LOG_E("Invalid config");
        return NULL;
    }

    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef *)config->hspi;
    SPI_Bus           *bus  = find_bus(hspi);
    if (bus == NULL)
    {
        /* 查找空闲槽位 */
        for (int i = 0; i < SPI_BUS_NUM; i++)
        {
            if (spi_bus_table[i].initialized == 0)
            {
                bus = &spi_bus_table[i];
                break;
            }
        }
    }
    if (bus == NULL)
    {
        LOG_E("No free bus slot (max %d)", SPI_BUS_NUM);
        return NULL;
    }

    /* 首次使用该总线时初始化 */
    if (bus->initialized == 0)
    {
        bus->devices_list = NULL; /* 初始化链表头 */
        if (tx_event_flags_create(&bus->event_flags, "spi_evt") != TX_SUCCESS)
        {
            LOG_E("Event flags create failed");
            return NULL;
        }
        if (tx_mutex_create(&bus->bus_mutex, "spi_mtx", TX_INHERIT) != TX_SUCCESS)
        {
            LOG_E("Bus mutex create failed");
            tx_event_flags_delete(&bus->event_flags);
            return NULL;
        }

        bus->hspi        = hspi;
        bus->initialized = 1;
        LOG_I("Bus %d initialized (hspi=%p)", (int)(bus - spi_bus_table), (void *)hspi);
    }

    /* 创建设备并挂载到总线链表 */
    SPI_Device *dev = NULL;
    BSP_MEM_ALLOC_WAIT(dev, sizeof(SPI_Device), TX_NO_WAIT);
    if (dev == NULL)
    {
        LOG_E("Failed to allocate device");
        return NULL;
    }

    memset(dev, 0, sizeof(SPI_Device));
    dev->hspi    = hspi;
    dev->cs_port = (GPIO_TypeDef *)config->cs_port;
    dev->cs_pin  = config->cs_pin;
    dev->tx_mode = config->tx_mode;
    dev->rx_mode = config->rx_mode;

    /* 初始化 CS 为高 */
    spi_cs_high(dev);

    /* 挂载到总线设备链表 */
    dev->next         = bus->devices_list;
    bus->devices_list = dev;

    LOG_I("Device init OK: bus=%d cs=P%c%d", (int)(bus - spi_bus_table), dev->cs_port ? 'A' + ((uint32_t)dev->cs_port & 0xF) : '?', dev->cs_pin);
    return dev;
}

uint8_t BSP_SPI_TransReceive(SPI_Device *dev, const uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    if (!dev || !tx_data || !rx_data || size == 0) return 1;

    SPI_Bus *bus = transfer_begin(dev, timeout);
    if (bus == NULL) return 1;

    uint8_t ret = transfer_once(dev, tx_data, rx_data, size, timeout, 1, 1);
    transfer_end(bus, dev);
    return ret;
}

uint8_t BSP_SPI_Transmit(SPI_Device *dev, const uint8_t *tx_data, uint16_t size, uint32_t timeout)
{
    if (!dev || !tx_data || size == 0) return 1;

    SPI_Bus *bus = transfer_begin(dev, timeout);
    if (bus == NULL) return 1;

    uint8_t ret = transfer_once(dev, tx_data, NULL, size, timeout, 1, 0);
    transfer_end(bus, dev);
    return ret;
}

uint8_t BSP_SPI_Receive(SPI_Device *dev, uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    if (!dev || !rx_data || size == 0) return 1;

    SPI_Bus *bus = transfer_begin(dev, timeout);
    if (bus == NULL) return 1;

    uint8_t ret = transfer_once(dev, NULL, rx_data, size, timeout, 0, 1);
    transfer_end(bus, dev);
    return ret;
}

uint8_t BSP_SPI_TransAndTrans(SPI_Device *dev, const uint8_t *tx_data1, uint16_t size1, const uint8_t *tx_data2, uint16_t size2, uint32_t timeout)
{
    if (!dev) return 1;

    SPI_Bus *bus = transfer_begin(dev, timeout);
    if (bus == NULL) return 1;

    uint8_t ret = 0;
    if (size1 > 0 && tx_data1) ret = transfer_once(dev, tx_data1, NULL, size1, timeout, 1, 0);
    if (ret == 0 && size2 > 0 && tx_data2) ret = transfer_once(dev, tx_data2, NULL, size2, timeout, 1, 0);

    transfer_end(bus, dev);
    return ret;
}

uint8_t BSP_SPI_ReceAndRece(SPI_Device *dev, uint8_t *rx_data1, uint16_t size1, uint8_t *rx_data2, uint16_t size2, uint32_t timeout)
{
    if (!dev) return 1;

    SPI_Bus *bus = transfer_begin(dev, timeout);
    if (bus == NULL) return 1;

    uint8_t ret = 0;
    if (size1 > 0 && rx_data1) ret = transfer_once(dev, NULL, rx_data1, size1, timeout, 0, 1);
    if (ret == 0 && size2 > 0 && rx_data2) ret = transfer_once(dev, NULL, rx_data2, size2, timeout, 0, 1);

    transfer_end(bus, dev);
    return ret;
}

/* HAL 回调  */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Bus *bus = find_bus(hspi);
    if (bus) tx_event_flags_set(&bus->event_flags, BSP_SPI_EVENT_TX, TX_OR);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Bus *bus = find_bus(hspi);
    if (bus) tx_event_flags_set(&bus->event_flags, BSP_SPI_EVENT_RX, TX_OR);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    SPI_Bus *bus = find_bus(hspi);
    if (bus) tx_event_flags_set(&bus->event_flags, BSP_SPI_EVENT_TXRX, TX_OR);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    /* 清除 OVR 错误 */
    if (hspi->ErrorCode & HAL_SPI_ERROR_OVR)
    {
#if defined(STM32H723xx)
        volatile uint32_t sr = hspi->Instance->SR;
        volatile uint32_t dr = hspi->Instance->RXDR;
#elif defined(STM32F407xx)
        volatile uint32_t sr = hspi->Instance->SR;
        volatile uint32_t dr = hspi->Instance->DR;
#elif defined(STM32F105xC) || defined(STM32F103xB)
        volatile uint32_t sr = hspi->Instance->SR;
        volatile uint32_t dr = hspi->Instance->DR;
#endif
        (void)sr;
        (void)dr;
    }

    SPI_Bus *bus = find_bus(hspi);
    if (bus)
    {
        HAL_SPI_Abort(hspi);
        hspi->State     = HAL_SPI_STATE_READY;
        hspi->ErrorCode = HAL_SPI_ERROR_NONE;
    }
}

#endif /* HAL_SPI_MODULE_ENABLED */
