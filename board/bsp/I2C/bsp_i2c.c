#include "bsp_i2c.h"
#include "bsp_def.h"
#include "i2c.h"
#ifdef HAL_I2C_MODULE_ENABLED
#include "tx_api.h"
#include <string.h>

#define LOG_TAG "bsp_i2c"
#define LOG_LVL LOG_LVL_WARNING
#include "ulog_def.h"

/* 事件标志位 */
#define BSP_I2C_EVENT_TX ((ULONG)0x01)
#define BSP_I2C_EVENT_RX ((ULONG)0x02)

/* I2C 设备 */
typedef struct I2C_Device
{
    I2C_HandleTypeDef *hi2c;        /* HAL 句柄 */
    uint16_t           dev_address; /* STM32 中为左移 1 位的地址 */
    I2C_Mode           tx_mode;     /* 发送模式 */
    I2C_Mode           rx_mode;     /* 接收模式 */
    I2C_Device        *next;        /* 指向下一个设备指针 */
} I2C_Device;

/* I2C 总线管理器 */
typedef struct
{
    I2C_HandleTypeDef   *hi2c;         /* HAL 句柄 */
    I2C_Device          *devices_list; /* 设备链表头 */
    TX_EVENT_FLAGS_GROUP event_flags;  /* I2C 总线事件组 */
    TX_MUTEX             bus_mutex;    /* 总线互斥锁 */
    uint8_t              initialized;  /* 是否已初始化 */
} I2C_Bus;

static I2C_Bus i2c_bus_table[I2C_BUS_NUM];

/* 内部辅助函数 */

/**
 * @brief 根据 HAL 句柄查找已初始化的总线
 */
static I2C_Bus *find_bus(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) return NULL;
    for (int i = 0; i < I2C_BUS_NUM; i++)
    {
        if (i2c_bus_table[i].initialized && i2c_bus_table[i].hi2c == hi2c)
        {
            return &i2c_bus_table[i];
        }
    }
    return NULL;
}

/**
 * @brief 传输前处理：获取总线锁 → 检查状态
 * @return NULL = 失败
 */
static I2C_Bus *transfer_begin(I2C_Device *dev, uint32_t timeout)
{
    if (dev == NULL) return NULL;

    I2C_Bus *bus = find_bus(dev->hi2c);
    if (bus == NULL)
    {
        LOG_E("Bus not found for hi2c=%p", (void *)dev->hi2c);
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

    /* 检查 I2C 外设就绪 */
    if (HAL_I2C_GetState(dev->hi2c) != HAL_I2C_STATE_READY)
    {
        LOG_E("I2C not ready");
        if (tx_thread_identify() != NULL) tx_mutex_put(&bus->bus_mutex);
        return NULL;
    }

    /* 清除残留事件标志（防止上一次异常退出的传输干扰本次等待） */
    ULONG dummy;
    tx_event_flags_get(&bus->event_flags, BSP_I2C_EVENT_TX | BSP_I2C_EVENT_RX, TX_OR_CLEAR, &dummy, TX_NO_WAIT);

    return bus;
}

/**
 * @brief 传输后：释放总线锁
 */
static void transfer_end(I2C_Bus *bus)
{
    if (tx_thread_identify() != NULL && bus != NULL)
    {
        tx_mutex_put(&bus->bus_mutex);
    }
}

/* 操作类型 */
typedef enum
{
    I2C_OP_TRANSMIT,
    I2C_OP_RECEIVE,
    I2C_OP_MEM_WRITE,
    I2C_OP_MEM_READ
} I2C_Op;

/**
 * @brief 通用传输（单次）
 */
static uint8_t transfer_once(I2C_Device *dev, I2C_Op op, const uint8_t *tx_data, uint8_t *rx_data, uint16_t size, uint16_t mem_address,
                             uint16_t mem_add_size, uint32_t timeout)
{
    HAL_StatusTypeDef hal_ret = HAL_OK;
    I2C_Bus          *bus     = find_bus(dev->hi2c);
    I2C_Mode          mode;
    ULONG             evt = 0;

    if (bus == NULL)
    {
        LOG_E("Bus not found in transfer_once");
        return 1;
    }

    /* 非任务上下文强制走 BLOCKING，保证同步完成 */
    if (tx_thread_identify() == NULL)
    {
        mode = I2C_MODE_BLOCKING;
    }
    else if (op == I2C_OP_TRANSMIT || op == I2C_OP_MEM_WRITE)
    {
        mode = dev->tx_mode;
        evt  = BSP_I2C_EVENT_TX;
    }
    else
    {
        mode = dev->rx_mode;
        evt  = BSP_I2C_EVENT_RX;
    }

    switch (mode)
    {
    case I2C_MODE_BLOCKING:
    {
        unsigned int tmo = (timeout == 0) ? HAL_MAX_DELAY : timeout;

        switch (op)
        {
        case I2C_OP_TRANSMIT:
            hal_ret = HAL_I2C_Master_Transmit(dev->hi2c, dev->dev_address, (uint8_t *)tx_data, size, tmo);
            break;
        case I2C_OP_RECEIVE:
            hal_ret = HAL_I2C_Master_Receive(dev->hi2c, dev->dev_address, rx_data, size, tmo);
            break;
        case I2C_OP_MEM_WRITE:
            hal_ret = HAL_I2C_Mem_Write(dev->hi2c, dev->dev_address, mem_address, mem_add_size, (uint8_t *)tx_data, size, tmo);
            break;
        case I2C_OP_MEM_READ:
            hal_ret = HAL_I2C_Mem_Read(dev->hi2c, dev->dev_address, mem_address, mem_add_size, rx_data, size, tmo);
            break;
        }
        return (hal_ret == HAL_OK) ? 0 : 1;
    }

    case I2C_MODE_IT:
    {
        switch (op)
        {
        case I2C_OP_TRANSMIT:
            hal_ret = HAL_I2C_Master_Transmit_IT(dev->hi2c, dev->dev_address, (uint8_t *)tx_data, size);
            break;
        case I2C_OP_RECEIVE:
            hal_ret = HAL_I2C_Master_Receive_IT(dev->hi2c, dev->dev_address, rx_data, size);
            break;
        case I2C_OP_MEM_WRITE:
            hal_ret = HAL_I2C_Mem_Write_IT(dev->hi2c, dev->dev_address, mem_address, mem_add_size, (uint8_t *)tx_data, size);
            break;
        case I2C_OP_MEM_READ:
            hal_ret = HAL_I2C_Mem_Read_IT(dev->hi2c, dev->dev_address, mem_address, mem_add_size, rx_data, size);
            break;
        }

        if (hal_ret != HAL_OK) return 1;

        {
            ULONG actual = 0;
            UINT  status = tx_event_flags_get(&bus->event_flags, evt, TX_OR_CLEAR, &actual, timeout);
            if (status != TX_SUCCESS) return 1;
        }
        return 0;
    }

    case I2C_MODE_DMA:
    {
        /* DMA 地址检查 */
        if (tx_data && BSP_IS_DMA_INACCESSIBLE(tx_data)) return 1;
        if (rx_data && BSP_IS_DMA_INACCESSIBLE(rx_data)) return 1;

        /* TX Cache Clean */
        if (tx_data) BSP_CACHE_CLEAN(tx_data, size);

        switch (op)
        {
        case I2C_OP_TRANSMIT:
            hal_ret = HAL_I2C_Master_Transmit_DMA(dev->hi2c, dev->dev_address, (uint8_t *)tx_data, size);
            break;
        case I2C_OP_RECEIVE:
            hal_ret = HAL_I2C_Master_Receive_DMA(dev->hi2c, dev->dev_address, rx_data, size);
            break;
        case I2C_OP_MEM_WRITE:
            hal_ret = HAL_I2C_Mem_Write_DMA(dev->hi2c, dev->dev_address, mem_address, mem_add_size, (uint8_t *)tx_data, size);
            break;
        case I2C_OP_MEM_READ:
            hal_ret = HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->dev_address, mem_address, mem_add_size, rx_data, size);
            break;
        }

        if (hal_ret != HAL_OK) return 1;

        {
            ULONG actual = 0;
            UINT  status = tx_event_flags_get(&bus->event_flags, evt, TX_OR_CLEAR, &actual, timeout);
            if (status != TX_SUCCESS) return 1;
        }

        /* RX Cache Invalidate */
        if (rx_data) BSP_CACHE_INVALIDATE(rx_data, size);
        return 0;
    }

    default:
        return 1;
    }
}

/* 对外函数 */

I2C_Device *BSP_I2C_Device_Init(I2C_Device_Init_Config *config)
{
    if (config == NULL || config->hi2c == NULL)
    {
        LOG_E("Invalid config");
        return NULL;
    }

    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)config->hi2c;
    I2C_Bus           *bus  = find_bus(hi2c);
    if (bus == NULL)
    {
        /* 查找空闲槽位 */
        for (int i = 0; i < I2C_BUS_NUM; i++)
        {
            if (i2c_bus_table[i].initialized == 0)
            {
                bus = &i2c_bus_table[i];
                break;
            }
        }
    }
    if (bus == NULL)
    {
        LOG_E("No free bus slot (max %d)", I2C_BUS_NUM);
        return NULL;
    }

    /* 首次使用该总线时初始化 */
    if (bus->initialized == 0)
    {
        bus->devices_list = NULL; /* 初始化链表头 */
        if (tx_event_flags_create(&bus->event_flags, "i2c_evt") != TX_SUCCESS)
        {
            LOG_E("Event flags create failed");
            return NULL;
        }
        if (tx_mutex_create(&bus->bus_mutex, "i2c_mtx", TX_INHERIT) != TX_SUCCESS)
        {
            LOG_E("Bus mutex create failed");
            tx_event_flags_delete(&bus->event_flags);
            return NULL;
        }

        bus->hi2c        = hi2c;
        bus->initialized = 1;
        LOG_I("Bus %d initialized (hi2c=%p)", (int)(bus - i2c_bus_table), (void *)hi2c);
    }

    /* 创建设备并挂载到总线链表 */
    I2C_Device *dev = NULL;
    BSP_MEM_ALLOC_WAIT(dev, sizeof(I2C_Device), TX_NO_WAIT);
    if (dev == NULL)
    {
        LOG_E("Failed to allocate device");
        return NULL;
    }

    memset(dev, 0, sizeof(I2C_Device));
    dev->hi2c        = hi2c;
    dev->dev_address = config->dev_address;
    dev->tx_mode     = config->tx_mode;
    dev->rx_mode     = config->rx_mode;

    /* 挂载到总线设备链表 */
    dev->next         = bus->devices_list;
    bus->devices_list = dev;

    LOG_I("Device init OK: bus=%d addr=0x%02X", (int)(bus - i2c_bus_table), dev->dev_address);
    return dev;
}

uint8_t BSP_I2C_Transmit(I2C_Device *dev, const uint8_t *tx_data, uint16_t size, uint32_t timeout)
{
    if (!dev || !tx_data || size == 0) return 1;

    I2C_Bus *bus = transfer_begin(dev, timeout);
    if (bus == NULL) return 1;

    uint8_t ret = transfer_once(dev, I2C_OP_TRANSMIT, tx_data, NULL, size, 0, 0, timeout);
    transfer_end(bus);
    return ret;
}

uint8_t BSP_I2C_Receive(I2C_Device *dev, uint8_t *rx_data, uint16_t size, uint32_t timeout)
{
    if (!dev || !rx_data || size == 0) return 1;

    I2C_Bus *bus = transfer_begin(dev, timeout);
    if (bus == NULL) return 1;

    uint8_t ret = transfer_once(dev, I2C_OP_RECEIVE, NULL, rx_data, size, 0, 0, timeout);
    transfer_end(bus);
    return ret;
}

uint8_t BSP_I2C_Mem_Write(I2C_Device *dev, uint16_t mem_address, uint16_t mem_add_size, const uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (!dev || !data || size == 0) return 1;

    I2C_Bus *bus = transfer_begin(dev, timeout);
    if (bus == NULL) return 1;

    uint8_t ret = transfer_once(dev, I2C_OP_MEM_WRITE, data, NULL, size, mem_address, mem_add_size, timeout);
    transfer_end(bus);
    return ret;
}

uint8_t BSP_I2C_Mem_Read(I2C_Device *dev, uint16_t mem_address, uint16_t mem_add_size, uint8_t *data, uint16_t size, uint32_t timeout)
{
    if (!dev || !data || size == 0) return 1;

    I2C_Bus *bus = transfer_begin(dev, timeout);
    if (bus == NULL) return 1;

    uint8_t ret = transfer_once(dev, I2C_OP_MEM_READ, NULL, data, size, mem_address, mem_add_size, timeout);
    transfer_end(bus);
    return ret;
}

/* HAL 回调 */

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    I2C_Bus *bus = find_bus(hi2c);
    if (bus) tx_event_flags_set(&bus->event_flags, BSP_I2C_EVENT_TX, TX_OR);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    I2C_Bus *bus = find_bus(hi2c);
    if (bus) tx_event_flags_set(&bus->event_flags, BSP_I2C_EVENT_RX, TX_OR);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    I2C_Bus *bus = find_bus(hi2c);
    if (bus) tx_event_flags_set(&bus->event_flags, BSP_I2C_EVENT_TX, TX_OR);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    I2C_Bus *bus = find_bus(hi2c);
    if (bus) tx_event_flags_set(&bus->event_flags, BSP_I2C_EVENT_RX, TX_OR);
}

#endif /* HAL_I2C_MODULE_ENABLED */
