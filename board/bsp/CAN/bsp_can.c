#include "bsp_can.h"
#include "bsp_can_task.h"
#include "bsp_def.h"
#include <string.h>

#define LOG_TAG "bsp_can"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

CAN_Bus_Manager g_can_bus[BSP_CAN_BUS_NUM];

/* Filter 配置 */

#if defined(STM32H723xx)
static uint16_t g_fdcan_filter_idx[BSP_CAN_BUS_NUM];     /* 标准帧过滤器计数 */
static uint16_t g_fdcan_ext_filter_idx[BSP_CAN_BUS_NUM]; /* 扩展帧过滤器计数 */

static int8_t fdcan_config_filter(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_id, uint8_t *out_bank_index)
{
    uint8_t bus_idx;
    if (hfdcan->Instance == FDCAN1)
        bus_idx = 0;
    else if (hfdcan->Instance == FDCAN2)
        bus_idx = 1;
    else if (hfdcan->Instance == FDCAN3)
        bus_idx = 2;
    else
        return -1;

    /* 标准帧与扩展帧各用一套独立计数器：HAL 对扩展过滤器的 FilterIndex 同样从 0 起，
     * 内部再偏移到 StdFiltersNbr 之后，扩展过滤器数量由 CubeMX 的 ExtFiltersNbr 决定，为 0 时此处拒绝扩展帧。 */
    FDCAN_FilterTypeDef cfg = {
        .FilterType   = FDCAN_FILTER_MASK,
        .FilterConfig = (rx_id & 1) ? FDCAN_FILTER_TO_RXFIFO1 : FDCAN_FILTER_TO_RXFIFO0,
    };
    uint16_t *idx;
    if (BSP_CAN_IS_EXT(rx_id))
    {
        if (g_fdcan_ext_filter_idx[bus_idx] >= hfdcan->Init.ExtFiltersNbr) return -1;
        cfg.FilterIndex = g_fdcan_ext_filter_idx[bus_idx];
        cfg.IdType      = FDCAN_EXTENDED_ID;
        cfg.FilterID1   = BSP_CAN_RAW_ID(rx_id);
        cfg.FilterID2   = 0x1FFFFFFF; /* 29 位 ID 全匹配 */
        idx             = &g_fdcan_ext_filter_idx[bus_idx];
    }
    else
    {
        if (g_fdcan_filter_idx[bus_idx] >= hfdcan->Init.StdFiltersNbr) return -1;
        cfg.FilterIndex = g_fdcan_filter_idx[bus_idx];
        cfg.IdType      = FDCAN_STANDARD_ID;
        cfg.FilterID1   = rx_id;
        cfg.FilterID2   = 0x7FF; /* 11 位 ID 全匹配 */
        idx             = &g_fdcan_filter_idx[bus_idx];
    }
    if (HAL_FDCAN_ConfigFilter(hfdcan, &cfg) != HAL_OK) return -1;
    if (out_bank_index) *out_bank_index = (uint8_t)cfg.FilterIndex;
    (*idx)++;
    return 0;
}

#elif defined(STM32F407xx) || defined(STM32F105xC)
static uint8_t g_can1_filter_idx;
static uint8_t g_can2_filter_idx = 14;

static bool can_config_filter(CAN_HandleTypeDef *hcan, uint32_t rx_id, uint8_t *out_bank_index)
{
    uint8_t filter_bank;
    if (hcan->Instance == CAN1)
    {
        if (g_can1_filter_idx >= 14) return false;
        filter_bank = g_can1_filter_idx++;
    }
    else if (hcan->Instance == CAN2)
    {
        if (g_can2_filter_idx >= 28) return false;
        filter_bank = g_can2_filter_idx++;
    }
    else
        return false;

    /* 标准帧用 16 位刻度，一个 bank 放两条同样的过滤器；
     * 扩展帧用 32 位刻度：(ID<<3 | IDE) 合成一个 32 位过滤器值；注意 32 位刻度下每条过滤器独占一个 bank。 */
    CAN_FilterTypeDef cfg = {
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = BSP_CAN_IS_EXT(rx_id) ? CAN_FILTERSCALE_32BIT : CAN_FILTERSCALE_16BIT,
        .FilterFIFOAssignment = (rx_id & 1) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1,
        .SlaveStartFilterBank = 14,
        .FilterBank           = filter_bank,
        .FilterActivation     = CAN_FILTER_ENABLE,
    };
    if (BSP_CAN_IS_EXT(rx_id))
    {
        uint32_t f           = (BSP_CAN_RAW_ID(rx_id) << 3) | CAN_ID_EXT;
        cfg.FilterIdHigh     = (uint16_t)(f >> 16);
        cfg.FilterIdLow      = (uint16_t)f;
        cfg.FilterMaskIdHigh = 0xFFFF;
        cfg.FilterMaskIdLow  = 0xFFFF; /* 精确匹配 ID + IDE + RTR(=0 数据帧) */
    }
    else
    {
        cfg.FilterIdHigh     = rx_id << 5;
        cfg.FilterIdLow      = rx_id << 5;
        cfg.FilterMaskIdHigh = 0x7FF << 5;
        cfg.FilterMaskIdLow  = 0x7FF << 5;
    }
    if (HAL_CAN_ConfigFilter(hcan, &cfg) != HAL_OK)
    {
        if (hcan->Instance == CAN1)
            g_can1_filter_idx--;
        else
            g_can2_filter_idx--;
        return false;
    }
    if (out_bank_index) *out_bank_index = filter_bank;
    return true;
}
#elif defined(STM32F103xB)
static uint8_t g_can_filter_idx;

static bool can_config_filter(CAN_HandleTypeDef *hcan, uint32_t rx_id, uint8_t *out_bank_index)
{
    uint8_t filter_bank;
    if (g_can_filter_idx >= 14) return false;
    filter_bank = g_can_filter_idx++;

    /* 标准帧用 16 位刻度，一个 bank 放两条同样的过滤器；
     * 扩展帧用 32 位刻度：(ID<<3 | IDE) 合成一个 32 位过滤器值；注意 32 位刻度下每条过滤器独占一个 bank。 */
    CAN_FilterTypeDef cfg = {
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = BSP_CAN_IS_EXT(rx_id) ? CAN_FILTERSCALE_32BIT : CAN_FILTERSCALE_16BIT,
        .FilterFIFOAssignment = (rx_id & 1) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1,
        .SlaveStartFilterBank = 14,
        .FilterBank           = filter_bank,
        .FilterActivation     = CAN_FILTER_ENABLE,
    };
    if (BSP_CAN_IS_EXT(rx_id))
    {
        uint32_t f           = (BSP_CAN_RAW_ID(rx_id) << 3) | CAN_ID_EXT;
        cfg.FilterIdHigh     = (uint16_t)(f >> 16);
        cfg.FilterIdLow      = (uint16_t)f;
        cfg.FilterMaskIdHigh = 0xFFFF;
        cfg.FilterMaskIdLow  = 0xFFFF; /* 精确匹配 ID + IDE + RTR(=0 数据帧) */
    }
    else
    {
        cfg.FilterIdHigh     = rx_id << 5;
        cfg.FilterIdLow      = rx_id << 5;
        cfg.FilterMaskIdHigh = 0x7FF << 5;
        cfg.FilterMaskIdLow  = 0x7FF << 5;
    }
    if (HAL_CAN_ConfigFilter(hcan, &cfg) != HAL_OK)
    {
        g_can_filter_idx--;
        return false;
    }
    if (out_bank_index) *out_bank_index = filter_bank;
    return true;
}
#endif

/* Bus 查找 */

static CAN_Bus_Manager *can_bus_find(void *hcan, bool require_exist)
{
    if (hcan == NULL) return NULL;
    for (int i = 0; i < BSP_CAN_BUS_NUM; i++)
    {
        if (g_can_bus[i].hcan == hcan) return &g_can_bus[i];
        if (!require_exist && !g_can_bus[i].initialized) return &g_can_bus[i];
    }
    return NULL;
}

/* 对外函数 */

Can_Device *BSP_CAN_Device_Init(Can_Device_Init_Config_s *config)
{
    if (config == NULL || config->hcan == NULL)
    {
        LOG_E("invalid config");
        return NULL;
    }
    /* ID 上界按扩展帧标记分别校验；低 29 位之外的值一律拒绝 */
    if ((BSP_CAN_IS_EXT(config->rx_id) && BSP_CAN_RAW_ID(config->rx_id) > BSP_CAN_MAX_EXT_ID) ||
        (!BSP_CAN_IS_EXT(config->rx_id) && config->rx_id >= BSP_CAN_MAX_STD_ID) ||
        (BSP_CAN_IS_EXT(config->tx_id) && BSP_CAN_RAW_ID(config->tx_id) > BSP_CAN_MAX_EXT_ID) ||
        (!BSP_CAN_IS_EXT(config->tx_id) && config->tx_id >= BSP_CAN_MAX_STD_ID))
    {
        LOG_E("CAN ID out of range");
        return NULL;
    }

    CAN_Bus_Manager *bus = can_bus_find((void *)config->hcan, false);
    if (bus == NULL)
    {
        LOG_E("no free bus slot");
        return NULL;
    }

    if (!bus->initialized)
    {
        memset(bus, 0, sizeof(*bus));
        bus->hcan = config->hcan;
        if (mpmc_queue_init(&bus->rx_queue, bus->rx_queue_buf, bus->rx_queue_seq, BSP_CAN_RX_QUEUE_SIZE, sizeof(BSP_CanMsg_t)) != 0 ||
            mpmc_queue_init(&bus->tx_queue, bus->tx_queue_buf, bus->tx_queue_seq, BSP_CAN_TX_QUEUE_SIZE, sizeof(BSP_CanMsg_t)) != 0)
        {
            LOG_E("can queue init failed");
            return NULL;
        }
#if defined(STM32H723xx)
        /* 全局过滤器只能在 READY 状态下配置，必须先于 Start。 */
        if (HAL_FDCAN_ConfigGlobalFilter(config->hcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
        {
            LOG_E("FDCAN global filter failed");
            return NULL;
        }
        HAL_FDCAN_ActivateNotification(config->hcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
        HAL_FDCAN_Start(config->hcan);
#elif defined(STM32F407xx) || defined(STM32F105xC)
        HAL_CAN_ActivateNotification(config->hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
        HAL_CAN_Start(config->hcan);
#elif defined(STM32F103xB)
        HAL_CAN_ActivateNotification(config->hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
        HAL_CAN_Start(config->hcan);
#endif
        bus->initialized = true;
    }

    if (can_dev_find(bus, config->rx_id) != NULL)
    {
        LOG_E("rx_id 0x%lx already registered", config->rx_id);
        return NULL;
    }

    int slot = can_dev_find_empty(bus);
    if (slot < 0)
    {
        LOG_E("bus device slots full");
        return NULL;
    }

    Can_Device *dev = NULL;
    BSP_MEM_ALLOC_WAIT(dev, sizeof(Can_Device), TX_NO_WAIT);
    if (dev == NULL)
    {
        LOG_E("alloc device failed");
        return NULL;
    }

    memset(dev, 0, sizeof(*dev));
    dev->_bus        = bus;
    dev->tx_id       = config->tx_id;
    dev->rx_id       = config->rx_id;
    dev->rx_callback = config->rx_callback;
    dev->user_arg    = config->user_arg;

#if defined(STM32H723xx)
    if (fdcan_config_filter(config->hcan, config->rx_id, &dev->filter_bank_index) != 0)
    {
        LOG_E("FDCAN filter failed");
        BSP_MEM_FREE(dev);
        return NULL;
    }
#elif defined(STM32F407xx) || defined(STM32F105xC)
    if (!can_config_filter(config->hcan, config->rx_id, &dev->filter_bank_index))
    {
        LOG_E("CAN filter failed");
        BSP_MEM_FREE(dev);
        return NULL;
    }
#elif defined(STM32F103xB)
    if (!can_config_filter(config->hcan, config->rx_id, &dev->filter_bank_index))
    {
        LOG_E("CAN filter failed");
        BSP_MEM_FREE(dev);
        return NULL;
    }
#endif

    can_dev_insert(bus, slot, config->rx_id, dev);

    LOG_I("dev init: rx=0x%lx tx=0x%lx", dev->rx_id, dev->tx_id);
    return dev;
}

int BSP_CAN_Send(Can_Device *device, const uint8_t *data, uint8_t len)
{
    if (device == NULL || data == NULL || len == 0 || len > 8) return -1;

    CAN_Bus_Manager *bus = (CAN_Bus_Manager *)device->_bus;
    if (bus == NULL) return -1;

    BSP_CanMsg_t msg = {.hcan = bus->hcan, .id = device->tx_id, .len = len};
    memcpy(msg.data, data, len);

    mpmc_queue_push(&bus->tx_queue, &msg, NULL);
    tx_semaphore_put(&g_can_tx_sem);
    return 0;
}

int BSP_CAN_SendMessage(BSP_CanMsg_t *msg)
{
    if (msg == NULL || msg->len == 0 || msg->len > 8) return -1;

    CAN_Bus_Manager *bus = can_bus_find((void *)msg->hcan, true);
    if (bus == NULL) return -1;

    mpmc_queue_push(&bus->tx_queue, msg, NULL);
    tx_semaphore_put(&g_can_tx_sem);
    return 0;
}

/* HAL 回调 */

#if defined(STM32H723xx)
static void can_fdcan_rx_callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo)
{
    CAN_Bus_Manager *bus = can_bus_find((void *)hfdcan, true);
    if (bus == NULL) return;

    bool         has_msg = false;
    BSP_CanMsg_t msg     = {.hcan = NULL};

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, RxFifo) > 0)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t               rx_data[8];
        if (HAL_FDCAN_GetRxMessage(hfdcan, RxFifo, &rx_header, rx_data) != HAL_OK) continue;
        msg.id  = (rx_header.IdType == FDCAN_EXTENDED_ID) ? BSP_CAN_EXT_ID(rx_header.Identifier) : rx_header.Identifier;
        msg.len = (uint8_t)(rx_header.DataLength & 0x0F);
        if (msg.len > 8) msg.len = 8;
        memcpy(msg.data, rx_data, msg.len);
        mpmc_queue_push(&bus->rx_queue, &msg, NULL);
        has_msg = true;
    }
    if (has_msg) tx_semaphore_put(&g_can_rx_sem);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) can_fdcan_rx_callback(hfdcan, FDCAN_RX_FIFO0);
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) can_fdcan_rx_callback(hfdcan, FDCAN_RX_FIFO1);
}

#elif defined(STM32F407xx) || defined(STM32F105xC) || defined(STM32F103xB)
static void can_f4_rx_callback(CAN_HandleTypeDef *hcan, uint32_t RxFifo)
{
    CAN_Bus_Manager *bus = can_bus_find((void *)hcan, true);
    if (bus == NULL) return;

    bool         has_msg = false;
    BSP_CanMsg_t msg     = {.hcan = NULL};

    while (HAL_CAN_GetRxFifoFillLevel(hcan, RxFifo) > 0)
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t             rx_data[8];
        if (HAL_CAN_GetRxMessage(hcan, RxFifo, &rx_header, rx_data) != HAL_OK) continue;
        msg.id  = (rx_header.IDE == CAN_ID_EXT) ? BSP_CAN_EXT_ID(rx_header.ExtId) : rx_header.StdId;
        msg.len = rx_header.DLC;
        if (msg.len > 8) msg.len = 8;
        memcpy(msg.data, rx_data, msg.len);
        mpmc_queue_push(&bus->rx_queue, &msg, NULL);
        has_msg = true;
    }
    if (has_msg) tx_semaphore_put(&g_can_rx_sem);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) { can_f4_rx_callback(hcan, CAN_RX_FIFO0); }
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) { can_f4_rx_callback(hcan, CAN_RX_FIFO1); }
#endif
