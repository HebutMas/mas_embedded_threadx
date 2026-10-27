/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-05 22:15:45
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-21 12:41:37
 * @FilePath: /mas_embedded_threadx/board/bsp/CAN/bsp_can.h
 * @Description: CAN BSP 公共接口与内部类型
 */
#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#if defined(STM32H723xx)
#include "fdcan.h"
#elif defined(STM32F407xx)
#include "can.h"
#elif defined(STM32F105xC) || defined(STM32F103xB)
#include "can.h"
#endif

#include "mpmc_queue.h"
#include <stdbool.h>
#include <stdint.h>

#if defined(STM32H723xx)
#define BSP_CAN_BUS_NUM 3
#elif defined(STM32F407xx) || defined(STM32F105xC)
#define BSP_CAN_BUS_NUM 2
#elif defined(STM32F103xB)
#define BSP_CAN_BUS_NUM 1
#endif

#if defined(STM32H723xx)
#define BSP_CAN_HANDLE1       &hfdcan1
#define BSP_CAN_HANDLE2       &hfdcan2
#define BSP_CAN_HANDLE3       &hfdcan3
#define BSP_CAN_IS_HANDLE1(p) ((p) == BSP_CAN_HANDLE1)
#define BSP_CAN_IS_HANDLE2(p) ((p) == BSP_CAN_HANDLE2)
#define BSP_CAN_IS_HANDLE3(p) ((p) == BSP_CAN_HANDLE3)
#elif defined(STM32F407xx) || defined(STM32F105xC)
#define BSP_CAN_HANDLE1       &hcan1
#define BSP_CAN_HANDLE2       &hcan2
#define BSP_CAN_HANDLE3       NULL
#define BSP_CAN_IS_HANDLE1(p) ((p) == BSP_CAN_HANDLE1)
#define BSP_CAN_IS_HANDLE2(p) ((p) == BSP_CAN_HANDLE2)
#define BSP_CAN_IS_HANDLE3(p) (0)
#elif defined(STM32F103xB)
#define BSP_CAN_HANDLE1       &hcan
#define BSP_CAN_HANDLE2       NULL
#define BSP_CAN_HANDLE3       NULL
#define BSP_CAN_IS_HANDLE1(p) ((p) == BSP_CAN_HANDLE1)
#define BSP_CAN_IS_HANDLE2(p) (0)
#define BSP_CAN_IS_HANDLE3(p) (0)
#endif

#define BSP_CAN_RX_QUEUE_SIZE   32 /* 接收队列，容量须为 2 的幂 */
#define BSP_CAN_TX_QUEUE_SIZE   8  /* 发送队列，容量须为 2 的幂 */
#define BSP_CAN_MAX_DEV_PER_BUS 8

/* 标准帧 / 扩展帧统一用一个 32 位 ID 表达：bit31 = 扩展帧标记，低 29 位 = 实际报文 ID。
 *   注册：  .rx_id = BSP_CAN_EXT_ID(0x100)   （标准帧照旧直接写数值）
 *   分发：  ISR 已按报文 IDE 置好标记，回调里拿到的 id 同样带标记 */
#define BSP_CAN_EXT_FLAG        0x80000000u
#define BSP_CAN_EXT_ID(id)      ((uint32_t)(id) | BSP_CAN_EXT_FLAG)
#define BSP_CAN_IS_EXT(id)      (((uint32_t)(id) & BSP_CAN_EXT_FLAG) != 0u)
#define BSP_CAN_RAW_ID(id)      ((uint32_t)(id) & ~BSP_CAN_EXT_FLAG)

#define BSP_CAN_MAX_STD_ID      2048        /* 标准帧 ID 上界 (合法 0 ~ 0x7FF) */
#define BSP_CAN_MAX_EXT_ID      0x1FFFFFFFu /* 扩展帧 ID 上界 (29 位) */

_Static_assert((BSP_CAN_RX_QUEUE_SIZE & (BSP_CAN_RX_QUEUE_SIZE - 1u)) == 0u, "BSP_CAN_RX_QUEUE_SIZE must set power of 2");
_Static_assert((BSP_CAN_TX_QUEUE_SIZE & (BSP_CAN_TX_QUEUE_SIZE - 1u)) == 0u, "BSP_CAN_TX_QUEUE_SIZE must set power of 2");

typedef struct Can_Device Can_Device;

typedef void (*BSP_CAN_RxCallback_t)(Can_Device *dev, const uint8_t *data, uint8_t len);

/* 初始化配置 */
typedef struct
{
#if defined(STM32H723xx)
    FDCAN_HandleTypeDef *hcan;
#elif defined(STM32F407xx)
    CAN_HandleTypeDef *hcan;
#elif defined(STM32F105xC) || defined(STM32F103xB)
    CAN_HandleTypeDef *hcan;
#endif
    uint32_t             tx_id;
    uint32_t             rx_id;
    BSP_CAN_RxCallback_t rx_callback;
    void                *user_arg;
} Can_Device_Init_Config_s;

/* CAN 消息 */
typedef struct
{
#if defined(STM32H723xx)
    FDCAN_HandleTypeDef *hcan;
#elif defined(STM32F407xx)
    CAN_HandleTypeDef *hcan;
#elif defined(STM32F105xC) || defined(STM32F103xB)
    CAN_HandleTypeDef *hcan;
#endif
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
} BSP_CanMsg_t;

/* 设备查找表条目 */
typedef struct
{
    uint32_t    rx_id; /* 0 = 空闲槽位 */
    Can_Device *dev;
} can_dev_slot_t;

/* CAN 设备 */
struct Can_Device
{
    uint32_t             tx_id;             /* 发送 ID */
    uint32_t             rx_id;             /* 接收 ID */
    uint8_t              filter_bank_index; /* 过滤器索引 */
    BSP_CAN_RxCallback_t rx_callback;       /* 接收回调 */
    void                *user_arg;          /* 用户参数 */
    void                *_bus;              /* 指向 CAN_Bus_Manager */
};

/* CAN 总线管理器 */
typedef struct
{
#if defined(STM32H723xx)
    FDCAN_HandleTypeDef *hcan;
#elif defined(STM32F407xx)
    CAN_HandleTypeDef *hcan;
#elif defined(STM32F105xC) || defined(STM32F103xB)
    CAN_HandleTypeDef *hcan;
#endif
    can_dev_slot_t devices[BSP_CAN_MAX_DEV_PER_BUS];    /* 设备查找表 */
    mpmc_queue_t   rx_queue;                            /* 接收队列 */
    BSP_CanMsg_t   rx_queue_buf[BSP_CAN_RX_QUEUE_SIZE]; /* 接收队列元素存储 */
    uint32_t       rx_queue_seq[BSP_CAN_RX_QUEUE_SIZE]; /* 接收队列槽位序号 */
    mpmc_queue_t   tx_queue;                            /* 发送队列*/
    BSP_CanMsg_t   tx_queue_buf[BSP_CAN_TX_QUEUE_SIZE]; /* 发送队列元素存储 */
    uint32_t       tx_queue_seq[BSP_CAN_TX_QUEUE_SIZE]; /* 发送队列槽位序号 */
    bool           initialized;                         /* 是否初始化 */
} CAN_Bus_Manager;

extern CAN_Bus_Manager g_can_bus[];

/* 辅助函数 */

static inline Can_Device *can_dev_find(CAN_Bus_Manager *bus, uint32_t rx_id)
{
    for (int i = 0; i < BSP_CAN_MAX_DEV_PER_BUS; i++)
    {
        if (bus->devices[i].rx_id == rx_id) return bus->devices[i].dev;
    }
    return NULL;
}

static inline int can_dev_find_empty(CAN_Bus_Manager *bus)
{
    for (int i = 0; i < BSP_CAN_MAX_DEV_PER_BUS; i++)
    {
        if (bus->devices[i].dev == NULL) return i;
    }
    return -1;
}

static inline void can_dev_insert(CAN_Bus_Manager *bus, int slot, uint32_t rx_id, Can_Device *dev)
{
    bus->devices[slot].rx_id = rx_id;
    bus->devices[slot].dev   = dev;
}

/**
 * @brief 初始化CAN设备
 * @param config CAN设备初始化配置
 * @return Can_Device* 成功返回CAN设备指针，失败返回NULL
 */
Can_Device *BSP_CAN_Device_Init(Can_Device_Init_Config_s *config);
/**
 * @brief 发送CAN消息
 * @param device CAN设备指针
 * @param data 发送数据
 * @param len 发送数据长度
 * @return int 成功返回0，失败返回-1
 */
int BSP_CAN_Send(Can_Device *device, const uint8_t *data, uint8_t len);
/**
 * @brief 发送CAN消息
 * @param msg CAN消息
 * @return int 成功返回0，失败返回-1
 */
int BSP_CAN_SendMessage(BSP_CanMsg_t *msg);

#endif /* _BSP_CAN_H_ */
