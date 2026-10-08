# BSP CAN 驱动文档

## 概述

BSP CAN 是基于 STM32 HAL 库的 CAN 总线驱动层，采用 **kfifo 缓冲 + 独立后台任务 + 接收回调** 异步架构，支持 STM32H723 (FDCAN) 和 STM32F407 (经典 CAN) 双平台。

## 架构设计

### 数据流

```
┌──────────┐   HAL IRQ (事件标志)  ┌──────────────┐   kfifo(SPSC)   ┌──────────────┐
│ 硬件     │ ───────────────────▶  │ ISR 回调     │ ──put─────────▶ │ CAN RX Task  │
│ RX FIFO  │   drain→put→set_evt   │ GetRxMsg→put │                 │ get→匹配→回调│
└──────────┘                       └──────────────┘                 └──────┬───────┘
                                                                           │
                                                                   rx_callback(dev, data, len)
                                                                           │
                                                                    ┌──────▼───────┐
                                                                    │ 用户回调函数 │
                                                                    └──────────────┘

┌─────────────┐    kfifo(覆盖写)  ┌──────────────┐    HAL (重试)   ┌──────────┐
│ 用户 Task   │ ──put──────────▶  │ CAN TX Task  │ ─────────────▶  │ 硬件     │
│ BSP_CAN_Send│    → set_evt      │ kfifo_get→HAL│  满则 sleep(1)  │ TX FIFO  │
└─────────────┘                   └──────────────┘                 └──────────┘
```

RX/TX 各自独立任务，各用独立计数信号量：
- `g_can_rx_sem`: ISR drain→put 后 `tx_semaphore_put` → 唤醒 CAN RX Task (prio 3)
- `g_can_tx_sem`: BSP_CAN_Send 入队后 `tx_semaphore_put` → 唤醒 CAN TX Task (prio 4)

TX kfifo 满时自动丢弃最老消息，保证最新数据总能入队。

### `CAN_Bus_Manager` 内部结构

| 字段 | 类型 | 说明 |
|------|------|------|
| `hcan` | `FDCAN/CAN_HandleTypeDef *` | HAL 库 CAN 句柄 |
| `devices` | `can_dev_slot_t [MAX_DEV]` | 设备查找表 |
| `rx_fifo` | `struct kfifo` | 接收 FIFO |
| `rx_fifo_buf` | `BSP_CanMsg_t [RX_FIFO_SIZE]` | 接收 kfifo 底层缓冲区 |
| `tx_fifo` | `struct kfifo` | 发送 FIFO (覆盖写) |
| `tx_fifo_buf` | `BSP_CanMsg_t [TX_FIFO_SIZE]` | 发送 kfifo 底层缓冲区 |
| `initialized` | `bool` | 总线是否已激活 |

### `Can_Device` 结构

| 字段 | 类型 | 说明 |
|------|------|------|
| `tx_id` | `uint32_t` | 发送 CAN ID |
| `rx_id` | `uint32_t` | 接收 CAN ID |
| `filter_bank_index` | `uint8_t` | 硬件过滤器编号 |
| `rx_callback` | `BSP_CAN_RxCallback_t` | 接收回调 |
| `user_arg` | `void *` | 用户参数 |
| `_bus` | `void *` | 指向所属 CAN_Bus_Manager |

### 事件标志与任务

| 信号量 | 类型 | 生产者 | 消费者 |
|--------|------|--------|--------|
| `g_can_rx_sem` | 计数信号量 | HAL_RxFifoXCallback (ISR) | CAN RX Task (prio 3) |
| `g_can_tx_sem` | 计数信号量 | BSP_CAN_Send / SendMessage | CAN TX Task (prio 4) |

> ISR 中 drain 整个硬件 RX FIFO 后只放一次 sem (批量 sem)。TX 侧每次入队后放一次。

---

## API 接口说明

```c
int BSP_CAN_TaskInit(void);
```

```c
Can_Device *BSP_CAN_Device_Init(Can_Device_Init_Config_s *config);
```

```c
int BSP_CAN_Send(Can_Device *device, const uint8_t *data, uint8_t len);
```

```c
int BSP_CAN_SendMessage(BSP_CanMsg_t *msg);
```

### 接收回调

```c
typedef void (*BSP_CAN_RxCallback_t)(Can_Device *dev, const uint8_t *data, uint8_t len);
```

## 使用示例
```c
#include "bsp_can.h"
#include "bsp_can_task.h"

static void motor_fb_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    int16_t angle = (data[0] << 8) | data[1];
    int idx = (int)(uintptr_t)dev->user_arg;
    g_motor[idx].angle = angle;
}

void can_demo_init(void)
{
    BSP_CAN_TaskInit();  /* 必须在 BSP_CAN_Device_Init 之前 */

    Can_Device_Init_Config_s cfg = {
        .hcan        = &hfdcan1,
        .tx_id       = 0x200,
        .rx_id       = 0x201,
        .rx_callback = motor_fb_callback,
        .user_arg    = (void *)0, 
    };
    Can_Device *motor0 = BSP_CAN_Device_Init(&cfg);
}

void can_send_task(ULONG unused)
{
    while (1) {
        uint8_t cmd[8] = {0};
        /* 填充控制指令... */
        BSP_CAN_Send(motor0, cmd, 8);
        tx_thread_sleep(1); /* 1kHz */
    }
}
```

### 直接发送 (不注册设备)

```c
BSP_CanMsg_t msg = {
    .hcan = &hfdcan1,
    .id   = 0x300,
    .len  = 8,
    .data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
};
BSP_CAN_SendMessage(&msg);
```

### 多设备注册

```c
Can_Device_Init_Config_s cfgs[] = {
    { .hcan = &hfdcan1, .tx_id = 0x200, .rx_id = 0x201, .rx_callback = cb0, .user_arg = (void *)0 },
    { .hcan = &hfdcan1, .tx_id = 0x1FF, .rx_id = 0x202, .rx_callback = cb1, .user_arg = (void *)1 },
    { .hcan = &hfdcan1, .tx_id = 0x1FE, .rx_id = 0x203, .rx_callback = cb2, .user_arg = (void *)2 },
};

Can_Device *devs[3];
for (int i = 0; i < 3; i++) {
    devs[i] = BSP_CAN_Device_Init(&cfgs[i]);
}
```

---

## 内部实现细节

### TX kfifo 覆盖写

```c
/* BSP_CAN_Send: 入队失败时丢弃最老消息 */
if (!kfifo_put(&bus->tx_fifo, &msg)) {
    BSP_CanMsg_t dummy;
    kfifo_get(&bus->tx_fifo, &dummy);  /* 丢弃最老 */
    kfifo_put(&bus->tx_fifo, &msg);    /* 写入最新 */
}
tx_semaphore_put(&g_can_tx_sem);
```

### ISR 批量事件标志

```c
bool has_msg = false;
while (HAL_GetRxFifoFillLevel > 0) {
    GetRxMessage → kfifo_put;
    has_msg = true;
}
if (has_msg) tx_semaphore_put(&g_can_rx_sem);
```

H7 FDCAN TX FIFO 仅 3 槽，F4 CAN 仅 3 邮箱。软件 kfifo (默认 8 槽) 提供额外缓冲，任务自动重试保证不丢帧。

### 过滤器配置

- **H7 FDCAN**：掩码模式，11-bit 全匹配 (`FilterID2 = 0x7FF`)，奇数 ID → FIFO1，偶数 ID → FIFO0，全局过滤器拒绝未匹配帧。
- **F4 CAN**：16-bit IDMASK 模式，`FilterId = rx_id << 5`，`FilterMaskId = 0x7FF << 5`。CAN1 占用 bank 0~13，CAN2 占用 bank 14~27。


## 可调参数 (bsp_can.h)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `BSP_CAN_BUS_NUM` | H7:3, F4:2 | 总线数量 (由芯片型号决定) |
| `BSP_CAN_RX_FIFO_SIZE` | 8 | per-bus RX kfifo 深度 (必须是 2 的幂) |
| `BSP_CAN_TX_FIFO_SIZE` | 8 | per-bus TX kfifo 深度 (必须是 2 的幂) |
| `BSP_CAN_MAX_DEV_PER_BUS` | 8 | 单总线最大设备数 |
| `BSP_CAN_MAX_STD_ID` | 2048 | CAN 标准 ID 最大值 (11-bit) |

### 任务优先级与栈大小 (bsp_can_task.c)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `BSP_CAN_RX_TASK_PRIORITY` | 3 | RX 任务优先级 (数值越小越高) |
| `BSP_CAN_TX_TASK_PRIORITY` | 4 | TX 任务优先级 |
| `BSP_CAN_TASK_STACK_SIZE` | 1024 | 任务栈大小 (字节) |

## 注意事项

- **BSP_CAN_TaskInit 必须在 BSP_CAN_Device_Init 之前调用**
- **rx_id 必须唯一** — 同一总线不支持多个设备注册相同 rx_id
- **回调在 CAN RX Task 上下文中执行** — 禁止长时间阻塞

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-05 | 初始版本 |
| v2.0 | 2026-05-21 | 移除侵入式链表, 信号量替换为事件标志组，TX kfifo 覆盖写 |
| v2.1 | 2026-05-21 | 事件标志组替换为信号量 |
