# BSP CAN 驱动文档

## 概述

BSP CAN 是基于 STM32 HAL 库的 CAN 总线驱动层，采用 **缓冲队列（mpmc） + 独立后台任务 + 接收回调** 异步架构，支持 STM32H723 (FDCAN) 和 STM32F407 (经典 CAN) 双平台，标准帧与扩展帧都能走完整的注册-过滤-分发-队列链路。

## 架构设计

### 数据流

```
┌──────────┐   HAL IRQ (事件标志)  ┌──────────────┐  mpmc(无锁)     ┌──────────────┐
│ 硬件     │ ───────────────────▶  │ ISR 回调     │ ──push────────▶ │ CAN RX Task  │
│ RX FIFO  │   drain→push→set_evt  │ GetRxMsg→push│  满则驱逐最老   │ pop→匹配→回调│
└──────────┘                       └──────────────┘                 └──────┬───────┘
                                                                           │
                                                                   rx_callback(dev, data, len)
                                                                           │
                                                                    ┌──────▼───────┐
                                                                    │ 用户回调函数 │
                                                                    └──────────────┘

┌─────────────┐  mpmc(无锁入队)  ┌──────────────┐    HAL (重试)   ┌──────────┐
│ 用户 Task   │ ──push─▶        │ CAN TX Task  │ ─────────────▶  │ 硬件     │
│ BSP_CAN_Send│   → set_evt      │  pop→HAL_Add │  满则 sleep(1)  │ TX FIFO  │
└─────────────┘                  └──────────────┘                 └──────────┘
```

RX/TX 各自独立任务，各用独立计数信号量：
- `g_can_rx_sem`: ISR drain→push 后 `tx_semaphore_put` → 唤醒 CAN RX Task (prio 3)
- `g_can_tx_sem`: BSP_CAN_Send 入队后 `tx_semaphore_put` → 唤醒 CAN TX Task (prio 4)

TX 队列满时由生产者丢弃最老一帧，保证新帧必然入队；

### `CAN_Bus_Manager` 内部结构

| 字段 | 类型 | 说明 |
|------|------|------|
| `hcan` | `FDCAN/CAN_HandleTypeDef *` | HAL 库 CAN 句柄 |
| `devices` | `can_dev_slot_t [MAX_DEV]` | 设备查找表 |
| `rx_queue` | `mpmc_queue_t` | 接收队列 |
| `rx_queue_buf` | `BSP_CanMsg_t [RX_QUEUE_SIZE]` | 接收队列元素存储 |
| `rx_queue_seq` | `uint32_t [RX_QUEUE_SIZE]` | 接收队列槽位发布序号 |
| `tx_queue` | `mpmc_queue_t` | 发送队列 |
| `tx_queue_buf` | `BSP_CanMsg_t [TX_QUEUE_SIZE]` | 发送队列元素存储 |
| `tx_queue_seq` | `uint32_t [TX_QUEUE_SIZE]` | 发送队列槽位发布序号 |
| `tx_evicted` | `uint32_t` | 队列满时被丢弃的最老帧累计数（调试用） |
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

### 标准帧与扩展帧

ID 字段（`rx_id` / `tx_id` / `BSP_CanMsg_t.id`）统一是 32 位：**bit31 = 扩展帧标记，低 29 位 = 实际报文 ID**。

```c
.cfg.rx_id = 0x205;                          /* 标准帧照旧 */
.cfg.rx_id = BSP_CAN_EXT_ID(0x100);          /* 29 位扩展帧 */
```

- `BSP_CAN_IS_EXT(id)` / `BSP_CAN_RAW_ID(id)` 供上层判断与取原始 ID；回调里收到的 `dev` 对应的 id 也带标记。
- 注册时按标记分别校验上界：标准 `0 ~ 0x7FF`，扩展 `0 ~ 0x1FFFFFFF`，越界直接返回 NULL。
- 接收 ISR 依据报文 IDE 置/清标记，因此分发和过滤都对两种帧型透明。

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

### ISR 批量事件标志

```c
bool has_msg = false;
while (HAL_GetRxFifoFillLevel > 0) {
    GetRxMessage → can_rx_enqueue (mpmc_queue_push);
    has_msg = true;
}
if (has_msg) tx_semaphore_put(&g_can_rx_sem);
```

### 过滤器配置

- **H7 FDCAN**：掩码模式，奇数 ID → FIFO1，偶数 ID → FIFO0，全局过滤器拒绝未匹配帧。
  标准帧 11-bit 全匹配 (`FilterID2 = 0x7FF`)，扩展帧 `IdType = FDCAN_EXTENDED_ID`、29-bit 全匹配 (`FilterID2 = 0x1FFFFFFF`)。
  标准帧与扩展帧各用一套独立计数器，`FilterIndex` 均从 0 起（HAL 内部把扩展索引偏移到 `StdFiltersNbr` 之后）
  扩展过滤器池大小由 CubeMX 的 `ExtFiltersNbr` 决定（未配置则为 0，此时扩展帧设备注册会失败）。
  首个设备注册时，必须在 `HAL_FDCAN_Start` 前配置全局过滤器；启动后 HAL 处于 BUSY 状态，不允许再配置全局过滤器。配置失败则设备初始化失败。
- **F4 CAN**：IDMASK 模式。标准帧用 16-bit 刻度，`FilterId = rx_id << 5`、`FilterMaskId = 0x7FF << 5`，一个 bank 的两半填同一条过滤器
  扩展帧用 **32-bit 刻度**：过滤器值 = `(raw_id << 3) | CAN_ID_EXT`，拆成 `FilterIdHigh:FilterIdLow`，掩码全 1（精确匹配 ID + IDE，RTR/FMT 位为 0 → 只收数据帧）。


## 可调参数 (bsp_can.h)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `BSP_CAN_BUS_NUM` | H7:3, F4:2 | 总线数量 (由芯片型号决定) |
| `BSP_CAN_RX_QUEUE_SIZE` | 8 | per-bus RX 无锁队列深度 (必须是 2 的幂) |
| `BSP_CAN_TX_QUEUE_SIZE` | 32 | per-bus TX 无锁队列深度 (必须是 2 的幂) |
| `BSP_CAN_MAX_DEV_PER_BUS` | 8 | 单总线最大设备数 |
| `BSP_CAN_MAX_STD_ID` | 2048 | 标准帧 ID 上界 (合法 0 ~ 0x7FF) |
| `BSP_CAN_MAX_EXT_ID` | 0x1FFFFFFF | 扩展帧 ID 上界 (29 位) |
| `BSP_CAN_EXT_FLAG` | 0x80000000 | ID 的扩展帧标记位，配合 `BSP_CAN_EXT_ID()` / `BSP_CAN_IS_EXT()` / `BSP_CAN_RAW_ID()` |

### 任务优先级与栈大小 (bsp_can_task.c)

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `BSP_CAN_RX_TASK_PRIORITY` | 3 | RX 任务优先级 (数值越小越高) |
| `BSP_CAN_TX_TASK_PRIORITY` | 4 | TX 任务优先级 |
| `BSP_CAN_TASK_STACK_SIZE` | 1024 | 任务栈大小 (字节) |

## 注意事项

- **BSP_CAN_TaskInit 必须在 BSP_CAN_Device_Init 之前调用**
- **rx_id 必须唯一** — 同一总线不支持多个设备注册相同 rx_id（扩展帧标记参与比较，标准 0x100 与扩展 0x100 视为不同设备）
- **回调在 CAN RX Task 上下文中执行** — 禁止长时间阻塞
- **不要在应用层直接调 HAL 收发或改过滤器/中断使能** — `HAL_CAN_DeactivateNotification` 会关掉整条总线该 FIFO 的中断

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-05 | 初始版本 |
| v2.0 | 2026-05-21 | 移除侵入式链表, 信号量替换为事件标志组，TX kfifo 覆盖写 |
| v2.1 | 2026-05-21 | 事件标志组替换为信号量 |
| v3.0 | 2026-10-24 | 收发缓冲统一改为无锁 MPMC 队列，增加扩展帧支持 |
