# BSP USB CDC ACM 驱动文档

## 概述

基于 CherryUSB 协议栈的 CDC ACM（虚拟串口）驱动层，支持多个虚拟串口。

- `damiao_h7`（STM32H723，USB_OTG_HS）默认枚举 **2 个**虚拟串口（双虚拟串口）；
- 其余板型（如 `dji_c` 的 F407 OTG_FS 仅 4 个端点）默认 **1 个**。
- 数量由宏 `CDC_ACM_PORT_NUM` 决定（见 `usbd_cdc_acm_user.h`），可用 `-DCDC_ACM_PORT_NUM=N` 覆盖。

## 架构设计

### 数据流

```
USB 硬件 OUT 端点 ──→ cdc_out_ep_buffer[i] (MPS) ──→ kfifo_in
         │                                                    │
         │ (OUT 端点回调)                                     │
         │                                                    ▼
         └── event_flags_set(RX) ──────────────────→ cdc_acm_recv(port)
                                                              │
                                              kfifo_out(buf, to_read) → 用户 buf
```

### 数据写入路径

```
cdc_acm_send(port) ──→ memcpy(cdc_write_buffer[port]) ──→ usbd_ep_start_write(IN)
                           │
                           │ (等待本端口上次 TX 完成事件)
                           │
                           └── ZLP 处理 (整除包长时自动发 0 长度包)
```

### 内部结构

每个虚拟串口由 `struct cdc_acm_port` 描述，缓冲区/状态均按端口独立：

| 字段 / 资源 | 类型 | 说明 |
|-------------|------|------|
| `cdc_ports[i].rx_fifo` | `struct kfifo` | 第 i 个串口接收 FIFO，底层 buffer 为 `cdc_read_buffer[i]` |
| `cdc_ports[i].event_flags` | `TX_EVENT_FLAGS_GROUP` | 第 i 个串口的事件标志组（bit0: RX, bit1: TX） |
| `cdc_ports[i].out_ep` / `.in_ep` | `uint8_t` | 第 i 个串口的 bulk 端点地址 |
| `cdc_read_buffer[i]` | `uint8_t[1024]` | 接收环形缓冲区（USB_NOCACHE_RAM_SECTION） |
| `cdc_write_buffer[i]` | `uint8_t[CDC_MAX_MPS]` | 发送缓冲区，单次最多一个 MPS 包 |
| `cdc_out_ep_buffer[i]` | `uint8_t[CDC_MAX_MPS]` | USB OUT 端点 DMA 目标缓冲区 |
| `CDC_MAX_MPS` | `512` (HS) / `64` (FS) | 批量传输最大包长 |

### 端点分配

| 串口 | bulk IN | bulk OUT | INT（描述符占位） |
|------|---------|----------|-------------------|
| 0 | `0x81` | `0x02` | `0x83` |
| 1 | `0x82` | `0x04` | `0x84` |

### 事件标志位

| 宏 | 值 | 触发者 | 等待者 |
|----|-----|--------|--------|
| `BSP_USB_EVENT_RX` | `0x01` | `usbd_cdc_acm_bulk_out` (端点回调) | `cdc_acm_recv` (任务) |
| `BSP_USB_EVENT_TX` | `0x02` | `usbd_cdc_acm_bulk_in` (端点回调) | `cdc_acm_send` (任务) |

## API 接口说明

```c
void cdc_acm_init(uint8_t busid, uintptr_t reg_base);
```

```c
int cdc_acm_send(uint8_t port, const uint8_t *data, uint32_t len, uint32_t timeout);
```

```c
int cdc_acm_recv(uint8_t port, uint8_t *data, uint32_t buf_size, uint32_t *rx_len, uint32_t timeout);
```


## 使用示例

```c
#include "usbd_cdc_acm_user.h"

void usb_demo_init(void)
{
    /* 初始化 CDC ACM，使用 USB1 (busid=0)，按 CDC_ACM_PORT_NUM 枚举串口 */
    cdc_acm_init(0, 0x40040000); /* USB1 OTG HS */
}

void usb_send_task(ULONG unused)
{
    uint8_t payload[] = "Hello USB\r\n";

    while (1)
    {
        /* 通过串口 1 发送 */
        int ret = cdc_acm_send(1, payload, sizeof(payload), 100);
        if (ret > 0)
        {
            /* 发送成功 */
        }
        tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND);
    }
}

void usb_recv_task(ULONG unused)
{
    uint8_t  buf[256];
    uint32_t rx_len;

    while (1)
    {
        /* 从串口 0 接收 */
        int ret = cdc_acm_recv(0, buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER);
        if (ret > 0)
        {
            /* 处理 buf[0..rx_len-1] */
        }
    }
}
```

## 内部实现细节

### 零拷贝 kfifo

`cdc_read_buffer[i]`（USB_NOCACHE_RAM_SECTION，放在 Cache 不缓存区域）直接作为第 i 个串口 kfifo 的底层 buffer：

```c
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t cdc_read_buffer[CDC_ACM_PORT_NUM][CDC_RX_FIFO_SIZE];
kfifo_init(&cdc_ports[i].rx_fifo, cdc_read_buffer[i], CDC_RX_FIFO_SIZE, 1);
```

OUT 端点回调收到一个 MPS 包后通过 `kfifo_in` 写入环形缓冲区，并置位本端口 RX 事件。

### 端点反查端口

共享端点回调 `usbd_cdc_acm_bulk_out` / `usbd_cdc_acm_bulk_in` 通过端点地址反查所属串口（`cdc_port_by_ep`），再操作对应的 kfifo / 事件标志组。

### ZLP 自动处理

IN 端点回调中，若发送字节数为端点 MPS 的整数倍且非零，自动追加一个零长度包（ZLP）标志传输结束：

```c
if ((nbytes % usbd_get_ep_mps(busid, ep)) == 0 && nbytes)
{
    usbd_ep_start_write(busid, cdc_ports[port].in_ep, NULL, 0); // ZLP
}
else
{
    tx_event_flags_set(&cdc_ports[port].event_flags, BSP_USB_EVENT_TX, TX_OR);
}
```

### 缓冲区位域

`cdc_read_buffer` / `cdc_write_buffer` / `cdc_out_ep_buffer` 通过 `USB_NOCACHE_RAM_SECTION` 放置在非 Cache 区域：

- H7 (`STM32H723xx`): `.RAM_D2` 段
- F4 (`STM32F407xx`): `.ram` 段

### FIFO / 端点资源约束

双虚拟串口需要 4 个非控制端点（2×bulk IN + 2×bulk OUT）。
`dji_c` 的 F407 OTG_FS 仅 EP0..EP3，因此 `CDC_ACM_PORT_NUM=1`；

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-09 | 初始版本 |
| v1.1 | 2026-05-23 | `cdc_acm_recv` 新增 `buf_size` 参数防止用户缓冲区溢出；OUT 端点回调增加 kfifo 容量保护 |
| v1.2 | 2026-10-10 | 支持多虚拟串口（h723） |
