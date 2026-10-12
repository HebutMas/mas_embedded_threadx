# VISION 自瞄通信模块文档

## 概述

VISION 模块通过 USB CDC ACM 的 **串口一（port0）**收发自瞄协议，内置 minipc 离线检测。
与 NAV 模块分别维护接收线程和双缓冲，USB 控制器仍由 BSP 统一初始化。

**端口分配:**

```
┌──────────────┐   USB CDC ACM
│  VISION      │ ─ port0 ─ 自瞄  板→上位机 0x5A / 上位机→板 0xA5
└──────────────┘
┌──────────────┐
│  NAV         │ ─ port1 ─ 导航  板→上位机 0x5B / 上位机→板 0xB5
└──────────────┘
```

## 架构设计

### 数据流

```
USB OUT → cdc_out_ep_buffer[0] → 接收 FIFO → RX 事件
                                               │
                                               ▼
                                        vision_thread
                                               │
                                     帧头 / CRC16 检查
                                               │
                                更新自瞄双缓冲 + 离线心跳
                                               │
                                               ▼
                               Module_Vision_Receive() → app
```

### 数据写入路径

```
app → Module_Vision_Send() → 帧头 + VisionSendPacket + CRC16
                            → cdc_acm_send(0, ...) → USB IN
```

发送在调用者线程中执行；VISION 线程负责接收。
接收线程使用 `TX_WAIT_FOREVER` 等待 CDC0 事件，读取失败时休眠 1 tick。
每批处理后让出同优先级线程，避免持续来包时阻碍 NAV。

### 内部结构

| 字段 / 资源 | 类型 | 说明 |
|-------------|------|------|
| `receive_packet[2]` | `VisionReceivePacket[2]` | 自瞄接收双缓冲 |
| `receive_idx` | `uint32_t` | 当前写入槽位，写完后切换 |
| `receive_snapshot` | `VisionReceivePacket` | 返回给 app 的最近有效包快照 |
| `vision_thread_stack` | `uint8_t[1024]` | 接收线程栈 |
| `offline_dev` | `Offline_Device *` | 原有 minipc 离线检测对象 |

## 帧格式

统一为定长帧：`[header(1)] [payload(N)] [crc16(2, 小端)]`。
使用现有 `crc_rm` 校验函数，保留原有协议布局。

| 方向 | 帧头 | payload | 总长度 |
|------|------|---------|--------|
| 板 → 上位机 | `0x5A` | `VisionSendPacket`：mode、四元数 q[4] | 20 字节 |
| 上位机 → 板 | `0xA5` | `VisionReceivePacket`：target_yaw、target_pitch、fire_advice | 12 字节 |

结构体按 1 字节对齐，float 沿用 STM32 的 32 位小端布局，角度单位 rad。

## API 接口说明

```c
int Module_Vision_Init(void);
```

由 `MODULE_Init` 在 OFFLINE 初始化后调用一次，不重复初始化 USB。
返回 0 成功；-1 表示离线设备注册或线程创建失败。

```c
void Module_Vision_Send(const VisionSendPacket *packet, uint32_t timeout);
```

通过 CDC0 发送姿态数据；`timeout` 单位为 ThreadX tick，返回不代表 USB 传输完成。

```c
VisionReceivePacket *Module_Vision_Receive(void);
```

复制最近有效包到快照并返回指针，指针有效至下次调用；尚无有效帧时为零值。

```c
uint8_t Module_Vision_Get_offline_state(void);
```

只检查自瞄通信；收到合法自瞄帧时更新心跳，超时沿用 100 ms。

### 宏参数

默认值在 `modules/module_config.cmake` 中配置。

| 宏 | 默认 | 说明 |
|----|------|------|
| `VISION_TASK_STACK_SIZE` | 1024 | 线程栈大小 |
| `VISION_TASK_PRIORITY` | 10 | 接收线程优先级 |
| `VISION_OFFLINE_ENABLE` | 1 | 自瞄离线检测开关 |

## 使用示例

应用 `MODULES_*` 中加入 `OFFLINE VISION`，系统统一完成初始化后，在控制任务中调用：

```c
#include "module_vision.h"
#include "tx_api.h"

VisionSendPacket send_packet = {.mode = 0, .q = {1.0f, 0.0f, 0.0f, 0.0f}};
Module_Vision_Send(&send_packet, TX_NO_WAIT);
VisionReceivePacket *receive_packet = Module_Vision_Receive();
```

## 内部实现细节

接收线程每次读取最多 64 字节，扫描帧头并检查 CRC16。
找到合法帧后写入当前槽位，切换索引；getter 从上次完成的槽位复制快照。
每批至多取第一个合法帧，沿用原有单次读取内扫描逻辑。

## 注意事项

1. **初始化顺序** — BSP USB → OFFLINE → VISION，每个模块仅初始化一次。
2. **接收使用** — 按一个控制任务读取快照使用，不保留跨次半帧。
3. **驱动行为** — FIFO 溢出、拔插恢复等继承现有 USB 驱动，本次不扩展可靠性。
4. **应用控制** — 哨兵自动控制中的视觉离线判断保持原样；导航分支同时检查 NAV 在线状态和报文有效状态，条件不满足时清零底盘平移速度。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.1 | 2026-10-10 | 数据包类型统一为 VisionSendPacket、VisionReceivePacket |
| v1.0 | 2026-10-10 | 从 PCCOMM 拆出自瞄通信，使用独立接收线程，移除回环测试 |
