# PCCOMM 上位机双虚拟串口通信文档

## 概述

PCCOMM 模块将板载 USB CDC ACM（虚拟串口）暴露为**两个独立端口**。**串口一（port0）收发自瞄协议，串口二（port1）收发导航协议**。两个端口各自独立双缓冲，互不影响。

**端口分配:**

```
┌──────────────┐   USB CDC ACM
│  板子        │  ─ port0 (/dev/ttyACM0) ─ 自瞄  板→上位机 0x5A / 上位机→板 0xA5
│  PCCOMM      │
│              │  ─ port1 (/dev/ttyACM1) ─ 导航  上位机→板 0xB5 / 板→上位机 0x5B
└──────────────┘
```

- 端口数量 `CDC_ACM_PORT_NUM`：`STM32H723xx` 默认 **2**，其它板默认 **1**；`dji_c`(F407) 为 OTG_FS 端点受限，保持单口。
- 底层依赖 `board/bsp/USB/usbd_cdc_acm_user.{c,h}` 

## 帧格式

统一为定长帧：`[header(1)] [payload(N)] [crc16(2, 小端)]`。
CRC16 由 `crc_rm` 的 `Append_CRC16_Check_Sum` / `Verify_CRC16_Check_Sum` 生成与校验，**校验范围 = header + payload**，CRC 以**小端**追加在帧尾。


## API 接口

```c
int Module_PCComm_Init(void);
```

```c
void Module_PCComm_Send(const SendPacket *packet, uint32_t timeout);
void Module_PCComm_Send_Nav(const NavSendPacket *packet, uint32_t timeout);
```

```c
ReceivePacket *Module_PCComm_Receive(void);
NavPacket     *Module_PCComm_Receive_Nav(void);
```

```c
uint8_t Module_PCComm_Get_offline_state(void);
```

### 宏参数

| 宏 | 默认 | 说明 |
|----|------|------|
| `PCCOMM_TASK_STACK_SIZE` | 1024 | 线程栈大小 |
| `PCCOMM_TASK_PRIORITY` | 10 | 线程优先级 |
| `PCCOMM_OFFLINE_ENABLE` | 1 | 离线检测开关 |
| `PCCOMM_AIM_PORT` | 0 | 自瞄端口号 |
| `PCCOMM_NAV_PORT` | 1 | 导航端口号 |

## 测试数据

### 1 kHz / 32B，双口，10 分钟测试

| 指标 | port0（自瞄） | port1（导航） |
|------|---------------|---------------|
| 发送 / 接收 | 599998 / 599997 | 599999 / 599999 |
| 丢包 | 1 (0.0002%) | 0 (0.0000%) |
| CRC 错 / 重复 / 乱序 | 0 / 0 / 0 | 0 / 0 / 0 |
| 延迟 min / avg / p50 | 158 / 710 / 693 µs | 163 / 752 / 754 µs |
| 延迟 p95 / p99 / max | 1176 / 1261 / 5984 µs | 1198 / 1286 / 6038 µs |

延迟分布：约 **79% <1 ms**，17–20% 在 1–1.2 ms，4–5% 在 1.2–1.5 ms

**超过阈值的帧占比：**

| 阈值 | port0 | port1 |
|------|-------|-------|
| >1.5 ms | 0.0085% (51 帧) | 0.0080% (48 帧) |
| >2.0 ms | 0.0020% (12 帧) | 0.0020% (12 帧) |
| >3.0 ms | 0.0008% (5 帧) | 0.0008% (5 帧) |
| >5.0 ms | 0.0003% (2 帧) | 0.0003% (2 帧) |

**偶发 max 的规律性**：>2 ms 事件 10 分钟内各 7 次，间隔 10.6/56.8/78.6/157.3/10.6/86.4/109.3 s，无固定周期（CV≈0.5，**不定时**）；且两口的事件几乎同时发生（78.6s、157.2s、314.5s、411.5s、520.9s 相差 <3 ms），属**上位机侧偶发调度抖动**

### 更高频率

| 目标速率 | 结果 |
|----------|------|
| 2 kHz | 基本无丢包，延迟升至 ~1.0–1.4 ms |
| 2 khz及以上 | 回环吞吐稳定卡在 ~2000 帧/s，下位机发送上限 |

## 注意事项

1. **端口数量与端点** — 双端口在 H723 的 USB_OTG_HS 上实现（端点 `0x81/0x02/0x83`、`0x82/0x04/0x84`）；单口板（如 F407）只有 1 组端点，`PCCOMM_NAV_PORT` 无效。
2. **帧长固定** — 解析按固定帧长扫描，上位机必须严格按上表组帧（含小端 CRC16）。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-10-10 | 初始版本 |
