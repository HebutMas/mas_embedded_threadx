# OFFLINE 离线检测模块文档

## 概述

OFFLINE 模块是一个基于心跳机制的设备离线检测框架。

## 架构设计

### 数据流

```
设备 A ──► Module_Offline_device_update() ──► last_time = now
设备 B ──► Module_Offline_device_update() ──► last_time = now
设备 C ──► Module_Offline_device_update() ──► last_time = now
                                                    │
                                                    ▼
┌─────────────────────────────────────────────────────────────┐
│           offline_detect_task_entry (10ms 周期)             │
│                                                             │
│  1. 刷新 IWDG (可选)                                        │
│  2. for 循环遍历所有设备                                    │
│     ├── now - last_time > timeout_ms  →  is_offline = true  │
│     └── now - last_time ≤ timeout_ms  →  is_offline = false │
│  3. 汇总离线状态, 决定告警等级                              │
│  4. LED / 蜂鸣器 状态机更新                                 │
└─────────────────────────────────────────────────────────────┘
        │                       │
        ▼                       ▼
   BSP_LED_Show()         BSP_BEEP_Set()
```

### 设备链表结构

所有注册设备通过 `Offline_Device.next` 指针串联，检测任务直接 for 循环遍历。

```
g_device_list (链表头)
    │
    ├── Offline_Device (设备 A)
    │       node.next ──► ...
    │
    ├── Offline_Device (设备 B)
    │       node.next ──► ...
    │
    └── Offline_Device (设备 C)
```

### `Offline_Device` 内部结构

| 字段 | 类型 | 说明 |
|------|------|------|
| `next` | `Offline_Device *` | 指向下一个注册设备的指针，NULL 表尾 |
| `name` | `char[32]` | 设备名称（最多 31 字符） |
| `timeout_ms` | `uint32_t` | 心跳超时时间（ms） |
| `beep_times` | `uint8_t` | 离线时每 2s 大周期内蜂鸣次数（0 = 静默告警） |
| `is_offline` | `bool` | 当前离线状态 |
| `last_time` | `uint32_t` | 上次心跳时间戳（DWT ms） |
| `enable` | `uint8_t` | 检测开关（1 = 启用，0 = 禁用） |

### 告警分级状态机

```
全部在线          → LED 绿灯, 蜂鸣关闭
存在 beep>0 离线  → LED 红灯闪烁 + 蜂鸣 (周期性)
存在 beep=0 离线  → LED 红灯常亮, 蜂鸣关闭 (静默)
两者同时存在      → 按 beep>0 处理 (蜂鸣优先)
```

#### LED / 蜂鸣器 时序

每个 2s 大周期内，蜂鸣次数 = `min(所有离线设备的 beep_times)`：

```
大周期 2000ms ────────────────────────────────────────►
│  子周期(200ms)  │  子周期(200ms)  │  子周期(200ms)   │ ...
│  0-100ms        │  0-100ms        │                  │
│  LED红+蜂鸣响   │  LED红+蜂鸣响   │  LED灭+蜂鸣关    │
│  100-200ms      │  100-200ms      │                  │
│  LED灭+蜂鸣关   │  LED灭+蜂鸣关   │                  │
└─────────────────┴─────────────────┴──────────────────┘
```

## API 接口说明

```c
void Module_Offline_init(void);
```

```c
Offline_Device *Module_Offline_register(const Offline_Init_config_t *init);
```

```c
void Module_Offline_device_update(Offline_Device *dev);
```

```c
void Module_Offline_device_enable(Offline_Device *dev);
```

```c
void Module_Offline_device_disable(Offline_Device *dev);
```

```c
uint8_t Module_Offline_get_device_status(Offline_Device *dev);
```

## 使用示例

### 基本用法

```c
#include "module_offline.h"

static Offline_Device *g_sensor_dev;

void app_init(void)
{
    /* 初始化离线模块 */
    if (Module_Offline_init() != 0) {
        return;
    }

    /* 注册传感器设备 (超时 500ms, 离线蜂鸣 3 次) */
    Offline_Init_config_t sensor_cfg = {
        .name       = "IMU_Sensor",
        .timeout_ms = 500,
        .beep_times = 3,
        .enable     = 1,
    };
    g_sensor_dev = Module_Offline_register(&sensor_cfg);

}

/* 传感器任务 — 每 100ms 喂狗 */
void sensor_task(ULONG unused)
{
    while (1)
    {
        read_sensor_data();
        Module_Offline_device_update(g_sensor_dev);
        tx_thread_sleep(100);
    }
}
```

## 内部实现细节

### 设备遍历与离线判定

检测任务直接 for 循环遍历所有设备，内联聚合离线状态：

```c
for (Offline_Device *dev = g_device_list; dev; dev = dev->next)
{
    if (!dev->enable) continue;

    uint32_t elapsed = now - dev->last_time;
    if (elapsed > dev->timeout_ms)
    {
        dev->is_offline = true;
        if (dev->beep_times > 0)
        {
            has_severe_fault = true;
            if (request_beep_times == 0 || dev->beep_times < request_beep_times)
                request_beep_times = dev->beep_times;
        }
        else
            has_silent_fault = true;
    }
    else
        dev->is_offline = false;
}
```

遍历结束后统一处理 LED / 蜂鸣器。

### 告警优先级

1. **存在 `beep_times > 0` 的离线设备** → 蜂鸣 + 红灯闪烁（`g_silent_error = false`）
2. **仅存在 `beep_times = 0` 的离线设备** → 红灯常亮，不蜂鸣（`g_silent_error = true`）
3. **全部在线** → 绿灯，不蜂鸣

当多个 `beep_times > 0` 设备同时离线时，取**最小的** `beep_times` 作为蜂鸣次数。

### LED / 蜂鸣器状态机

使用静态变量在 10ms 周期调用间保持状态：

| 变量 | 说明 |
|------|------|
| `beep_period_start_time` | 当前 2s 大周期起始时刻 |
| `beep_cycle_start_time` | 当前闪烁子周期起始时刻 |
| `remaining_beep_cycles` | 当前大周期内剩余闪烁次数 |

每个子周期 200ms：前 100ms 红灯亮 + 蜂鸣，后 100ms 全灭。大周期结束后重新装载 `remaining_beep_cycles`。

### IWDG 看门狗刷新

若启用 `OFFLINE_WATCHDOG_ENABLE`，检测任务每 10ms 刷新一次独立看门狗：
调试模式下通过 `__HAL_DBGMCU_FREEZE_IWDG` 冻结看门狗，避免断点时复位。

## 注意事项

1. **初始化顺序** — 必须先调用 `Module_Offline_init()`，再调用 `Module_Offline_register()`，否则注册会失败（`g_initialized` 为 `false`）。

2. **心跳周期建议** — 设备侧心跳更新周期应远小于 `timeout_ms`，建议 `timeout_ms ≥ 3 × 心跳周期`，避免正常抖动触发离线。

3. **离线检测** — 离线设备的 `beep_times` 应 <= 10。(2s大周期每个子周期200ms -- 10个周期)

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2025-09-11 | 初始版本|
| v1.1 | 2026-05-20 | 增加调试输出，方便调试 |
| v2.0 | 2026-05-21 | 使用单向链表，方便调试 |