# ZDT 步进电机模块文档

## 概述

ZDT 步进电机驱动模块，通过 UART 串口协议控制 ZDT 系列闭环步进电机。

## 架构设计

### 数据结构

```c
typedef enum {
    UART_MODE_SPEED    = 0,  // 速度模式: ref = RPM
    UART_MODE_POSITION = 1,  // 位置模式: ref = 脉冲数 (相对)
} UART_Mode_e;

typedef struct {
    Motor_Base   base;
    UART_Device *uart_dev;       // UART 设备句柄
    uint8_t      addr;           // 从机地址 (1-255, 0=广播)
    uint8_t      tx_buf[16];     // 发送缓冲区
    uint8_t      rx_buf[16];     // 接收缓冲区
    uint8_t      enable_state;   // 使能状态缓存 (0=失能, 1=使能, 0xFF=强制刷新)
    uint16_t     pulse_per_rev;  // 每圈脉冲数 (如 3200 = 16细分)
    uint8_t      accel;          // 加速度档位 (0-255, 0=无加减速曲线)
    UART_Mode_e  mode;           // 控制模式 (UART_MODE_SPEED / UART_MODE_POSITION)
    float        position_rad;   // 实时位置 (rad)
    float        velocity_rpm;   // 实时转速 (RPM)
} UART_Motor_t;
```

### 协议命令字

| 命令 | 字节 | 说明 |
|------|------|------|
| `CMD_ENABLE` | `0xF3` | 使能/失能电机 |
| `CMD_SPEED` | `0xF6` | 速度模式控制 |
| `CMD_POSITION` | `0xFD` | 位置模式控制 (相对) |
| `CMD_STOP` | `0xFE` | 立即停止 |
| `CMD_CLR_POS` | `0x0A` | 清零位置 (保留) |
| `CHECKSUM` | `0x6B` | 固定校验和 |


## 协议帧格式

### 使能帧 

```
| addr | 0xF3 | 0xAB | state | 0x00 | 0x6B |
```
- `state`: 1=使能, 0=失能
- `0x00`: 不同步标志

### 停止帧 

```
| addr | 0xFE | 0x98 | 0x00 | 0x6B |
```

### 速度控制帧

```
| addr | 0xF6 | dir | SPD_H | SPD_L | accel | 0x00 | 0x6B |
```
- `dir`: 0=CW (正转), 1=CCW (反转)
- `SPD_H:SPD_L`: 目标转速 RPM (uint16, big-endian)
- `accel`: 加速度档位

### 位置控制帧

```
| addr | 0xFD | dir | SPD_H | SPD_L | accel | PULSE[31:24] | PULSE[23:16] | PULSE[15:8] | PULSE[7:0] | 0x00 | 0x00 | 0x6B |
```
- `dir`: 0=CW, 1=CCW
- `SPD_H:SPD_L`: 位置模式运行速度 RPM
- `PULSE[31:0]`: 目标脉冲数 (uint32, big-endian, 绝对值)
- `0x00`: 相对位置模式
- `0x00`: 不同步标志

## 控制流程

```
Motor_ZDT_Init()
  ├── BSP_MEM_ALLOC (UART_Motor_t)
  ├── loop_type 强制 = OPEN_LOOP      ── 开环直通, 基类跳过控制计算
  ├── mode = UART_MODE_SPEED          ── 默认速度模式
  ├── BSP_UART_Device_Init()         ── 注册 UART 设备
  ├── base.Apply = zdt_apply         (无离线检测, 步进默认在线)
  └── Motor_Register()               ── 注册到全局链表

每周期调度:
Motor_ControlAll()
  └── 统一控制计算                    ── 阶段1 (基类): loop_type==OPEN_LOOP → 跳过

Motor_ApplyAll()
  └── zdt_apply()                    ── 阶段2:
        ├── if (!enableflag)
        │     └── enable_state != 0 ? → zdt_enable(m, 0), enable_state = 0
        │     └── return
        ├── if (enable_state != 1) → zdt_enable(m, 1), enable_state = 1
        └── 按 m->mode 分发:
              ├── UART_MODE_POSITION → zdt_position_cmd(pulses, speed_rpm)
              └── UART_MODE_SPEED    → zdt_speed_cmd(rpm)
```

### 使能状态机

```
enableflag=0, enable_state!=0  →  发送失能帧 (0xF3+state=0)  →  enable_state=0
enableflag=1, enable_state!=1  →  发送使能帧 (0xF3+state=1)  →  enable_state=1
```

通过 `enable_state` 缓存避免每周期重复发送使能命令。`Start/Stop` 将 `enable_state` 置为 `0xFF` 强制下一周期刷新。

### 位置模式速度

位置模式下的运行速度由 `base->info.max_torque` 字段复用提供（`uint16_t` 截断），默认为 300 RPM。

## API 接口说明

```c
UART_Motor_t *Motor_ZDT_Init(Motor_Init_Config_s *config,
                             uint8_t addr, uint16_t pulse_per_rev, uint8_t accel);
```

```c
void Motor_ZDT_Start(UART_Motor_t *motor);   /* 使能 (强制刷新使能命令) */
```

```c
void Motor_ZDT_Stop(UART_Motor_t *motor);    /* 失能 + 发送停止命令 */
```

```c
void Motor_ZDT_SetMode(UART_Motor_t *motor, UART_Mode_e mode);  /* 切换速度/位置模式 */
```

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-11 | 初始版本|
| v1.1 | 2026-05-18 | 移除掉线检测，默认步进在线，不接收电机反馈信息 |
| v2.0 | 2026-08-07 | 适配新架构: 开环直通 + mode 字段独立控制模式 + 统一运行 API |
