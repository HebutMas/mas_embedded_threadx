# ZDT 步进电机模块文档

## 概述

ZDT 系列闭环步进电机驱动模块，支持 **UART** 和 **CAN** 两种传输。

> **协议版本**：张大头 Emm_V5.0 步进闭环驱动说明书 **Rev1.3（2023/11/20）**。

- **UART**：`地址 + 命令 + 校验` 一帧
- **CAN**：ZDT 扩展帧协议 —— 帧 ID = `(地址 << 8) | 包号`，命令去掉地址字节后按 8 字节拆包

## 架构设计

### 数据结构

```c
typedef enum {
    ZDT_MODE_SPEED    = 0,  // 速度模式: ref = RPM
    ZDT_MODE_POSITION = 1,  // 位置模式: 相对脉冲增量
} ZDT_Mode_e;

#define ZDT_CAN_MAX_PKT 2   // 位置命令 12 字节 payload → 2 包

typedef struct
{
    Motor_Base base;

    uint8_t    addr;          /* 从机地址*/
    uint8_t    accel;         /* 加速度档位*/
    uint16_t   pulse_per_rev; /* 每圈脉冲数*/
    ZDT_Mode_e mode;          /* 当前控制模式 */
    uint8_t    enable_state;  /* 使能状态缓存 */

    UART_Device *uart_dev;                 /* UART 传输 */
    Can_Device  *can_dev[ZDT_CAN_MAX_PKT]; /* CAN 传输：每个包号一个设备 */

    /* 位置模式增量 */
    int32_t  pending_pulses;
    uint16_t pending_speed_rpm;
    uint8_t  pending_absolute; /* 0=相对, 1=绝对 */
    uint8_t  pending_sync;     /* 0=立即执行, 1=挂起等待多机同步 */

} ZDT_Motor_t;
```


## 协议帧格式

命令 payload 统一构建为「功能码 + 参数 + 校验」（**不含地址**），再由传输层封装：

### UART 封装

```
| addr | payload[0] | payload[1] | ... | payload[len-1] |
```

### CAN 封装

- 帧 ID = `(addr << 8) | 包号`，29 位扩展帧；
- payload 按每包 8 字节拆包，**每包首字节重复功能码 `payload[0]`**，每包有效数据 7 字节；
- **校验字节只出现在末包末尾**；
- 位置命令 payload 12 字节 → 2 包（`0x0100`、`0x0101`）；使能/停止/速度 ≤ 8 字节 → 1 包。

### 各命令字段

#### 使能帧

```
payload = | 0xF3 | 0xAB | state | 0x00 | 0x6B |
```
- `state`: 1=使能, 0=失能
- `0x00`: 不同步标志

#### 停止帧

```
payload = | 0xFE | 0x98 | 0x00 | 0x6B |
```

#### 速度控制帧

```
payload = | 0xF6 | dir | SPD_H | SPD_L | accel | 0x00 | 0x6B |
```
- `dir`: 0=CW (正转), 1=CCW (反转)
- `SPD_H:SPD_L`: 目标转速 RPM (uint16, big-endian)
- `accel`: 加速度档位

#### 位置控制帧

```
payload = | 0xFD | dir | SPD_H | SPD_L | accel | PULSE[31:24] | PULSE[23:16] | PULSE[15:8] | PULSE[7:0] | ABS | SYNC | 0x6B |
```
- `dir`: 0=CW, 1=CCW
- `SPD_H:SPD_L`: 位置模式运行速度 RPM
- `PULSE[31:0]`: 目标脉冲数 (uint32, big-endian, 绝对值)
- `ABS`: 0=相对上一目标位置, 1=绝对位置（V5.0 新增）
- `SYNC`: 0=立即执行, 1=挂起（需发 `0xFF 0x66` 同步触发）

CAN 拆包结果（以地址 1 为例）：

```
包0  ID=0x0100: FD dir spdH spdL accel p0 p1 p2
包1  ID=0x0101: FD p3 00 00 6B
```

#### 清零位置

```
payload = | 0x0A | 0x6D | 0x6B |
```

#### 多机同步触发

```
payload = | 0xFF | 0x66 | 0x6B |    （CAN 用扩展帧 ID=0x0000；UART 用地址 0）
```

> 用法：先给各电机发 `SYNC=1` 的位置命令（挂起），再广播本命令统一启动。

## 控制流程

```
Motor_ZDT_Init()
  ├── BSP_MEM_ALLOC (ZDT_Motor_t)
  ├── loop_type 强制 = OPEN_LOOP      ── 开环直通, 基类跳过控制计算
  ├── mode = ZDT_MODE_SPEED
  ├── transport == CAN ?
  │     ├── 每包号 BSP_CAN_Device_Init(tx_id=rx_id=(addr<<8)|i, 回调=NULL)
  │     └── transport_dev = can_dev[0]
  │   : transport == UART ?
  │     └── BSP_UART_Device_Init(transport_config.uart)
  ├── base.Apply = zdt_apply
  └── Motor_Register()

每周期调度:
Motor_ControlAll()
  └── loop_type==OPEN_LOOP → 跳过控制计算

Motor_ApplyAll()
  └── zdt_apply()
        ├── if (!enableflag)
        │     └── enable_state != 0 ? → zdt_enable(m, 0), enable_state = 0
        │     └── return
        ├── if (enable_state != 1) → zdt_enable(m, 1), enable_state = 1
        └── 按 m->mode 分发:
              ├── ZDT_MODE_POSITION → pending_pulses != 0 ? 发位置命令并清零
              └── ZDT_MODE_SPEED    → zdt_speed_cmd(ref)
```

### 使能状态机

```
enableflag=0, enable_state!=0  →  发送失能命令  →  enable_state=0
enableflag=1, enable_state!=1  →  发送使能命令  →  enable_state=1
```

通过 `enable_state` 缓存避免每周期重复发送使能命令。`Motor_ZDT_Start/Stop` 将 `enable_state` 置为 `0xFF` 强制下一周期刷新。

### 位置模式

位置命令是**相对上一目标位置的增量**。调用方用 `Motor_ZDT_SendPosition()` 写入 `pending_pulses`，
`zdt_apply` 在下一周期发送并清零，避免每周期重复发送导致累积。

## API 接口说明

```c
ZDT_Motor_t *Motor_ZDT_Init(Motor_Init_Config_s *config, uint8_t addr,
                            uint16_t pulse_per_rev, uint8_t accel);
void Motor_ZDT_Start(ZDT_Motor_t *motor);    // 使能 (强制刷新使能命令)
void Motor_ZDT_Stop(ZDT_Motor_t *motor);     // 失能 + 发送停止命令
void Motor_ZDT_SetMode(ZDT_Motor_t *motor, ZDT_Mode_e mode);
void Motor_ZDT_SendPosition(ZDT_Motor_t *motor, int32_t pulses, uint16_t speed_rpm,
                            uint8_t absolute, uint8_t sync); // absolute: 0=相对 1=绝对; sync: 0=立即 1=挂起
void Motor_ZDT_ClearPosition(ZDT_Motor_t *motor);          // 0x0A 清零位置
void Motor_ZDT_SyncTrigger(ZDT_Motor_t *motor);            // 0xFF 0x66 多机同步触发
```

## 使用示例（CAN）

```c
Motor_Init_Config_s cfg = {
    .transport            = MOTOR_TRANSPORT_CAN,
    .transport_config.can = {.hcan = &hcan2},
    .setting_init_config  = {.loop_type = OPEN_LOOP},
    .motor_init_info      = {.motor_type = ZDT_STEEP_MOTOR, .gear_ratio = 1.0f},
};
ZDT_Motor_t *yaw = Motor_ZDT_Init(&cfg, /*addr=*/1, /*ppr=*/3200, /*accel=*/0);
Motor_ZDT_Stop(yaw);                    // 上电先失能
...
Motor_ZDT_Start(yaw);                   // 使能
Motor_ZDT_SendPosition(yaw, 320, 300, 0, 0);  // 相对 +320 脉冲，300 RPM，立即执行
```

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-11 | 初始版本（UART） |
| v1.1 | 2026-05-18 | 移除掉线检测，默认步进在线，不接收电机反馈信息 |
| v2.0 | 2026-08-07 | 适配新架构: 开环直通 + mode 字段独立控制模式 + 统一运行 API |
| v3.0 | 2026-10-25 | 统一 UART / CAN 双传输 |
