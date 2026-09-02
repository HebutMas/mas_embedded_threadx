# 达妙 (DM) 电机模块文档

## 概述

达妙电机驱动模块，通过 CAN 总线控制达妙电机。支持 **MIT、位置-速度、纯速度、PSI** 四种控制模式。

> [参考链接](https://gitee.com/kit-miao/damiao)

**支持的控制模式：**

| 模式 | 宏 | CAN ID 偏移 | 说明 |
|------|-----|-----------|------|
| MIT 模式 | `DM_MIT_MODE` | `+0x000` | 位置-速度-Kp-Kd-扭矩联合控制 |
| 位置-速度模式 | `DM_POS_MODE` | `+0x100` | 浮点位置 + 浮点速度 |
| 速度模式 | `DM_SPD_MODE` | `+0x200` | 纯速度控制 |
| PSI 模式 | `DM_PSI_MODE` | `+0x300` | 位置-速度-电流联合控制 |

## 架构设计

### 数据结构

```c
typedef enum {
    DM_NO_ERROR               = 0x00,
    OVERVOLTAGE_ERROR         = 0x08,
    UNDERVOLTAGE_ERROR        = 0x09,
    OVERCURRENT_ERROR         = 0x0A,
    MOS_OVERTEMP_ERROR        = 0x0B,
    MOTOR_COIL_OVERTEMP_ERROR = 0x0C,
    COMMUNICATION_LOST_ERROR  = 0x0D,
    OVERLOAD_ERROR            = 0x0E,
} DMMotorError_t;

typedef enum {
    DM_CMD_MOTOR_START   = 0xFC,
    DM_CMD_MOTOR_STOP    = 0xFD,
    DM_CMD_ZERO_POSITION = 0xFE,
    DM_CMD_CLEAR_ERROR   = 0xFB,
} DMMotor_Mode_e;

/* 达妙电机参数配置 */
typedef struct
{
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
} DM_Motor_Params_t;

typedef struct
{
    uint8_t        id;                      /* 电机ID */
    uint8_t        state;                   /* 电机状态 */
    float          last_single_round_angle; /* 上一次单圈角度(rad) */
    float          torque;                  /* 输出力矩(Nm) */
    float          T_Mos;                   /* MOS 温度 (℃) */
    float          T_Rotor;                 /* 线圈温度 (℃) */
    int32_t        total_round;             /* 总圈数 */
    DMMotorError_t Error_Code;              /* 错误码 */
} DM_Motor_Measure_s;

typedef struct {
    Motor_Base               base;    // 公共基类 (首字段)
    DM_Motor_Measure_s       measure; // 达妙测量数据
    uint32_t                 mode_type; // 控制模式 (DM_MIT_MODE / DM_POS_MODE / ...)
    const DM_Motor_Params_t *params;  // 电机参数配置指针 (参数映射表)
} DM_Motor_t;
```

### 参数映射范围

CAN 帧中整数值与物理量的映射（`dm_motor_params[]` 按电机型号查表）：

| 参数 | 物理范围 | 位宽 | 说明 |
|------|---------|------|-----|
| 位置 | -12.5 ~ +12.5 rad | 16 bit | `p_min` / `p_max` |
| 速度 | -30.0 ~ +30.0 rad/s | 12 bit | `v_min` / `v_max` |
| Kp | 0 ~ 500 | 12 bit | `kp_min` / `kp_max` |
| Kd | 0 ~ 5 | 12 bit | `kd_min` / `kd_max` |
| 扭矩 | -10.0 ~ +10.0 Nm | 12 bit | `t_min` / `t_max` |

## 控制流程

```
Motor_DM_Init(config, DM_Mode_type)
  ├── BSP_MEM_ALLOC (DM_Motor_t)
  ├── Master ID 校验 (rx_id 必须大于 tx_id)
  ├── BSP_CAN_Device_Init()          ── 注册 CAN 设备 + Filter
  ├── PIDInit() / LQRInit()          ── 初始化控制器
  ├── Module_Offline_register()       ── 离线检测
  ├── Motor_DM_Cmd(CLEAR_ERROR)      ── 清除错误
  ├── Motor_DM_Cmd(MOTOR_START)      ── 启动电机
  ├── base.Apply = dm_apply
  └── Motor_Register()               ── 注册到全局链表

每周期调度:
Motor_ControlAll()
  └── 统一控制计算                    ── 阶段1 (基类): 离线/禁用→ZeroState, 开环→跳过, 闭环→Motor_CalculateTorque

Motor_ApplyAll()
  └── dm_apply()                     ── 阶段2: 离线/禁用发零值帧, 否则按 mode_type 分发:
        ├── DM_MIT_MODE  → mit_ctrl(pos, vel, kp, kd, output_torque)
        ├── DM_POS_MODE  → pos_speed_ctrl(ref, PI)
        ├── DM_SPD_MODE  → speed_ctrl(ref)
        └── DM_PSI_MODE  → psi_ctrl(0, 0, ref)
```

### 各模式发包行为

| 模式 | 调用函数 | 发送内容 | len |
|------|---------|---------|-----|
| MIT | `mit_ctrl()` | pos + vel + kp + kd + torque (MIT 协议编码) | 8 |
| POS | `pos_speed_ctrl()` | 浮点 pos (4B) + 浮点 vel (4B) | 8 |
| SPD | `speed_ctrl()` | 浮点 vel (4B) | 4 |
| PSI | `psi_ctrl()` | 浮点 pos (4B) + uint16 vel×100 (2B) + uint16 cur×10000 (2B) | 8 |

## API 接口说明

```c
DM_Motor_t *Motor_DM_Init(Motor_Init_Config_s *config, uint32_t DM_Mode_type);
```

```c
void Motor_DM_Cmd(DM_Motor_t *motor, DMMotor_Mode_e cmd);
```

## 错误码

| 错误码 | 枚举 | 说明 |
|--------|------|------|
| 0x00 | `DM_NO_ERROR` | 正常 |
| 0x08 | `OVERVOLTAGE_ERROR` | 过压 |
| 0x09 | `UNDERVOLTAGE_ERROR` | 欠压 |
| 0x0A | `OVERCURRENT_ERROR` | 过流 |
| 0x0B | `MOS_OVERTEMP_ERROR` | MOS 过温 |
| 0x0C | `MOTOR_COIL_OVERTEMP_ERROR` | 线圈过温 |
| 0x0D | `COMMUNICATION_LOST_ERROR` | 通信丢失 |
| 0x0E | `OVERLOAD_ERROR` | 过载 |

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-11 | 初始版本 |
| v1.1 | 2026-05-18 | fix:掉线失能问题，增加前馈力矩补偿 |
| v1.2 | 2026-05-23 | fix:增加外部指针检查，取消延时, 并增加id检查 |
| v2.0 | 2026-08-07 | 适配新架构: 控制计算上移基类 + 统一运行 API + 两阶段调度 |
