# DJI 电机模块文档

## 概述

DJI 电机驱动模块，通过 CAN 总线控制 RoboMaster 系列电机。

## 架构设计

### 数据结构

```c
typedef struct
{
    uint16_t last_ecd;     /* 上一次编码器值 */
    uint16_t ecd;          /* 0-8191 */
    float    speed_rpm;    /* 角速度 (RPM) */
    int16_t  real_current; /* 实际电流 (A) */
    uint8_t  temperature;  /* 温度 (℃) */
    int32_t  total_round;  /* 总圈数 */
} DJI_Measure_s;

typedef struct {
    Motor_Base    base;         // 公共基类 (首字段)
    DJI_Measure_s measure;      // DJI 测量数据
    uint8_t       sender_group; // CAN 发送分组编号
    uint8_t       message_num;  // 组内序号 (0-3)
} DJI_Motor_t;
```

### CAN 发送分组 (sender_assignment)

每个 CAN 总线预分配 5 个发送槽：

| 分组 | CAN 总线 | CAN ID | 用途 |
|------|---------|--------|------|
| 0/1 | CAN1 | 0x1FF / 0x200 | M2006/M3508 (ID 1-8) |
| 2/3/4 | CAN1 | 0x2FF / 0x1FE / 0x2FE | GM6020 (ID 1-8) |
| 5/6 | CAN2 | 0x1FF / 0x200 | M2006/M3508 |
| 7/8/9 | CAN2 | 0x2FF / 0x1FE / 0x2FE | GM6020 |
| 10-14 | CAN3 | (同上) | H723 独有 |

## 控制流程

```
Motor_DJI_Init()
  ├── BSP_MEM_ALLOC (DJI_Motor_t)
  ├── MotorSenderGrouping()          ── 分配 CAN 发送槽
  ├── BSP_CAN_Device_Init()          ── 注册 CAN 设备 + Filter
  ├── PIDInit() / LQRInit()          ── 初始化控制器
  ├── Module_Offline_register()       ── 离线检测
  └── Motor_Register()               ── 注册到全局链表

每周期调度:
Motor_ControlAll()
  └── 统一控制计算                    ── 阶段1 (基类): 离线/禁用→ZeroState, 开环→跳过,
        └── Motor_CalculateTorque()     ── 扭矩 = PID/LQR(measure, ref) + 前馈 + 限幅

Motor_ApplyAll()
  └── dji_apply()                      ── 阶段2: 扭矩 → 电流整型
        ├── 离线/禁用 → 槽位写零
        └── 填充 sender_assignment[group].data[num*2, num*2+1]
        └── dji_batch_send()            ── 组内计数, 最后一个电机发送整帧
              └── BSP_CAN_SendMessage(&sender_assignment[group])
```

## API 接口说明

```c
DJI_Motor_t *Motor_DJI_Init(Motor_Init_Config_s *config);
```

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-11 | 初始版本|
| v1.1 | 2026-05-18 | fix:掉线失能问题，增加前馈力矩补偿 |
| v1.2 | 2026-05-23 | fix:增加外部指针检查 |
| v2.0 | 2026-08-07 | 适配新架构: 控制计算上移基类 + 统一运行 API + 组内计数发送 |
