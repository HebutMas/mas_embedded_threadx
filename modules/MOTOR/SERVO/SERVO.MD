# 舵机模块文档

## 概述

PWM 舵机驱动模块，通过 PWM 脉宽控制标准舵机角度。舵机为**开环电机**（`loop_type` 在 Init 时强制设为 `OPEN_LOOP`）
## 架构设计

### 数据结构

```c
typedef struct {
    Motor_Base  base;              // 公共基类 (首字段)
    PWM_Device *pwm_dev;           // PWM 设备句柄
    uint16_t    min_pulse;         // 最小脉宽 (对应最小角度)
    uint16_t    max_pulse;         // 最大脉宽 (对应最大角度)
    float       min_angle_deg;     // 最小角度 (度)
    float       max_angle_deg;     // 最大角度 (度)
} Servo_Motor_t;
```

### 角度-脉宽映射

```
ratio  = (target_angle - min_angle) / (max_angle - min_angle)
pulse  = min_pulse + ratio * (max_pulse - min_pulse)
```

`ratio` 被限制在 `[0, 1]` 范围内，超出角度范围时饱和到边界脉宽。

---

## 控制流程

```
Motor_Servo_Init()
  ├── BSP_MEM_ALLOC (Servo_Motor_t)
  ├── loop_type 强制 = OPEN_LOOP      ── 开环电机, 基类跳过控制计算
  ├── BSP_PWM_Device_Init()          ── 注册 PWM 设备
  ├── BSP_PWM_Start()                ── 启动 PWM 输出
  ├── Module_Offline_register()       ── 离线检测
  ├── base.Apply = servo_apply
  └── Motor_Register()               ── 注册到全局链表

每周期调度:
Motor_ControlAll()
  └── 统一控制计算                    ── 阶段1 (基类): loop_type==OPEN_LOOP → 跳过

Motor_ApplyAll()
  └── servo_apply()                  ── 阶段2: 
        ├── if (!enableflag) → return (保持当前角度)
        └── pulse = angle_to_pulse(base->controller.ref)
        └── BSP_PWM_SetPulse(pwm_dev, pulse)
```

## PWM 配置

| 参数 | 说明 | 常见值 |
|------|------|--------|
| `min_pulse` | 最小角度脉宽 | 500 (0°) |
| `max_pulse` | 最大角度脉宽 | 2500 (180°) |
| `min_angle_deg` | 最小角度 (度) | 0.0 |
| `max_angle_deg` | 最大角度 (度) | 180.0 |


## API 接口说明

```c
Servo_Motor_t *Motor_Servo_Init(Motor_Init_Config_s *config,
                                uint16_t min_pulse, uint16_t max_pulse,
                                float min_angle, float max_angle);
```

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-11 | 初始版本 |
| v2.0 | 2026-08-07 | 适配新架构: 开环直通 + 统一运行 API + 两阶段调度 |
