# 电机模块框架文档

## 概述

电机模块是一个**面向对象风格**的电机驱动框架，基于继承和多态统一管理不同类型的电机（CAN 总线电机、UART 步进电机、PWM 舵机等）。所有电机通过全局链表注册，由一个独立的 ThreadX 任务周期性调度。

### 已支持的电机

| 模块 | 传输层 | 电机型号 | 控制方式 |
|------|--------|---------|---------|
| [DJI](DJI/DJI.MD) | CAN | M2006/M3508/GM6020 | PID / LQR + 电流环 |
| [DAMIAO](DAMIAO/DAMIAO.MD) | CAN | DM4310/DM6220 | PID / LQR + MIT/POS/SPD/PSI |
| [LK](LK/LK.MD) | CAN | MG8016 | PID / LQR + 转矩闭环 |
| [SERVO](SERVO/SERVO.MD) | PWM | 标准舵机 | 角度→脉宽 |
| [ZDT_STEPMOTOR](ZDT_STEPMOTOR/ZDT_STEPMOTOR.MD) | UART | ZDT 步进电机 | 速度/位置 UART 协议 |

## 架构设计

### 继承体系

```
Motor_Base (基类)                   ← motor_base.h
  ├── Motor_Base  *next             ← 指向下一个电机
  ├── Motor_Type_e type             ← 电机型号
  ├── Motor_Transport_e transport   ← CAN / UART / PWM
  ├── Motor_Info_s       info       ← 物理参数
  ├── Motor_Setting_s    setting    ← 闭环类型, 反转标志, 反馈源...
  ├── Motor_Controller_s controller ← PID / LQR 实例 + ref + output_torque
  ├── Motor_Measure_s    measure    ← 公共测量
  ├── Offline_Device    *offline_dev
  ├── void              *transport_dev  ← 底层设备句柄
  └── void (*Apply)(Motor_Base*)    ← 阶段2: 输出应用 (协议层实现, 唯一虚函数)

子类（继承 Motor_Base，base 必须是首字段）:
  ├── DJI_Motor_t      (CAN)     ← DJI motor
  ├── DM_Motor_t       (CAN)     ← DAMIAO motor
  ├── LK_Motor_t       (CAN)     ← 翎控 motor
  ├── Servo_Motor_t    (PWM)     ← Servo
  └── UART_Motor_t     (UART)    ← ZDT stepper
```

通过 `MOTOR_GET_DERIVED(base_ptr, derived_type)` 宏将基类指针还原为子类指针（利用首字段地址相同的特性）。

### 两阶段调度

电机驱动只需实现一个 `Apply` 函数指针，由 motor task 分两阶段统一调度：

```
motor_task (2ms):
  Motor_ControlAll()     控制计算 → controller.output_torque
  PowerControl_Update()  功率限制 → 覆写 output_torque 
  Motor_ApplyAll()       输出应用 → 传输层 (读取限后扭矩发帧)
```

### 控制算法层 (motor_algorithm.c)

控制计算独立于基类，`Motor_CalculateTorque(m)` 用 `setting.algorithm_type` switch 分派，增加新算法只需两步：

```c
/* 1. 写实现 */
static float motor_calc_adrc(Motor_Base *m) { /* ... */ }

/* 2. switch 加一个 case (motor_def.h 枚举加 CONTROL_ADRC) */
case CONTROL_ADRC:
    torque = motor_calc_adrc(m);
    break;
```

随后叠加前馈并限幅。PID 的位置/速度双环、反馈源选择、反转标志处理均封装在 `motor_calc_pid` 内，所有闭环电机共享。

## 核心数据结构

### Motor_Setting_s — 控制设置

```c
typedef struct {
    Closeloop_Type_e         loop_type;             // 闭环类型 (位掩码, OPEN_LOOP 为开环)
    Control_Algorithm_Type_e algorithm_type;        // PID / LQR
    uint8_t                  enableflag;            // 使能标志
    uint8_t                  motor_reverse_flag;    // 输出反转
    uint8_t                  feedback_reverse_flag; // 反馈反转
    uint8_t                  angle_feedback_source; // 0=电机, 1=外部
    uint8_t                  speed_feedback_source; // 0=电机, 1=外部
} Motor_Setting_s;
```

### Motor_Controller_s — 控制器

```c
typedef struct {
    const float *other_angle_feedback_ptr;  // 外部角度反馈指针
    const float *other_speed_feedback_ptr;  // 外部速度反馈指针
    PIDInstance  speed_PID;                 // 速度环 PID
    PIDInstance  angle_PID;                 // 位置环 PID
    LQRInstance  lqr;                       // LQR 控制器
    float        ref;                       // 当前参考值
    float        output;                    // 当前输出值 (协议层换算值)
    float        output_torque;             // 当前力矩输出 (Nm) — 功率控制操作对象
    float        feedforward_torque;        // 前馈力矩 (Nm)
} Motor_Controller_s;
```

### Motor_Measure_s — 公共测量数据

```c
typedef struct
{
    float speed_rad;          /* 角速度 (rad/s) */
    float single_round_angle; /* 单圈角度 (rad) */
    float total_angle;        /* 总角度 (rad)   */
    float torque_nm;          /* 当前力矩 (Nm)  */
} Motor_Measure_s;
```

## 初始化流程

```
用户代码:
  Motor_DJI_Init(&config)            ── 品牌 Init, 保留各自签名与独有参数
  ├── BSP_MEM_ALLOC (子类)
  ├── 填充 base 字段
  ├── BSP_xxx_Device_Init()          ── 注册传输层设备 → transport_dev
  ├── 设置接收回调 (rx_callback → 解析帧 → measure)
  ├── PIDInit() / LQRInit()
  ├── Module_Offline_register()
  ├── base.Apply   = xx_apply         ← 品牌实现 (发帧, 唯一虚函数)
  └── Motor_Register(&base)
```

## 如何添加一个新电机

以添加一个**"XX 品牌 CAN 电机"**为例，按以下步骤操作：

### 目录结构

```
modules/MOTOR/XX/
  ├── motor_xx.h
  └── motor_xx.c
```

### 定义子类结构 (`motor_xx.h`)

```c
typedef struct {
    Motor_Base   base;         // [必须首字段]
    XX_Measure_s measure;      // 扩展测量数据 (可选)
    // ... 子类专属字段
} XX_Motor_t;

XX_Motor_t *Motor_XX_Init(Motor_Init_Config_s *config);
```

### 实现接收回调 (`motor_xx.c`)

```c
static void xx_rx_callback(Can_Device *dev, const uint8_t *data, uint8_t len)
{
    XX_Motor_t *motor = (XX_Motor_t *)dev->user_arg;
    // 解析 data → base.measure.speed_rad / total_angle / torque_nm
    Module_Offline_device_update(motor->base.offline_dev);
}
```

### 实现控制函数 (`motor_xx.c`)

```c
/* 闭环电机: 读取 output_torque 构造帧 */
static void xx_apply(Motor_Base *base)
{
    XX_Motor_t *motor = MOTOR_GET_DERIVED(base, XX_Motor_t);
    uint8_t offline = (base->offline_dev && Module_Offline_get_device_status(base->offline_dev) == STATE_OFFLINE);

    if (offline || !base->setting.enableflag) { /* 发零值帧 */ return; }

    float torque = base->controller.output_torque;
    /* 扭矩 → 驱动指令 → BSP_CAN_SendMessage (或 DJI 式分组帧写槽位) */
}

/* 开环电机 (舵机/步进): loop_type 设为 OPEN_LOOP, 基类跳过控制计算, 直接使用 controller.ref */
static void xx_apply(Motor_Base *base)
{
    if (!base->setting.enableflag) return;
    /* base->controller.ref → 输出 */
}
```

### 实现 Init 函数

```c
XX_Motor_t *Motor_XX_Init(Motor_Init_Config_s *config)
{
    XX_Motor_t *motor = NULL;
    BSP_MEM_ALLOC_WAIT(motor, sizeof(XX_Motor_t), TX_NO_WAIT);
    if (!motor) return NULL;
    memset(motor, 0, sizeof(XX_Motor_t));

    motor->base.type      = config->motor_init_info.motor_type;
    motor->base.transport = MOTOR_TRANSPORT_CAN;  // 或 UART / PWM
    motor->base.info      = config->motor_init_info;
    motor->base.setting   = config->setting_init_config;

    void *dev = BSP_CAN_Device_Init(&config->transport_config.can);
    if (!dev) { BSP_MEM_FREE(motor); return NULL; }
    motor->base.transport_dev = dev;

    ((Can_Device *)dev)->rx_callback = xx_rx_callback;
    ((Can_Device *)dev)->user_arg    = motor;

    PIDInit(&motor->base.controller.speed_PID, &config->controller_init_config.speed_PID);
    PIDInit(&motor->base.controller.angle_PID, &config->controller_init_config.angle_PID);

    motor->base.offline_dev = Module_Offline_register(&config->offline_init_config);

    motor->base.Apply = xx_apply;    /* 阶段1 控制计算由 Motor_ControlAll 统一执行 */
    Motor_Register(&motor->base);
    return motor;
}
```

### 注册

1. `motor_def.h` 的 `Motor_Type_e` 枚举添加型号
2. `module_motor.h` 添加 `#include "motor_xx.h"`
3. `CMakeLists.txt` 的 `MODULE_MOTOR` 分支添加源文件与 include

## API 接口说明

```c
void Motor_Register(Motor_Base *motor);     
```
```c
void Motor_ControlAll(void);         
```
```c
void Motor_ApplyAll(void);          
```
```c
float Motor_CalculateTorque(Motor_Base *motor);
```
```c
void Motor_Start/Stop/SetRef/ChangeFeed/OuterLoop/SetForwardTorque/SetOutputTorque(Motor_Base*, ...);
```
```c
void Module_Motor_Init(void);                       
```

- 遗留：轴端统一后仍有待实车验证项：按输出轴域重标的内环 LQR/PID 增益与功率控制阈值

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-11 | 初始版本 |
| v1.1 | 2026-05-15 | 将内部指针改为const，增加前馈力矩补偿，fix：掉线失能问题 |
| v1.2 | 2026-05-21 | 使用单向链表，方便调试 |
| v2.0 | 2026-08-07 | 架构重构: 控制算法独立文件(PID/LQR switch 分派) + 两阶段调度 |
| v2.1 | 2026-09-05 | 统一轴端契约: base.measure 与扭矩参数一律为输出轴端; LK/DJI 反馈按 gear_ratio 折算到输出轴, 发送电流改用输出轴 Kt(配置 torque_constant 已 ×gear 配平, 电流指令不变); LK total_angle 改用 total_round 重建(编码器绝对零点)并修正单圈角; 新增 DM4340/DM8009/DM3507/DM3519 达妙参数; 上层(power_control/chassis_type/发射)去掉二次 ratio 折算并统一使用 Motor_Base 指针 |
