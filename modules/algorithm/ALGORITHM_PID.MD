
# 算法层 PID 模块文档

## 1. 概述

STM32H7 和 STM32F4 系列通用的高级 PID 算法处理模块。该模块在基础位置式 PID 的基础上，集成了多种优化环节（如梯形积分、微分先行、变速积分等），并内置电机堵转保护机制。

---

## 2. 架构设计

### 2.1 数据结构

#### PID_Improve_e - PID 优化环节枚举

通过按位或（`|`）开启不同的优化功能，支持组合使用。

```c
typedef enum {
    PID_IMPROVE_NONE = 0x00,
    PID_Trapezoid_Intergral = 0x01,       // 梯形积分
    PID_ChangingIntegrationRate = 0x02,  // 变速积分
    PID_Derivative_On_Measurement = 0x04,// 微分先行
    PID_DerivativeFilter = 0x08,         // 微分滤波
    PID_Integral_Limit = 0x10,           // 积分限幅
    PID_OutputFilter = 0x20,             // 输出滤波
    PID_ErrorHandle = 0x40               // 错误处理（堵转检测）
} PID_Improve_e;

```

#### PID_Init_Config_s - PID 初始化配置

```c
typedef struct {
    float Kp, Ki, Kd;           // PID 增益
    float MaxOut;               // 输出限幅
    float IntegralLimit;        // 积分限幅
    float DeadBand;             // 死区阈值
    uint32_t Improve;           // 启用的优化环节位掩码
    float CoefA, CoefB;         // 变速积分系数（误差在区间 [B, A+B] 内积分线性衰减）
    float Output_LPF_RC;        // 输出滤波截止频率相关常数
    float Derivative_LPF_RC;    // 微分滤波截止频率相关常数
} PID_Init_Config_s;

```

#### PIDInstance - PID 设备实例

```c
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    // improve parameter
    PID_Improvement_e Improve;
    float             IntegralLimit; // 积分限幅
    float             CoefA;         // 变速积分 For Changing Integral
    float             CoefB;         // 变速积分 ITerm = Err*((A-fabsf(err)+B)/A)  when B<|err|<A+B
    float             Output_LPF_RC; // 输出滤波器 RC = 1/omegac
    float             Derivative_LPF_RC; // 微分滤波器系数
  
    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t DWT_CNT;
    float    dt;

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

```

### 2.2 代码框架

```c
内部辅助函数层（static）：
├─ f_Trapezoid_Intergral()         // 采用 (Err + Last_Err)/2 进行积分
├─ f_Changing_Integration_Rate()   // 根据误差大小动态调整 Ki
├─ f_Integral_Limit()              // 积分抗饱和（Anti-Windup）
├─ f_Derivative_On_Measurement()   // 微分项仅取决于测量值变化
├─ f_Derivative_Filter()           // 对微分项进行一阶低通滤波
├─ f_Output_Filter()               // 对总输出进行一阶低通滤波
└─ f_PID_ErrorHandle()             // 持续大偏差检测（堵转保护）

公开 API 层：
├─ PIDInit()                       // 实例初始化与参数拷贝
└─ PIDCalculate()                  // 执行完整控制循环并返回输出

```

---

## 3. 控制原理

本模块基于**位置式 PID** 实现。通过 `Improve` 配置，可将标准公式扩展为更鲁棒的形式。

* 微分先行：将对误差的微分替换为对实际值的微分。
* 变速积分：根据误差的大小调整积分的速度。
* 梯形积分：将矩形积分改为梯形积分。
* 微分滤波：给微分项加入一阶惯性单元。
* 积分限幅：限制积分的幅度，防止积分深度饱和。
* 输出滤波：给输出项加入一阶惯性单元
---

## 4. API 接口说明
```c
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

```
```c
float PIDCalculate(PIDInstance *pid, float measure, float ref);

```
---

## 5. 注意事项

- 周期调用：必须确保 PIDCalculate() 被周期性调用，持续计算输出。 
- 依赖模块：本模块依赖 BSP_DWT (获取时间戳)。 
- 变速积分系数：`CoefA` 必须为正数，代表积分衰减的区间长度；`CoefB` 为开始衰减的误差阈值。
- 浮点绝对值用 `fabsf`，不要用 `abs`

---
## 6. 常见问题排查
- 问题：pid输出为0 
    - 排查：`PIDInstance`类型的实例中，`ERRORHandler`是否为`PID_ERROR_NONE`
- 问题：输出跳到极大值或 NaN
    - 排查：`Iout` 是否被 NaN 污染（微分在 `dt=0` 时除零），或电机反馈本身是 NaN/Inf

---

## 6. 使用示例

```c
PIDInstance speed_pid;
// 1. 定义实例与配置
PID_Init_Config_s config = {
        .Kp = 20.0f, .Ki = 0.5f, .Kd = 0.1f,
        .MaxOut = 10000.0f,
        .IntegralLimit = 4000.0f,
        .DeadBand = 0.1f,
        // 开启积分限幅、微分先行和堵转保护
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ErrorHandle
    };
// 2. 初始化
PIDInit(&speed_pid, &config);
// 3. 计算输出
float output = PIDCalculate(&speed_pid, fb_speed, set_speed);
```
