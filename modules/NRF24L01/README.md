# NRF24L01 模块

nRF24L01 2.4GHz 无线模块驱动，能力注册式收发框架，基于 HAL + ThreadX + BSP SPI。一份源码同时支持 f103_c8(F103) 与 dji_c(F407)，靠芯片宏条件编译自动切换引脚/分频/中断线。

## 功能特性

- **能力注册式**：`Register("名字", &变量, 类型)` 后自动收发，用户不用管发送线程
- **动态包长 (DPL)**：包长由注册总字节数决定，单包最多 32 字节（硬件限制）
- **Enhanced ShockBurst**：硬件自动 ACK + 自动重传 + CRC，丢包率低
- **8 种数据类型**：int8/16/32、uint8/16/32、float、double，小端序列化
- **IRQ 中断接收**：收到数据触发 EXTI 中断，信号量唤醒线程，低延迟
- **OFFLINE 集成**：收到数据自动喂心跳，超时触发离线报警
- **单线程架构**：一个线程同时处理定时发送和 IRQ 唤醒接收
- **跨芯片**：F103/F407 同一套源码，选对 Build 任务即可，射频参数两端一致时可互通

---

## 快速接入步骤（新板 / 新兵种）

1. **CubeMX 配置**：配好 SPI、CE/CSN/IRQ、NVIC 并重新生成（详见下文「CubeMX 配置」）。
2. **挂进构建（本框架三处，一般已预置）**：
   - `modules/CMakeLists.txt`：`if(MODULE_NRF24L01)` 段加入 `.c` 源文件与 include 目录；
   - `modules/module_init.c`：`#include "module_nrf24l01.h"` 并在 `MODULE_Init()` 里 `#if MODULE_NRF24L01  Module_NRF24L01_Init(); #endif`；
   - `apps/<robot>/robot.cmake`：把 NRF24L01 加入模块列表并设置射频参数/主从（见下）。
   ```cmake
   set(MODULES_SINGLE   OFFLINE NRF24L01)
   set(NRF24L01_RF_CHANNEL  2)   # 2402MHz，两端一致
   set(NRF24L01_RF_DATARATE 2)   # 2Mbps，两端一致
   set(NRF24L01_TX_ENABLE  1)    # 1=发送端, 0=纯接收端，两块板分别编译
   ```
3. **新芯片适配（F1/F4 已内置，可跳过）**：换其它芯片时，在 `module_nrf24l01.h` 硬件段照 F1/F4 加一个 `#if defined(STM32xxx)` 分支（6 个引脚宏 + SPI 分频宏 + EXTI 线宏），并在 `.c` Init 里补对应 GPIO 端口时钟使能。
4. **应用层注册变量**：初始化后调用 `Module_NRF24L01_Register(...)`（见「使用方法」），收发两端注册顺序/类型/个数必须一致。
5. **定主从、选板编译**：`NRF24L01_TX_ENABLE` 一块设 1、一块设 0，分别编译烧录。VS Code 里 `Ctrl+Shift+P → Tasks: Run Task`，选 **Build f103_c8** 或 **Build dji_c**——芯片宏（`STM32F103xB`/`STM32F407xx`）由板级 CMake 自动传入，**源码不用切来切去**。
6. **上电自检**：看开机 `Reg check` 是否为标准值（见「上电自检与故障排查」）。

---

## 硬件连接（F103 / F407）

SPI 三根线两板完全相同，差别只在 CE/CSN/IRQ：

| nRF24L01 | 信号方向 | f103_c8 (F103) | dji_c (F407) | 说明 |
|---|---|---|---|---|
| VCC | 电源 | **3.3V** | **3.3V** | **必须 3.3V，严禁 5V** |
| GND | 地 | GND | GND | 与主控共地 |
| SCK | MCU→模块 | PB13 | PB13 | SPI2 时钟 |
| MISO | 模块→MCU | PB14 | PB14 | 主入从出 |
| MOSI | MCU→模块 | PB15 | PB15 | 主出从入 |
| CSN | MCU→模块 | PB1 | **PB12** | SPI 片选（BSP 软件管理，默认高） |
| CE | MCU→模块 | PB0 | **PF0** | 收发使能，默认低 |
| IRQ | 模块→MCU | PA0 (EXTI0) | **PF1 (EXTI1)** | 中断输出，下降沿，建议必接 |

硬件接线注意：

- **SPI 同名脚直连**：MOSI→MOSI、MISO→MISO、SCK→SCK，不要像串口 TX/RX 那样交叉。
- nRF 发射瞬间电流较大，建议在模块 **VCC-GND 间并 100nF + 10µF 电容**，否则电源纹波大时易丢包、假死。
- IO 电平为 3.3V 逻辑；模块丝印脚序若为 2×4 排列以丝印名字为准对接。

---

## CubeMX 配置

### 为什么 SPI 必须分频

nRF24L01 的 SPI 接口**最高只支持 10MHz**，超过就会读写出错（寄存器全 00/全 FF、ACTIVATE 失败）。而 STM32 的 SPI 时钟来自 APB 总线，主频不同总线频率也不同：

| 板卡 | 系统主频 | SPI2 时钟源 | 可选分频与结果 | 选用 |
|---|---|---|---|---|
| f103_c8 | 72MHz | APB1 = **36MHz** | /4=9MHz，/2=18MHz(超) | **/4 = 9MHz** |
| dji_c | 168MHz | APB1 = **42MHz** | /8=5.25MHz，/4=10.5MHz(略超) | **/8 = 5.25MHz** |

原则：**选「结果 ≤10MHz 里最快」的那一档**。分频太大通信变慢，太小（超过 10MHz）模块识别不了。驱动 Init 里也会用 `NRF24L01_SPI_PRESCALER` 宏再强制设置一次，保证不依赖 CubeMX 的初始值。

### 1. SPI2 配置（两板一致）

- **Mode**: Full-Duplex Master
- **Hardware NSS Signal**: Disable（NSS 用软件，CSN 由普通 GPIO 控制）
- **Parameter Settings**:
  - Frame Format: Motorola
  - Data Size: 8 Bits
  - First Bit: **MSB First**
  - Clock Polarity (CPOL): **Low**
  - Clock Phase (CPHA): **1 Edge**（即 SPI 模式 0，nRF 强制要求）
  - NSS Signal Type: Software
  - Baud Rate Prescaler: **F103 选 4，F407 选 8**（理由见上表）
- **GPIO**: PB13=SCK、PB14=MISO、PB15=MOSI（复用推挽）

### 2. GPIO 配置（CE / CSN / IRQ）

| 信号 | f103_c8 | dji_c | GPIO 模式 | 初始电平 | 上下拉 |
|---|---|---|---|---|---|
| CE | PB0 | PF0 | GPIO_Output | **Low** | No pull |
| CSN | PB1 | PB12 | GPIO_Output | **High** | Pull-up |
| IRQ | PA0 | PF1 | **GPIO_EXTI，下降沿 (Falling edge)** | — | **Pull-up** |

输出速度都可设 High。IRQ 配成外部中断后还要在 NVIC 使能对应中断线。

### 3. NVIC 配置

- F103：使能 **EXTI line0 interrupt**；F407：使能 **EXTI line1 interrupt**。
- Preemption Priority = 5，Sub Priority = 0（驱动里也会再设一次）。

> CubeMX 会在 `stm32f1xx_it.c` / `stm32f4xx_it.c` 生成 `EXTIx_IRQHandler → HAL_GPIO_EXTI_IRQHandler`。模块通过 `BSP_GPIO_EXTI_Register` 注册回调，**不要自己再定义 `HAL_GPIO_EXTI_Callback`**，以免和其它模块冲突。

### 4. 时钟配置

- **f103_c8**：HSE 8MHz → PLL×9 → 72MHz；AHB 不分频；APB1 /2 → 36MHz（SPI2 输入时钟即 PCLK1=36MHz，/4=9MHz）；APB2 不分频 72MHz。
- **dji_c**：HSE → PLL → 168MHz；AHB /1；APB1 /4 → 42MHz（SPI2=42MHz，/8=5.25MHz）；APB2 /2 → 84MHz。

### 5. F407 特殊：CubeMX 重新生成后要删冲突中断

dji_c 用了 ThreadX 和 CherryUSB，而 CubeMX 不认识它们，每次重新生成都会在 `board/dji_c/Src/stm32f4xx_it.c` 写回三个空壳/PCD 中断，导致链接 `multiple definition`：

- `PendSV_Handler`、`SysTick_Handler`（应由 ThreadX 端口汇编提供）
- `OTG_FS_IRQHandler`（应由 CherryUSB 提供）

**每次 CubeMX 重新生成后，把这三个函数从 it.c 删掉**（文件里已留中文说明注释），再编译。f103_c8 无此问题。

---

## 使用方法

### 基本用法

```c
#include "module_nrf24l01.h"

/* 定义要传输的变量 */
float    motor_rpm = 0.0f;
int16_t  encoder   = 0;
uint8_t  mode      = 0;

/* 在初始化后注册（顺序很重要，收发两端必须一致） */
Module_NRF24L01_Register("motor_rpm", &motor_rpm, NRF24L01_TYPE_FLOAT);
Module_NRF24L01_Register("encoder",   &encoder,   NRF24L01_TYPE_INT16);
Module_NRF24L01_Register("mode",      &mode,      NRF24L01_TYPE_UINT8);

/* 之后自动收发：
 * - 你改变量值 → 下一个发送周期自动发出去
 * - 对端发过来 → CRC校验通过后自动写进你的变量
 */
```

### 数据类型枚举

| 枚举 | 类型 | 字节数 |
|------|------|--------|
| `NRF24L01_TYPE_INT8` | int8_t | 1 |
| `NRF24L01_TYPE_UINT8` | uint8_t | 1 |
| `NRF24L01_TYPE_INT16` | int16_t | 2 |
| `NRF24L01_TYPE_UINT16` | uint16_t | 2 |
| `NRF24L01_TYPE_INT32` | int32_t | 4 |
| `NRF24L01_TYPE_UINT32` | uint32_t | 4 |
| `NRF24L01_TYPE_FLOAT` | float | 4 |
| `NRF24L01_TYPE_DOUBLE` | double | 8 |

### 手动触发发送

一般不需要，线程自动定时发送。如需立即发送：

```c
Module_NRF24L01_TriggerTx();
```

### 查询在线状态

```c
if (Module_NRF24L01_GetStatus() == 0) {
    /* 在线 */
} else {
    /* 离线 */
}
```

## 可配置参数

在 `robot.cmake` 中覆盖（或 `module_config.h` 中定义），`.h` 里的 `#ifndef` 只作最后兜底：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `NRF24L01_TASK_STACK_SIZE` | 1024 | 线程栈大小 |
| `NRF24L01_TASK_PRIORITY` | 1 | 线程优先级（别太低，否则会被打印线程抢占导致收发卡顿） |
| `NRF24L01_TX_INTERVAL_MS` | 10 | 发送间隔（ms），默认 100Hz |
| `NRF24L01_TX_ENABLE` | 1 | 1=发送端，0=纯接收端（主从一发一收） |
| `NRF24L01_MAX_CAPS` | 16 | 最多注册数据项数 |
| `NRF24L01_OFFLINE_TIMEOUT_MS` | 500 | OFFLINE 心跳超时（ms） |
| `NRF24L01_RF_CHANNEL` | 2 | 射频通道：2400+N MHz |
| `NRF24L01_RF_DATARATE` | 2 | 空中速率：1=1Mbps, 2=2Mbps |
| `NRF24L01_RF_POWER` | 0 | 发射功率：0=0dBm, 1=-6, 2=-12, 3=-18 |
| `NRF24L01_RETR_COUNT` | 3 | 自动重传次数（0~15） |
| `NRF24L01_RETR_DELAY` | 0 | 重传间隔：0=250us, 1=500us... |

> **收发两端必须一致的参数**：`RF_CHANNEL`、`RF_DATARATE`、固定 5 字节地址、CRC/EN_AA/地址宽度（驱动写死）、以及注册顺序与类型。

## 架构说明

```
应用层: Module_NRF24L01_Register("名字", &var, TYPE)  → 变量自动收发
    │
框架层: 能力注册列表 → 序列化(小端) → 写TX FIFO → CE脉冲触发
    │                    ↑ 接收: IRQ中断 → BSP回调 → 信号量 → 线程读RX FIFO → 反序列化写回
协议层: Enhanced ShockBurst(自动ACK+重传) + 动态包长(DPL) + 硬件CRC
    │
寄存器层: SPI指令封装(读/写寄存器, 读/写载荷, 清FIFO)
    │
底层: BSP SPI 驱动(硬件SPI2, 线程安全+互斥锁) + GPIO(CE/CSN) + EXTI(IRQ)
```

### 收发流程

**发送**：线程到点 → 遍历注册列表序列化 → 清 TX FIFO → 写载荷 → 切 TX 模式（CE 高脉冲）→ 硬件自动 ACK+重传 → 切回 RX 模式

**接收**：模块常驻 RX → 收到数据 → IRQ 拉低 → EXTI 中断 → BSP 回调置信号量 → 线程被唤醒 → 读 STATUS → 读动态包长 → 读载荷 → 长度匹配则反序列化写回变量 → OFFLINE 心跳 → 清中断标志

---

## 注意事项（Checklist）

**硬件**
1. VCC 只接 **3.3V，严禁 5V**；必须共地；建议并 100nF+10µF 电容。
2. SPI 同名脚直连（MOSI/MISO/SCK 不交叉）；IRQ 建议必接（中断驱动，不接会退化为低效轮询）。

**SPI / 时序**
3. SPI 必须**模式 0（CPOL=0/CPHA=0）、MSB First、8bit、软件 NSS**，SCK ≤10MHz（F103 用 /4=9M，F407 用 /8=5.25M）。
4. CE 默认低、CSN 默认高（上拉）、IRQ 下降沿上拉。

**协议一致性（两端必须相同，否则收不到）**
5. 射频参数一致：信道、5 字节地址、空中速率、CRC、EN_AA、地址宽度；别只给一端烧旧固件。
6. 两端 **Register 顺序/类型/个数完全一致**，否则解码错位；单包载荷 **≤32 字节**，超了注册返回 -1。
7. **一发一收**：`NRF24L01_TX_ENABLE` 一块 1 一块 0，分别编译烧录；不要两块同时发（半双工，会空中冲突）。

**软件**
8. nRF 线程优先级别太低（曾因与打印线程同级被抢占，出现"通一会断"，默认已设为 1）。
9. 模块通过 `BSP_GPIO_EXTI_Register` 统一注册 EXTI 回调，**不要再自定义 `HAL_GPIO_EXTI_Callback`**。
10. F407 每次 CubeMX 重新生成后，删掉 it.c 里 `PendSV_Handler`/`SysTick_Handler`/`OTG_FS_IRQHandler`（见「CubeMX 配置 5」）。

## 上电自检与故障排查

上电会打印一行寄存器自检，**标准值**为：

```
Reg check: CONFIG=0x0F EN_AA=0x01 AW=0x03 RF_CH=2 FEATURE=0x06 DYNPD=0x01
```

| 现象 | 含义 / 排查方向 |
|---|---|
| 值与标准一致 | SPI、CE/CSN、ACTIVATE/DPL 全部正常 |
| 全 `0x00` | SPI 没通，查 SCK / MOSI / CSN 接线与模式0 |
| 全 `0xFF` | 查 MISO（总线一直被拉高/没接） |
| `FEATURE=0` / ACTIVATE failed | SPI 通信异常，DPL 没激活，查接线和供电 |
| TX 端 `ok=0、fail 一直涨`（no ACK） | 发送端自身正常、**对端没应答**：接收板没上电/角色不是 RX/接收端 CE 没拉高/两端参数不一致 |
| RX 端一直 0 + OFFLINE | 没收到包，查发送端角色、信道地址、距离与供电 |
| 开机短暂 ONLINE 后才 OFFLINE | 正常：初始默认在线，超过 `OFFLINE_TIMEOUT` 没收到包才判离线 |

> `TX failed(no ACK)` 每累计 50 次失败才打印一条，**不是"偶发失败"，看到它在涨就是这段时间一直没收到 ACK**；调通后 `ok` 持续增长、该警告自然消失。

## 文件结构

```
modules/NRF24L01/
├── nrf24l01_reg.h      # 寄存器地址、SPI指令码、位定义
├── module_nrf24l01.h   # 对外接口、可配置参数、数据类型枚举、F1/F4硬件条件编译
├── module_nrf24l01.c   # 完整实现
└── README.md            # 本文件
```
