# NAV 导航通信模块文档

## 概述

NAV 模块通过 USB CDC ACM 的 **串口二（port1）**接收导航速度、发送裁判系统数据。
从 PCCOMM 中拆出原有导航分支，独立维护接收线程与双缓冲。导航下发包含 `vx`、`vy`、`small_yaw_ref` 和 `follow_gimbal`。NAV 模块将后两项保存到接收快照，应用层如何使用由 `nav_auto_func` 决定；下位机在自动模式且导航在线、速度有效时固定以 `+4 rad/s` 自转。

**端口分配:**

```
┌──────────────┐   USB CDC ACM
│  NAV         │ ─ port1 ─ 导航  上位机→板 0xB5 / 板→上位机 0x5B
└──────────────┘
```

H723 默认提供两路 CDC；单 CDC 板没有 port1，NAV 初始化跳过离线设备注册和接收线程。

## 架构设计

### 数据流

```
USB OUT → cdc_out_ep_buffer[1] → 接收 FIFO → RX 事件
                                               │
                                               ▼
                                          nav_thread
                                               │
                                     帧头 / CRC16 检查
                                               │
                                      更新导航双缓冲
                                               │
                                               ▼
                                  Module_Nav_Receive() → app
```

### 数据写入路径

```
app → Module_Nav_Send() → 帧头 + NavSendPacket + CRC16
                         → cdc_acm_send(1, ...) → USB IN
```

发送仍在调用者线程中执行；NAV 线程负责接收。
接收线程沿用 `TX_NO_WAIT` 非阻塞读取，无数据时休眠 1 tick。
每批处理后让出同优先级线程，避免拆分后持续来包时占住 CPU。

### 内部结构

| 字段 / 资源 | 类型 | 说明 |
|-------------|------|------|
| `nav_packet[2]` | `NavReceivePacket[2]` | 导航接收双缓冲 |
| `nav_idx` | `uint32_t` | 当前写入槽位，写完后切换 |
| `nav_snapshot` | `NavReceivePacket` | 返回给 app 的最近有效包快照 |
| `nav_thread_stack` | `uint8_t[1024]` | 接收线程栈 |
| `offline_dev` | `Offline_Device *` | minipc_nav 离线检测对象 |

## 帧格式

统一为定长帧：`[header(1)] [payload(N)] [crc16(2, 小端)]`。
使用现有 `crc_rm` 校验函数。

| 方向 | 帧头 | payload | 总长度 |
|------|------|---------|--------|
| 上位机 → 板 | `0xB5` | `NavReceivePacket`：vx、vy、small_yaw_ref、follow_gimbal | 16 字节 |
| 板 → 上位机 | `0x5B` | `NavSendPacket`：robot_color、game_progress、current_hp、bullet_allow、shooter_heat_pct | 11 字节 |

结构体按 1 字节对齐，float 沿用 STM32 的 32 位小端布局，速度单位 m/s。
坐标转换仍由应用处理，裁判数据字段顺序和类型不变。

下发字段偏移从帧头起算：

| 偏移 | 字段 | 类型 / 含义 |
|------|------|-------------|
| 0 | header | `0xB5` |
| 1～4 | vx | float32，m/s |
| 5～8 | vy | float32，m/s |
| 9～12 | small_yaw_ref | float32，deg，`rad_to_deg(wrap(入口路径航向 - 当前 LIO yaw))`，范围 [-180°, 180°]，逆时针为正 |
| 13 | follow_gimbal | uint8，0=不请求跟随，1=请求进入/保持跟随云台模式 |
| 14～15 | crc16 | 小端，覆盖前 14 字节；初值 0xFFFF，反射多项式 0x8408 |

上位机从坡道/隧道/起伏路的 `PREPARING` 阶段提前发送对齐请求，持续至释放点，每个控制周期更新相对 LIO 的偏角。无请求时 ref=0、follow_gimbal=0；导航输入/安全门失效或桥接指令超过 300 ms 时清除请求。`follow_gimbal` 用于请求进入/保持跟随云台模式，特殊地形准备窗至释放点为 1，其他情况为 0；它不是导航在线状态或区域 mode。当前工作区的应用层已用该标志请求跟随云台模式，并将 ref 交给 `gimbal_gyro_mode` 的角度制 yaw 输入；本次单位调整保留这些已有控制改动。

该 ref 不是下位机 INS 的绝对角，也不是每次收到后累加的增量。LIO 固定在大 yaw 上，LIO 相对偏角与小 yaw 的 INS 绝对目标角之间仍须核对大小 yaw 相对角和安装零点。当前 `gimbal_gyro_mode` 将输入 yaw（deg）乘 `DEGREE_2_RAD` 后加 `auto_yaw_offset` 再调用 `Motor_SetRef`；上位机和接收包中的 ref 均为角度制，不可重复进行 rad→deg 转换。本次仅统一 ref 单位，不修改已有控制算法。

下发包不含 `w` 或 `nav_state`，导航在线状态由有效帧超时检测决定。新格式与旧版 11/12 字节帧以及含 w/nav_state 的旧 16 字节帧均不兼容；即使长度同为 16 字节，字段含义也不同，上下位机必须同步更新。

哨兵沿用现有 `vx`、`vy` 符号与坐标转换。自动模式的角速度由云台板内部生成，不来自导航包；板间角速度编码仍为 0.1 rad/s，`+4 rad/s` 编码为 40，底盘板按相同比例恢复。

## API 接口说明

```c
int Module_Nav_Init(void);
```

由 `MODULE_Init` 在 OFFLINE 初始化后调用一次，不重复初始化 USB。
返回 0 成功或单 CDC 板跳过；-1 表示离线设备注册或线程创建失败。

```c
void Module_Nav_Send(const NavSendPacket *packet, uint32_t timeout);
```

通过 CDC1 发送裁判数据；`timeout` 单位为 ThreadX tick，返回不代表 USB 传输完成。
单 CDC 板调用时由驱动拒绝不存在的通道。

```c
NavReceivePacket *Module_Nav_Receive(void);
```

复制最近有效包到快照并返回指针，指针有效至下次调用；尚无有效帧时为零值。

```c
uint8_t Module_Nav_Get_offline_state(void);
```

只检查导航通信，设备名为 `minipc_nav`；收到帧头和 CRC16 校验通过的导航帧时更新心跳。
默认超过 100 ms 未收到有效帧后，由 OFFLINE 检测任务标记离线；恢复有效帧后，在下一次检测时恢复在线。
返回 `STATE_ONLINE`（0）或 `STATE_OFFLINE`（1）；单 CDC 板未注册设备时返回离线。

### 宏参数

默认值在 `modules/module_config.cmake` 中配置。

| 宏 | 默认 | 说明 |
|----|------|------|
| `NAV_TASK_STACK_SIZE` | 1024 | 线程栈大小 |
| `NAV_TASK_PRIORITY` | 10 | 接收线程优先级 |
| `NAV_OFFLINE_ENABLE` | 1 | 导航离线检测开关 |

## 使用示例

应用 `MODULES_*` 中加入 `OFFLINE NAV`，系统统一完成初始化后，在控制任务中调用：

```c
#include "module_nav.h"
#include "module_offline.h"
#include "tx_api.h"

NavSendPacket send_packet = {0};
Module_Nav_Send(&send_packet, TX_NO_WAIT);
NavReceivePacket *nav_packet = Module_Nav_Receive();
uint8_t nav_offline = Module_Nav_Get_offline_state();
```

## 内部实现细节

接收线程每次读取最多 64 字节，扫描帧头并检查 CRC16。
找到合法帧后使用 `memcpy` 写入 `nav_packet[nav_idx]`，切换索引，并更新 `minipc_nav` 心跳。
getter 从上次完成的槽位复制快照；每批至多取第一个合法帧。

## 注意事项

1. **端口数量** — 仅有 CDC1 时启动导航线程，单 CDC 板保持无导航数据的行为。
2. **接收使用** — 按一个控制任务读取快照使用，不保留跨次半帧。
3. **状态判断** — 下发包没有导航状态字段，离线接口表示有效帧是否超时；离线时接收快照保留最近有效值，应用必须结合离线状态使用。
4. **应用控制** — 每拍按 `RemoteControlSet` → `nav_auto_func` → `gimbal_auto_func` 调用。NAV 不接管手动/停止模式；仅在导航在线、快照非空、`vx`/`vy`/`small_yaw_ref` 均为有限值、角度在 [-180°, 180°] 且 `follow_gimbal` 为 0/1 时应用命令。有效跟随请求选择 `chassis_follow_gimbal_yaw` 和 `gimbal_gyro_nav_mode`，平移继续使用 NAV；随后视觉处理函数屏蔽视觉目标/开火建议，并强制拨盘 `load_stop`，包括遥控器预先设置的发射命令。有效普通模式固定 `wz=+4 rad/s`，零平移也保持自转。导航离线、快照为空或数据无效时，清零三路速度并切回 `chassis_automode` 退出跟随，云台按现有行为恢复 `gimbal_auto_mode`；无效 NAV 角度不会写入云台命令。视觉离线不阻止有效导航处理。

## 底盘状态回传约定（待实现）

当前 `0x5B` 上行仍只有裁判数据，不能作为底盘状态或控制 tick；尚未回传相对角度、实测角速度、采样时间或控制有效状态。

后续状态至少需要：云台相对底盘角度 `δ`（rad）、实测底盘 `w`（rad/s）、各反馈的采样时间，以及反馈有效和自动控制有效状态。已确认 LIO 固定在大 yaw 上，因此 `δ` 取大 yaw 相对底盘的角度，不叠加小 yaw；可以复用当前大 yaw 编码器及 `YAW_CHASSIS_ALIGN_ECD` 对齐零点。实测 `w` 的反馈来源仍需确定。

统一定义 `δ = wrap(yaw_gimbal - yaw_chassis)` 后，上位机按 `yaw_chassis = wrap(yaw_LIO - δ)` 计算底盘朝向。编码器方向、安装零点与 LIO 外参需要实机核对。独立 INS 的启动 yaw 没有与 LIO 对齐，不能直接替代底盘 yaw。

时间戳必须来自反馈采样并说明时钟域、单位、回绕与主机时钟映射；仅标记 USB 发送或主机收包时刻不能保证姿态对齐。两板各自的运行时间也不能直接相减。上位机应按时间插值匹配 LIO 和相对角度，反馈过期或无效时停止自动控制。

稳定定频回传并验证后，上位机可用新状态触发平移控制 tick；当前导航下发含平移速度和小 yaw 对齐请求，底盘角速度由下位机生成。小 yaw 对齐字段由现有应用层读取，LIO/INS 零点与安装外参尚需实车核对；底盘特殊地形朝向闭环仍需另行明确旋转控制接口。状态断流停车由独立看门狗执行，不能依赖状态回调触发。下位机保留独立 NAV 超时停车和板间通信超时失能。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.6 | 2026-10-17 | NAV 先校验数据再切换模式，无效命令退出底盘跟随；道路对齐期间屏蔽视觉控制和拨盘发射 |
| v1.5 | 2026-10-17 | small_yaw_ref 改用角度制 deg、范围 [-180°, 180°]；布局/CRC/follow_gimbal 不变，保留工作区已有跟随控制改动 |
| v1.4 | 2026-10-17 | 下发扩为 16 字节：vx、vy、相对 LIO 的 small_yaw_ref 和 follow_gimbal 跟随模式请求标志；下位机仅接收新字段，尚未用于云台控制 |
| v1.3 | 2026-10-17 | 导航下发缩为 11 字节，仅保留 vx、vy；移除 w、nav_state，自动模式由下位机固定 +4 rad/s 自转，导航离线或速度无效时停车 |
| v1.2 | 2026-10-13 | 导航下发增加 w，自动模式执行上位机角速度，无效或离线时三路速度清零 |
| v1.1 | 2026-10-10 | 新增 minipc_nav 离线检测及状态查询接口，默认超时 100 ms |
| v1.0 | 2026-10-10 | 从 PCCOMM 拆出导航通信，使用独立接收线程，移除回环测试 |
