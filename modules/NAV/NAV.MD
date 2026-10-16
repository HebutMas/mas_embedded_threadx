# NAV 导航通信模块文档

## 概述

NAV 模块通过 USB CDC ACM 的 **串口二（port1）**接收导航速度、发送裁判系统数据。
从 PCCOMM 中拆出原有导航分支，独立维护接收线程与双缓冲。下发角速度由上位机统一生成，下位机自动模式仅执行收到的角速度。

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
| 上位机 → 板 | `0xB5` | `NavReceivePacket`：vx、vy、w、nav_state | 16 字节 |
| 板 → 上位机 | `0x5B` | `NavSendPacket`：robot_color、game_progress、current_hp、bullet_allow、shooter_heat_pct | 11 字节 |

结构体按 1 字节对齐，float 沿用 STM32 的 32 位小端布局，速度单位 m/s。
坐标转换仍由应用处理，裁判数据字段顺序和类型不变。

下发字段偏移从帧头起算：`vx` 在 1、`vy` 在 5、`w` 在 9，均为 float32；`nav_state` 在 13，CRC16 小端位于 14～15。`w` 单位 rad/s，底盘逆时针为正。旧版 12 字节导航帧不再兼容，上位机必须同步更新。

哨兵沿用现有 `vx`、`vy` 符号与坐标转换。`w` 不做平移坐标转换；云台板将其限幅到 ±12.7 rad/s 后按最近值量化为 0.1 rad/s，底盘板按相同比例恢复，不再乘以线速度上限。小于 0.05 rad/s 的修正会量化为零；如果对正控制需要更细分辨率，应调整板间角速度编码。

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
3. **状态判断** — `nav_state` 表示报文中的导航状态，离线接口表示有效帧是否超时；离线时接收快照保留最近有效值，应用按需判断。
4. **应用控制** — 哨兵控制任务独立调用 `nav_auto_func`；底盘自动模式下，仅在导航在线、快照非空、`nav_state == 1` 且 `vx`、`vy`、`w` 均为有限值时采用导航速度，否则将三路速度清零。视觉离线不阻止导航处理，非自动模式保留遥控器控制。自动模式不叠加内部自转，上位机需要自转时必须持续下发相应 `w`。

## 底盘状态回传约定（待实现）

当前 `0x5B` 上行仍只有裁判数据，不能作为底盘状态或控制 tick；尚未回传相对角度、实测角速度、采样时间或控制有效状态。

后续状态至少需要：云台相对底盘角度 `δ`（rad）、实测底盘 `w`（rad/s）、各反馈的采样时间，以及反馈有效和自动控制有效状态。已确认 LIO 固定在大 yaw 上，因此 `δ` 取大 yaw 相对底盘的角度，不叠加小 yaw；可以复用当前大 yaw 编码器及 `YAW_CHASSIS_ALIGN_ECD` 对齐零点。实测 `w` 的反馈来源仍需确定。

统一定义 `δ = wrap(yaw_gimbal - yaw_chassis)` 后，上位机按 `yaw_chassis = wrap(yaw_LIO - δ)` 计算底盘朝向。编码器方向、安装零点与 LIO 外参需要实机核对。独立 INS 的启动 yaw 没有与 LIO 对齐，不能直接替代底盘 yaw。

时间戳必须来自反馈采样并说明时钟域、单位、回绕与主机时钟映射；仅标记 USB 发送或主机收包时刻不能保证姿态对齐。两板各自的运行时间也不能直接相减。上位机应按时间插值匹配 LIO 和相对角度，反馈过期或无效时停止自动控制。

稳定定频回传并验证后，上位机再用新状态触发控制 tick：正常行走下发固定 `w`，接近坡道或隧道时按目标方向误差对正，进入后保持误差修正，离开后恢复固定 `w`。状态断流停车由独立看门狗执行，不能依赖状态回调触发。下位机保留独立 NAV 超时停车和板间通信超时失能。

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.2 | 2026-10-13 | 导航下发增加 w，自动模式执行上位机角速度，无效或离线时三路速度清零 |
| v1.1 | 2026-10-10 | 新增 minipc_nav 离线检测及状态查询接口，默认超时 100 ms |
| v1.0 | 2026-10-10 | 从 PCCOMM 拆出导航通信，使用独立接收线程，移除回环测试 |
