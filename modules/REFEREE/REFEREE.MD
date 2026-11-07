# 裁判系统模块

## 协议版本与线格式

`referee_protocol.h` 对应《RoboMaster 2027 机甲大师高校系列赛通信协议》V1.0.0（20261029）的串口协议。结构体使用 packed 和 ARM GCC 低位优先位域布局；不能直接用于自定义终端的 MQTT/Protobuf 消息。

与原2026定义相比：

- `robot_hp_data_t` 为24字节，新增双方累计控制度；工程血量位置为保留位，前哨血量字段为固定 `outpost_r_hp` / `outpost_b_hp`，不随队伍颜色交换。
- `event_data_t` 为8字节，以 `event_data` / `event_data_2` 两个联合体访问状态位或原始 `value`。
- `buff_t` 为7字节，删除 `remaining_energy`；缓冲能量仍从 `power_heat_data_t.buffer_energy` 获取。
- `hurt_status_t` 为7字节，`status` 是首字节位域，其后为命中序号和有符号毫米坐标。
- `rfid_status_t` 为6字节，包含32位和16位两个状态字，位定义已按2027重排。
- `map_robot_data_t` 为48字节，先对方坐标，后己方坐标，单位cm。
- ID `1/101` 和选手端 `0x0101/0x0165` 对应重装；原工程ID为保留位，相关宏已移除。前哨ID使用 `ROBOT_ID_R_OUTPOST` / `ROBOT_ID_B_OUTPOST`。
- 头文件同时定义雷达同步、雷达指令、图传自定义数据以及 `0x0A01`～`0x0A06` 雷达无线链路数据。定义线格式不代表常规链路线程已经接入这些数据。

R/B均表示开局红方侧/蓝方侧。RFID字段 `trap` 表示靠近中央高地的梯形高地侧，`far` 表示远离中央高地侧；`far_far/mid/near` 对应距己方或对方梯形高地较远/中间/较近的位置。

## 初始化与接收

配置 `REFEREE_UART`、任务栈和优先级后调用 `Module_Referee_Init()`，返回0表示成功、-1表示失败；由模块初始化流程调用一次。UART使用DMA，模块线程读取常规链路数据，检查SOF、数据长度、CRC8及CRC16后更新缓存。

当前缓存命令：`0x0001`、`0x0003`、`0x0201`、`0x0202`、`0x0204`、`0x0207`、`0x0208`、`0x020D`。`Module_Referee_Get_cmd_data()` 返回对应缓存地址；未初始化或不支持的命令返回NULL。固定长度命令必须与结构体大小完全一致，防止旧版或异常长度数据混入。返回地址是接收线程更新的共享缓存，并非一致性快照。

当前接收实现只解析单次UART读取中完整的帧，读取末尾的不完整帧不会跨次拼接。

## 发送与链路边界

`Module_Referee_Send_Interaction()` 发送常规链路 `0x0301`，用户内容最多112字节，加6字节交互头后数据段上限118字节，整帧最多127字节。模块限频10Hz，未到发送间隔时直接返回。发送者使用本机器人ID，接收者通常为自身选手端；哨兵/飞镖/雷达/前哨/基地使用服务器ID `0x8080`，当前接口不提供任意机间接收者选择。

`REFEREE_REGULAR_DATA_MAX_LEN` 是常规链路接收上限118；`REFEREE_INTERACTION_FRAME_MAX_LEN` 是交互整帧上限127。`REFEREE_DATA_MAX_LEN` / `REFEREE_FRAME_MAX_LEN` 为图传 `0x0310` 的300/309字节上限，不用于放宽常规UART的校验。图传需要独立物理接口，雷达无线数据需要独立接收路径。
