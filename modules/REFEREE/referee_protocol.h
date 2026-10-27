/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2026-03-12 14:20:23
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-11 15:50:13
 * @FilePath: /mas_embedded_threadx/modules/REFEREE/referee_protocol.h
 * @Description: 适配2026 robomaster v1.2.0 协议
 */
#ifndef _REFEREE_PROTOCOL_H_
#define _REFEREE_PROTOCOL_H_

#include "stdint.h"
#include "stdbool.h"

// 协议宏定义

// 帧固定参数
#define REFEREE_SOF                  0xA5 // 帧起始字节固定值
#define FRAME_HEADER_LEN             5    // 帧头长度
#define CMD_ID_LEN                   2    // 命令码长度
#define FRAME_TAIL_LEN               2    // 帧尾CRC16长度
#define REFEREE_USER_DATA_MAX_LEN    112  // 最大数据段长度
// 最大整帧长度：帧头5 + cmd_id2 + 数据118 + 帧尾2 = 127
#define REFEREE_FRAME_MAX_LEN        127

// 命令码ID定义

// 比赛状态类
#define CMD_ID_GAME_STATUS           0x0001 // 比赛状态数据
#define CMD_ID_GAME_RESULT           0x0002 // 比赛结果数据
#define CMD_ID_ROBOT_HP              0x0003 // 机器人血量数据
#define CMD_ID_FIELD_EVENT           0x0101 // 场地事件数据
#define CMD_ID_REFEREE_WARNING       0x0104 // 裁判警告数据
#define CMD_ID_DART_INFO             0x0105 // 飞镖发射相关数据

// 机器人状态类
#define CMD_ID_ROBOT_STATUS          0x0201 // 机器人性能体系数据
#define CMD_ID_POWER_HEAT_DATA       0x0202 // 底盘缓冲能量和射击热量数据
#define CMD_ID_ROBOT_POSITION        0x0203 // 机器人位置数据
#define CMD_ID_BUFF_DATA             0x0204 // 机器人增益和底盘能量数据
#define CMD_ID_HURT_STATUS           0x0206 // 伤害状态数据
#define CMD_ID_SHOOT_DATA            0x0207 // 实时射击数据
#define CMD_ID_ALLOWED_BULLET        0x0208 // 允许发弹量数据
#define CMD_ID_RFID_STATUS           0x0209 // 机器人RFID模块状态
#define CMD_ID_DART_CLIENT_CMD       0x020A // 飞镖选手端指令数据
#define CMD_ID_GROUND_ROBOT_POS      0x020B // 地面机器人位置数据
#define CMD_ID_RADAR_MARK_PROGRESS   0x020C // 雷达标记进度数据
#define CMD_ID_SENTRY_AUTO_SYNC      0x020D // 哨兵自主决策信息同步
#define CMD_ID_RADAR_AUTO_SYNC       0x020E // 雷达自主决策信息同步

// 交互类
#define CMD_ID_ROBOT_INTERACTION     0x0301 // 机器人交互数据
#define CMD_ID_MAP_COMMAND           0x0303 // 选手端小地图交互数据
#define CMD_ID_MAP_RADAR_DATA        0x0305 // 选手端小地图接收雷达数据
#define CMD_ID_MAP_PATH_DATA         0x0307 // 选手端小地图接收路径数据
#define CMD_ID_MAP_CUSTOM_INFO       0x0308 // 选手端小地图接收机器人数据

// 机器人交互子内容ID
#define INTERACTION_ID_ROBOT_COM     0x0200 // 机器人间通信（0x0200~0x02FF）
#define INTERACTION_ID_DELETE_LAYER  0x0100 // 选手端删除图层
#define INTERACTION_ID_DRAW_1_FIGURE 0x0101 // 选手端绘制1个图形
#define INTERACTION_ID_DRAW_2_FIGURE 0x0102 // 选手端绘制2个图形
#define INTERACTION_ID_DRAW_5_FIGURE 0x0103 // 选手端绘制5个图形
#define INTERACTION_ID_DRAW_7_FIGURE 0x0104 // 选手端绘制7个图形
#define INTERACTION_ID_DRAW_STRING   0x0110 // 选手端绘制字符
#define INTERACTION_ID_SENTRY_CMD    0x0120 // 哨兵自主决策指令
#define INTERACTION_ID_RADAR_CMD     0x0121 // 雷达自主决策指令

// 机器人ID定义

// 红方机器人ID
#define ROBOT_ID_RED_HERO            1
#define ROBOT_ID_RED_ENGINEER        2
#define ROBOT_ID_RED_INFANTRY_3      3
#define ROBOT_ID_RED_INFANTRY_4      4
#define ROBOT_ID_RED_INFANTRY_5      5
#define ROBOT_ID_RED_AIR             6
#define ROBOT_ID_RED_SENTRY          7
#define ROBOT_ID_RED_DART            8
#define ROBOT_ID_RED_RADAR           9
#define ROBOT_ID_RED_BASE            10
#define ROBOT_ID_RED_OUTPOST         11

// 蓝方机器人ID
#define ROBOT_ID_BLUE_HERO           101
#define ROBOT_ID_BLUE_ENGINEER       102
#define ROBOT_ID_BLUE_INFANTRY_3     103
#define ROBOT_ID_BLUE_INFANTRY_4     104
#define ROBOT_ID_BLUE_INFANTRY_5     105
#define ROBOT_ID_BLUE_AIR            106
#define ROBOT_ID_BLUE_SENTRY         107
#define ROBOT_ID_BLUE_DART           108
#define ROBOT_ID_BLUE_RADAR          109
#define ROBOT_ID_BLUE_BASE           110
#define ROBOT_ID_BLUE_OUTPOST        111

// 选手端ID
#define CLIENT_ID_RED_HERO           0x0101
#define CLIENT_ID_RED_ENGINEER       0x0102
#define CLIENT_ID_RED_INFANTRY_3     0x0103
#define CLIENT_ID_RED_INFANTRY_4     0x0104
#define CLIENT_ID_RED_INFANTRY_5     0x0105
#define CLIENT_ID_RED_AIR            0x0106
#define CLIENT_ID_BLUE_HERO          0x0165
#define CLIENT_ID_BLUE_ENGINEER      0x0166
#define CLIENT_ID_BLUE_INFANTRY_3    0x0167
#define CLIENT_ID_BLUE_INFANTRY_4    0x0168
#define CLIENT_ID_BLUE_INFANTRY_5    0x0169
#define CLIENT_ID_BLUE_AIR           0x016A
#define SERVER_ID                    0x8080 // 裁判系统服务器

// 协议数据结构体定义

// 帧头结构体
typedef struct __attribute__((packed))
{
    uint8_t  sof;         // 起始字节，固定0xA5
    uint16_t data_length; // 数据段长度
    uint8_t  seq;         // 包序号
    uint8_t  crc8;        // 帧头CRC8校验
} frame_header_t;

// 0x0001 比赛状态数据
typedef union
{
    struct __attribute__((packed))
    {
        uint8_t game_type     : 4; // 比赛类型 1~5
        uint8_t game_progress : 4; // 比赛进程 0~5
    };
    uint8_t byte;
} game_type_progress_t;

typedef struct __attribute__((packed))
{
    game_type_progress_t type_progress;     // 联合体解析比赛类型和进程
    uint16_t             stage_remain_time; // 当前阶段剩余时间(秒)
    uint64_t             sync_time_stamp;   // UNIX时间戳
} game_status_t;

// 0x0002 比赛结果数据
typedef struct __attribute__((packed))
{
    uint8_t winner; // 0:平局 1:红方胜 2:蓝方胜
} game_result_t;

// 0x0003 机器人血量数据
typedef struct __attribute__((packed))
{
    uint16_t ally_hero_hp;       // 己方英雄血量
    uint16_t ally_engineer_hp;   // 己方工程血量
    uint16_t ally_infantry_3_hp; // 己方3号步兵血量
    uint16_t ally_infantry_4_hp; // 己方4号步兵血量
    int16_t  damage_difference;  // 己方全队总伤害与对方总伤害之差
    uint16_t ally_sentry_hp;     // 己方哨兵血量
    uint16_t ally_outpost_hp;    // 己方前哨站血量
    uint16_t ally_base_hp;       // 己方基地血量
    uint16_t enemy_outpost_hp;   // 对方前哨站血量
    uint16_t enemy_base_hp;      // 对方基地血量
} robot_hp_data_t;

// 0x0101 场地事件数据
typedef union
{
    struct __attribute__((packed))
    {
        uint32_t supply_zone_1         : 1; // bit0: 己方补给区1占领
        uint32_t supply_zone_2         : 1; // bit1: 己方补给区2占领
        uint32_t supply_zone_rmul      : 1; // bit2: 己方补给区(仅RMUL)
        uint32_t small_rune            : 2; // bit3-4: 己方小能量机关
        uint32_t big_rune              : 2; // bit5-6: 己方大能量机关
        uint32_t central_high_ground   : 2; // bit7-8: 己方中央高地
        uint32_t trapezoid_high_ground : 2; // bit9-10: 己方梯形高地
        uint32_t dart_hit_time         : 9; // bit11-19: 飞镖击中时间
        uint32_t dart_hit_target       : 3; // bit20-22: 飞镖击中目标
        uint32_t center_gain_rmul      : 2; // bit23-24: 中心增益点(仅RMUL)
        uint32_t fortress_gain         : 2; // bit25-26: 己方堡垒增益点
        uint32_t outpost_gain          : 2; // bit27-28: 己方前哨站增益点
        uint32_t base_gain             : 1; // bit29: 己方基地增益点
        uint32_t reserved              : 2; // bit30-31: 保留
    };
    uint32_t value;
} event_data_t;

// 0x0104 裁判警告数据
typedef struct __attribute__((packed))
{
    uint8_t level;    // 判罚等级 1~4
    uint8_t robot_id; // 违规机器人ID
    uint8_t count;    // 违规次数
} referee_warning_t;

// 0x0105 飞镖发射相关数据
typedef union
{
    struct __attribute__((packed))
    {
        uint16_t last_hit_target : 3; // bit0-2: 最近击中目标
        uint16_t hit_count       : 3; // bit3-5: 被击中累计次数
        uint16_t selected_target : 3; // bit6-8: 当前选定击打目标
        uint16_t reserved        : 7; // bit9-15: 保留
    };
    uint16_t value;
} dart_info_bits_t;

typedef struct __attribute__((packed))
{
    uint8_t          dart_remaining_time; // 飞镖发射剩余时间(秒)
    dart_info_bits_t dart_info;           // 飞镖信息位域
} dart_info_t;

// 0x0201 机器人性能体系数据
typedef union
{
    struct __attribute__((packed))
    {
        uint8_t gimbal_output  : 1; // bit0: gimbal口24V输出
        uint8_t chassis_output : 1; // bit1: chassis口24V输出
        uint8_t shooter_output : 1; // bit2: shooter口24V输出
        uint8_t reserved       : 5;
    };
    uint8_t byte;
} power_mgmt_output_t;

typedef struct __attribute__((packed))
{
    uint8_t             robot_id;              // 本机器人ID
    uint8_t             robot_level;           // 机器人等级
    uint16_t            current_hp;            // 当前血量
    uint16_t            maximum_hp;            // 血量上限
    uint16_t            shooter_cooling_value; // 射击热量冷却值
    uint16_t            shooter_heat_limit;    // 射击热量上限
    uint16_t            chassis_power_limit;   // 底盘功率上限
    float               bullet_speed_limit;    // 射击初速度上限
    power_mgmt_output_t power_output;          // 电源管理输出状态（联合体）
} robot_status_t;

// 0x0202 底盘缓冲能量和射击热量数据
typedef struct __attribute__((packed))
{
    uint16_t reserved1;
    uint16_t reserved2;
    float    reserved3;
    uint16_t buffer_energy;     // 缓冲能量(J)
    uint16_t shooter_17mm_heat; // 17mm发射机构热量
    uint16_t shooter_42mm_heat; // 42mm发射机构热量
} power_heat_data_t;

// 0x0203 机器人位置数据
typedef struct __attribute__((packed))
{
    float x;     // x坐标(m)
    float y;     // y坐标(m)
    float angle; // 朝向(度)，正北为0
} robot_position_t;

// 0x0204 机器人增益和底盘能量数据
typedef union
{
    struct __attribute__((packed))
    {
        uint8_t energy_125 : 1; // bit0: 剩余能量≥125%
        uint8_t energy_100 : 1; // bit1: 剩余能量≥100%
        uint8_t energy_50  : 1; // bit2: 剩余能量≥50%
        uint8_t energy_30  : 1; // bit3: 剩余能量≥30%
        uint8_t energy_15  : 1; // bit4: 剩余能量≥15%
        uint8_t energy_5   : 1; // bit5: 剩余能量≥5%
        uint8_t energy_1   : 1; // bit6: 剩余能量≥1%
        uint8_t reserved   : 1; // bit7: 保留
    };
    uint8_t byte;
} remaining_energy_t;

typedef struct __attribute__((packed))
{
    uint8_t            recovery_buff;      // 回血增益(百分比)
    uint16_t           cooling_buff;       // 热量冷却增益
    uint8_t            defence_buff;       // 防御增益(百分比)
    uint8_t            vulnerability_buff; // 负防御增益(百分比)
    uint16_t           attack_buff;        // 攻击增益(百分比)
    remaining_energy_t remaining_energy;   // 底盘剩余能量（联合体）
} buff_t;

// 0x0206 伤害状态数据
typedef union
{
    struct __attribute__((packed))
    {
        uint8_t armor_id  : 4; // bit0-3: 被击中装甲/测速模块ID
        uint8_t hurt_type : 4; // bit4-7: 血量变化类型
    };
    uint8_t byte;
} hurt_status_t;

// 0x0207 实时射击数据
typedef struct __attribute__((packed))
{
    uint8_t bullet_type;  // 弹丸类型: 1=17mm 2=42mm
    uint8_t shooter_id;   // 发射机构ID: 1=17mm 3=42mm
    uint8_t fire_freq;    // 射速(Hz)
    float   bullet_speed; // 弹丸初速度(m/s)
} shoot_data_t;

// 0x0208 允许发弹量数据
typedef struct __attribute__((packed))
{
    uint16_t bullet_17mm_allowed;   // 17mm允许发弹量
    uint16_t bullet_42mm_allowed;   // 42mm允许发弹量
    uint16_t remaining_gold;        // 剩余金币
    uint16_t fortress_17mm_allowed; // 堡垒增益点储备17mm允许发弹量
} allowed_bullet_t;

// 0x0209 RFID模块状态
typedef union
{
    struct __attribute__((packed))
    {
        uint32_t base_gain            : 1; // bit0: 己方基地增益点
        uint32_t central_high_ally    : 1; // bit1: 己方中央高地
        uint32_t central_high_enemy   : 1; // bit2: 对方中央高地
        uint32_t trap_high_ally       : 1; // bit3: 己方梯形高地
        uint32_t trap_high_enemy      : 1; // bit4: 对方梯形高地
        uint32_t fly_ramp_ally_front  : 1; // bit5: 己方飞坡前
        uint32_t fly_ramp_ally_back   : 1; // bit6: 己方飞坡后
        uint32_t fly_ramp_enemy_front : 1; // bit7: 对方飞坡前
        uint32_t fly_ramp_enemy_back  : 1; // bit8: 对方飞坡后
        uint32_t central_lower_ally   : 1; // bit9
        uint32_t central_upper_ally   : 1; // bit10
        uint32_t central_lower_enemy  : 1; // bit11
        uint32_t central_upper_enemy  : 1; // bit12
        uint32_t road_lower_ally      : 1; // bit13
        uint32_t road_upper_ally      : 1; // bit14
        uint32_t road_lower_enemy     : 1; // bit15
        uint32_t road_upper_enemy     : 1; // bit16
        uint32_t fortress_gain_ally   : 1; // bit17: 己方堡垒增益点
        uint32_t outpost_gain_ally    : 1; // bit18: 己方前哨站增益点
        uint32_t supply_no_overlap    : 1; // bit19: 不重叠补给区
        uint32_t supply_overlap       : 1; // bit20: 重叠补给区
        uint32_t assembly_ally        : 1; // bit21: 己方装配增益点
        uint32_t assembly_enemy       : 1; // bit22: 对方装配增益点
        uint32_t center_gain_rmul     : 1; // bit23: 中心增益点(仅RMUL)
        uint32_t fortress_gain_enemy  : 1; // bit24: 对方堡垒增益点
        uint32_t outpost_gain_enemy   : 1; // bit25: 对方前哨站增益点
        uint32_t tunnel_ally_road_low : 1; // bit26
        uint32_t tunnel_ally_road_mid : 1; // bit27
        uint32_t tunnel_ally_road_up  : 1; // bit28
        uint32_t tunnel_ally_trap_low : 1; // bit29
        uint32_t tunnel_ally_trap_mid : 1; // bit30
        uint32_t tunnel_ally_trap_up  : 1; // bit31
    };
    uint32_t value;
} rfid_status_word0_t;

typedef union
{
    struct __attribute__((packed))
    {
        uint8_t tunnel_enemy_road_low : 1; // bit0
        uint8_t tunnel_enemy_road_mid : 1; // bit1
        uint8_t tunnel_enemy_road_up  : 1; // bit2
        uint8_t tunnel_enemy_trap_low : 1; // bit3
        uint8_t tunnel_enemy_trap_mid : 1; // bit4
        uint8_t tunnel_enemy_trap_up  : 1; // bit5
        uint8_t reserved              : 2;
    };
    uint8_t byte;
} rfid_status_word1_t;

typedef struct __attribute__((packed))
{
    rfid_status_word0_t rfid_status;   // 主RFID状态位域
    rfid_status_word1_t rfid_status_2; // 扩展RFID状态位域
} rfid_status_t;

// 0x020A 飞镖选手端指令数据
typedef struct __attribute__((packed))
{
    uint8_t  dart_launch_opening_status; // 飞镖发射站状态: 0=已开启 1=关闭 2=开关中
    uint8_t  reserved;
    uint16_t target_change_time;     // 切换目标时剩余比赛时间(秒)
    uint16_t latest_launch_cmd_time; // 最后发射指令时剩余比赛时间(秒)
} dart_client_cmd_t;

// 0x020B 地面机器人位置数据
typedef struct __attribute__((packed))
{
    float hero_x;       // 己方英雄x(m)
    float hero_y;       // 己方英雄y(m)
    float engineer_x;   // 己方工程x(m)
    float engineer_y;   // 己方工程y(m)
    float infantry_3_x; // 己方3步兵x(m)
    float infantry_3_y; // 己方3步兵y(m)
    float infantry_4_x; // 己方4步兵x(m)
    float infantry_4_y; // 己方4步兵y(m)
    float reserved_x;   // 保留位
    float reserved_y;   // 保留位
} ground_robot_position_t;

// 0x020C 雷达标记进度数据
typedef union
{
    struct __attribute__((packed))
    {
        uint16_t enemy_hero_vulnerable      : 1; // bit0: 对方英雄易伤
        uint16_t enemy_engineer_vulnerable  : 1; // bit1: 对方工程易伤
        uint16_t enemy_infantry3_vulnerable : 1; // bit2: 对方3步兵易伤
        uint16_t enemy_infantry4_vulnerable : 1; // bit3: 对方4步兵易伤
        uint16_t enemy_air_special          : 1; // bit4: 对方空中特殊标识
        uint16_t enemy_sentry_vulnerable    : 1; // bit5: 对方哨兵易伤
        uint16_t ally_hero_special          : 1; // bit6: 己方英雄特殊标识
        uint16_t ally_engineer_special      : 1; // bit7: 己方工程特殊标识
        uint16_t ally_infantry3_special     : 1; // bit8: 己方3步兵特殊标识
        uint16_t ally_infantry4_special     : 1; // bit9: 己方4步兵特殊标识
        uint16_t ally_air_special           : 1; // bit10: 己方空中特殊标识
        uint16_t ally_sentry_special        : 1; // bit11: 己方哨兵特殊标识
        uint16_t reserved                   : 4; // bit12-15
    };
    uint16_t value;
} radar_mark_data_t;

// 0x020D 哨兵自主决策信息同步
typedef union
{
    struct __attribute__((packed))
    {
        uint32_t bullet_exchange_cnt    : 11; // bit0-10: 成功兑换发弹量
        uint32_t remote_bullet_exchange : 4;  // bit11-14: 远程兑换发弹量次数
        uint32_t remote_heal_exchange   : 4;  // bit15-18: 远程兑换血量次数
        uint32_t can_free_respawn       : 1;  // bit19: 可以免费复活
        uint32_t can_pay_respawn        : 1;  // bit20: 可以兑换立即复活
        uint32_t pay_respawn_cost       : 10; // bit21-30: 立即复活花费金币
        uint32_t reserved               : 1;  // bit31
    };
    uint32_t value;
} sentry_info_word0_t;

typedef union
{
    struct __attribute__((packed))
    {
        uint16_t is_out_of_combat  : 1;  // bit0: 脱战状态
        uint16_t team_17mm_quota   : 11; // bit1-11: 17mm可兑换发弹量
        uint16_t posture           : 2;  // bit12-13: 哨兵姿态 1=攻击 2=防御 3=移动
        uint16_t rune_can_activate : 1;  // bit14: 能量机关可激活
        uint16_t enhanced_posture  : 1;  // bit15: 当前姿态是否为强化姿态
    };
    uint16_t value;
} sentry_info_word1_t;

typedef struct __attribute__((packed))
{
    sentry_info_word0_t sentry_info;   // 主信息位域
    sentry_info_word1_t sentry_info_2; // 扩展信息位域
    uint64_t            sentry_info_3; // 各姿态弱化前剩余可持续时长，每 8bit 一种姿态
} sentry_auto_sync_t;

// 0x0301 机器人交互数据头
typedef struct __attribute__((packed))
{
    uint16_t data_cmd_id; // 子内容ID
    uint16_t sender_id;   // 发送者ID
    uint16_t receiver_id; // 接收者ID
    // user_data紧随其后，最大112字节
} robot_interaction_header_t;

// 0x0120: 哨兵自主决策指令
typedef union
{
    struct __attribute__((packed))
    {
        uint32_t confirm_resurrect     : 1;  // bit0: 确认复活
        uint32_t confirm_pay_resurrect : 1;  // bit1: 确认金币复活
        uint32_t bullet_exchange_num   : 11; // bit2-12: 兑换发弹量值（单调递增）
        uint32_t remote_bullet_req_cnt : 4;  // bit13-16: 远程补弹请求次数（单调递增）
        uint32_t remote_heal_req_cnt   : 4;  // bit17-20: 远程补血请求次数（单调递增）
        uint32_t change_posture        : 3;  // bit21-23: 修改姿态 1~3普通 4~6强化，默认3
        uint32_t confirm_activate_rune : 1;  // bit24: 确认使能量机关进入激活状态
        uint32_t reserved              : 7;  // bit25-31
    };
    uint32_t value;
} sentry_cmd_t;

// 0x0303 小地图交互数据
typedef struct __attribute__((packed))
{
    float    target_x;        // 目标x坐标(m)
    float    target_y;        // 目标y坐标(m)
    uint8_t  cmd_keyboard;    // 键盘按键值
    uint8_t  target_robot_id; // 目标机器人ID
    uint16_t cmd_source;      // 信息来源ID
} map_command_t;

// 0x0305 选手端小地图接收雷达数据
typedef struct __attribute__((packed))
{
    uint16_t hero_position_x;       // 英雄x(cm)
    uint16_t hero_position_y;       // 英雄y(cm)
    uint16_t engineer_position_x;   // 工程x(cm)
    uint16_t engineer_position_y;   // 工程y(cm)
    uint16_t infantry_3_position_x; // 3步兵x(cm)
    uint16_t infantry_3_position_y; // 3步兵y(cm)
    uint16_t infantry_4_position_x; // 4步兵x(cm)
    uint16_t infantry_4_position_y; // 4步兵y(cm)
    uint16_t reserved_x;            // 保留
    uint16_t reserved_y;            // 保留
    uint16_t sentry_position_x;     // 哨兵x(cm)
    uint16_t sentry_position_y;     // 哨兵y(cm)
} map_robot_data_t;

// 0x0307 选手端小地图接收路径数据
typedef struct __attribute__((packed))
{
    uint8_t  intention;        // 1=攻击 2=防守 3=移动
    uint16_t start_position_x; // 路径起点x(dm)
    uint16_t start_position_y; // 路径起点y(dm)
    int8_t   delta_x[49];      // 路径点x增量数组(dm)
    int8_t   delta_y[49];      // 路径点y增量数组(dm)
    uint16_t sender_id;        // 发送者ID
} map_data_t;

// 0x0308 选手端小地图接收机器人数据
typedef struct __attribute__((packed))
{
    uint16_t sender_id;     // 发送者ID
    uint16_t receiver_id;   // 接收者ID（选手端）
    uint8_t  user_data[30]; // UTF-16编码字符
} custom_info_t;

#endif // _REFEREE_PROTOCOL_H_
