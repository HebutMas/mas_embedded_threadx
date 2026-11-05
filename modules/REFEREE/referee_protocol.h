/*
 * @Author: laladuduqq 17503181697@163.com
 * @Date: 2026-03-12 14:20:23
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-11 15:50:13
 * @FilePath: /mas_embedded_threadx/modules/REFEREE/referee_protocol.h
 * @Description: 适配 RoboMaster 2027 V1.0.0（20261029）串口协议
 */
#ifndef _REFEREE_PROTOCOL_H_
#define _REFEREE_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

// 协议宏定义

// 帧固定参数
#define REFEREE_SOF                       0xA5 // 帧起始字节固定值
#define FRAME_HEADER_LEN                  5    // 帧头长度
#define CMD_ID_LEN                        2    // 命令码长度
#define FRAME_TAIL_LEN                    2    // 帧尾CRC16长度
#define REFEREE_USER_DATA_MAX_LEN         112  // 0x0301 内容数据段上限，不含6字节交互头
#define REFEREE_REGULAR_DATA_MAX_LEN      118  // 常规链路数据段上限
#define REFEREE_INTERACTION_FRAME_MAX_LEN 127  // 0x0301 整帧上限
#define REFEREE_DATA_MAX_LEN              300  // 图传链路 0x0310 数据段长度
#define REFEREE_FRAME_MAX_LEN             (FRAME_HEADER_LEN + CMD_ID_LEN + REFEREE_DATA_MAX_LEN + FRAME_TAIL_LEN)

// 命令码ID定义

// 比赛状态类
#define CMD_ID_GAME_STATUS                0x0001 // 比赛状态数据
#define CMD_ID_GAME_RESULT                0x0002 // 比赛结果数据
#define CMD_ID_ROBOT_HP                   0x0003 // 机器人血量数据
#define CMD_ID_FIELD_EVENT                0x0101 // 场地事件数据
#define CMD_ID_REFEREE_WARNING            0x0104 // 裁判警告数据
#define CMD_ID_DART_INFO                  0x0105 // 飞镖发射相关数据

// 机器人状态类
#define CMD_ID_ROBOT_STATUS               0x0201 // 机器人性能体系数据
#define CMD_ID_POWER_HEAT_DATA            0x0202 // 底盘缓冲能量和射击热量数据
#define CMD_ID_ROBOT_POSITION             0x0203 // 机器人位置数据
#define CMD_ID_BUFF_DATA                  0x0204 // 机器人增益数据
#define CMD_ID_HURT_STATUS                0x0206 // 伤害状态数据
#define CMD_ID_SHOOT_DATA                 0x0207 // 实时射击数据
#define CMD_ID_ALLOWED_BULLET             0x0208 // 允许发弹量数据
#define CMD_ID_RFID_STATUS                0x0209 // 机器人RFID模块状态
#define CMD_ID_DART_CLIENT_CMD            0x020A // 飞镖选手端指令数据
#define CMD_ID_GROUND_ROBOT_POS           0x020B // 地面机器人位置数据
#define CMD_ID_RADAR_MARK_PROGRESS        0x020C // 雷达标记进度数据
#define CMD_ID_SENTRY_AUTO_SYNC           0x020D // 哨兵自主决策信息同步
#define CMD_ID_RADAR_AUTO_SYNC            0x020E // 雷达自主决策信息同步

// 交互类
#define CMD_ID_ROBOT_INTERACTION          0x0301 // 机器人交互数据
#define CMD_ID_MAP_COMMAND                0x0303 // 选手端小地图交互数据
#define CMD_ID_MAP_RADAR_DATA             0x0305 // 选手端小地图接收雷达数据
#define CMD_ID_MAP_PATH_DATA              0x0307 // 选手端小地图接收路径数据
#define CMD_ID_MAP_CUSTOM_INFO            0x0308 // 选手端小地图接收机器人数据

// 图传链路（与常规链路使用不同物理串口）
#define CMD_ID_ROBOT_CUSTOM_DATA          0x0310 // 机器人发送自定义数据，300字节，≤50Hz
#define CMD_ID_TERMINAL_CUSTOM_DATA       0x0311 // 自定义终端下发指令，60字节，≤75Hz

// 雷达无线链路
#define CMD_ID_RADAR_ENEMY_POSITION       0x0A01
#define CMD_ID_RADAR_ENEMY_HP             0x0A02
#define CMD_ID_RADAR_ENEMY_BULLET         0x0A03
#define CMD_ID_RADAR_ENEMY_STATUS         0x0A04
#define CMD_ID_RADAR_ENEMY_BUFF           0x0A05
#define CMD_ID_RADAR_ENEMY_KEY            0x0A06

// 机器人交互子内容ID
#define INTERACTION_ID_ROBOT_COM          0x0200 // 机器人间通信（0x0200~0x02FF）
#define INTERACTION_ID_DELETE_LAYER       0x0100 // 选手端删除图层
#define INTERACTION_ID_DRAW_1_FIGURE      0x0101 // 选手端绘制1个图形
#define INTERACTION_ID_DRAW_2_FIGURE      0x0102 // 选手端绘制2个图形
#define INTERACTION_ID_DRAW_5_FIGURE      0x0103 // 选手端绘制5个图形
#define INTERACTION_ID_DRAW_7_FIGURE      0x0104 // 选手端绘制7个图形
#define INTERACTION_ID_DRAW_STRING        0x0110 // 选手端绘制字符
#define INTERACTION_ID_SENTRY_CMD         0x0120 // 哨兵自主决策指令
#define INTERACTION_ID_RADAR_CMD          0x0121 // 雷达自主决策指令

// 机器人ID定义：1/101为重装；2/102及选手端0x0102/0x0166为保留位

// 红方机器人ID
#define ROBOT_ID_RED_HEAVY                1
#define ROBOT_ID_RED_INFANTRY_3           3
#define ROBOT_ID_RED_INFANTRY_4           4
#define ROBOT_ID_RED_INFANTRY_5           5
#define ROBOT_ID_RED_AIR                  6
#define ROBOT_ID_RED_SENTRY               7
#define ROBOT_ID_RED_DART                 8
#define ROBOT_ID_RED_RADAR                9
#define ROBOT_ID_RED_BASE                 10
#define ROBOT_ID_R_OUTPOST                11

// 蓝方机器人ID
#define ROBOT_ID_BLUE_HEAVY               101
#define ROBOT_ID_BLUE_INFANTRY_3          103
#define ROBOT_ID_BLUE_INFANTRY_4          104
#define ROBOT_ID_BLUE_INFANTRY_5          105
#define ROBOT_ID_BLUE_AIR                 106
#define ROBOT_ID_BLUE_SENTRY              107
#define ROBOT_ID_BLUE_DART                108
#define ROBOT_ID_BLUE_RADAR               109
#define ROBOT_ID_BLUE_BASE                110
#define ROBOT_ID_B_OUTPOST                111

// 选手端ID
#define CLIENT_ID_RED_HEAVY               0x0101
#define CLIENT_ID_RED_INFANTRY_3          0x0103
#define CLIENT_ID_RED_INFANTRY_4          0x0104
#define CLIENT_ID_RED_INFANTRY_5          0x0105
#define CLIENT_ID_RED_AIR                 0x0106
#define CLIENT_ID_BLUE_HEAVY              0x0165
#define CLIENT_ID_BLUE_INFANTRY_3         0x0167
#define CLIENT_ID_BLUE_INFANTRY_4         0x0168
#define CLIENT_ID_BLUE_INFANTRY_5         0x0169
#define CLIENT_ID_BLUE_AIR                0x016A
#define SERVER_ID                         0x8080 // 裁判系统服务器

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
    uint16_t ally_heavy_hp;        // 己方重装血量
    uint16_t reserved;             // 保留位
    uint16_t ally_infantry_3_hp;   // 己方3号步兵血量
    uint16_t ally_infantry_4_hp;   // 己方4号步兵血量
    int16_t  damage_difference;    // 己方全队总伤害与对方总伤害之差
    uint16_t ally_sentry_hp;       // 己方哨兵血量
    uint16_t outpost_r_hp;         // R前哨血量（固定为开局红方侧）
    uint16_t ally_base_hp;         // 己方基地血量
    uint16_t outpost_b_hp;         // B前哨血量（固定为开局蓝方侧）
    uint16_t enemy_base_hp;        // 对方基地血量
    uint16_t ally_control_degree;  // 己方累计控制度
    uint16_t enemy_control_degree; // 对方累计控制度
} robot_hp_data_t;

// 0x0101 场地事件：增益点3bit状态0=未激活、1=己方、2=对方、3=双方、4=己方未控制阵地；
// 中央高地2bit状态0=未激活、1=己方、2=对方、3=己方未控制阵地。
// 阵地控制状态0=己方完全控制、1=己方临时控制、2=己方未控制。
typedef union
{
    struct __attribute__((packed))
    {
        uint32_t supply_zone            : 1; // bit0: 己方补给区激活
        uint32_t small_rune             : 2; // bit1-2: 0=未激活、1=已激活、2=正在激活
        uint32_t big_rune               : 2; // bit3-4
        uint32_t fortress_r_gain        : 3; // bit5-7
        uint32_t fortress_b_gain        : 3; // bit8-10
        uint32_t base_r_gain            : 3; // bit11-13
        uint32_t base_b_gain            : 3; // bit14-16
        uint32_t commanding_height_gain : 1; // bit17: 己方制高点
        uint32_t central_r_gain         : 2; // bit18-19
        uint32_t central_b_gain         : 2; // bit20-21
        uint32_t outpost_r_gain         : 3; // bit22-24
        uint32_t outpost_b_gain         : 3; // bit25-27
        uint32_t outpost_r_control      : 2; // bit28-29
        uint32_t outpost_b_control      : 2; // bit30-31
    };
    uint32_t value;
} event_word0_t;

typedef union
{
    struct __attribute__((packed))
    {
        uint32_t base_r_control       : 2; // bit0-1
        uint32_t base_b_control       : 2; // bit2-3
        uint32_t dart_hit_time        : 9; // bit4-12: 对方飞镖最后击中时间，秒
        uint32_t dart_hit_target      : 3; // bit13-15: 0=默认、1=R前哨、2~5=基地目标、6=B前哨
        uint32_t high_r_gain_rmul     : 2; // bit16-17: RMUL激活状态0=无、1=己方、2=对方、3=双方
        uint32_t high_r_control_rmul  : 2; // bit18-19: RMUL控制状态0=中立、1=己方、2=对方
        uint32_t high_b_gain_rmul     : 2; // bit20-21
        uint32_t high_b_control_rmul  : 2; // bit22-23
        uint32_t central_gain_rmul    : 2; // bit24-25
        uint32_t central_control_rmul : 2; // bit26-27
        uint32_t reserved             : 4; // bit28-31
    };
    uint32_t value;
} event_word1_t;

typedef struct __attribute__((packed))
{
    event_word0_t event_data;
    event_word1_t event_data_2;
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
        uint16_t last_hit_target : 3; // bit0-2: 0=默认、1=R前哨、2~5=基地目标、6=B前哨
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

// 0x0204 机器人增益数据（7字节）
typedef struct __attribute__((packed))
{
    uint8_t  recovery_buff;      // 回血增益(百分比)
    uint16_t cooling_buff;       // 热量冷却增益
    uint8_t  defence_buff;       // 防御增益(百分比)
    uint8_t  vulnerability_buff; // 负防御增益(百分比)
    uint16_t attack_buff;        // 攻击增益(百分比)
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
} hurt_status_bits_t;

typedef struct __attribute__((packed))
{
    hurt_status_bits_t status;
    uint16_t           hit_sequence;   // 每发命中递增
    int16_t            hit_position_x; // mm，装甲检测面左下角为原点，向右为x正方向
    int16_t            hit_position_y; // mm，向上为y正方向
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

// 0x0209 RFID模块状态，位值1表示检测到对应卡，仅赛内生效。
// R/B指开局红/蓝方侧；trap为靠近中央高地的梯形高地侧，far为远离中央高地侧。
typedef union
{
    struct __attribute__((packed))
    {
        uint32_t supply_zone            : 1; // bit0
        uint32_t assembly_ally          : 1; // bit1
        uint32_t base_r_gain            : 1; // bit2
        uint32_t base_b_gain            : 1; // bit3
        uint32_t fortress_r_gain        : 1; // bit4
        uint32_t fortress_b_gain        : 1; // bit5
        uint32_t central_r_gain         : 1; // bit6
        uint32_t central_b_gain         : 1; // bit7
        uint32_t outpost_r_gain         : 1; // bit8
        uint32_t outpost_b_gain         : 1; // bit9
        uint32_t commanding_height_gain : 1; // bit10
        uint32_t high_r_rmul            : 1; // bit11
        uint32_t high_b_rmul            : 1; // bit12
        uint32_t central_rmul           : 1; // bit13
        uint32_t central_lower_ally     : 1; // bit14
        uint32_t central_upper_ally     : 1; // bit15
        uint32_t central_lower_enemy    : 1; // bit16
        uint32_t central_upper_enemy    : 1; // bit17
        uint32_t road_lower_ally        : 1; // bit18
        uint32_t road_upper_ally        : 1; // bit19
        uint32_t road_lower_enemy       : 1; // bit20
        uint32_t road_upper_enemy       : 1; // bit21
        uint32_t tunnel_ally_road_low   : 1; // bit22
        uint32_t tunnel_ally_road_mid   : 1; // bit23
        uint32_t tunnel_ally_road_up    : 1; // bit24
        uint32_t tunnel_ally_trap_low   : 1; // bit25
        uint32_t tunnel_ally_trap_mid   : 1; // bit26
        uint32_t tunnel_ally_trap_up    : 1; // bit27
        uint32_t tunnel_ally_far_far    : 1; // bit28
        uint32_t tunnel_ally_far_mid    : 1; // bit29
        uint32_t tunnel_ally_far_near   : 1; // bit30
        uint32_t reserved               : 1;
    };
    uint32_t value;
} rfid_status_word0_t;

typedef union
{
    struct __attribute__((packed))
    {
        uint16_t tunnel_enemy_road_low : 1; // bit0
        uint16_t tunnel_enemy_road_mid : 1; // bit1
        uint16_t tunnel_enemy_road_up  : 1; // bit2
        uint16_t tunnel_enemy_trap_low : 1; // bit3
        uint16_t tunnel_enemy_trap_mid : 1; // bit4
        uint16_t tunnel_enemy_trap_up  : 1; // bit5
        uint16_t tunnel_enemy_far_far  : 1; // bit6
        uint16_t tunnel_enemy_far_mid  : 1; // bit7
        uint16_t tunnel_enemy_far_near : 1; // bit8
        uint16_t reserved              : 7;
    };
    uint16_t value;
} rfid_status_word1_t;

typedef struct __attribute__((packed))
{
    rfid_status_word0_t rfid_status;
    rfid_status_word1_t rfid_status_2;
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
    float heavy_x;      // 己方重装x(m)
    float heavy_y;      // 己方重装y(m)
    float reserved_1;   // 保留位
    float reserved_2;   // 保留位
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
        uint16_t enemy_heavy_vulnerable     : 1; // bit0: 对方重装易伤
        uint16_t reserved_1                 : 1; // 保留位
        uint16_t enemy_infantry3_vulnerable : 1; // bit2: 对方3步兵易伤
        uint16_t enemy_infantry4_vulnerable : 1; // bit3: 对方4步兵易伤
        uint16_t reserved_4                 : 1; // 保留位
        uint16_t enemy_sentry_vulnerable    : 1; // bit5: 对方哨兵易伤
        uint16_t ally_heavy_special         : 1; // bit6: 己方重装特殊标识
        uint16_t reserved_7                 : 1; // 保留位
        uint16_t ally_infantry3_special     : 1; // bit8: 己方3步兵特殊标识
        uint16_t ally_infantry4_special     : 1; // bit9: 己方4步兵特殊标识
        uint16_t reserved_10                : 1; // 保留位
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

// 0x020E 雷达自主决策信息同步
typedef struct __attribute__((packed))
{
    uint8_t  radar_info;       // bit0-1:双倍易伤机会；bit2:生效；bit3-4:加密等级；bit5:可改密钥
    uint8_t  radar_info_2;     // bit0:有加固区域；bit1-3:剩余加固机会
    uint16_t fortify_center_x; // mm
    uint16_t fortify_center_y; // mm
    uint16_t fortify_radius;   // mm
} radar_info_t;

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

// 0x0121 雷达自主决策指令；机会请求计数必须单调递增且每次仅加1。
typedef struct __attribute__((packed))
{
    uint8_t  radar_cmd;         // 双倍易伤请求计数
    uint8_t  password_cmd;      // 1=更新己方密钥，2=验证对方密钥（验证更新间隔至少10秒）
    uint8_t  password[6];       // ASCII字母或数字
    uint8_t  armor_fortify_cmd; // 装甲加固请求计数
    uint16_t fortify_center_x;  // mm
    uint16_t fortify_center_y;  // mm
} radar_cmd_t;

// 0x0303 小地图交互数据
typedef struct __attribute__((packed))
{
    float    target_x;        // 目标x坐标(m)
    float    target_y;        // 目标y坐标(m)
    uint8_t  cmd_keyboard;    // 键盘按键值
    uint8_t  target_robot_id; // 目标机器人ID
    uint16_t cmd_source;      // 信息来源ID
} map_command_t;

// 0x0305 双方位置坐标，单位cm；x/y均为0表示未发送该机器人位置。
typedef struct __attribute__((packed))
{
    uint16_t enemy_heavy_position_x;
    uint16_t enemy_heavy_position_y;
    uint16_t enemy_reserved_1;
    uint16_t enemy_reserved_2;
    uint16_t enemy_infantry_3_position_x;
    uint16_t enemy_infantry_3_position_y;
    uint16_t enemy_infantry_4_position_x;
    uint16_t enemy_infantry_4_position_y;
    uint16_t enemy_reserved_3;
    uint16_t enemy_reserved_4;
    uint16_t enemy_sentry_position_x;
    uint16_t enemy_sentry_position_y;
    uint16_t ally_heavy_position_x;
    uint16_t ally_heavy_position_y;
    uint16_t ally_reserved_1;
    uint16_t ally_reserved_2;
    uint16_t ally_infantry_3_position_x;
    uint16_t ally_infantry_3_position_y;
    uint16_t ally_infantry_4_position_x;
    uint16_t ally_infantry_4_position_y;
    uint16_t ally_reserved_3;
    uint16_t ally_reserved_4;
    uint16_t ally_sentry_position_x;
    uint16_t ally_sentry_position_y;
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

// 图传链路固定长度数据；此处仅定义线格式，不由常规链路接收线程处理。
typedef struct __attribute__((packed))
{
    uint8_t data[300];
} robot_custom_data_t;

typedef struct __attribute__((packed))
{
    uint8_t data[60];
} terminal_custom_data_t;

// 0x0A01 雷达无线链路对方坐标，单位cm
typedef struct __attribute__((packed))
{
    uint16_t heavy_x, heavy_y;
    uint16_t reserved_1, reserved_2;
    uint16_t infantry_3_x, infantry_3_y;
    uint16_t infantry_4_x, infantry_4_y;
    uint16_t reserved_3, reserved_4;
    uint16_t sentry_x, sentry_y;
} radar_enemy_position_t;

// 0x0A02 对方血量
typedef struct __attribute__((packed))
{
    uint16_t heavy_hp;
    uint16_t reserved_1;
    uint16_t infantry_3_hp;
    uint16_t infantry_4_hp;
    uint16_t reserved_2;
    uint16_t sentry_hp;
} radar_enemy_hp_t;

// 0x0A03 对方允许发弹量（步兵/哨兵含堡垒储备）
typedef struct __attribute__((packed))
{
    uint16_t heavy_bullet;
    uint16_t infantry_3_bullet;
    uint16_t infantry_4_bullet;
    uint16_t reserved;
    uint16_t sentry_bullet;
} radar_enemy_bullet_t;

// 0x0A04 对方宏观状态，位定义见协议表1-44，视角为对方。
typedef struct __attribute__((packed))
{
    uint16_t remaining_gold;
    uint16_t total_gold;
    uint32_t gain_status;
    uint32_t crossing_control_status;
} radar_enemy_status_t;

// 0x0A05 对方增益、状态和自定义装甲加固情报
// 主要状态：0=存活、1=战亡、2=无敌但不虚弱、3=无敌且虚弱。
typedef struct __attribute__((packed))
{
    buff_t   heavy_buff;
    uint8_t  reserved_buff[7];
    buff_t   infantry_3_buff;
    buff_t   infantry_4_buff;
    buff_t   sentry_buff;
    uint8_t  sentry_posture; // 1~3普通、4~6强化
    uint8_t  heavy_status;
    uint8_t  reserved_status;
    uint8_t  infantry_3_status;
    uint8_t  infantry_4_status;
    uint8_t  sentry_status;
    uint16_t fortify_center_x; // mm，装甲检测面左下角为原点
    uint16_t fortify_center_y; // mm
    uint16_t fortify_radius;   // mm，0=无加固情报
} radar_enemy_buff_t;

// 0x0A06 对方干扰波密钥
typedef struct __attribute__((packed))
{
    uint8_t key[6]; // ASCII字母或数字
} radar_enemy_key_t;

#endif // _REFEREE_PROTOCOL_H_
