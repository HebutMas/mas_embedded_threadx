#ifndef _SENTRY_BOARDCOMM_PROTOCOL_H_
#define _SENTRY_BOARDCOMM_PROTOCOL_H_

#include <stdint.h>

// clang-format off
// 小云台参数
#define SMALL_YAW_ALIGN_ECD           4692            // 小yaw与大yaw保持居中对齐时的编码器角度
#define SMALL_YAW_PITCH_HORIZON_ANGLE 0.0f            // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define SMALL_YAW_PITCH_MAX_ANGLE     40.0f           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define SMALL_YAW_PITCH_MIN_ANGLE     -20.0f          // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD         4096            // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
// 底盘参数
#define CHASSIS_MAX_SPEED_MPS         3.0f            // 底盘最大线速度 (m/s), 遥控满杆映射到此速度
// 发射参数
#define REDUCTION_RATIO_LOADER         36.0f          // 2006拨盘电机的减速比
#define ONE_BULLET_DELTA_ANGLE         60.0f          // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define NUM_PER_CIRCLE                 6              // 拨盘一圈的装载量

#define GRAVITY_GAMMA 1.58f                              // 重力前馈补偿项
#define GRAVITY_K_PITCH -0.49f                           // pitch轴重力前馈系数
// clang-format on

#pragma pack(1)

// 云台模式设置
typedef enum
{
    gimbal_zero_force = 0, /* 云台停止模式 */
    gimbal_gyro_mode,      /*云台控制模式*/
    gimbal_auto_mode,      /*云台自动模式*/
} gimbal_mode_e;

typedef struct
{
    float         yaw;
    float         pitch;
    uint8_t       auto_search;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_t;

// 发射模式设置
typedef enum
{
    shoot_off = 0,
    shoot_on,
} shoot_mode_e;
typedef enum
{
    friction_off = 0, /* 摩擦轮关闭 */
    friction_on,      /* 摩擦轮开启 */
} friction_mode_e;
typedef enum
{
    load_stop = 0,  /* 停止发射 */
    load_reverse,   /* 反转 */
    load_1_bullet,  /* 单发 */
    load_burstfire, /* 连发 */
} loader_mode_e;

typedef struct
{
    shoot_mode_e    shoot_mode;
    loader_mode_e   load_mode;
    friction_mode_e friction_mode;
} Shoot_Ctrl_Cmd_t;

// 底盘模式设置
typedef enum
{
    chassis_zero_force = 0,    /* 底盘停止模式 */
    chassis_follow_gimbal_yaw, /* 跟随云台模式 */
    chassis_rotate,            /* 旋转 */
    chassis_rotate_reverse,    /* 旋转反向 */
    chassis_automode,          /* 自动模式 */
} chassis_mode_e;

typedef struct
{
    float          vx;
    float          vy;
    float          wz;
    float          offset_angle;
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_t;

/* 板间通讯结构体 */
typedef struct
{
    int8_t  vx;           /* X速度, -10 ～ +10 */
    int8_t  vy;           /* Y速度, -10 ～ +10 */
    int8_t  wz;           /* 旋转速度, -10 ～ +10 */
    int16_t offset_angle; /* yaw偏移角度 */
    uint8_t chassis_mode; /* 底盘控制指令 (chassis_mode_e) */
    int8_t  reserved[2];  /* 填充至8字节 */
} GimbalToChassis_cmd_t;
_Static_assert(sizeof(GimbalToChassis_cmd_t) == 8, "GimbalToChassis_cmd_t must be 8 bytes");

typedef struct
{
    uint8_t  robot_color;      /* 机器人颜色 (0=红,1=蓝) */
    uint8_t  game_progress;    /* 比赛阶段 */
    uint16_t current_hp;       /* 当前血量 */
    uint16_t bullet_allow;     /* 可发射17mm弹丸剩余量 */
    uint16_t shooter_heat_pct; /* 枪管热量% = heat/limit*100 */
} ChassisToGimbal_referee_t;
_Static_assert(sizeof(ChassisToGimbal_referee_t) == 8, "ChassisToGimbal_referee_t must be 8 bytes");

#pragma pack()

#endif /* _SENTRY_BOARDCOMM_PROTOCOL_H_ */
