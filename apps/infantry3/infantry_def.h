#ifndef _INFANTRY_DEF_H_
#define _INFANTRY_DEF_H_

#include <stdint.h>

// clang-format off
// 云台参数
#define PITCH_HORIZON_ANGLE 0.0f            // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE     20.0f           // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE     -40.0f          // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define YAW_CHASSIS_ALIGN_ECD        5995     // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
// 底盘参数
#define CHASSIS_MAX_SPEED_MPS         3.0f            // 底盘最大线速度 (m/s), 遥控满杆映射到此速度
// 发射参数
#define REDUCTION_RATIO_LOADER         36.0f          // 2006拨盘电机的减速比
#define ONE_BULLET_DELTA_ANGLE         60.0f          // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出
#define NUM_PER_CIRCLE                 6              // 拨盘一圈的装载量

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
    chassis_rotate,            /* 旋转(小陀螺): 固定 0.2转/s 自旋 + 全向移动 */
    chassis_rotate_reverse,    /* 旋转反向 */
} chassis_mode_e;

typedef struct
{
    float          vx;
    float          vy;
    float          wz;
    float          offset_angle;
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_t;

#pragma pack()

#endif // _INFANTRY_DEF_H_