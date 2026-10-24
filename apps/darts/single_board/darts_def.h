/*
 * @Description: 机器人定义模板
 *
 * 使用说明:
 *   1. 复制本文件到你的机器人目录, 改名为 <机器人名>_def.h (如 hero_def.h)
 *   2. 修改机械参数 (角度、速度、减速比等)
 *   3. 如需要单板/双板模式, 参考 sentry 添加板间通讯结构体
 *   4. 如底盘模式与模板不同, 修改 chassis_mode_e 枚举
 */

#ifndef _ROBOT_DEF_H_
#define _ROBOT_DEF_H_

#include <stdint.h>

/* ================================================================
 * 机械参数 (按实际机械图纸修改)
 * ================================================================ */
// clang-format off
  #define YAW_CHASSIS_ALIGN_ECD     2048      /* 云台与底盘对齐的电机编码器值, 机械改动后需修改 */

// ── 3508 同步带拉簧轴 (电机反馈 total_angle 是转子侧, 输出侧=同步轮, 差 gear_ratio(19) 倍) ──
#define FRICTION_PULLEY_RADIUS_MM 17.0f     /* 同步轮半径, 行程/前馈都按带直线行程 (mm) 给 */
#define FRICTION_TRAVEL_MM        710.0f    /* 待机位 → 工作位 带行程 = 360° × 2πr/360 (占位换算值) */
#define FRICTION_FIRST_PULL_MM    689.0f    /* 第2~4发首次下拉位置距本发 home 的带行程(mm)，独立于工作位 */
#define FRICTION_RELOAD_MM        400.0f     /* 回程中停下等拨盘 reload 的带行程 = 100° 换算 (值待定) */
#define FRICTION_SPRING_START_MM  130.0f      /* 带起点到拉簧开始伸长的空行程 (mm, >=0), 不计入拉簧前馈位移 */
#define FRICTION_MOVE_STEP          0.5f  /* 同步带全程调速：每周期目标推进量，越小越慢，须 >0；内部按 mm/周期换算 */
#define FRICTION_FF_K             0.00450f  /* 拉簧前馈 N·m/mm; 带位移为弹簧伸长的2倍, 绳传力再折半, 按每轴1400 N/m折算 */
#define FRICTION_RIGHT_SIGN       (-1.0f)   /* 右轴镜像: 只把目标值取反, 反馈/电机反转标志位保持 0 */
#define FRICTION_POSITION_TOLERANCE_MM  7.0f  /* 非home停点到位: 位置误差绝对值 <= 此值 (mm) */
#define FRICTION_SPEED_TOLERANCE_MM_S   0.5f /* 非home停点停稳: 速度绝对值 < 此值 (mm/s) */
#define FRICTION_HOME_POSITION_TOLERANCE_MM 50.0f /* home到位: 位置误差绝对值 <= 此值 (mm) */
#define FRICTION_HOME_SPEED_TOLERANCE_MM_S  2.0f  /* home停稳: 速度绝对值 < 此值 (mm/s) */
/* 离散 LQR：状态为输出轴角误差(rad)、角速度(rad/s)，输入为输出轴力矩(N·m)。
 * 暂定 J=5kg×0.017²=0.001445kg·m²（等效惯量待实测，50N 阻力不能换算成惯量）；Ts=0.002s。
 * A=[1,Ts;0,1], B=[Ts²/(2J);Ts/J]；Q=diag(900,40), R=1，速度权重由10增至40。
 * DARE 得 K_out=[3.36755183,0.71676373]，除以19适配转子反馈；拉簧由前馈补偿。
 * 50N 恒阻力的预计静差约4.29mm；位置权重兼顾当前约5mm的到位窗口。 */
#define FRICTION_LQR_K0           0.177240f /* 输出轴力矩 / 转子角误差 */
#define FRICTION_LQR_K1           0.037724f /* 输出轴力矩 / 转子角速度；离散解不能直接套连续增益 */

// ── 扳机舵机 (位置搬 E: 60 上膛 / 15 开火) ──
#define TRIGGER_READY_DEG         60.0f     /* 上膛 */
#define TRIGGER_FIRE_DEG          40.0f     /* 开火 */
#define TRIGGER_READY_WAIT_MS     500U      /* 工作位发出上膛指令后保持多久再回home (ms)，四发共用 */

// ── 拨盘 DM4310 (CAN1, MIT 模式 = 纯力矩, 位置环在模块里跑) ──
// 换弹只在第 2~4 发 (第 1 发跳过小回程), 每个换弹周期转 2 次 (60° + 60°),
// 转动惯量在弹被取走后变, 所以每转一组增益; 命名 K0_<发>_<转>, 转 1 = 弹取走前 (首次下拉位), 转 2 = 弹取走后
#define ONE_BULLET_DELTA_ANGLE    60.0f     /* 单发拨盘转角 (输出侧度, 一圈 6 发) */
#define LOADER_ZERO_OFFSET_DEG    69.0f      /* 拨盘基准 = 电机自身零点 + 此偏置 (输出侧度, 可正可负) */
/* DM 经验调参（拨盘惯量待辨识）：提高位置刚度，K1 近似按 sqrt(K0) 增大以维持阻尼比。
 * 静止且原目标已到位时，60° 阶跃初始位置力矩 ≈ 8.38 N·m，低于 10 N·m 限幅。 */
#define LOADER_LQR_K0_2_1         8.0f      /* 第 2 发转 1 */
#define LOADER_LQR_K1_2_1         0.72f     /* 第 2 发转 1 */
#define LOADER_LQR_K0_2_2         8.0f      /* 第 2 发转 2 */
#define LOADER_LQR_K1_2_2         0.72f     /* 第 2 发转 2 */
#define LOADER_LQR_K0_3_1         8.0f      /* 第 3 发转 1 */
#define LOADER_LQR_K1_3_1         0.72f     /* 第 3 发转 1 */
#define LOADER_LQR_K0_3_2         8.0f      /* 第 3 发转 2 */
#define LOADER_LQR_K1_3_2         0.72f     /* 第 3 发转 2 */
#define LOADER_LQR_K0_4_1         8.0f      /* 第 4 发转 1 */
#define LOADER_LQR_K1_4_1         0.72f     /* 第 4 发转 1 */
#define LOADER_LQR_K0_4_2         8.0f      /* 第 4 发转 2 */
#define LOADER_LQR_K1_4_2         0.72f     /* 第 4 发转 2 */

// ── 云台 (yaw = ZDT 闭环步进 / CAN2 扩展帧, pitch = 3508 / CAN1 id 2) ──
#define GIMBAL_YAW_PULSES_PER_REV 3200.0f  /* yaw 输出轴一圈的脉冲数 = 200 × 细分 × 减速比 (Emm42 出厂 16 细分 = 3200), 待确认 */
#define GIMBAL_YAW_SPEED_RPM      300.0f   /* yaw 位置模式的运行速度 (RPM) */
#define GIMBAL_STICK_TH           200      /* 摇杆过此值就动 (CH3/4 值域 ±660, 底层死区 ±10 已滤) */
#define GIMBAL_YAW_RATE_DPS       120.0f   /* 压住时的 yaw 速度 (120 °/s = 每 2ms 周期 0.24°) */
#define GIMBAL_PITCH_RATE_DPS     240.0f   /* 压住时的 pitch 速度 (每 2ms 周期 0.48°) */
#define GIMBAL_PITCH_LQR_K0       0.240f   /* pitch LQR 位置项 N·m/(转子侧 rad), 占位: 拉簧轴值减半, 上机标定 */
#define GIMBAL_PITCH_LQR_K1       0.0265f  /* pitch LQR 阻尼项 N·m/(转子侧 rad/s), 占位 */

// clang-format on

/* ================================================================
 * 控制模式枚举
 * ================================================================ */
#pragma pack(1)

// ── 云台模式 ──
typedef enum
{
    gimbal_zero_force = 0, /* 云台无力 */
    gimbal_gyro_mode,      /* 手动跟随 */
    gimbal_auto_mode,      /* 自动瞄准 */
} gimbal_mode_e;

typedef struct
{
    float         yaw;
    float         pitch;
    uint8_t       auto_search;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_t;

// ── 发射模式 ──
typedef enum
{
    shoot_off = 0,
    shoot_on,
} shoot_mode_e;

typedef enum
{
    friction_off = 0,
    friction_on,
} friction_mode_e;

typedef enum
{
    load_stop   = 0,
    load_reverse,
    load_1_bullet,
    load_burstfire,
} loader_mode_e;

typedef struct
{
    shoot_mode_e    shoot_mode;
    loader_mode_e   load_mode;
    friction_mode_e friction_mode;
} Shoot_Ctrl_Cmd_t;

// ── 底盘模式 ──
typedef enum
{
    chassis_zero_force       = 0, /* 无力            */
    chassis_follow_gimbal_yaw,    /* 跟随云台         */
    chassis_rotate,               /* 旋转            */
    chassis_rotate_reverse,       /* 反向旋转         */
    /* sentry 用: chassis_automode,  自动模式 (多板) */
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

/* ================================================================
 * 板间通讯结构体 (仅多板机器人需要, 单板可删除)
 *
 * 取消 #if 0 以启用
 * ================================================================ */
#if 0

#pragma pack(1)

/**
 * @brief 云台板 → 底盘板 命令
 *   vx/vy/wz: int8 速度比例 (-10 ~ +10), 底盘解析为 (int8/10)*CHASSIS_MAX_SPEED_MPS
 */
typedef struct
{
    int8_t  vx;
    int8_t  vy;
    int8_t  wz;
    int16_t offset_angle;
    uint8_t chassis_mode;
    int8_t  reserved[2];
} GimbalToChassis_cmd_t;
_Static_assert(sizeof(GimbalToChassis_cmd_t) == 8, "must be 8 bytes");

/**
 * @brief 底盘板 → 云台板 裁判系统数据
 */
typedef struct
{
    uint8_t  robot_color;
    uint8_t  game_progress;
    uint16_t current_hp;
    uint16_t bullet_allow;
    uint16_t shooter_heat_pct;
} ChassisToGimbal_referee_t;
_Static_assert(sizeof(ChassisToGimbal_referee_t) == 8, "must be 8 bytes");

#pragma pack()

#endif /* 板间通讯 */

#endif /* _ROBOT_DEF_H_ */
