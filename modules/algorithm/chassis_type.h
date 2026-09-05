#ifndef _CHASSIS_TYPE_H_
#define _CHASSIS_TYPE_H_

#include "motor_base.h"
#include <stdint.h>

/* 舵轮底盘几何/机械配置 */
typedef struct
{
    float wheel_r;        /* 舵轮投影点到几何中心距离 (m) */
    float radius_wheel_m; /* 舵轮轮子半径 (m) */
    float align_rad[4];   /* 机械零点对齐角度 (rad): LF LB RB RF  */
} Chassis_Swerve_Config_s;

/* 麦轮/全向轮底盘几何配置 */
typedef struct
{
    float wheel_base_x; /* 前后轮距 (m), X方向两轮中心距 */
    float wheel_base_y; /* 左右轮距 (m), Y方向两轮中心距 */
    float wheel_radius; /* 轮子半径 (m) */
} Chassis_Diff_Config_s;

/* 底盘速度向量 (正运动学输出 / 逆运动学输入) */
typedef struct
{
    float vx; /* X 方向速度 (m/s), 前进为正 */
    float vy; /* Y 方向速度 (m/s), 左移为正 */
    float vw; /* 旋转角速度 (rad/s), 逆时针为正 */
} Chassis_Velocity_s;

/* 里程计状态 (全局位姿) */
typedef struct
{
    float x;   /* 全局 X 坐标 (m), 前进为正 */
    float y;   /* 全局 Y 坐标 (m), 左移为正 */
    float yaw; /* 全局航向角 (rad), 逆时针为正 */
} Chassis_Odom_s;

/* 逆运动学 (底盘速度 → 电机指令) */
/**
 * @brief 舵轮逆运动学计算
 * @param motors 舵轮电机数组
 * @param cfg    舵轮配置参数
 * @param vx     X 方向速度 (m/s), 前进为正
 * @param vy     Y 方向速度 (m/s), 左移为正
 * @param vw     旋转角速度 (rad/s), 逆时针为正
 */
void Chassis_Swerve_Calc(Motor_Base *motors[8], const Chassis_Swerve_Config_s *cfg, float vx, float vy, float vw);
/**
 * @brief 麦轮逆运动学计算
 * @param motors 麦轮电机数组
 * @param cfg    麦轮配置参数
 * @param vx     X 方向速度 (m/s), 前进为正
 * @param vy     Y 方向速度 (m/s), 左移为正
 * @param vw     旋转角速度 (rad/s), 逆时针为正
 */
void Chassis_Mecanum_Calc(Motor_Base *motors[4], const Chassis_Diff_Config_s *cfg, float vx, float vy, float vw);
/**
 * @brief 全向轮逆运动学计算
 * @param motors 全向轮电机数组
 * @param cfg    全向轮配置参数
 * @param vx     X 方向速度 (m/s), 前进为正
 * @param vy     Y 方向速度 (m/s), 左移为正
 * @param vw     旋转角速度 (rad/s), 逆时针为正
 */
void Chassis_Omni_Calc(Motor_Base *motors[4], const Chassis_Diff_Config_s *cfg, float vx, float vy, float vw);

/* 正运动学 (电机反馈 → 底盘速度) */

/**
 * @brief 舵轮正运动学计算
 * @param motors 舵轮电机数组
 * @param cfg    舵轮配置参数
 * @return 底盘速度结构体
 */
Chassis_Velocity_s Chassis_Swerve_Fwd(const Motor_Base *motors[8], const Chassis_Swerve_Config_s *cfg);
/**
 * @brief 麦轮正运动学计算
 * @param motors 麦轮电机数组
 * @param cfg    麦轮配置参数
 * @return 底盘速度结构体
 */
Chassis_Velocity_s Chassis_Mecanum_Fwd(const Motor_Base *motors[4], const Chassis_Diff_Config_s *cfg);
/**
 * @brief 全向轮正运动学计算
 * @param motors 全向轮电机数组
 * @param cfg    全向轮配置参数
 * @return 底盘速度结构体
 */
Chassis_Velocity_s Chassis_Omni_Fwd(const Motor_Base *motors[4], const Chassis_Diff_Config_s *cfg);

/* 里程计 (速度积分 → 位姿) */

/**
 * @brief 里程计复位 (x=y=yaw=0)
 * @param odom 里程计结构体
 */
void Chassis_Odom_Reset(Chassis_Odom_s *odom);
/**
 * @brief 舵轮里程计
 * @param odom   里程计结构体
 * @param motors 舵轮电机数组
 * @param cfg    舵轮配置参数
 * @param dt     时间间隔 (s)
 */
void Chassis_Swerve_Odom(Chassis_Odom_s *odom, const Motor_Base *motors[8], const Chassis_Swerve_Config_s *cfg, float dt);
/**
 * @brief 麦轮里程计
 * @param odom   里程计结构体
 * @param motors 麦轮电机数组
 * @param cfg    麦轮配置参数
 * @param dt     时间间隔 (s)
 */
void Chassis_Mecanum_Odom(Chassis_Odom_s *odom, const Motor_Base *motors[4], const Chassis_Diff_Config_s *cfg, float dt);
/**
 * @brief 全向轮里程计
 * @param odom   里程计结构体
 * @param motors 全向轮电机数组
 * @param cfg    全向轮配置参数
 * @param dt     时间间隔 (s)
 */
void Chassis_Omni_Odom(Chassis_Odom_s *odom, const Motor_Base *motors[4], const Chassis_Diff_Config_s *cfg, float dt);

#endif /* _CHASSIS_TYPE_H_ */
