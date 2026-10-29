#ifndef _CAR_STATE_H_
#define _CAR_STATE_H_

/* 调姿到位窗口 (luntui1 mode 1 判据): car_state 判据与 pose_adjust 摆腿共用 */
#define CAR_POSE_THETA_LO (-0.90f)
#define CAR_POSE_THETA_HI (1.00f)

/* 腿部运动状态机 (预留动作状态) */
typedef enum
{
    STATE_STANDUP = 0, /* 起立: 收腿+降阶偏置LQR 并行 */
    STATE_BALANCE,     /* 正常平衡 */
    STATE_FALLEN,      /* 摔倒: 摆角超限 latch, 仅 reset 可出 (置位权在 VMC) */
    STATE_POSE_ADJUST, /* 调姿: 摆腿到位 (起立前的姿态准备) */
    /* 预留: STATE_JUMP, STATE_STEP, ... */
    STATE_NUM
} car_state_e;

void        car_state_init(void);       /* 上电默认 POSE_ADJUST */
void        car_state_update(void);     /* 每周期调用(需 kinematics 已更新): 判据+转移+latch */
car_state_e car_state_get(void);
void        car_state_reset(void);      /* 掉线: 回 POSE_ADJUST (含清 FALLEN, 不碰 x) */
void        car_state_set_fallen(void); /* 摔倒置位: 仅 VMC 平衡态摆角超限调用 */

#endif
