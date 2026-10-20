/**
 ******************************************************************************
 * @file    X_KF.c
 * @version V1.1.0
 * @date    2026/10/16
 * @brief   displacement kalman filter implementation
 ******************************************************************************
 */
#include "X_KF.h"
#include "kalman_filter.h"
#include <string.h>

/* ══════════════════ 矩阵数据 (与 luntui1 Slip_dection.c 逐位一致) ══════════════════ */

static float F_data[4] = { 1.0f, 0.002f,   0.0f, 1.0f };   /* dt 写死 0.002 (同 luntui1 DT) */
static float H_data[4] = { 0.0f, 1.0f,      1.0f, 0.0f };
static float P_data[4] = { 1.0f, 0.0f,      0.0f, 1.0f };
static float Q_data[4] = { 0.01f, 0.0f,     0.0f, 0.1f };
static float R_data[4] = { 10.0f, 0.0f,     0.0f, 500.0f };

/* ══════════════════ 内部实例 (文件内不可见) ══════════════════ */

static KalmanFilter_t kf;
static X_KF_out       out;

/* ══════════════════ 初始化 (仅一次! 禁止重复调用) ══════════════════ */

void X_KF_init(void)
{
    out.x_hat = 0.0f;
    out.dx    = 0.0f;

    /* Init 内部 user_malloc 分配 (~284B) 且框架无 Free, 生命周期内只能调这一次 */
    Kalman_Filter_Init(&kf, 2, 0, 2);
    kf.UseAutoAdjustment = 0;   /* 模式0: 0 是正经量测值(车静止速度真为0), 走完整 H/R 路径 */

    memcpy(kf.F_data, F_data, sizeof(F_data));
    memcpy(kf.H_data, H_data, sizeof(H_data));
    memcpy(kf.P_data, P_data, sizeof(P_data));
    memcpy(kf.Q_data, Q_data, sizeof(Q_data));
    memcpy(kf.R_data, R_data, sizeof(R_data));
}

/* ══════════════════ 更新 (每 2ms, 由 observer_update 调用) ══════════════════ */

void X_KF_update(float body_velocity, float body_accel_y)
{
    /* 框架每拍"拷走即清零" MeasuredVector, 故两槽每拍必填、先填后调 */
    kf.MeasuredVector[0] = body_velocity;   /* z0: 机体前进速度 (轮面速度+腿运动学项) */
    kf.MeasuredVector[1] = body_accel_y;    /* z1: 体轴 Y 加速度 (MotionAccel_b[1]) */

    Kalman_Filter_Update(&kf);

    out.dx    = kf.FilteredValue[1];
    out.x_hat = kf.FilteredValue[0];   /* 参考值, 无消费点 (与 luntui1 一致) */
}

/* ══════════════════ 只读 getter ══════════════════ */

const X_KF_out *X_KF_get(void)
{
    return &out;
}

/* ══════════════════ 重重置: 只清滤波器状态, 禁止重新 Init ══════════════════ */
/* 原因: 框架无 Free, 反复 Init 泄漏字节池 (~284B/次, dji_c 池 20KB 约 70 次耗尽),
 *       耗尽后 user_malloc 返回 NULL → memset(NULL) 硬故障 */
void X_KF_reset(void)
{
    out.x_hat = 0.0f;
    out.dx    = 0.0f;

    memset(kf.xhat_data,      0, 2 * sizeof(float));
    memset(kf.xhatminus_data, 0, 2 * sizeof(float));
    memset(kf.MeasuredVector, 0, 2 * sizeof(float));

    /* P 不动: 对齐 luntui1 (其 Kalman_Init 不清 P, 保留收敛值)。
     * 若想保守重置, 改为: memcpy(kf.P_data, P_data, sizeof(P_data)); */
}
