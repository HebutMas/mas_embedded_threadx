/**
 ******************************************************************************
 * @file    X_KF.h
 * @version V1.1.0
 * @date    2026/10/16
 * @brief   displacement kalman filter / 位移估计线性卡尔曼滤波(纯滤波层)
 ******************************************************************************
 * @attention
 *  基于通用 KalmanFilter_t 框架，功能等价于 luntui1 Slip_dection 手写版。
 *  状态 2 维 [位移 x, 速度 dx]，量测 2 维 [机体速度, 体轴Y加速度]。
 *  本模块只做滤波: 输出滤波速度 dx (及参考位移 x_hat, 无消费点)。
 *  位移积分器在流水线层 observer.c, 对应 luntui1 KF_x 在 x_observe.c。
 *  dt 写死 0.002 (与 luntui1 DT 一致); 待 exp/remove_200us_delay 实测后
 *  再定是否改 DWT 实测 (BSP_DWT_GetDeltaT)。
 ******************************************************************************
 */
#ifndef _X_KF_H_
#define _X_KF_H_

typedef struct
{
    float x_hat;   /**< 滤波器内部位移估计 (参考值, 对齐 luntui1 FilteredValue[0], 无消费点) */
    float dx;      /**< 滤波后速度 */
} X_KF_out;

void           X_KF_init(void);
void           X_KF_update(float body_velocity, float body_accel_y);
const X_KF_out *X_KF_get(void);
void           X_KF_reset(void);   /* 重重置: 只清滤波器状态, 禁止重新 Init */

#endif /* _X_KF_H_ */
