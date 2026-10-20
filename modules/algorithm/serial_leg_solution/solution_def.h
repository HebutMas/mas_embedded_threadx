/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-26 12:10:31
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-07-27 13:38:15
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\solution_def.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _SOLUTION_DEF_H_
#define _SOLUTION_DEF_H_

#include <stdint.h>

#define WHEEL_R 0.06f
#define phi_1_offset 173.92f
#define phi_4_offset 6.08f

#define ROBOT_MASS 16.0f
//观测器
typedef struct
{
    float phi_1_L;
    float phi_4_L;
    float phi_1_R;
    float phi_4_R;
    float dphi_1_L;
    float dphi_4_L;
    float dphi_1_R;
    float dphi_4_R;
    float x_L;
    float dx_L;
    float x_R;
    float dx_R;
    /* 机体位移估计 (KF 流水线层, 对应 luntui1 Body.dx_nofilter / Body.dx / Body.x) */
    float kf_dx_nofilter;   /* KF 量测输入: 轮面速度 + 腿运动学项 */
    float kf_dx;            /* KF 滤波后机体速度 (X_KF 输出) */
    float kf_x;             /* KF 位移积分器 (等价 luntui1 KF_x) */
    float pitch;
    float dpitch;
    float roll;
    float droll;
    float yaw;
    uint8_t initialized;
} observer;

//运动学解算
typedef struct
{
    float L_0;
    float dL_0;
    float ddL_0;
    float phi_0;
    float dphi_0;
    float theta;
    float dtheta;
    float phi_2;
    float J_11;
    float J_12;
    float J_21;
    float J_22;
    uint8_t initialized;
} fk;

//LQR解算
typedef struct
{
    float TL;
    float TR;
    float Tpl;
    float Tpr;
} LQR;

//VMC解算
typedef struct
{
    float T_1_L;   
    float T_2_L;
    float T_1_R;
    float T_2_R;
} VMC;    

//腿长控制
typedef struct
{
    float target_length_L;
    float target_length_R;
    float F_L_pid;
    float F_R_pid;//vmc解算之前再加和roll补偿和重力补偿，双腿协调同理
} Length;

//roll补偿
typedef struct
{
    float F_L_roll;
    float F_R_roll;
} roll_compf;
 
//双腿协调
typedef struct
{
    float Tp_L_coor;
    float Tp_R_coor;
} coordination ;

//气弹簧等效力
typedef struct
{
    float Fs_L;
    float Fs_R;
} GasSpring;



#endif /* _SOLUTION_DEF_H_ */
