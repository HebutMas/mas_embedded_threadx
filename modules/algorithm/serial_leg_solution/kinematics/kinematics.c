/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-26 12:10:31
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-07-27 17:53:13
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\kinematics\kinematics.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#include "kinematics.h"
#include "arm_math.h"
#include "solution_def.h"
#include "observer.h"
#include "user_lib.h"

static fk fk_left;
static fk fk_right;

static void _kinematics_solve(float phi_1, float phi_4, float pitch, float dpitch, float dphi_1, float dphi_4, fk *out)
{
    // 1. 计算B点坐标
    float XB = L_1 * arm_cos_f32(phi_1);   // XB = l1 × cos(phi1)
    float YB = L_1 * arm_sin_f32(phi_1);   // YB = l1 × sin(phi1)

    // 2. 计算D点坐标
    float XD = L_4 * arm_cos_f32(phi_4);   // XD = l4 × cos(phi4)
    float YD = L_4 * arm_sin_f32(phi_4);   // YD = l4 × sin(phi4)

    // 3. 计算BD的距离
    float LBD_2 = (XD - XB) * (XD - XB) + (YD - YB) * (YD - YB);

    // 4. 使用余弦定理求解phi2
    float A0 = 2 * L_2 * (XD - XB);
    float B0 = 2 * L_2 * (YD - YB);
    float C0 = L_2 * L_2 + LBD_2 - L_3 * L_3;
    /* 数值保护: sqrt 参数为负时置0, 防 NaN 传播 */
    float disc = A0*A0 + B0*B0 - C0*C0;
    if (disc < 0.0f) disc = 0.0f;
    float phi_2 = 2 * atan2f((B0 + sqrtf(disc)), A0 + C0);
    out->phi_2 = phi_2;

    // 5. 求解phi3
    float phi_3 = atan2f(YB - YD + L_2 * arm_sin_f32(phi_2), XB - XD + L_2 * arm_cos_f32(phi_2));

    // 6. 计算C点坐标
    float XC = L_1 * arm_cos_f32(phi_1) + L_2 * arm_cos_f32(phi_2);
    float YC = L_1 * arm_sin_f32(phi_1) + L_2 * arm_sin_f32(phi_2);

    // 7. 计算摆杆长度L0
    float L_0 = sqrtf(XC * XC + YC * YC);
    /* 数值保护: 腿折叠 L0→0 时防除零 */
    if (L_0 < KIN_EPS) L_0 = KIN_EPS;
    out->L_0 = L_0;

    // 8. 计算摆杆角度phi0
    float phi_0 = atan2f(YC, XC);
    out->phi_0 = phi_0;

    // 9. 计算腿部摆动角度theta
    out->theta = -(PI/2.0f - phi_0 + pitch);

    float sigma1=arm_sin_f32(phi_3-phi_2);
	/* 数值保护: 共线奇异 sin(phi3-phi2)→0 时防除零 */
	if (fabsf(sigma1) < KIN_EPS)
	    sigma1 = (sigma1 >= 0.0f) ? KIN_EPS : -KIN_EPS;
	float sigma2=arm_sin_f32(phi_3-phi_4);
	float sigma3=arm_sin_f32(phi_1-phi_2);
	float sigma4=arm_sin_f32(phi_0-phi_3);
	float sigma5=arm_cos_f32(phi_0-phi_3);
	float sigma6=arm_sin_f32(phi_0-phi_2);
	float sigma7=arm_cos_f32(phi_0-phi_2);
	
	(out->J_11)=(L_1*sigma4*sigma3)/sigma1;
	(out->J_12)=(L_4*sigma6*sigma2)/sigma1;
	(out->J_21)=(L_1*sigma5*sigma3)/((L_0)*sigma1);
	(out->J_22)=(L_4*sigma7*sigma2)/((L_0)*sigma1);
	
	// //获取VMC逆解矩阵元素
	 float sigma8 = L_4*sigma2;
	// float sigma9=l_1*sigma3;
	// (Leg->T_11)=-sigma7/sigma9;
	// (Leg->T_12)=sigma5/sigma8;
	// (Leg->T_21)=(Leg->L_0)*sigma6/sigma9;
	// (Leg->T_22)=-(Leg->L_0)*sigma4/sigma8;

    float sigma10 = L_1*dphi_1;
	float sigma11 = -XC;
	float sigma12 = sigma10*arm_sin_f32(phi_1-phi_3)+sigma8*dphi_4;
	float sigma13 = sigma12/sigma1;
	float sigma14 = sigma10*arm_cos_f32(phi_1)+sigma13*arm_cos_f32(phi_2);
	float sigma15 = sigma10*arm_sin_f32(phi_1)+sigma13*arm_sin_f32(phi_2);
	out->dL_0    = (YC*sigma14+sigma11*sigma15)/(L_0);
	/* 数值保护: dphi_0 分母 L0^2 防除零 */
	float dphi_den = YC*YC + sigma11*sigma11;
	if (dphi_den < KIN_EPS) dphi_den = KIN_EPS;
	out->dphi_0  = -(sigma14*sigma11-YC*sigma15)/(dphi_den);

    out->dtheta = out->dphi_0 - dpitch;
}



void kinematics_calc(void)
{
    const observer *obs = observer_get();
    if (obs == NULL)
        return;
    
    _kinematics_solve(obs->phi_1_L, obs->phi_4_L, obs->pitch, obs->dpitch, obs->dphi_1_L, obs->dphi_4_L, &fk_left);
    _kinematics_solve(obs->phi_1_R, obs->phi_4_R, obs->pitch, obs->dpitch, obs->dphi_1_R, obs->dphi_4_R, &fk_right);
}

void kinematics_init(void)
{
    kinematics_calc();
    fk_left.initialized = 1;
    fk_right.initialized = 1;
}

const fk *kinematics_get_left(void)
{
    if (fk_left.initialized)
        return &fk_left;
    return NULL;
}

const fk *kinematics_get_right(void)
{
    if (fk_right.initialized)
        return &fk_right;
    return NULL;
}
 