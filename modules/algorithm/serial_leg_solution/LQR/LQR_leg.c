/*
 * @Author: 550wsleep 1329258004@qq.com
 * @Date: 2026-07-26 12:10:31
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-07-26 19:24:01
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\LQR\LQR_leg.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "LQR_leg.h"
#include "solution_def.h"
#include "observer.h"
#include "kinematics.h"
#include "car_state.h"

static LQR LQR_ctrl;

float K_p0[4] = {49.4497, 32.9892, -88.6024, 1.2490};    // K[0]
float K_p1[4] = {46.7838, -40.9180, -2.2866, 0.1495};    // K[1]
float K_p2[4] = {-124.1995, 172.1004, -80.9787, 3.7286}; // K[2]
float K_p3[4] = {-54.7260, 87.8378, -49.9663, 2.1165};   // K[3]
float K_p4[4] = {51.4053, 12.8363, -41.1310, 17.1957};   // K[4]
float K_p5[4] = {-6.1781, 6.7417, -3.1872, 0.8723};      // K[5]
float K_p6[4] = {865.2406, -816.2623, 254.3094, -6.3306};// K[6]
float K_p7[4] = {24.4881, -43.6621, 25.6240, -0.8286};   // K[7]
float K_p8[4] = {1258.4626, -1009.4580, 237.0215, -2.8866};// K[8]
float K_p9[4] = {757.4736, -618.4545, 151.2204, -1.8544};// K[9]
float K_p10[4] = {521.5005, -581.3964, 235.1001, 16.1606};// K[10]
float K_p11[4] = {-8.5828, 1.9856, 2.1412, 1.0268};      // K[11]


void LQR_K_calc(float *K, float L_0)
{
    float L0_2 = L_0 * L_0;
    float L0_3 = L0_2 * L_0;
    
    K[0] = K_p0[0]*L0_3 + K_p0[1]*L0_2 + K_p0[2]*L_0 + K_p0[3];
    K[1] = K_p1[0]*L0_3 + K_p1[1]*L0_2 + K_p1[2]*L_0 + K_p1[3];
    K[2] = K_p2[0]*L0_3 + K_p2[1]*L0_2 + K_p2[2]*L_0 + K_p2[3];
    K[3] = K_p3[0]*L0_3 + K_p3[1]*L0_2 + K_p3[2]*L_0 + K_p3[3];
    K[4] = K_p4[0]*L0_3 + K_p4[1]*L0_2 + K_p4[2]*L_0 + K_p4[3];
    K[5] = K_p5[0]*L0_3 + K_p5[1]*L0_2 + K_p5[2]*L_0 + K_p5[3];
    K[6] = K_p6[0]*L0_3 + K_p6[1]*L0_2 + K_p6[2]*L_0 + K_p6[3];
    K[7] = K_p7[0]*L0_3 + K_p7[1]*L0_2 + K_p7[2]*L_0 + K_p7[3];
    K[8] = K_p8[0]*L0_3 + K_p8[1]*L0_2 + K_p8[2]*L_0 + K_p8[3];
    K[9] = K_p9[0]*L0_3 + K_p9[1]*L0_2 + K_p9[2]*L_0 + K_p9[3];
    K[10] =	K_p10[0]*L0_3 +	K_p10[1]*L0_2 +	K_p10[2]*L_0 +  K_p10[3];
   	K[11] =	K_p11[0]*L0_3 +	K_p11[1]*L0_2 +	K_p11[2]*L_0 +	K_p11[3];
}

void LQR_calc(float target_x, float target_dx)
{
    const fk *fk_left = kinematics_get_left();
    const fk *fk_right = kinematics_get_right();
    const observer *obs = observer_get();

    // 计算自适应K矩阵
    float K_l[12], K_r[12];
    LQR_K_calc(K_l, fk_left->L_0);
    LQR_K_calc(K_r, fk_right->L_0);
    
    // 获取状态量并取反（配合 VMC 中 Tp 的加号组装）
    float theta_l = -fk_left->theta;
    float dtheta_l = -fk_left->dtheta;
    float theta_r = -fk_right->theta;
    float dtheta_r = -fk_right->dtheta;
    /* 位移/速度反馈换源: X_KF 滤波输出 (等价 luntui1 Body.x/Body.dx; 取负约定不变) */
    float x = -obs->kf_x;
    float dx = -obs->kf_dx;

    if (car_state_get() == STATE_BALANCE)
    {
        // ========== 正常平衡: 全量 6 状态反馈 ==========
        float Pitch  = -obs->pitch;
        float dPitch = -obs->dpitch;

        // 左腿控制（TL再取反正回来，Tp保持反的）
        LQR_ctrl.TL  = -(K_l[0]*theta_l + K_l[1]*dtheta_l + K_l[2]*(x-target_x) + K_l[3]*(dx-target_dx) + K_l[4]*Pitch + K_l[5]*dPitch);
        LQR_ctrl.Tpl = K_l[6]*theta_l + K_l[7]*dtheta_l + K_l[8]*(x-target_x) + K_l[9]*(dx-target_dx)+ K_l[10]*Pitch + K_l[11]*dPitch;

        // 右腿控制（TL再取反正回来，Tp保持反的）
        LQR_ctrl.TR  = -(K_r[0]*theta_r + K_r[1]*dtheta_r + K_r[2]*(x-target_x) + K_r[3]*(dx-target_dx) + K_r[4]*Pitch + K_r[5]*dPitch);
        LQR_ctrl.Tpr = K_r[6]*theta_r + K_r[7]*dtheta_r + K_r[8]*(x-target_x) + K_r[9]*(dx-target_dx) + K_r[10]*Pitch + K_r[11]*dPitch;
    }
    else if (car_state_get() == STATE_POSE_ADJUST)
    {
        // ========== 调姿: LQR 不参与, 轮矩/摆矩全 0 (Tp 由 pose_adjust 直接给 VMC) ==========
        LQR_ctrl.TL  = 0.0f;
        LQR_ctrl.TR  = 0.0f;
        LQR_ctrl.Tpl = 0.0f;
        LQR_ctrl.Tpr = 0.0f;
    }
    else
    {
        // ========== 起立: 轮矩 4项×5%, Tp 4项×1.5±8/12, 无 pitch 项 ==========
        float s_l = K_l[0]*theta_l + K_l[1]*dtheta_l + K_l[2]*(x-target_x) + K_l[3]*(dx-target_dx);
        float s_r = K_r[0]*theta_r + K_r[1]*dtheta_r + K_r[2]*(x-target_x) + K_r[3]*(dx-target_dx);
        LQR_ctrl.TL = -s_l * 0.05f;
        LQR_ctrl.TR = -s_r * 0.05f;

        float t_l = K_l[6]*theta_l + K_l[7]*dtheta_l + K_l[8]*(x-target_x) + K_l[9]*(dx-target_dx);
        float t_r = K_r[6]*theta_r + K_r[7]*dtheta_r + K_r[8]*(x-target_x) + K_r[9]*(dx-target_dx);
        LQR_ctrl.Tpl = t_l * 1.5f - 8.0f;
        LQR_ctrl.Tpr = t_r * 1.5f - 8.0f;
        if (fk_left->theta  < 0.0f) LQR_ctrl.Tpl += 12.0f;
        if (fk_right->theta < 0.0f) LQR_ctrl.Tpr += 12.0f;
    }
}

const LQR *LQR_get(void)
{
    return &LQR_ctrl;
}

