/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-07-25 19:41:18
 * @LastEditors: 550wsleep 1329258004@qq.com
 * @LastEditTime: 2026-09-17 17:39:34
 * @FilePath: \mas_embedded_threadx\modules\algorithm\serial_leg_solution\observer\observer.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "observer.h"
#include "module_ins.h"
#include "module_bmi088.h"
#include "solution_def.h"
#include "X_KF.h"
#include "kinematics.h"
#include "user_lib.h"
#include "motor_damiao.h"
#include "motor_dji.h"

static observer obs;  
const Ins_t           *ins = NULL;
const Bmi088_device_t *bmi = NULL;


void observer_init(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[])
{
    ins = Module_INS_get();
    bmi = Module_BMI088_get_device();
    obs.phi_1_L = motor_8009[3]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_L = motor_8009[3]->base.measure.speed_rad;
    obs.phi_4_L = motor_8009[0]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_L = motor_8009[0]->base.measure.speed_rad;
    obs.phi_1_R = - motor_8009[2]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_R = - motor_8009[2]->base.measure.speed_rad;
    obs.phi_4_R = - motor_8009[1]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_R = - motor_8009[1]->base.measure.speed_rad;
    /* 左轮反馈取负 (两轮镜像安装, 前进时左轮 3508 转速为负) 存疑待修改 */
    /* 3508 反馈为转子量, 轮子位移/速度需 ÷gear_ratio */
    obs.x_L     = -motor_3508[0]->base.measure.total_angle / motor_3508[0]->base.info.gear_ratio * WHEEL_R;
    obs.dx_L    = -motor_3508[0]->base.measure.speed_rad / motor_3508[0]->base.info.gear_ratio * WHEEL_R;
    obs.x_R     = motor_3508[1]->base.measure.total_angle / motor_3508[1]->base.info.gear_ratio * WHEEL_R;
    obs.dx_R    = motor_3508[1]->base.measure.speed_rad / motor_3508[1]->base.info.gear_ratio * WHEEL_R;

    // C板安装: gyro[0]=左, gyro[1]=后, gyro[2]=上
    // INS euler: euler_rad[0]=后, euler_rad[1]=左, euler_rad[2]=上
    // pitch=绕左转, roll=绕前转=-绕后转
    obs.pitch   = -ins->euler_rad[1];  /* 机体俯仰角 (绕左轴, 与 INS 约定相反) */
    obs.dpitch  = -bmi->gyro[0];       /* 机体俯仰角速度 */
    obs.roll    = -ins->euler_rad[0];
    obs.droll   = -bmi->gyro[1];
    obs.yaw     = ins->euler_rad[2];

    /* ========== 位移卡尔曼滤波 (纯滤波层) ========== */
    X_KF_init();
    obs.kf_dx_nofilter = 0.0f;
    obs.kf_dx = 0.0f;
    obs.kf_x = 0.0f;
    obs.initialized = 1;
}

void observer_update(DM_Motor_t *motor_8009[], DJI_Motor_t *motor_3508[])
{
    obs.phi_1_L = motor_8009[3]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_L = motor_8009[3]->base.measure.speed_rad;
    obs.phi_4_L = motor_8009[0]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_L = motor_8009[0]->base.measure.speed_rad;
    obs.phi_1_R = - motor_8009[2]->base.measure.single_round_angle+phi_1_offset*DEGREE_2_RAD;
    obs.dphi_1_R = - motor_8009[2]->base.measure.speed_rad;
    obs.phi_4_R = - motor_8009[1]->base.measure.single_round_angle+phi_4_offset*DEGREE_2_RAD;
    obs.dphi_4_R = - motor_8009[1]->base.measure.speed_rad;
    /* 左轮反馈取负 (两轮镜像安装, 前进时左轮 3508 转速为负) 存疑待修改 */
    /* 3508 反馈为转子量, 轮子位移/速度需 ÷gear_ratio */
    obs.x_L     = -motor_3508[0]->base.measure.total_angle / motor_3508[0]->base.info.gear_ratio * WHEEL_R;
    obs.dx_L    = -motor_3508[0]->base.measure.speed_rad / motor_3508[0]->base.info.gear_ratio * WHEEL_R;
    obs.x_R     = motor_3508[1]->base.measure.total_angle / motor_3508[1]->base.info.gear_ratio * WHEEL_R;
    obs.dx_R    = motor_3508[1]->base.measure.speed_rad / motor_3508[1]->base.info.gear_ratio * WHEEL_R;

    obs.pitch   = -ins->euler_rad[1];  /* 机体俯仰角 (绕左轴, 与 INS 约定相反) */
    obs.dpitch  = -bmi->gyro[0];       /* 机体俯仰角速度 */
    obs.roll    = -ins->euler_rad[0];
    obs.droll   = -bmi->gyro[1];
    obs.yaw     = ins->euler_rad[2];

    /* ---- 机体速度估计: 轮面速度 + 腿运动学贡献 (对应 luntui1 dx_nofilter) ----
     * dx = wheel*R + dL0*sin(theta) + L0*dtheta*cos(theta)
     * 注意: observer_update 先于 kinematics_calc 执行, 此处 fk 为上一拍结果 (滞后 2ms,
     * 滤波周期内无感)。左右腿公式同构; 符号链待实车验证 */
    const fk *fk_l = kinematics_get_left();
    const fk *fk_r = kinematics_get_right();
    float leg_dx_l = fk_l->dL_0 * arm_sin_f32(fk_l->theta) + fk_l->L_0 * fk_l->dtheta * arm_cos_f32(fk_l->theta);
    float leg_dx_r = fk_r->dL_0 * arm_sin_f32(fk_r->theta) + fk_r->L_0 * fk_r->dtheta * arm_cos_f32(fk_r->theta);

    obs.kf_dx_nofilter = (obs.dx_L + obs.dx_R) / 2.0f + (leg_dx_l + leg_dx_r) / 2.0f;

    /* ---- KF 滤波 + 位移积分器 (积分器在流水线层, 对应 luntui1 KF_x 在 x_observe.c) ----
     * dt 写死 0.002 (与 luntui1 DT 一致); 待 200μs delay 实测后再定是否改 DWT 实测 */
    X_KF_update(obs.kf_dx_nofilter, ins->MotionAccel_b[1]);
    obs.kf_dx = X_KF_get()->dx;
    obs.kf_x += obs.kf_dx * 0.002f;
}

const observer *observer_get(void)
{
    if (obs.initialized)
        return &obs;
    return NULL;
}

/* 掉线轻重置: 只清位移积分器 (等价 luntui1 if(flag!=0) KF_x=0)
 * 由 chassis_func 掉线分支调用; 流水线掉线期间停跑, 清一次即冻结, 恢复后从 0 重新积分 */
void observer_reset_kf_x(void)
{
    obs.kf_x = 0.0f;
}
