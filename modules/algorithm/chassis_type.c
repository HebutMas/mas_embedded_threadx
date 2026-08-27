#include "chassis_type.h"
#include "motor_dji.h"
#include "user_lib.h"
#include <math.h>

#define TWO_PI  (2.0f * PI)
#define HALF_PI (PI / 2.0f)
#define SQRT2_2 0.70710678f

/* 内部辅助函数 */

/* 角度回环 - 将角度限制在 [-max/2, +max/2) 范围内 */
static void AngleLoop_f(float *angle, float max)
{
    while ((*angle < -(max / 2)) || (*angle > (max / 2)))
    {
        if (*angle < -(max / 2))
            *angle += max;
        else if (*angle > (max / 2))
            *angle -= max;
    }
}

/* 对外函数 */

void Chassis_Swerve_Calc(DJI_Motor_t *motors[8], const Chassis_Swerve_Config_s *cfg, float vx, float vy, float vw)
{
    const float *align_rad = cfg->align_rad; /* LF LB RB RF */

    static float last_target_angle_rad[4] = {0};

    uint8_t is_zero_speed = (fabsf(vx) < 0.01f && fabsf(vy) < 0.01f && fabsf(vw) < 0.01f);

    for (int i = 0; i < 4; i++)
    {
        float local_vx = 0.0f, local_vy = 0.0f;

        /* 运动学分解: 底盘速度 → 各轮组局部坐标系 */
        switch (i)
        {
        case 0: /* 左前 LF: (+x, +y) */
            local_vx = vx + vw * (cfg->wheel_r * SQRT2_2);
            local_vy = vy + vw * (cfg->wheel_r * SQRT2_2);
            break;
        case 1: /* 左后 LB: (+x, -y) */
            local_vx = vx + vw * (cfg->wheel_r * SQRT2_2);
            local_vy = vy - vw * (cfg->wheel_r * SQRT2_2);
            break;
        case 2: /* 右后 RB: (-x, -y) */
            local_vx = vx - vw * (cfg->wheel_r * SQRT2_2);
            local_vy = vy - vw * (cfg->wheel_r * SQRT2_2);
            break;
        case 3: /* 右前 RF: (-x, +y) */
            local_vx = vx - vw * (cfg->wheel_r * SQRT2_2);
            local_vy = vy + vw * (cfg->wheel_r * SQRT2_2);
            break;
        }

        /* 驱动电机 (3508) 目标速度 */
        float velocity_vector  = sqrtf(local_vx * local_vx + local_vy * local_vy);
        float target_speed_rad = (velocity_vector / cfg->radius_wheel_m) * cfg->decele_ratio;

        /* 转向电机 (6020) 目标角度 */
        float  target_steer_rad;
        int8_t drct_factor = 1;

        if (is_zero_speed)
        {
            target_steer_rad = last_target_angle_rad[i];
        }
        else
        {
            float vector_rad       = atan2f(local_vy, local_vx);
            float target_abs_angle = align_rad[i] + vector_rad;
            AngleLoop_f(&target_abs_angle, TWO_PI);

            float current_single = motors[i + 4]->base.measure.total_angle;
            AngleLoop_f(&current_single, TWO_PI);

            float diff = target_abs_angle - current_single;
            AngleLoop_f(&diff, TWO_PI);

            if (diff > HALF_PI)
            {
                target_steer_rad = target_abs_angle - PI;
                drct_factor      = -1;
            }
            else if (diff < -HALF_PI)
            {
                target_steer_rad = target_abs_angle + PI;
                drct_factor      = -1;
            }
            else
            {
                target_steer_rad = target_abs_angle;
            }

            AngleLoop_f(&target_steer_rad, TWO_PI);
            last_target_angle_rad[i] = target_steer_rad;
        }

        Motor_DJI_SetRef(motors[i], target_speed_rad * (float)drct_factor);

        float delta = target_steer_rad - motors[i + 4]->base.measure.total_angle;
        AngleLoop_f(&delta, TWO_PI);
        Motor_DJI_SetRef(motors[i + 4], motors[i + 4]->base.measure.total_angle + delta);
    }
}

void Chassis_Mecanum_Calc(DJI_Motor_t *motors[4], const Chassis_Diff_Config_s *cfg, float vx, float vy, float vw)
{
    const float L            = (cfg->wheel_base_x + cfg->wheel_base_y) / 2.0f;
    const float speed_factor = cfg->decele_ratio / cfg->wheel_radius;

    float wheel_speed[4];
    wheel_speed[0] = (+vx - vy - vw * L) * speed_factor;
    wheel_speed[3] = (+vx + vy + vw * L) * speed_factor;
    wheel_speed[1] = (+vx + vy - vw * L) * speed_factor;
    wheel_speed[2] = (+vx - vy + vw * L) * speed_factor;

    for (int i = 0; i < 4; i++) Motor_DJI_SetRef(motors[i], wheel_speed[i]);
}

void Chassis_Omni_Calc(DJI_Motor_t *motors[4], const Chassis_Diff_Config_s *cfg, float vx, float vy, float vw)
{
    const float a            = cfg->wheel_base_x / 2.0f;
    const float b            = cfg->wheel_base_y / 2.0f;
    const float rot_diff     = vw * (a - b);
    const float speed_factor = cfg->decele_ratio / (cfg->wheel_radius * SQRT2_2);

    float wheel_speed[4];
    wheel_speed[0] = (+vx + vy + rot_diff) * speed_factor;
    wheel_speed[3] = (+vx - vy - rot_diff) * speed_factor;
    wheel_speed[1] = (-vx + vy - rot_diff) * speed_factor;
    wheel_speed[2] = (-vx - vy + rot_diff) * speed_factor;

    for (int i = 0; i < 4; i++) Motor_DJI_SetRef(motors[i], wheel_speed[i]);
}


Chassis_Velocity_s Chassis_Swerve_Fwd(const DJI_Motor_t *motors[8], const Chassis_Swerve_Config_s *cfg)
{
    const float a  = cfg->wheel_r * SQRT2_2;
    const float r2 = cfg->wheel_r * cfg->wheel_r;
    const float R  = cfg->radius_wheel_m / cfg->decele_ratio;

    float vx = 0.0f, vy = 0.0f, vw = 0.0f;
    float px[4] = {+a, +a, -a, -a};
    float py[4] = {+a, -a, -a, +a};

    for (int i = 0; i < 4; i++)
    {
        float steer = motors[i + 4]->base.measure.total_angle;
        AngleLoop_f(&steer, TWO_PI);

        float vi  = motors[i]->base.measure.speed_rad * R;
        float vix = vi * cosf(steer);
        float viy = vi * sinf(steer);

        vx += vix;
        vy += viy;
        vw += (px[i] * viy - py[i] * vix) / r2;
    }

    Chassis_Velocity_s vel;
    vel.vx = vx / 4.0f;
    vel.vy = vy / 4.0f;
    vel.vw = vw / 4.0f;
    return vel;
}

Chassis_Velocity_s Chassis_Mecanum_Fwd(const DJI_Motor_t *motors[4], const Chassis_Diff_Config_s *cfg)
{
    const float L = (cfg->wheel_base_x + cfg->wheel_base_y) / 2.0f;
    const float R = cfg->wheel_radius / cfg->decele_ratio;

    float w[4];
    for (int i = 0; i < 4; i++) w[i] = motors[i]->base.measure.speed_rad;

    Chassis_Velocity_s vel;
    vel.vx = (R / 4.0f) * (w[0] + w[1] + w[2] + w[3]);
    vel.vy = (R / 4.0f) * (-w[0] + w[1] - w[2] + w[3]);
    vel.vw = (R / (4.0f * L)) * (-w[0] - w[1] + w[2] + w[3]);
    return vel;
}

Chassis_Velocity_s Chassis_Omni_Fwd(const DJI_Motor_t *motors[4], const Chassis_Diff_Config_s *cfg)
{
    const float a = cfg->wheel_base_x / 2.0f;
    const float b = cfg->wheel_base_y / 2.0f;
    const float R = cfg->wheel_radius / cfg->decele_ratio;

    float w[4];
    for (int i = 0; i < 4; i++) w[i] = motors[i]->base.measure.speed_rad;

    Chassis_Velocity_s vel;
    vel.vx = (w[0] - w[1] - w[2] + w[3]) * R / (2.0f * SQRT2_2);
    vel.vy = (w[0] + w[1] - w[2] - w[3]) * R / (2.0f * SQRT2_2);

    if (fabsf(a - b) > 0.001f)
        vel.vw = (w[0] - w[1] - w[2] + w[3]) * R * SQRT2_2 / (4.0f * (a - b));
    else
        vel.vw = 0.0f;

    return vel;
}


void Chassis_Odom_Reset(Chassis_Odom_s *odom)
{
    odom->x   = 0.0f;
    odom->y   = 0.0f;
    odom->yaw = 0.0f;
}

void Chassis_Odom_Update(Chassis_Odom_s *odom, const Chassis_Velocity_s *vel, float dt)
{
    float cos_yaw = cosf(odom->yaw);
    float sin_yaw = sinf(odom->yaw);

    /* 机体速度 → 全局坐标系积分 */
    odom->x += (vel->vx * cos_yaw - vel->vy * sin_yaw) * dt;
    odom->y += (vel->vx * sin_yaw + vel->vy * cos_yaw) * dt;
    odom->yaw += vel->vw * dt;
}

void Chassis_Swerve_Odom(Chassis_Odom_s *odom, const DJI_Motor_t *motors[8], const Chassis_Swerve_Config_s *cfg, float dt)
{
    Chassis_Velocity_s vel = Chassis_Swerve_Fwd(motors, cfg);
    Chassis_Odom_Update(odom, &vel, dt);
}

void Chassis_Mecanum_Odom(Chassis_Odom_s *odom, const DJI_Motor_t *motors[4], const Chassis_Diff_Config_s *cfg, float dt)
{
    Chassis_Velocity_s vel = Chassis_Mecanum_Fwd(motors, cfg);
    Chassis_Odom_Update(odom, &vel, dt);
}

void Chassis_Omni_Odom(Chassis_Odom_s *odom, const DJI_Motor_t *motors[4], const Chassis_Diff_Config_s *cfg, float dt)
{
    Chassis_Velocity_s vel = Chassis_Omni_Fwd(motors, cfg);
    Chassis_Odom_Update(odom, &vel, dt);
}
