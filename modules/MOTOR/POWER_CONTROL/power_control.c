#include "power_control.h"
#include "user_lib.h"
#include <math.h>
#include <string.h>

#define LOG_TAG "PowerCtrl"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 可调参数 */
#define STEER_RATIO          0.8f   /* 舵向轮最大功率占比 */
#define ERROR_THRESHOLD      20.0f  /* 完全按误差分配: sumErr 上限 (rad/s) */
#define PROP_THRESHOLD       15.0f  /* 完全按比例分配: sumErr 下限 (rad/s) */
#define IDLE_WHEEL_ERR_RAD   10.47f /* 悬空轮保护: 速度误差阈值 */
#define CLAMP(x, lo, hi)     ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define ABS(x)               ((x) >= 0.0f ? (x) : -(x))

typedef struct
{
    Motor_Base       *motor;
    PowerCtrl_Role_e  role;
    PowerCtrl_Param_t param;
    float             raw_power;      /* 原始估算功率 */
    float             assigned_power; /* 分配后功率     */
} PC_Node_t;

static PC_Node_t    s_nodes[POWER_CTRL_MAX_MOTORS];
static uint8_t      s_count;

static float        s_power_limit   = 45.0f;
static float        s_buffer_energy = 60.0f;
static uint8_t      s_use_buffer    = 1;
static PIDInstance  s_buffer_pid;

/**
 * @brief 对一组电机执行混合权重功率分配
 * @param nodes       电机节点数组
 * @param N           节点数量
 * @param max_power   该组可用功率上限 (W)
 * @param cmd_sum_out [out] 命令功率之和, 可为 NULL
 */
static void limit_group(PC_Node_t **nodes, uint8_t N, float max_power, float *cmd_sum_out)
{
    if (N == 0)
    {
        if (cmd_sum_out) *cmd_sum_out = 0.0f;
        return;
    }

    float omega[POWER_CTRL_MAX_MOTORS];  /* 输出轴角速度 (rad/s) */
    float tau[POWER_CTRL_MAX_MOTORS];    /* 输出轴扭矩   (Nm) */
    float cmdPow[POWER_CTRL_MAX_MOTORS]; /* 估算功率     (W) */
    float errRad[POWER_CTRL_MAX_MOTORS]; /* |ref - speed| (rad/s) */

    float sum_cmd = 0.0f, allocatable = 0.0f, sum_pos = 0.0f, sum_err = 0.0f;

    for (uint8_t i = 0; i < N; i++)
    {
        Motor_Base *m  = nodes[i]->motor;
        float       k1 = nodes[i]->param.k1;
        float       k2 = nodes[i]->param.k2;
        float       k3 = nodes[i]->param.k3;

        omega[i]  = m->measure.speed_rad / m->info.gear_ratio;
        tau[i]    = m->controller.output_torque; /* 通用: 命令扭矩 (Nm) */

        /* P = τ·Ω + k1·|Ω| + k2·τ² + k3 */
        cmdPow[i] = tau[i] * omega[i] + k1 * ABS(omega[i]) + k2 * tau[i] * tau[i] + k3;

        nodes[i]->raw_power = cmdPow[i];
        sum_cmd += cmdPow[i];
        errRad[i] = ABS(m->controller.ref - m->measure.speed_rad);

        if (cmdPow[i] <= 0.0f)
            allocatable += (-cmdPow[i]);
        else
        {
            sum_pos += cmdPow[i];
            sum_err += errRad[i];
        }
    }

    if (cmd_sum_out) *cmd_sum_out = sum_cmd;

    /* 不超限: 保持原始输出 */
    if (sum_cmd <= max_power) return;

    /* 超限: 分配功率预算 */
    allocatable += max_power;

    /* 误差置信度: 0=按功率比例, 1=按误差比例, 中间线性插值 */
    float confidence;
    if (sum_err >= ERROR_THRESHOLD)
        confidence = 1.0f;
    else if (sum_err > PROP_THRESHOLD)
        confidence = (sum_err - PROP_THRESHOLD) / (ERROR_THRESHOLD - PROP_THRESHOLD);
    else
        confidence = 0.0f;

    for (uint8_t i = 0; i < N; i++)
    {
        if (cmdPow[i] <= 0.0f) continue; /* 再生功率不限制 */

        Motor_Base *m = nodes[i]->motor;

        /* 混合权重 */
        float w_err  = (sum_err  > 1e-5f) ? (errRad[i] / sum_err)  : (1.0f / N);
        float w_prop = (sum_pos  > 1e-5f) ? (cmdPow[i] / sum_pos)  : (1.0f / N);
        float w      = confidence * w_err + (1.0f - confidence) * w_prop;

        float p_alloc = w * allocatable;
        nodes[i]->assigned_power = p_alloc;

        /* 悬空轮保护: 速度已接近目标的电机不抢占接地轮功率 */
        if (errRad[i] < IDLE_WHEEL_ERR_RAD && p_alloc > cmdPow[i])
            p_alloc = cmdPow[i];

        /* 反解扭矩: k2·τ² + Ω·τ + (k1|Ω| + k3 - p_alloc) = 0 */
        float Omega  = omega[i];
        float a_coef = nodes[i]->param.k2;
        float b_coef = Omega;
        float c_coef = nodes[i]->param.k1 * ABS(Omega) + nodes[i]->param.k3 - p_alloc;

        float tau_new;
        if (a_coef < 1e-8f)
        {
            tau_new = (ABS(Omega) > 1e-5f)
                          ? (p_alloc - nodes[i]->param.k1 * ABS(Omega) - nodes[i]->param.k3) / Omega
                          : 0.0f;
        }
        else
        {
            float delta = b_coef * b_coef - 4.0f * a_coef * c_coef;
            if (delta < 0.0f)
            {
                tau_new = -b_coef / (2.0f * a_coef);
            }
            else
            {
                float sqrt_delta = sqrtf(delta);
                float root1      = (-b_coef + sqrt_delta) / (2.0f * a_coef);
                float root2      = (-b_coef - sqrt_delta) / (2.0f * a_coef);
                tau_new = (m->controller.output_torque >= 0.0f) ? root1 : root2;
            }
        }

        tau_new = CLAMP(tau_new, -m->info.max_torque, m->info.max_torque);
        m->controller.output_torque = tau_new;
    }
}

/* 对外接口 */

void PowerControl_Register(Motor_Base *motor, PowerCtrl_Role_e role, PowerCtrl_Param_t param)
{
    if (!motor || s_count >= POWER_CTRL_MAX_MOTORS) return;

    /* 已注册则更新参数 */
    for (uint8_t i = 0; i < s_count; i++)
    {
        if (s_nodes[i].motor == motor)
        {
            s_nodes[i].role  = role;
            s_nodes[i].param = param;
            return;
        }
    }

    s_nodes[s_count].motor = motor;
    s_nodes[s_count].role  = role;
    s_nodes[s_count].param = param;
    s_count++;
}

void PowerControl_SetLimit(float power_limit_w, float buffer_energy_j, uint8_t use_buffer)
{
    s_power_limit   = power_limit_w;
    s_buffer_energy = buffer_energy_j;
    s_use_buffer    = use_buffer;
}

void PowerControl_Update(void)
{
    if (s_count == 0) return;

    /* 按角色分组 */
    PC_Node_t *drives[POWER_CTRL_MAX_MOTORS];
    PC_Node_t *steers[POWER_CTRL_MAX_MOTORS];
    uint8_t    n_drive = 0, n_steer = 0;

    for (uint8_t i = 0; i < s_count; i++)
    {
        if (s_nodes[i].role == PC_ROLE_DRIVE)
            drives[n_drive++] = &s_nodes[i];
        else if (s_nodes[i].role == PC_ROLE_STEER)
            steers[n_steer++] = &s_nodes[i];
    }

    /* 计算可用功率 */
    float effective = s_power_limit;
    if (s_use_buffer)
    {
        if (s_buffer_pid.Kp == 0.0f) /* lazy init */
        {
            PID_Init_Config_s cfg = {.MaxOut = 50, .DeadBand = 5, .Kp = 5, .Improve = 0x01};
            PIDInit(&s_buffer_pid, &cfg);
        }
        PIDCalculate(&s_buffer_pid, s_buffer_energy, 30.0f);
        effective -= s_buffer_pid.Output;
    }

    /* 分配 */
    if (n_steer == 0)
    {
        limit_group(drives, n_drive, effective, NULL);
    }
    else
    {
        float steer_sum = 0.0f;
        limit_group(steers, n_steer, effective * STEER_RATIO, &steer_sum);

        float steer_actual = (steer_sum < effective * STEER_RATIO) ? steer_sum : effective * STEER_RATIO;
        float drive_limit  = effective - steer_actual;
        if (drive_limit < 0.0f) drive_limit = 0.0f;

        limit_group(drives, n_drive, drive_limit, NULL);
    }
}
