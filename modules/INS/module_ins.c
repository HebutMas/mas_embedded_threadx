#include "module_ins.h"
#include "QuaternionEKF.h"
#include "bsp_dwt.h"
#include "module_bmi088.h"
#include "user_lib.h"

#define LOG_TAG "module_ins"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#define INIT_SAMPLE_COUNT  100     // 初始化采样次数
#define INS_ACCEL_LPF_COEF 0.0085f // 加速度低通滤波系数
#define GRAVITY_CONSTANT   9.81f   // 重力加速度常数

static Ins_t                      ins;               /* INS状态结构体 */
static Bmi088_device_t           *bmi088_dev = NULL; /* BMI088设备指针 */
static Ins_param_t                IMU_Param;         /* 陀螺仪参数结构体 */
static TX_THREAD                  g_ins_task;
APPS_STACK_SECTION static uint8_t g_ins_stack[INS_TASK_STACK_SIZE];

// 基向量定义
static const float xb[3] = {1.0f, 0.0f, 0.0f};
static const float yb[3] = {0.0f, 1.0f, 0.0f};
static const float zb[3] = {0.0f, 0.0f, 1.0f};
// 重力向量定义
static const float gravity[3] = {0.0f, 0.0f, GRAVITY_CONSTANT};

/* 内部算法函数 */

// 三维向量归一化
float *Norm3d(float *v)
{
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}
// 计算模长
float NormOf3d(float *v) { return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
// 三维向量叉乘v1 x v2
void Cross3d(const float *v1, const float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}
// 三维向量点乘
float Dot3d(const float *v1, const float *v2) { return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]; }
/**
 * @brief 使用加速度计的数据初始化Roll和Pitch,Yaw置0
 * @param init_q4 输出的初始四元数
 */
void InitQuaternion(float *init_q4)
{
    float              acc_init[3]     = {0.0f, 0.0f, 0.0f};
    static const float gravity_norm[3] = {0.0f, 0.0f, 1.0f}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float              axis_rot[3]     = {0.0f, 0.0f, 0.0f}; // 旋转轴

    // 读取多次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < INIT_SAMPLE_COUNT; ++i)
    {
        Module_BMI088_get_accel();
        acc_init[0] += bmi088_dev->acc[0];
        acc_init[1] += bmi088_dev->acc[1];
        acc_init[2] += bmi088_dev->acc[2];
        BSP_DWT_Delay(0.001f);
    }

    // 计算平均值
    acc_init[0] /= INIT_SAMPLE_COUNT;
    acc_init[1] /= INIT_SAMPLE_COUNT;
    acc_init[2] /= INIT_SAMPLE_COUNT;

    // 归一化加速度向量
    Norm3d(acc_init);

    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);

    // 转换为四元数 (轴角公式)
    init_q4[0] = cosf(angle / 2.0f);
    init_q4[1] = axis_rot[0] * sinf(angle / 2.0f);
    init_q4[2] = axis_rot[1] * sinf(angle / 2.0f);
    init_q4[3] = axis_rot[2] * sinf(angle / 2.0f);
}
/**
 * @brief 将三维向量从机体坐标系转换到地球坐标系
 * @param vecBF 机体坐标系向量
 * @param vecEF 地球坐标系向量
 * @param q 四元数
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] =
        2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] + (q[1] * q[2] - q[0] * q[3]) * vecBF[1] + (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] =
        2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] + (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] =
        2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] + (q[2] * q[3] + q[0] * q[1]) * vecBF[1] + (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}
/**
 * @brief 将三维向量从地球坐标系转换到机体坐标系
 * @param vecEF 地球坐标系向量
 * @param vecBF 机体坐标系向量
 * @param q 四元数
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] =
        2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] + (q[1] * q[2] + q[0] * q[3]) * vecEF[1] + (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] =
        2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] + (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] + (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] =
        2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] + (q[2] * q[3] - q[0] * q[1]) * vecEF[1] + (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}
/**
 * @brief IMU参数修正，用于修正IMU安装误差与标度因数误差
 * @param gyro  角速度
 * @param accel 加速度
 */
void IMU_Param_Correction(float gyro[3], float accel[3])
{
    static float lastYawOffset   = 0.0f;
    static float lastPitchOffset = 0.0f;
    static float lastRollOffset  = 0.0f;
    static float c_11 = 1.0f, c_12 = 0.0f, c_13 = 0.0f;
    static float c_21 = 0.0f, c_22 = 1.0f, c_23 = 0.0f;
    static float c_31 = 0.0f, c_32 = 0.0f, c_33 = 1.0f;

    // 检查是否需要重新计算变换矩阵
    if (fabsf(IMU_Param.Yaw - lastYawOffset) > 0.001f || fabsf(IMU_Param.Pitch - lastPitchOffset) > 0.001f ||
        fabsf(IMU_Param.Roll - lastRollOffset) > 0.001f || IMU_Param.flag)
    {
        // 角度转弧度
        const float yawRad   = IMU_Param.Yaw / 57.295779513f;
        const float pitchRad = IMU_Param.Pitch / 57.295779513f;
        const float rollRad  = IMU_Param.Roll / 57.295779513f;

        // 计算三角函数值
        const float cosYaw   = arm_cos_f32(yawRad);
        const float cosPitch = arm_cos_f32(pitchRad);
        const float cosRoll  = arm_cos_f32(rollRad);
        const float sinYaw   = arm_sin_f32(yawRad);
        const float sinPitch = arm_sin_f32(pitchRad);
        const float sinRoll  = arm_sin_f32(rollRad);

        // 计算变换矩阵元素
        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;

        IMU_Param.flag = 0;
    }

    // 应用标度因子并进行坐标变换
    float gyro_temp[3];
    gyro_temp[0] = gyro[0] * IMU_Param.scale[0];
    gyro_temp[1] = gyro[1] * IMU_Param.scale[1];
    gyro_temp[2] = gyro[2] * IMU_Param.scale[2];

    gyro[0] = c_11 * gyro_temp[0] + c_12 * gyro_temp[1] + c_13 * gyro_temp[2];
    gyro[1] = c_21 * gyro_temp[0] + c_22 * gyro_temp[1] + c_23 * gyro_temp[2];
    gyro[2] = c_31 * gyro_temp[0] + c_32 * gyro_temp[1] + c_33 * gyro_temp[2];

    // 对加速度进行同样的变换
    const float accel_temp[3] = {accel[0], accel[1], accel[2]};

    accel[0] = c_11 * accel_temp[0] + c_12 * accel_temp[1] + c_13 * accel_temp[2];
    accel[1] = c_21 * accel_temp[0] + c_22 * accel_temp[1] + c_23 * accel_temp[2];
    accel[2] = c_31 * accel_temp[0] + c_32 * accel_temp[1] + c_33 * accel_temp[2];

    lastYawOffset   = IMU_Param.Yaw;
    lastPitchOffset = IMU_Param.Pitch;
    lastRollOffset  = IMU_Param.Roll;
}

static void ins_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        if (ins.initialized)
        {
            // 获取时间间隔
            ins.dt = BSP_DWT_GetDeltaT(&ins.dwt_cnt);

            // 获取IMU数据
            Module_BMI088_get_accel();
            Module_BMI088_get_gyro();

            // IMU参数修正（安装误差修正）
            IMU_Param_Correction(bmi088_dev->gyro, bmi088_dev->acc);

            // 核心函数,EKF更新四元数
            IMU_QuaternionEKF_Update(bmi088_dev->gyro[0], bmi088_dev->gyro[1], bmi088_dev->gyro[2], bmi088_dev->acc[0], bmi088_dev->acc[1],
                                     bmi088_dev->acc[2], ins.dt);

            // 复制四元数结果
            memcpy(ins.q, QEKF_INS.q, sizeof(QEKF_INS.q));

            // 机体系基向量转换到导航坐标系（本例选取惯性系为导航系）
            BodyFrameToEarthFrame(xb, ins.xn, ins.q);
            BodyFrameToEarthFrame(yb, ins.yn, ins.q);
            BodyFrameToEarthFrame(zb, ins.zn, ins.q);

            // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
            float gravity_b[3];
            EarthFrameToBodyFrame(gravity, gravity_b, ins.q);

            // 计算运动加速度（去除重力分量），并进行低通滤波
            for (uint8_t i = 0; i < 3; ++i)
            {
                const float acc_diff = bmi088_dev->acc[i] - gravity_b[i];
                ins.MotionAccel_b[i] = acc_diff * ins.dt / (ins.AccelLPF + ins.dt) + ins.MotionAccel_b[i] * ins.AccelLPF / (ins.AccelLPF + ins.dt);
            }

            // 将运动加速度转换回导航系
            BodyFrameToEarthFrame(ins.MotionAccel_b, ins.MotionAccel_n, ins.q);

            ins.euler_angle[0] = QEKF_INS.Roll;
            ins.euler_angle[1] = QEKF_INS.Pitch;
            ins.euler_angle[2] = QEKF_INS.Yaw;

            ins.euler_rad[0] = QEKF_INS.Roll * DEGREE_2_RAD;
            ins.euler_rad[1] = QEKF_INS.Pitch * DEGREE_2_RAD;
            ins.euler_rad[2] = QEKF_INS.Yaw * DEGREE_2_RAD;

            ins.YawTotalAngle_rad = QEKF_INS.YawTotalAngle * DEGREE_2_RAD;
            ins.YawRoundCount     = QEKF_INS.YawRoundCount;

            Module_BMI088_get_temp();
            // 温度控制
            Module_BMI088_temp_ctrl();
            tx_thread_sleep(1);
        }
        else
        {
            tx_thread_sleep(100);
        }
    }
}

/* 对外接口 */
int Module_INS_Init(void)
{
    bmi088_dev = Module_BMI088_get_device();
    if (bmi088_dev == NULL)
    {
        LOG_E("Failed to init bmi088!");
        return -1;
    }
    // 初始化IMU参数修正结构体
    IMU_Param.scale[0] = 1.0f;
    IMU_Param.scale[1] = 1.0f;
    IMU_Param.scale[2] = 1.0f;
    IMU_Param.Yaw      = 0.0f;
    IMU_Param.Pitch    = 0.0f;
    IMU_Param.Roll     = 0.0f;
    IMU_Param.flag     = 1;
    // 初始化四元数
    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    // 初始化EKF
    IMU_QuaternionEKF_Init(init_quaternion, 10.0f, 0.001f, 1000000.0f, 0.9996f, 0.0f);
    // 设置加速度低通滤波系数
    ins.AccelLPF = INS_ACCEL_LPF_COEF;
    // 初始化标志
    ins.initialized = 1;

    UINT status = tx_thread_create(&g_ins_task, "INS Task", ins_task_entry, 0, g_ins_stack, INS_TASK_STACK_SIZE, INS_TASK_PRIORITY, INS_TASK_PRIORITY,
                                   TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("Failed to create INS task");
        return -1;
    }

    return 0;
}

const Ins_t *Module_INS_get()
{
    if (ins.initialized)
    {
        return &ins;
    }
    return NULL;
}
