#ifndef _MODULE_INS_H_
#define _MODULE_INS_H_

#include <stdint.h>

#ifndef INS_TASK_STACK_SIZE
#define INS_TASK_STACK_SIZE 1024 /* 检测任务栈大小 */
#endif
#ifndef INS_TASK_PRIORITY
#define INS_TASK_PRIORITY 7 /* 检测任务优先级 */
#endif

// INS结构体
typedef struct
{
    // 四元数估计值，wxyz
    float q[4];
    // 机体坐标加速度
    float MotionAccel_b[3];
    // 绝对系加速度
    float MotionAccel_n[3];
    // 加速度低通滤波系数
    float AccelLPF;
    // bodyframe在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];
    // 加速度在机体系和XY两轴的夹角
    // float atanxz;
    // float atanyz;
    // 位姿
    float   euler_angle[3]; // roll pitch yaw(deg)
    float   euler_rad[3];   // roll pitch yaw(rad)
    float   YawTotalAngle_rad;// yaw总角度(rad)
    int32_t YawRoundCount;// yaw圆周次数
    // 采样时间间隔
    float    dt;
    uint32_t dwt_cnt;
    // 初始化标志
    uint8_t initialized;
} Ins_t;

/* 用于修正安装误差的参数 */
typedef struct
{
    uint8_t flag;
    float   scale[3];
    float   Yaw;
    float   Pitch;
    float   Roll;
} Ins_param_t;

/**
 * @description: 初始化INS模块
 * @return {*}
 */
void Module_INS_Init();
/**
 * @description: 获取Ins_t指针
 * @return {Ins_t *}，返回Ins_t指针
 */
const Ins_t *Module_INS_get();

#endif // _MODULE_INS_H_
