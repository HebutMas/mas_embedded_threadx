#ifndef _POSE_ADJUST_H_
#define _POSE_ADJUST_H_

#include <stdint.h>

/* 调姿(摆腿)控制输出: VMC 的 POSE_ADJUST 分支直接消费 */
typedef struct
{
    float Tp_L; /* 左腿摆腿力矩 (N·m) */
    float Tp_R; /* 右腿摆腿力矩 (N·m) */
    float F_L;  /* 左腿支撑力 (N): 摆腿中 8N, 已到位 0(放松等待) */
    float F_R;  /* 右腿支撑力 (N) */
} PoseAdjust;

void               pose_adjust_init(void);
void               pose_adjust_control(void); /* 每周期调用, 内部自判状态 */
const PoseAdjust *pose_adjust_get(void);

#endif /* _POSE_ADJUST_H_ */
