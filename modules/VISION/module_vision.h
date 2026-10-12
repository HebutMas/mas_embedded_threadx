#ifndef _MODULE_VISION_H_
#define _MODULE_VISION_H_

#include <stdint.h>

#ifndef VISION_TASK_STACK_SIZE
#define VISION_TASK_STACK_SIZE 1024
#endif
#ifndef VISION_TASK_PRIORITY
#define VISION_TASK_PRIORITY 10
#endif
#ifndef VISION_OFFLINE_ENABLE
#define VISION_OFFLINE_ENABLE 1 /* 离线检测开启 */
#endif

#pragma pack(push, 1)

/* 自瞄*/
typedef struct
{
    uint8_t mode; /* 自瞄模式 */
    float   q[4]; /* 云台姿态四元数 w,x,y,z */
} VisionSendPacket;

typedef struct
{
    float   target_yaw;   /* 目标偏航角 (rad) */
    float   target_pitch; /* 目标俯仰角 (rad) */
    uint8_t fire_advice;  /* 0:不射击, 1:射击 */
} VisionReceivePacket;

#pragma pack(pop)

/**
 * @brief 初始化自瞄通信模块
 * @return 0 成功 / -1 失败
 */
int Module_Vision_Init(void);

/**
 * @brief 发送自瞄数据包到上位机 (串口一)
 * @param packet  发送包指针
 * @param timeout 超时时间（ThreadX tick）
 */
void Module_Vision_Send(const VisionSendPacket *packet, uint32_t timeout);

/**
 * @brief 接收自瞄数据包 (串口一)
 * @return 最近一帧有效包的指针
 */
VisionReceivePacket *Module_Vision_Receive(void);

/**
 * @brief 获取minipc离线状态,这里只看串口一的更新
 * @return STATE_ONLINE (0) 或 STATE_OFFLINE (1)
 */
uint8_t Module_Vision_Get_offline_state(void);

#endif // _MODULE_VISION_H_
