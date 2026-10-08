/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-13 13:14:16
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-14 16:53:43
 * @FilePath: /mas_embedded_threadx/modules/VISION/module_vision.h
 * @Description:
 */
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

/* 帧头区分方向 */
#define VISION_TX_HEADER 0x5A
#define VISION_RX_HEADER 0xA5

#pragma pack(1)

/* 发送包 */
typedef struct
{
    uint8_t mode; /* 自瞄模式 */
    float   q[4]; /* 云台姿态四元数 w,x,y,z */
} SendPacket;

/* 接收包 */
typedef struct
{
    float   target_yaw;   /* 目标偏航角 (rad) */
    float   target_pitch; /* 目标俯仰角 (rad) */
    uint8_t fire_advice;  /* 0:不射击, 1:射击 */
} ReceivePacket;

#pragma pack()

/* 定长帧: header + payload + CRC16(小端) */
#define VISION_FRAME_TX_SIZE (1 + sizeof(SendPacket) + 2)
#define VISION_FRAME_RX_SIZE (1 + sizeof(ReceivePacket) + 2)

/**
 * @brief 初始化视觉模块
 * @return 0 成功 / -1 失败
 */
int Module_Vision_Init(void);

/**
 * @brief 发送数据包到上位机
 * @param packet  发送包指针
 * @param timeout 超时时间（毫秒）
 */
void Module_Vision_Send(const SendPacket *packet, uint32_t timeout);

/**
 * @brief 接收上位机数据包
 * @return 最近一帧有效包的指针
 */
ReceivePacket *Module_Vision_Receive(void);

/**
 * @brief 获取视觉设备离线状态
 * @return STATE_ONLINE (0) 或 STATE_OFFLINE (1)
 */
uint8_t Module_Vision_Get_offline_state(void);

#endif // _MODULE_VISION_H_
