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

#pragma pack(1)

/* 发送包: MCU → 上位机 */
typedef struct
{
    uint8_t header; /* 帧头 0xAA */
    uint8_t mode;   /* 模式 */
    float   q[4];   /* 四元数 w,x,y,z */
    uint8_t tail;   /* 帧尾 0x55 */
} SendPacket;

/* 接收包: 上位机 → MCU */
typedef struct
{
    uint8_t header;       /* 帧头 0xBB */
    uint8_t found;        /* 是否找到目标 */
    uint8_t fire_advice;  /* 开火建议 */
    float   target_yaw;   /* 目标偏航角 (rad) */
    float   target_pitch; /* 目标俯仰角 (rad) */
    float   vx;          /* 目标速度 x (m/s) */
    float   vy;          /* 目标速度 y (m/s) */
    uint8_t nav_state;    /* 导航状态 */
    uint8_t tail;         /* 帧尾 0x55 */
} ReceivePacket;

#pragma pack()

/**
 * @brief 初始化视觉模块
 */
void Module_Vision_Init(void);

/**
 * @brief 发送数据包到上位机
 * @param packet  发送包指针
 * @param timeout 超时时间（毫秒）
 */
void Module_Vision_Send(SendPacket *packet,uint32_t timeout);

/**
 * @brief 接收上位机数据包
 * @return 有效包指针, 无新数据时返回 NULL
 */
ReceivePacket *Module_Vision_Receive(void);

/**
 * @brief 获取视觉设备离线状态
 * @return STATE_ONLINE (0) 或 STATE_OFFLINE (1)
 */
uint8_t Module_Vision_Get_offline_state(void);

#endif // _MODULE_VISION_H_
