/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-13 13:14:16
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-14 16:53:43
 * @FilePath: /mas_embedded_threadx/modules/PCCOMM/module_pccomm.h
 * @Description:
 */
#ifndef _MODULE_PCCOMM_H_
#define _MODULE_PCCOMM_H_

#include <stdint.h>

#ifndef PCCOMM_TASK_STACK_SIZE
#define PCCOMM_TASK_STACK_SIZE 1024
#endif
#ifndef PCCOMM_TASK_PRIORITY
#define PCCOMM_TASK_PRIORITY 10
#endif

#ifndef PCCOMM_OFFLINE_ENABLE
#define PCCOMM_OFFLINE_ENABLE 1 /* 离线检测开启 */
#endif

#ifndef PCCOMM_LOOPBACK_TEST
#define PCCOMM_LOOPBACK_TEST 0 /* 1=大包回环测试模式 */
#endif

/* 虚拟串口分配 */
#define PCCOMM_AIM_PORT      0 /* 串口一: 自瞄 */
#define PCCOMM_NAV_PORT      1 /* 串口二: 导航 */

/* 定长帧头 (TX 方向 0x5A/0x5B 为板 → 上位机) */
#define PCCOMM_AIM_TX_HEADER 0x5A
#define PCCOMM_AIM_RX_HEADER 0xA5
#define PCCOMM_NAV_RX_HEADER 0xB5
#define PCCOMM_NAV_TX_HEADER 0x5B

#pragma pack(1)

/* 自瞄*/
typedef struct
{
    uint8_t mode; /* 自瞄模式 */
    float   q[4]; /* 云台姿态四元数 w,x,y,z */
} SendPacket;

typedef struct
{
    float   target_yaw;   /* 目标偏航角 (rad) */
    float   target_pitch; /* 目标俯仰角 (rad) */
    uint8_t fire_advice;  /* 0:不射击, 1:射击 */
} ReceivePacket;

/* 导航 */
typedef struct
{
    float   vx;        /* 导航速度 x (m/s) */
    float   vy;        /* 导航速度 y (m/s) */
    uint8_t nav_state; /* 0:无效, 1:有效 */
} NavPacket;

/* 导航发送: 裁判系统数据 (layout 同 boards 间 ChassisToGimbal_referee_t) */
typedef struct
{
    uint8_t  robot_color;      /* 机器人颜色 (0=红,1=蓝) */
    uint8_t  game_progress;    /* 比赛阶段 */
    uint16_t current_hp;       /* 当前血量 */
    uint16_t bullet_allow;     /* 可发射17mm弹丸剩余量 */
    uint16_t shooter_heat_pct; /* 枪管热量% */
} NavSendPacket;

#pragma pack()

_Static_assert(sizeof(NavSendPacket) == 8, "NavSendPacket must be 8 bytes");

/* 定长帧: header + payload + CRC16(小端) */
#define PCCOMM_AIM_FRAME_TX_SIZE (1 + sizeof(SendPacket) + 2)
#define PCCOMM_AIM_FRAME_RX_SIZE (1 + sizeof(ReceivePacket) + 2)
#define PCCOMM_NAV_FRAME_RX_SIZE (1 + sizeof(NavPacket) + 2)
#define PCCOMM_NAV_FRAME_TX_SIZE (1 + sizeof(NavSendPacket) + 2)

/**
 * @brief 初始化上位机通信模块
 * @return 0 成功 / -1 失败
 */
int Module_PCComm_Init(void);

/**
 * @brief 发送自瞄数据包到上位机 (串口一)
 * @param packet  发送包指针
 * @param timeout 超时时间（毫秒）
 */
void Module_PCComm_Send(const SendPacket *packet, uint32_t timeout);

/**
 * @brief 发送裁判系统数据包到导航上位机 (串口二)
 * @param packet  发送包指针
 * @param timeout 超时时间（毫秒）
 */
void Module_PCComm_Send_Nav(const NavSendPacket *packet, uint32_t timeout);

/**
 * @brief 接收自瞄数据包 (串口一)
 * @return 最近一帧有效包的指针
 */
ReceivePacket *Module_PCComm_Receive(void);

/**
 * @brief 接收导航数据包 (串口二)
 * @return 最近一帧有效包的指针
 */
NavPacket *Module_PCComm_Receive_Nav(void);

/**
 * @brief 获取minipc离线状态,这里只看串口一的更新
 * @return STATE_ONLINE (0) 或 STATE_OFFLINE (1)
 */
uint8_t Module_PCComm_Get_offline_state(void);

#endif // _MODULE_PCCOMM_H_
