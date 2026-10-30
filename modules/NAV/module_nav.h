#ifndef _MODULE_NAV_H_
#define _MODULE_NAV_H_

#include <stdint.h>

#ifndef NAV_TASK_STACK_SIZE
#define NAV_TASK_STACK_SIZE 1024
#endif
#ifndef NAV_TASK_PRIORITY
#define NAV_TASK_PRIORITY 10
#endif
#ifndef NAV_OFFLINE_ENABLE
#define NAV_OFFLINE_ENABLE 1 /* 离线检测开启 */
#endif

#pragma pack(push, 1)

/* 导航 */
typedef struct
{
    float   vx;            /* 导航速度 x (m/s) */
    float   vy;            /* 导航速度 y (m/s) */
    float   small_yaw_ref; /* 相对当前 LIO 朝向的偏角 (deg)，逆时针为正，[-180, 180] */
    uint8_t follow_gimbal; /* 0=不跟随，1=请求进入跟随云台模式 */
} NavReceivePacket;

/* 导航发送: 裁判系统数据 (layout 同 boards 间 ChassisToGimbal_referee_t) */
typedef struct
{
    uint8_t  robot_color;      /* 机器人颜色 (0=红,1=蓝) */
    uint8_t  game_progress;    /* 比赛阶段 */
    uint16_t current_hp;       /* 当前血量 */
    uint16_t bullet_allow;     /* 可发射17mm弹丸剩余量 */
    uint16_t shooter_heat_pct; /* 枪管热量% */
} NavSendPacket;

#pragma pack(pop)

_Static_assert(sizeof(NavReceivePacket) == 13, "NavReceivePacket must be 13 bytes");
_Static_assert(sizeof(NavSendPacket) == 8, "NavSendPacket must be 8 bytes");

/**
 * @brief 初始化导航通信模块
 * @return 0 成功 / -1 失败
 */
int Module_Nav_Init(void);

/**
 * @brief 发送裁判系统数据包到导航上位机 (串口二)
 * @param packet  发送包指针
 * @param timeout 超时时间（ThreadX tick）
 */
void Module_Nav_Send(const NavSendPacket *packet, uint32_t timeout);

/**
 * @brief 接收导航数据包 (串口二)
 * @return 最近一帧有效包的指针
 */
NavReceivePacket *Module_Nav_Receive(void);

/**
 * @brief 获取minipc_nav离线状态,这里只看串口二的更新
 * @return STATE_ONLINE (0) 或 STATE_OFFLINE (1)
 */
uint8_t Module_Nav_Get_offline_state(void);

#endif // _MODULE_NAV_H_
