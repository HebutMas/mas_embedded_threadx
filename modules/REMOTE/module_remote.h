/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-10 17:40:19
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-25 21:43:18
 * @FilePath: /mas_embedded_threadx/modules/REMOTE/module_remote.h
 * @Description:
 */
#ifndef _MODULE_REMOTE_H_
#define _MODULE_REMOTE_H_

#include <stdint.h>

#ifndef REMOTE_UART
#define REMOTE_UART huart5 /* 遥控器串口 */
#endif
#ifndef REMOTE_VT_UART
#define REMOTE_VT_UART huart1 /* 图传串口 */
#endif

#ifndef REMOTE_SOURCE
#define REMOTE_SOURCE 1 /* 遥控器选择: 0=none, 1=sbus, 2=dt7 */
#endif
#ifndef REMOTE_VT_SOURCE
#define REMOTE_VT_SOURCE 0 /* 图传选择:   0=none, 1=vt02, 2=vt03 */
#endif
#ifndef REMOTE_DEAD_ZONE
#define REMOTE_DEAD_ZONE 10 /* 遥控器死区范围 */
#endif

/* 任务配置 */
#ifndef REMOTE_TASK_STACK_SIZE
#define REMOTE_TASK_STACK_SIZE 1024 /* 每个解码任务栈大小 */
#endif
#ifndef REMOTE_TASK_PRIORITY
#define REMOTE_TASK_PRIORITY 9 /* 遥控器解码任务优先级 */
#endif
#ifndef REMOTE_TASK_VT_PRIORITY
#define REMOTE_TASK_VT_PRIORITY 8 /* 图传任务优先级 */
#endif

/* 离线检测 */
#ifndef REMOTE_OFFLINE_ENABLE
#define REMOTE_OFFLINE_ENABLE 1 /* 遥控器离线检测 */
#endif
#ifndef REMOTE_VT_OFFLINE_ENABLE
#define REMOTE_VT_OFFLINE_ENABLE 1 /* 图传离线检测 */
#endif
/* SBUS 通道值定义 */
#define SBUS_CHX_BIAS        ((uint16_t)1024)
#define SBUS_CHX_UP          ((uint16_t)240)
#define SBUS_CHX_DOWN        ((uint16_t)1807)
/* DT7 通道值定义 */
#define DT7_CH_VALUE_MIN     ((uint16_t)364)
#define DT7_CH_VALUE_OFFSET  ((uint16_t)1024)
#define DT7_CH_VALUE_MAX     ((uint16_t)1684)
#define DT7_SW_UP            ((uint16_t)1)
#define DT7_SW_MID           ((uint16_t)3)
#define DT7_SW_DOWN          ((uint16_t)2)
/* VT03 通道值定义 */
#define VT03_CH_VALUE_MIN    ((uint16_t)364)
#define VT03_CH_VALUE_OFFSET ((uint16_t)1024)
#define VT03_CH_VALUE_MAX    ((uint16_t)1684)

/* 统一的按键类型定义 */
typedef enum
{
    KEY_W = 0,
    KEY_S,
    KEY_A,
    KEY_D,
    KEY_SHIFT,
    KEY_CTRL,
    KEY_Q,
    KEY_E,
    KEY_R,
    KEY_F,
    KEY_G,
    KEY_Z,
    KEY_X,
    KEY_C,
    KEY_V,
    KEY_B,
    KEY_COUNT // 用于计数
} remote_key_e;

/* keyboard state structure */
typedef union
{
    uint16_t key_code;
    struct
    {
        uint16_t KEY_W     : 1;
        uint16_t KEY_S     : 1;
        uint16_t KEY_A     : 1;
        uint16_t KEY_D     : 1;
        uint16_t KEY_SHIFT : 1;
        uint16_t KEY_CTRL  : 1;
        uint16_t KEY_Q     : 1;
        uint16_t KEY_E     : 1;
        uint16_t KEY_R     : 1;
        uint16_t KEY_F     : 1;
        uint16_t KEY_G     : 1;
        uint16_t KEY_Z     : 1;
        uint16_t KEY_X     : 1;
        uint16_t KEY_C     : 1;
        uint16_t KEY_V     : 1;
        uint16_t KEY_B     : 1;
    } bit;
} keyboard_state_t;

typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_l;
    uint8_t mouse_r;
    uint8_t mouse_m;
} mouse_state_t;

typedef struct
{
    int16_t wheel; /* 滚轮 */
    uint8_t sw1;   /* 拨杆1 */
    uint8_t sw2;   /* 拨杆2 */
} dt7_custom_t;

typedef struct
{
    int16_t wheel;
    uint8_t switch_pos;
    uint8_t pause;
    uint8_t custom_left;
    uint8_t custom_right;
    uint8_t trigger;
} vt03_custom_t;

/**
 * @brief 统一遥控数据
 */
typedef struct
{
    /* 摇杆/通道数据 */
    int16_t       channels[16]; /* ch[0..3]: 摇杆通道, ch[4..15]: 扩展通道 (SBUS) */
    dt7_custom_t  dt7;          /* DT7 自定义数据 */
    vt03_custom_t vt03;         /* VT03 自定义数据 */

    /* 键鼠数据 */
    mouse_state_t    mouse;    /* 鼠标状态 */
    keyboard_state_t keyboard; /* 键盘状态 */
} Remote_Data_t;

/**
 * @brief 初始化遥控/图传模块
 */
int Module_Remote_init(void);

/**
 * @brief 获取统一遥控数据指针
 * @return 指向全局 Remote_Data_t 的指针
 */
Remote_Data_t *Module_Remote_get_data(void);

/**
 * @brief 获取指定通道值
 * @param ch 通道号 (1-based, 1..16)
 * @return 通道值, 无效通道返回 0
 */
int16_t Module_Remote_get_channel(uint8_t ch);

/**
 * @brief 获取鼠标状态指针
 * @return 鼠标状态指针, 未初始化返回 NULL
 */
mouse_state_t *Module_Remote_get_mouse(void);

/**
 * @brief 获取键盘状态指针
 * @return 键盘状态指针, 未初始化返回 NULL
 */
keyboard_state_t *Module_Remote_get_keyboard(void);

/**
 * @brief 获取 DT7 自定义数据指针
 * @return DT7 自定义数据指针, 未初始化返回 NULL
 */
dt7_custom_t *Module_Remote_get_dt7_custom(void);

/**
 * @brief 获取 VT03 自定义数据指针
 * @return VT03 自定义数据指针, 未初始化返回 NULL
 */
vt03_custom_t *Module_Remote_get_vt03_custom(void);

/**
 * @brief 获取遥控器与图传联合离线状态
 * @return 0: 双离线
 *         1: 遥控器在线, 图传离线
 *         2: 遥控器离线, 图传在线
 *         3: 双在线
 */
uint8_t Module_Remote_get_offline_status(void);

#endif // _MODULE_REMOTE_H_
