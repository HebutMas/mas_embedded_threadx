/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2026-05-13 15:10:40
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-05-14 16:47:22
 * @FilePath: /mas_embedded_threadx/modules/BOARDCOMM/module_boardcomm.h
 * @Description:
 */
#ifndef _MODULE_BOARDCOMM_H_
#define _MODULE_BOARDCOMM_H_

#include <stdint.h>

/* CAN ID 定义 */
#define BOARDCOMM_GIMBAL_ID  0x310 /* 云台 → 底盘 */
#define BOARDCOMM_CHASSIS_ID 0x311 /* 底盘 → 云台 */

/* CAN帧 DLC 上限 */
#define BOARDCOMM_FRAME_MAX  8

/* 使用的 CAN 总线 */
#ifndef BOARDCOMM_CAN
#define BOARDCOMM_CAN BSP_CAN_HANDLE2
#endif

#ifndef BOARDCOMM_OFFLINE_ENABLE
#define BOARDCOMM_OFFLINE_ENABLE 1 /* 离线检测开启 */
#endif

/**
 * @brief 板间通信接收回调原型
 * @param data 接收到的 CAN 数据（最多 8 字节）
 * @param len  数据长度
 */
typedef void (*BoardComm_RxCallback_t)(const uint8_t *data, uint8_t len);

/**
 * @brief 初始化板间通信模块（注册 CAN 过滤器、启动接收）
 * @note 单板模式下直接跳过
 */
int Module_BoardComm_Init(void);

/**
 * @brief 发送原始 CAN 数据
 * @param data   数据指针（最多 8 字节）
 * @param len    数据长度
 */
void Module_BoardComm_Send(uint8_t *data, uint8_t len);

/**
 * @brief 注册接收回调（收到数据时由 BOARDCOMM 自动调用）
 * @param callback app 层解析函数
 * @note 只能注册一个回调，重复调用会覆盖；NULL 表示注销
 */
void Module_BoardComm_RegisterRx(BoardComm_RxCallback_t callback);

/**
 * @brief 取出一份接收帧快照
 * @param dst  目标缓冲区
 * @param len  期望长度（须与收到的 CAN 帧长度一致）
 * @return 1 = 已拷出最近一帧；0 = 尚无帧、长度不匹配，或 CAN 中断正在写
 * @note 与 RegisterRx 互斥;
 */
uint8_t Module_BoardComm_Receive(void *dst, uint8_t len);

/**
 * @brief 获取板间通信离线状态
 * @return 0 表示在线，1 表示离线
 */
uint8_t Module_BoardComm_Get_Offline_State(void);

#endif // _MODULE_BOARDCOMM_H_
