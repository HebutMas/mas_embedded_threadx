/*
 * @Description: 张大头 ZDT 闭环步进 CAN 直发驱动 (darts 本地, 仅 dji_c 编译)
 *               协议: X42S 手册 4.2 章 — 扩展帧, ID = (电机地址<<8)|包序号,
 *               命令去掉地址字节后拆 8 字节包, 每包首字节重复功能码
 */
#ifndef _ZDT_CAN_H_
#define _ZDT_CAN_H_

#include <stdbool.h>
#include <stdint.h>
#include "can.h"

/* 初始化并失能, addr = 电机地址 (模块默认 1) */
void zdt_can_init(CAN_HandleTypeDef *hcan, uint8_t addr);

/* 使能/失能 (失能 = 松轴), 失能持续发送，入队失败时下周期重试 */
void zdt_can_set_enable(bool enable);

/* 位置模式: 相对上一目标位置运动 pulses 个脉冲 (有符号, 负 = 反向), speed_rpm 0~6000 */
void zdt_can_send_position(int32_t pulses, uint16_t speed_rpm);

/* 每周期轮询 FIFO1 读模块回包, 更新 Ozone 观察变量 zdt_dbg_reply_status/count */
void zdt_can_poll_reply(void);

/* Ozone 调试观察: 模块回包状态字节 (02=接受 EE=格式错 E2=参数错, 0xFF=没收到) 与回包数 */
extern uint8_t  zdt_dbg_reply_status;
extern uint32_t zdt_dbg_reply_count;
extern uint32_t zdt_dbg_can_tsr; /* bit28/27/26 邮箱空闲, 0x1C000000=全空闲, 全0=堵死 */
extern uint32_t zdt_dbg_can_esr; /* bit23..16=TEC, 0=模块在ACK, 在涨=无ACK */

#endif /* _ZDT_CAN_H_ */
