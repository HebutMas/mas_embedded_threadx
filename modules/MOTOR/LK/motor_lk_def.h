#ifndef __LK_MOTOR_DEF_H__
#define __LK_MOTOR_DEF_H__

/*----------------------------------------------- CAN 总线参数 -------------------------------------------------*/
#define LK_CAN_CMD_BASE_ID  0x140  /* 主机→电机 命令帧基址，实际 ID = 0x140 + motor_id (ID 范围 1~32)           */
#define LK_CAN_REPLY_BASE_ID 0x140 /* 电机→主机 回复帧基址，实际 ID = 0x140 + motor_id (ID 范围 1~32)(手册不对)           */
#define LK_CAN_DLC           8     /* 数据长度固定 8 字节                                                       */                                         
/*----------------------------------------------- 控制命令字节 --------------------------------------------------*/
/* 基本状态控制 */
#define LK_CMD_READ_STATUS1_ERROR 0x9A /* 读取电机状态1和错误标志                                               */
#define LK_CMD_CLEAR_ERROR        0x9B /* 清除电机错误标志                                                       */
#define LK_CMD_READ_STATUS2       0x9C /* 读取电机状态2                                                          */
#define LK_CMD_READ_STATUS3       0x9D /* 读取电机状态3（除 MS 外）                                              */
#define LK_CMD_MOTOR_OFF          0x80 /* 电机关闭 — 清除转动圈数及控制指令，LED 慢闪                            */
#define LK_CMD_MOTOR_RUN          0x88 /* 电机运行 — 切换到开启状态，LED 常亮                                    */
#define LK_CMD_MOTOR_STOP         0x81 /* 电机停止 — 停止但不改变运行状态                                         */
/* 辅助功能 */
#define LK_CMD_BRAKE_CTRL         0x8C /* 抱闸器控制和状态读取                                                    */
/* 闭环控制，一般只用转矩闭环控制，扭矩闭环参数出场已经设置，可读不用调整 */
#define LK_CMD_OPEN_LOOP          0xA0 /* 开环控制（仅 MS）                                                        */
#define LK_CMD_TORQUE_CLOSED_LOOP 0xA1 /* 转矩闭环控制（电流值）（仅 MF、MH、MG）                                            */
#define LK_CMD_SPEED_CLOSED_LOOP  0xA2 /* 速度闭环控制                                                             */
#define LK_CMD_MULTI_POS_CTRL1    0xA3 /* 多圈位置闭环控制1                                                        */
#define LK_CMD_MULTI_POS_CTRL2    0xA4 /* 多圈位置闭环控制2（可指定 maxSpeed）                                     */
#define LK_CMD_SINGLE_POS_CTRL1   0xA5 /* 单圈位置闭环控制1                                                        */
#define LK_CMD_SINGLE_POS_CTRL2   0xA6 /* 单圈位置闭环控制2（可指定 maxSpeed）                                     */
#define LK_CMD_INCREMENT_POS_CTRL1 0xA7 /* 增量位置闭环控制1                                                       */
#define LK_CMD_INCREMENT_POS_CTRL2 0xA8 /* 增量位置闭环控制2（可指定 maxSpeed）                                    */
/* 编码器与角度，看返回帧就行，这些命令不用 */
#define LK_CMD_READ_ENCODER       0x90 /* 读取编码器数据                                                           */
#define LK_CMD_SET_ZERO_ROM       0x19 /* 设置当前位置到 ROM 作为电机零点（需重新上电生效）                        */
#define LK_CMD_READ_MULTI_ANGLE   0x92 /* 读取多圈角度                                                             */
#define LK_CMD_READ_SINGLE_ANGLE  0x94 /* 读取单圈角度                                                             */
#define LK_CMD_SET_CURRENT_ANGLE  0x95 /* 设置当前位置为任意角度（写 RAM，即时生效，断电失效）                     */
/* 参数读写 */
#define LK_CMD_READ_PARAM         0xC0 /* 读取控制参数                                                             */
#define LK_CMD_WRITE_PARAM        0xC1 /* 写入控制参数（写 RAM，即时生效，断电失效）                               */
/*----------------------------------------------- 电机状态标志 --------------------------------------------------*/
/* 电机运行状态 (DATA[6] of 0x9A) */
#define LK_MOTOR_STATUS_ON        0x00 /* 电机开启                                                                 */
#define LK_MOTOR_STATUS_OFF       0x10 /* 电机关闭                                                                 */
/*----------------------------------------------- 错误标志位 ----------------------------------------------------*/
/* 错误标志位 (DATA[7] of 0x9A, bit 0~7) */
#define LK_ERROR_UNDER_VOLTAGE    (1 << 0) /* 低电压保护                                                              */
#define LK_ERROR_OVER_VOLTAGE     (1 << 1) /* 高压保护                                                                */
#define LK_ERROR_DRIVER_OVER_TEMP (1 << 2) /* 驱动过温                                                                */
#define LK_ERROR_MOTOR_OVER_TEMP  (1 << 3) /* 电机过温                                                                */
#define LK_ERROR_OVER_CURRENT     (1 << 4) /* 电机过流                                                                */
#define LK_ERROR_SHORT_CIRCUIT    (1 << 5) /* 电机短路                                                                */
#define LK_ERROR_STALL            (1 << 6) /* 电机堵转                                                                */
#define LK_ERROR_SIGNAL_LOST      (1 << 7) /* 输入信号丢失超时                                                        */
/*----------------------------------------------- 控制参数范围（仅需要的） --------------------------------------------------*/                                                       
/* 转矩闭环控制 (仅 MF、MH、MG) */
#define LK_TORQUE_IQ_MIN          (-2048)             /* 转矩电流 iqControl 下限                                          */
#define LK_TORQUE_IQ_MAX          2048                /* 转矩电流 iqControl 上限                                          */
/* MG 电机实际转矩电流: iqControl 映射到 -33A ~ 33A */
#define LK_MG_TORQUE_CURRENT_MIN    (-33.0f) /* MG 实际转矩电流下限 (A)                                               */
#define LK_MG_TORQUE_CURRENT_MAX    33.0f    /* MG 实际转矩电流上限 (A)                                               */
#define LK_MG_TORQUE_CURRENT_RES    (66.0f / 4096.0f) /* MG 转矩电流分辨率 (A/LSB)                                       */
/*----------------------------------------------- 数学常量 --------------------------------------------------*/
/* PI / DEGREE_2_RAD 复用 user_lib.h */
#define LK_DEG_001_TO_RAD    (DEGREE_2_RAD / 100.0f)    /* 0.01° → rad   */
/*----------------------------------------------- 编码器位数 --------------------------------------------------*/
/* 根据电机型号选择，不同电机编码器位数不同 (14/15/16 bit) */
#define LK_ENCODER_MAX_14BIT 16383u
#define LK_ENCODER_MAX_15BIT 32767u
#define LK_ENCODER_MAX_16BIT 65535u
/*----------------------------------------------- 系列差异对照 -------------------------------------------------*/
/*  特性              │ MF              │ MH           │ MG              │ MS                */
/*  ─────────────────│─────────────────│──────────────│─────────────────│────────────────── */
/*  转矩闭环          │ ✓               │ ✓            │ ✓               │ ✗                 */
/*  开环控制          │ ✗               │ ✗            │ ✗               │ ✓                 */
/*  状态3 (相电流)    │ ✓               │ ✓            │ ✓               │ ✗                 */
/*  转矩电流分辨率    │ (33/4096) A/LSB │ —            │ (66/4096) A/LSB │ —                 */
/*  相电流分辨率      │ (33/4096) A/LSB │ —            │ (66/4096) A/LSB │ —                 */
/*  状态2 中的值      │ 转矩电流 iq     │ 转矩电流 iq  │ 转矩电流 iq     │ 输出功率 power    */
/*  编码器位数(根据电机)│ 14/15/16 bit    │ 14/15/16 bit │ 14/15/16 bit    │ 14/15/16 bit      */
#endif /* __LK_MOTOR_DEF_H__ */
