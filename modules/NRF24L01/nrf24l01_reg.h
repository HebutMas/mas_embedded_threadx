/**
 * @file    nrf24l01_reg.h
 * @brief   nRF24L01 寄存器地址与指令码定义
 * @note    移植自江协科技标准库驱动，补充动态包长(DPL)相关寄存器
 */
#ifndef _NRF24L01_REG_H_
#define _NRF24L01_REG_H_

#include <stdint.h>

/* ================= SPI 指令码 ================= */
#define NRF24L01_R_REGISTER          0x00 /* 读寄存器: 000A AAAA, 后跟1~5字节数据 */
#define NRF24L01_W_REGISTER          0x20 /* 写寄存器: 001A AAAA, 后跟1~5字节数据(仅待机/掉电模式可写) */
#define NRF24L01_R_RX_PAYLOAD        0x61 /* 读RX有效载荷: 后跟1~32字节 */
#define NRF24L01_W_TX_PAYLOAD        0xA0 /* 写TX有效载荷: 后跟1~32字节 */
#define NRF24L01_FLUSH_TX            0xE1 /* 清空TX FIFO */
#define NRF24L01_FLUSH_RX            0xE2 /* 清空RX FIFO */
#define NRF24L01_REUSE_TX_PL         0xE3 /* 重发最后一次载荷 */
#define NRF24L01_R_RX_PL_WID         0x60 /* 读RX FIFO最前面包的宽度(仅DPL模式) */
#define NRF24L01_W_ACK_PAYLOAD       0xA8 /* 写应答附带载荷(高5位指令+低3位通道号) */
#define NRF24L01_W_TX_PAYLOAD_NOACK  0xB0 /* 写TX载荷不要求应答 */
#define NRF24L01_NOP                 0xFF /* 空操作, 可用来读STATUS */
#define NRF24L01_ACTIVATE            0x50 /* 激活FEATURE寄存器(后跟0x73), nRF24L01+必需 */
#define NRF24L01_ACTIVATE_DATA       0x73 /* ACTIVATE命令的固定跟随字节 */

/* ================= 寄存器地址 ================= */
#define NRF24L01_CONFIG              0x00 /* 配置寄存器 */
#define NRF24L01_EN_AA               0x01 /* 使能自动应答 */
#define NRF24L01_EN_RXADDR           0x02 /* 使能接收通道 */
#define NRF24L01_SETUP_AW            0x03 /* 设置地址宽度 */
#define NRF24L01_SETUP_RETR          0x04 /* 设置自动重传 */
#define NRF24L01_RF_CH               0x05 /* 射频通道 */
#define NRF24L01_RF_SETUP            0x06 /* 射频设置 */
#define NRF24L01_STATUS              0x07 /* 状态寄存器 */
#define NRF24L01_OBSERVE_TX          0x08 /* 发送观察 */
#define NRF24L01_RPD                 0x09 /* 接收功率检测 */
#define NRF24L01_RX_ADDR_P0          0x0A /* 接收通道0地址(5字节) */
#define NRF24L01_RX_ADDR_P1          0x0B /* 接收通道1地址(5字节) */
#define NRF24L01_RX_ADDR_P2          0x0C /* 接收通道2地址(1字节, 高4字节同P1) */
#define NRF24L01_RX_ADDR_P3          0x0D /* 接收通道3地址(1字节) */
#define NRF24L01_RX_ADDR_P4          0x0E /* 接收通道4地址(1字节) */
#define NRF24L01_RX_ADDR_P5          0x0F /* 接收通道5地址(1字节) */
#define NRF24L01_TX_ADDR             0x10 /* 发送地址(5字节) */
#define NRF24L01_RX_PW_P0            0x11 /* 通道0载荷宽度(1~32) */
#define NRF24L01_RX_PW_P1            0x12 /* 通道1载荷宽度 */
#define NRF24L01_RX_PW_P2            0x13 /* 通道2载荷宽度 */
#define NRF24L01_RX_PW_P3            0x14 /* 通道3载荷宽度 */
#define NRF24L01_RX_PW_P4            0x15 /* 通道4载荷宽度 */
#define NRF24L01_RX_PW_P5            0x16 /* 通道5载荷宽度 */
#define NRF24L01_FIFO_STATUS         0x17 /* FIFO状态 */
#define NRF24L01_DYNPD               0x1C /* 使能动态包长 */
#define NRF24L01_FEATURE             0x1D /* 使能高级功能 */

/* ================= CONFIG 寄存器位定义 ================= */
#define NRF24L01_CONFIG_MASK_RX_DR   (1 << 6) /* 屏蔽RX_DR中断 */
#define NRF24L01_CONFIG_MASK_TX_DS   (1 << 5) /* 屏蔽TX_DS中断 */
#define NRF24L01_CONFIG_MASK_MAX_RT  (1 << 4) /* 屏蔽MAX_RT中断 */
#define NRF24L01_CONFIG_EN_CRC       (1 << 3) /* 使能CRC */
#define NRF24L01_CONFIG_CRCO         (1 << 2) /* CRC宽度: 0=1字节, 1=2字节 */
#define NRF24L01_CONFIG_PWR_UP       (1 << 1) /* 上电: 0=掉电, 1=上电 */
#define NRF24L01_CONFIG_PRIM_RX      (1 << 0) /* 收发模式: 0=TX, 1=RX */

/* ================= STATUS 寄存器位定义 ================= */
#define NRF24L01_STATUS_RX_DR        (1 << 6) /* 接收数据就绪中断 */
#define NRF24L01_STATUS_TX_DS        (1 << 5) /* 发送完成中断 */
#define NRF24L01_STATUS_MAX_RT       (1 << 4) /* 达到最大重传次数中断 */
#define NRF24L01_STATUS_RX_P_NO_MASK (0x0E)   /* 接收通道号掩码(bit3:1) */
#define NRF24L01_STATUS_TX_FULL      (1 << 0) /* TX FIFO满 */

/* ================= FIFO_STATUS 位定义 ================= */
#define NRF24L01_FIFO_TX_REUSE       (1 << 6)
#define NRF24L01_FIFO_TX_FULL        (1 << 5)
#define NRF24L01_FIFO_TX_EMPTY       (1 << 4)
#define NRF24L01_FIFO_RX_FULL        (1 << 1)
#define NRF24L01_FIFO_RX_EMPTY       (1 << 0)

/* ================= FEATURE 寄存器位定义 ================= */
#define NRF24L01_FEATURE_EN_DPL      (1 << 2) /* 使能动态包长 */
#define NRF24L01_FEATURE_EN_ACK_PAY  (1 << 1) /* 使能应答附带载荷 */
#define NRF24L01_FEATURE_EN_DYN_ACK  (1 << 0) /* 使能W_TX_PAYLOAD_NOACK指令 */

/* ================= 常用配置值 ================= */
#define NRF24L01_ADDR_WIDTH_5        0x03 /* 5字节地址 */
#define NRF24L01_PAYLOAD_MAX         32   /* 单包最大载荷字节数(硬件限制) */

#endif /* _NRF24L01_REG_H_ */
