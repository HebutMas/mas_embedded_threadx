/**
 * @file    module_nrf24l01.h
 * @brief   nRF24L01 2.4GHz 无线模块 —— 能力注册式收发框架
 *
 *  使用流程:
 *    0. 先在 CubeMX 里配好 SPI2 与 CE/CSN/IRQ 引脚(见下方"硬件"), 生成代码
 *    1. 在 robot.cmake 里选定角色(见下方 NRF24L01_TX_ENABLE / NRF24L01_RX_ENABLE, 二者互斥)
 *    2. Module_NRF24L01_Init()           初始化(配置寄存器 + 按角色起线程)
 *    3. Module_NRF24L01_Register("名字", &var, TYPE)  注册变量
 *    4. 之后变量自动收发, 收到数据自动写回
 *
 *  硬件(需自行在 CubeMX 配置, 与下方宏保持一致):
 *    SPI2: PB13=SCK PB14=MISO PB15=MOSI (模式0, ≤10MHz)
 *          f103_c8: CE=PB0  CSN=PB1  IRQ=PA0(EXTI0)
 *          dji_c  : CE=PF0  CSN=PB12 IRQ=PF1(EXTI1)
 *    IRQ 需配为下降沿外部中断(EXTI), 且 it.c 中有对应 EXTIx_IRQHandler
 *
 *  @note    nRF24L01 单包载荷最大 32 字节(硬件限制), 注册总字节数不能超 32
 *  @note    收发两端注册顺序必须一致, 否则解码错位
 */
#ifndef _MODULE_NRF24L01_H_
#define _MODULE_NRF24L01_H_

#include <stdint.h>

/* ================= 可配置参数(可被 module_config.h 覆盖) ================= */
#ifndef NRF24L01_TASK_STACK_SIZE
#define NRF24L01_TASK_STACK_SIZE 1024 /* 线程栈大小 */
#endif
#ifndef NRF24L01_TASK_PRIORITY
#define NRF24L01_TASK_PRIORITY 10 /* 线程优先级; 与其它模块同档(INS=7/REMOTE=8-9/本模块=VISION=REFEREE=10/MOTOR=12) */
#endif
#ifndef NRF24L01_TX_INTERVAL_MS
#define NRF24L01_TX_INTERVAL_MS 10 /* 自动发送间隔(ms), 默认100Hz */
#endif
#ifndef NRF24L01_TX_ENABLE
#define NRF24L01_TX_ENABLE 0 /* 1=注册发送线程(本板做发送端) */
#endif
#ifndef NRF24L01_RX_ENABLE
#define NRF24L01_RX_ENABLE 0 /* 1=注册接收线程(本板做接收端) */
#endif
/* 两者必须且只能选一个。
 * 同时开会 #error(nRF24L01 半双工, 同板收发没有意义); 都不开会 #error(模块什么都不做)。
 * 需要双向通信就两块板各选一个角色。 */
#ifndef NRF24L01_OFFLINE_ENABLE
#define NRF24L01_OFFLINE_ENABLE 1 /* 离线检测开启 */
#endif
#ifndef NRF24L01_ADDR
#define NRF24L01_ADDR 0x11, 0x22, 0x33, 0x44, 0x55 /* 5字节通信地址(逗号分隔); 收发两端必须一致 */
#endif

/* ================= 驱动内部定死(不对外暴露) ================= */
#define NRF24L01_MAX_CAPS    16 /* 最多注册多少个数据项(编译期数组大小, 与32字节载荷上限一起卡住) */
#define NRF24L01_RETR_COUNT  3  /* 自动重传次数(0~15) */
#define NRF24L01_RETR_DELAY  0  /* 自动重传间隔: 0=250us, 1=500us, ... 15=4000us */

/* 射频参数(定值, 不复盖): 需要换信道/速率/功率时直接改这里 */
#define NRF24L01_RF_CHANNEL  2 /* 射频通道: 2400+2=2402MHz */
#define NRF24L01_RF_DATARATE 2 /* 空中速率: 1=1Mbps, 2=2Mbps */
#define NRF24L01_RF_POWER    0 /* 发射功率: 0=0dBm, 1=-6dBm, 2=-12dBm, 3=-18dBm */

/* ================= 硬件相关(按芯片给默认, 仍可被外部覆盖) =================
 * 引脚随芯片自动选择, 须与板级 CubeMX(.ioc) 配置一致(模块不配引脚, 只强制 SPI 参数);
 * 射频参数(信道/速率/地址)收发两端必须一致。 */

/* 使用的 SPI 外设(默认 hspi2; 换其它 SPI 时覆盖此宏即可, 模块不再硬编码) */
#ifndef NRF24L01_SPI
#define NRF24L01_SPI hspi2
#endif

#if defined(STM32F407xx)
/* ---- dji_c (STM32F407) ---- */
#ifndef NRF24L01_CE_PORT
#define NRF24L01_CE_PORT GPIOF
#endif
#ifndef NRF24L01_CE_PIN
#define NRF24L01_CE_PIN GPIO_PIN_0
#endif
#ifndef NRF24L01_CSN_PORT
#define NRF24L01_CSN_PORT GPIOB
#endif
#ifndef NRF24L01_CSN_PIN
#define NRF24L01_CSN_PIN GPIO_PIN_12
#endif
#ifndef NRF24L01_IRQ_PIN
#define NRF24L01_IRQ_PIN GPIO_PIN_1
#endif
#ifndef NRF24L01_SPI_PRESCALER
#define NRF24L01_SPI_PRESCALER SPI_BAUDRATEPRESCALER_8 /* APB1=42M, /8=5.25MHz */
#endif
#else
/* ---- f103_c8 (STM32F103, 默认, 已验证) ---- */
#ifndef NRF24L01_CE_PORT
#define NRF24L01_CE_PORT GPIOB
#endif
#ifndef NRF24L01_CE_PIN
#define NRF24L01_CE_PIN GPIO_PIN_0
#endif
#ifndef NRF24L01_CSN_PORT
#define NRF24L01_CSN_PORT GPIOB
#endif
#ifndef NRF24L01_CSN_PIN
#define NRF24L01_CSN_PIN GPIO_PIN_1
#endif
#ifndef NRF24L01_IRQ_PIN
#define NRF24L01_IRQ_PIN GPIO_PIN_0
#endif
#ifndef NRF24L01_SPI_PRESCALER
#define NRF24L01_SPI_PRESCALER SPI_BAUDRATEPRESCALER_4 /* PCLK1=36M, /4=9MHz */
#endif
#endif

/* ================= 数据类型枚举 ================= */
typedef enum
{
    NRF24L01_TYPE_INT8 = 0,
    NRF24L01_TYPE_UINT8,
    NRF24L01_TYPE_INT16,
    NRF24L01_TYPE_UINT16,
    NRF24L01_TYPE_INT32,
    NRF24L01_TYPE_UINT32,
    NRF24L01_TYPE_FLOAT,
    NRF24L01_TYPE_DOUBLE,
    NRF24L01_TYPE_COUNT
} NRF24L01_DataType_e;

/* ================= 对外接口 ================= */

/**
 * @brief 初始化 nRF24L01 模块(SPI参数强制 + 注册BSP设备/EXTI + 配置寄存器 + 起线程)
 * @note  失败只打日志; 与其它模块一致, 不返回错误码
 */
void Module_NRF24L01_Init(void);

/**
 * @brief 注册一个数据项到收发列表
 * @param name      名称字符串(调试用, 不传输)
 * @param data_ptr  指向变量的指针(发送时读, 接收时写)
 * @param type      数据类型
 * @return >=0 注册索引; -1 失败(列表满/超32B载荷/参数非法)
 *
 * @note  收发两端注册顺序必须一致
 * @note  注册总字节数不能超过 32(nRF24L01 单包硬件上限)
 */
int8_t Module_NRF24L01_Register(const char *name, void *data_ptr, NRF24L01_DataType_e type);

/**
 * @brief 获取模块在线状态(集成OFFLINE)
 * @return 0=在线, 1=离线
 */
uint8_t Module_NRF24L01_GetStatus(void);

#endif /* _MODULE_NRF24L01_H_ */
