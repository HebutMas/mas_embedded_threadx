/**
 * @file    module_nrf24l01.h
 * @brief   nRF24L01 2.4GHz 无线模块 —— 能力注册式收发框架
 *
 *  使用流程:
 *    1. Module_NRF24L01_Init()           初始化(SPI+GPIO+IRQ+线程)
 *    2. Module_NRF24L01_Register("名字", &var, TYPE)  注册变量
 *    3. 之后变量自动收发, 收到数据自动写回
 *
 *  硬件(SPI2 均为 PB13=SCK PB14=MISO PB15=MOSI):
 *    f103_c8(F103): CE=PB0 CSN=PB1 IRQ=PA0(EXTI0), SPI 36M/4=9MHz
 *    dji_c  (F407): CE=PF0 CSN=PB12 IRQ=PF1(EXTI1), SPI 42M/8=5.25MHz
 *
 *  @note    nRF24L01 单包载荷最大 32 字节(硬件限制), 注册总字节数不能超 32
 *  @note    收发两端注册顺序必须一致, 否则解码错位
 */
#ifndef _MODULE_NRF24L01_H_
#define _MODULE_NRF24L01_H_

#include <stdint.h>
#include <stdbool.h>

/* ================= 可配置参数(可被 module_config.h 覆盖) ================= */
#ifndef NRF24L01_TASK_STACK_SIZE
#define NRF24L01_TASK_STACK_SIZE 1024 /* 线程栈大小 */
#endif
#ifndef NRF24L01_TASK_PRIORITY
#define NRF24L01_TASK_PRIORITY 1 /* 线程优先级 */
#endif
#ifndef NRF24L01_TX_INTERVAL_MS
#define NRF24L01_TX_INTERVAL_MS                                                                                                                      \
    10 /* 自动发送间隔(ms), 默认100Hz                                                                                                        \
          nRF24L01 2Mbps空中速率, 单包32B约1ms,                                                                                               \
          100Hz远低于硬件上限, 留足余量 */
#endif
#ifndef NRF24L01_TX_ENABLE
#define NRF24L01_TX_ENABLE                                                                                                                           \
    1 /* 是否自动发送: 1=发送端, 0=纯接收端                                                                                             \
         主从模式下一块设1一块设0, 避免空中冲突 */
#endif
#ifndef NRF24L01_MAX_CAPS
#define NRF24L01_MAX_CAPS 16 /* 最多注册多少个数据项 */
#endif
#ifndef NRF24L01_OFFLINE_TIMEOUT_MS
#define NRF24L01_OFFLINE_TIMEOUT_MS 500 /* OFFLINE心跳超时(ms) */
#endif

/* ================= 硬件相关(按芯片给默认, 仍可被外部覆盖) =================
 * 引脚/SPI分频/EXTI中断线 随芯片自动选择; 本地SPI参数两块板可以不同,
 * 但射频参数(信道/速率/地址)收发两端必须一致。 */
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
#ifndef NRF24L01_IRQ_PORT
#define NRF24L01_IRQ_PORT GPIOF
#endif
#ifndef NRF24L01_IRQ_PIN
#define NRF24L01_IRQ_PIN GPIO_PIN_1
#endif
#ifndef NRF24L01_SPI_PRESCALER
#define NRF24L01_SPI_PRESCALER SPI_BAUDRATEPRESCALER_8 /* APB1=42M, /8=5.25MHz */
#endif
#ifndef NRF24L01_EXTI_IRQn
#define NRF24L01_EXTI_IRQn EXTI1_IRQn /* PF1 = EXTI1 */
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
#ifndef NRF24L01_IRQ_PORT
#define NRF24L01_IRQ_PORT GPIOA
#endif
#ifndef NRF24L01_IRQ_PIN
#define NRF24L01_IRQ_PIN GPIO_PIN_0
#endif
#ifndef NRF24L01_SPI_PRESCALER
#define NRF24L01_SPI_PRESCALER SPI_BAUDRATEPRESCALER_4 /* PCLK1=36M, /4=9MHz */
#endif
#ifndef NRF24L01_EXTI_IRQn
#define NRF24L01_EXTI_IRQn EXTI0_IRQn /* PA0 = EXTI0 */
#endif
#endif

/* ================= 射频参数(可被覆盖, 收发两端必须一致) ================= */
#ifndef NRF24L01_RF_CHANNEL
#define NRF24L01_RF_CHANNEL 2 /* 射频通道: 2400+2=2402MHz */
#endif
#ifndef NRF24L01_RF_DATARATE
#define NRF24L01_RF_DATARATE 2 /* 空中速率: 1=1Mbps, 2=2Mbps */
#endif
#ifndef NRF24L01_RF_POWER
#define NRF24L01_RF_POWER 0 /* 发射功率: 0=0dBm, 1=-6dBm, 2=-12dBm, 3=-18dBm */
#endif
#ifndef NRF24L01_RETR_COUNT
#define NRF24L01_RETR_COUNT 3 /* 自动重传次数(0~15) */
#endif
#ifndef NRF24L01_RETR_DELAY
#define NRF24L01_RETR_DELAY 0 /* 自动重传间隔: 0=250us, 1=500us, ... 15=4000us */
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
 * @brief 初始化 nRF24L01 模块(SPI+GPIO+IRQ+配置寄存器+启动线程)
 * @return 0=成功, 其他=失败
 */
int Module_NRF24L01_Init(void);

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
 * @brief 手动触发一次发送(一般不用, 线程自动定时发)
 */
void Module_NRF24L01_TriggerTx(void);

/**
 * @brief 获取模块在线状态(集成OFFLINE)
 * @return 0=在线, 1=离线
 */
uint8_t Module_NRF24L01_GetStatus(void);

#endif /* _MODULE_NRF24L01_H_ */
