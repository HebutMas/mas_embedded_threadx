/*
 * robot_control.c
 *
 * 单板测试
 *   云台: 达妙电机 DM4310 (仅初始化)
 *   底盘: 留空
 *   发射: 无
 *   遥控器: 由 module_remote 独立任务处理
 */

#include "robot_control.h"
#include "tx_api.h"
#include "bsp_def.h"
#include "test_def.h"
#include "gimbal_func.h"
#include "chassis_func.h"
#include "robot_func.h"

#include "module_remote.h"

/* REMOTE_UART 宏展开为 huart1/huart3/huart4，需要 usart.h 提供声明 */
#include "usart.h"

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

static Gimbal_Ctrl_Cmd_t  gimbal_cmd;
static Chassis_Ctrl_Cmd_t chassis_cmd;
static uint16_t           yaw_ecd;

/* ========== 遥控器串口寄存器调试 ========== */
static void dbg_dump_remote_uart_regs(void)
{
    USART_TypeDef *uart = REMOTE_UART.Instance;

    /* 确定串口号 */
    const char *uart_name = "UNKNOWN";
    int8_t irqn = -1;
    uint32_t base = (uint32_t)uart;

#if defined(USART1_BASE)
    if (base == USART1_BASE)      { uart_name = "USART1"; irqn = 37; }
#endif
#if defined(USART2_BASE)
    else if (base == USART2_BASE) { uart_name = "USART2"; irqn = 38; }
#endif
#if defined(USART3_BASE)
    else if (base == USART3_BASE) { uart_name = "USART3"; irqn = 39; }
#endif
#if defined(UART4_BASE)
    else if (base == UART4_BASE)  { uart_name = "UART4";  irqn = 52; }
#endif
#if defined(UART5_BASE)
    else if (base == UART5_BASE)  { uart_name = "UART5";  irqn = 53; }
#endif

    uint32_t sr  = uart->SR;
    uint32_t cr1 = uart->CR1;
    uint32_t cr2 = uart->CR2;
    uint32_t cr3 = uart->CR3;
    uint32_t brr = uart->BRR;

    uint8_t ue     = (cr1 >> 13) & 1;
    uint8_t re     = (cr1 >> 2)  & 1;
    uint8_t te     = (cr1 >> 3)  & 1;
    uint8_t rxneie = (cr1 >> 5)  & 1;
    uint8_t idleie = (cr1 >> 4)  & 1;
    uint8_t dma_re = (cr3 >> 6)  & 1;

    uint8_t rxne = (sr >> 5) & 1;
    uint8_t idle = (sr >> 4) & 1;
    uint8_t ore  = (sr >> 3) & 1;
    uint8_t fe   = (sr >> 1) & 1;
    uint8_t nf   = (sr >> 0) & 1;
    uint8_t tc   = (sr >> 6) & 1;

    uint16_t mantissa = (brr >> 4) & 0xFFF;
    uint8_t  fraction = brr & 0x0F;

    LOG_I("--- %s Regs ---", uart_name);
    LOG_I("  CR1: 0x%08lX (UE=%d RE=%d TE=%d RXNEIE=%d IDLEIE=%d)",
          (unsigned long)cr1, ue, re, te, rxneie, idleie);
    LOG_I("  CR2: 0x%08lX | CR3: 0x%08lX (DMAR=%d)",
          (unsigned long)cr2, (unsigned long)cr3, dma_re);
    LOG_I("  BRR: 0x%08lX (mantissa=%u fraction=%u baud_div=%.2f)",
          (unsigned long)brr, mantissa, fraction, mantissa + fraction / 16.0f);
    LOG_I("  SR:  0x%08lX (RXNE=%d IDLE=%d ORE=%d FE=%d NF=%d TC=%d)",
          (unsigned long)sr, rxne, idle, ore, fe, nf, tc);
    LOG_I("  HAL gState: %d", (int)REMOTE_UART.gState);

    /* 信号检测：采样 RX 引脚 1000 次，看电平有无跳变 */
    {
        volatile uint32_t prev = 0, changes = 0;
        for (int i = 0; i < 1000; i++)
        {
            uint32_t cur = REMOTE_UART.Instance->SR;
            (void)cur;
            uint32_t sample = REMOTE_UART.Instance->DR;
            /* 读 DR 会清 RXNE，不影响 */
            if (i == 0)
                prev = sample;
            else if (sample != prev)
                changes++;
            prev = sample;
        }
        /* 上面的 DR 采样没意义，换 GPIO IDR 采样 */
        changes = 0;
        uint32_t prev_idr = 0;
        for (int i = 0; i < 10000; i++)
        {
            uint32_t cur = GPIOC->IDR;
            if (i == 0)
                prev_idr = (cur >> 11) & 1;
            else
            {
                uint32_t bit = (cur >> 11) & 1;
                if (bit != prev_idr)
                {
                    changes++;
                    prev_idr = bit;
                }
            }
        }
        LOG_I("  PC11 signal: %lu changes in 10000 samples (last=%lu)",
              (unsigned long)changes, (unsigned long)prev_idr);
    }

    /* NVIC: IRQn 决定 ISER 索引和位 */
    if (irqn >= 0)
    {
        uint32_t iser_idx = (uint32_t)irqn / 32;
        uint32_t iser_bit = (uint32_t)irqn % 32;
        uint32_t iser_val = NVIC->ISER[iser_idx];
        uint8_t  enabled  = (iser_val >> iser_bit) & 1;
        LOG_I("  NVIC ISER[%lu]: 0x%08lX (%s_IRQn=%d enabled=%d)",
              (unsigned long)iser_idx, (unsigned long)iser_val, uart_name, irqn, enabled);
    }
}

/* ========== 控制线程 ========== */
static void robot_control_task(ULONG thread_input)
{
    (void)thread_input;

    uint32_t loop_count = 0;

    while (1)
    {

        /* 每 500 个循环 (~1s) 打印遥控器串口寄存器状态 */
        loop_count++;
        if (loop_count >= 500)
        {
            loop_count = 0;
            dbg_dump_remote_uart_regs();
        }

        tx_thread_sleep(2);
    }
}

/* ========== 初始化 ========== */
void robot_control_init(void)
{
    UINT status;

    gimbal_init();
    chassis_init();


    status = tx_thread_create(&robot_control_thread, "robot_control_thread",
                              robot_control_task, 0,
                              robot_control_thread_stack, 1024,
                              30, 30, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_thread create failed!");
        return;
    }

    LOG_I("robot_control init success!");
}
