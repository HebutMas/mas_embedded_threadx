/*
 * @file        vofa.c
 * @brief       VOFA (Visualization of Floats Add-on) 串口波形协议模块
 *
 *              本文件 = 协议核心 + 模块入口 合二为一:
 *                - 协议: JustFloat(二进制帧尾 00 00 80 7f) / FireWater(文本前缀,浮点)
 *                - 模块入口 Module_VOFA_Init(): 初始化 UART + 创建 RX/TX 双线程
 *                  RX 线程周期解析下行数据; TX 线程按 VOFA_TX_INTERVAL_MS 周期自动上报
 *
 *              默认不启用: 需在 module_config.cmake 的 MODULES_* 中加入 VOFA
 *              才会参与编译与初始化 (见 modules/module_init.c #if MODULE_VOFA)
 *
 *              配置宏见 vofa.h 顶部 (VOFA_UART / VOFA_FORMAT / VOFA_TX_INTERVAL_MS ...)
 */
#include "vofa.h"
#include "bsp_def.h"
#include "tx_api.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define LOG_TAG "module_vofa"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 帧尾常量 */
static const uint8_t VOFA_TAIL_JUSTFLOAT[] = {0x00, 0x00, 0x80, 0x7f};

/* ========== 内部数据结构 (动态链表节点) ========== */

/* 发送帧节点 */
typedef struct VOFA_TxNode
{
    struct VOFA_TxNode *next;                 // 链表下一节点
    uint8_t             string_data[VOFA_STRING_DATA_LEN]; // 参数名
    uint8_t             string_len;           // 参数名长度
    float              *float_data;           // 浮点数指针
} VOFA_TxNode;

/* 接收帧节点 */
typedef struct VOFA_RxNode
{
    struct VOFA_RxNode *next;                 // 链表下一节点
    uint8_t             string_data[VOFA_STRING_DATA_LEN]; // 参数名
    uint8_t             string_len;           // 参数名长度
    float              *float_data;           // 浮点数指针
    VOFA_RxCallback     callback;             // 回调
} VOFA_RxNode;

/* ========== 全局状态 ========== */

static UART_Device *g_uart_dev         = NULL;
static VOFA_Format  g_current_format   = VOFA_FORMAT_FIREWATER;
static const char  *g_firewater_prefix = NULL;
static uint8_t      g_tx_buffer[VOFA_TX_BUFFER_SIZE];

/* 发送/接收链表头 */
static VOFA_TxNode *g_tx_head = NULL;
static VOFA_RxNode *g_rx_head = NULL;

/* ========== 模块任务状态 ========== */
static volatile bool     g_initialized = false;
static TX_THREAD         g_vofa_rx_task;
APPS_STACK_SECTION static uint8_t g_vofa_rx_stack[VOFA_TASK_STACK_SIZE];
static TX_THREAD         g_vofa_tx_task;
APPS_STACK_SECTION static uint8_t g_vofa_tx_stack[VOFA_TASK_STACK_SIZE];

/* UART6 接收缓冲 (双缓冲, 见 bsp_uart 说明) */
static uint8_t g_vofa_rx_buf[2][128];

/* ============ API 函数实现 ============ */

/**
 * @brief 初始化VOFA设备
 */
vofa_err_t VOFA_Init(UART_Device *uart_dev, VOFA_Format format, const char *firewater_prefix)
{
    if (!uart_dev) return VOFA_ERR;

    g_uart_dev         = uart_dev;
    g_current_format   = format;
    g_firewater_prefix = firewater_prefix;
    g_tx_head          = NULL;
    g_rx_head          = NULL;

    return VOFA_OK;
}

/**
 * @brief 注册待发送数据 (动态分配发送节点, 入发送链表)
 */
vofa_err_t VOFA_RegisterTx(const char *name, float *float_ptr)
{
    if (!name || !float_ptr) return VOFA_ERR;

    VOFA_TxNode *node = NULL;
    BSP_MEM_ALLOC_WAIT(node, sizeof(VOFA_TxNode), TX_NO_WAIT);
    if (node == NULL) return VOFA_ERR;

    memset(node, 0, sizeof(VOFA_TxNode));

    /* 复制参数名 */
    size_t len = strlen(name);
    if (len > VOFA_STRING_DATA_LEN) len = VOFA_STRING_DATA_LEN;
    memcpy(node->string_data, name, len);
    node->string_len = (uint8_t)len;
    node->float_data = float_ptr;

    /* 头插法入发送链表 */
    node->next = g_tx_head;
    g_tx_head  = node;

    return VOFA_OK;
}

/**
 * @brief Just Float格式发送函数
 */
static vofa_err_t VOFA_SendJustFloat(void)
{
    if (!g_uart_dev || g_tx_head == NULL) return VOFA_ERR;

    float    *pf          = (float *)g_tx_buffer;
    uint16_t  float_count = 0;

    /* 遍历发送链表, 提取所有浮点数据 */
    for (VOFA_TxNode *node = g_tx_head; node != NULL && float_count < (VOFA_JUSTFLOAT_MAX_LEN / 4); node = node->next)
    {
        if (node->float_data != NULL)
        {
            pf[float_count++] = *node->float_data;
        }
    }

    if (float_count == 0) return VOFA_ERR;

    /* 添加帧尾 */
    uint16_t send_len = float_count * 4 + VOFA_JUSTFLOAT_TAIL_LEN;
    memcpy(&g_tx_buffer[float_count * 4], VOFA_TAIL_JUSTFLOAT, VOFA_JUSTFLOAT_TAIL_LEN);

    /* 通过UART发送 (RM2027: Send(dev,data,len,timeout)) */
    int sent = BSP_UART_Send(g_uart_dev, g_tx_buffer, send_len, 100);

    return (sent == send_len) ? VOFA_OK : VOFA_ERR;
}

/**
 * @brief Fire Water格式发送函数
 */
static vofa_err_t VOFA_SendFireWater(void)
{
    if (!g_uart_dev || g_tx_head == NULL || !g_firewater_prefix) return VOFA_ERR;

    /* 清空缓冲区 */
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));

    /* 添加前缀 */
    size_t prefix_len = strlen(g_firewater_prefix);
    if (prefix_len >= sizeof(g_tx_buffer) - 10) return VOFA_ERR;

    if (prefix_len > 0)
    {
        memcpy(g_tx_buffer, g_firewater_prefix, prefix_len);
    }
    size_t offset = prefix_len;

    /* 遍历发送链表, 添加浮点数据 */
    int data_count = 0;

    for (VOFA_TxNode *node = g_tx_head; node != NULL; node = node->next)
    {
        if (node->float_data == NULL) continue;

        /* 计算缓冲区剩余空间 */
        size_t remaining = sizeof(g_tx_buffer) - offset;
        if (remaining < 15) break;

        /* 手动格式化浮点数 */
        float value     = *node->float_data;
        int   int_part  = (int)value;
        float frac_temp = value - int_part;
        if (frac_temp < 0) frac_temp = -frac_temp;
        int frac_part = (int)(frac_temp * 100 + 0.5f);

        int ret = snprintf((char *)&g_tx_buffer[offset], remaining, "%d.%02d,", int_part, frac_part);
        if (ret <= 0 || ret >= (int)remaining) break;

        offset += (size_t)ret;
        data_count++;
    }

    /* 如果没有成功添加任何数据, 至少发送一个默认的 "0.0" */
    if (data_count == 0 && (offset + 10) < sizeof(g_tx_buffer))
    {
        int ret = snprintf((char *)&g_tx_buffer[offset], sizeof(g_tx_buffer) - offset, "0.0,");
        if (ret > 0 && (size_t)ret < (sizeof(g_tx_buffer) - offset))
            offset += (size_t)ret;
    }

    /* 移除最后的逗号, 添加换行符 */
    if (offset > prefix_len && g_tx_buffer[offset - 1] == ',') offset--;

    if ((offset + 2) >= sizeof(g_tx_buffer)) return VOFA_ERR;

    int ret = snprintf((char *)&g_tx_buffer[offset], sizeof(g_tx_buffer) - offset, "\n");
    if (ret <= 0 || (size_t)ret >= (sizeof(g_tx_buffer) - offset)) return VOFA_ERR;
    offset += (size_t)ret;

    /* 通过UART发送 */
    int sent = BSP_UART_Send(g_uart_dev, g_tx_buffer, (uint16_t)offset, 100);
    return (sent == (int)offset) ? VOFA_OK : VOFA_ERR;
}

/**
 * @brief 注册接收数据 (动态分配接收节点, 入接收链表)
 */
vofa_err_t VOFA_RegisterRx(const char *name, float *float_ptr, VOFA_RxCallback callback)
{
    if (!name || !float_ptr) return VOFA_ERR;

    VOFA_RxNode *node = NULL;
    BSP_MEM_ALLOC_WAIT(node, sizeof(VOFA_RxNode), TX_NO_WAIT);
    if (node == NULL) return VOFA_ERR;

    memset(node, 0, sizeof(VOFA_RxNode));

    /* 复制参数名 */
    size_t len = strlen(name);
    if (len > VOFA_STRING_DATA_LEN) len = VOFA_STRING_DATA_LEN;
    memcpy(node->string_data, name, len);
    node->string_len = (uint8_t)len;
    node->float_data = float_ptr;
    node->callback   = callback;

    /* 头插法入接收链表 */
    node->next = g_rx_head;
    g_rx_head  = node;

    return VOFA_OK;
}

/**
 * @brief 接收数据处理函数
 */
void VOFA_ReceiveHandler(void)
{
    if (!g_uart_dev || g_rx_head == NULL) return;

    uint8_t  rx_buf[VOFA_STRING_DATA_LEN];
    uint32_t total_rx_len = 0;

    /* 从UART读取接收数据 (RM2027: Read(dev,buf,size,&len,timeout), 非阻塞) */
    int rc = BSP_UART_Read(g_uart_dev, rx_buf, sizeof(rx_buf), &total_rx_len, 0);
    if (rc <= 0 || total_rx_len == 0) return;

    if (total_rx_len > VOFA_STRING_DATA_LEN) total_rx_len = VOFA_STRING_DATA_LEN;

    /* 将接收到的字节数据转换为C字符串 */
    char rx_string[VOFA_STRING_DATA_LEN + 1] = {0};
    memcpy(rx_string, rx_buf, total_rx_len);
    rx_string[total_rx_len] = '\0';

    /* 从后往前查找数值部分的起始位置 */
    int float_start_idx = -1;
    for (int i = (int)total_rx_len - 1; i >= 0; i--)
    {
        char c = rx_string[i];
        if (!((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+'))
        {
            float_start_idx = i + 1;
            break;
        }
    }
    if (float_start_idx == -1) float_start_idx = 0;

    /* 提取参数名部分 */
    size_t param_name_len = (size_t)float_start_idx;
    if (param_name_len > VOFA_STRING_DATA_LEN) param_name_len = VOFA_STRING_DATA_LEN;

    /* 解析浮点数部分 */
    float parsed_float = 0.0f;
    int   float_valid  = 0;
    if (float_start_idx < (int)total_rx_len)
    {
        char *endptr = NULL;
        parsed_float = strtof(&rx_string[float_start_idx], &endptr);
        if (endptr != &rx_string[float_start_idx]) float_valid = 1;
    }

    /* 遍历接收链表, 查找匹配的参数名 */
    for (VOFA_RxNode *node = g_rx_head; node != NULL; node = node->next)
    {
        /* 比对参数名是否匹配 */
        if (node->string_len != param_name_len) continue;
        if (param_name_len > 0 && memcmp(node->string_data, rx_string, param_name_len) != 0) continue;

        /* 参数名匹配成功, 写入浮点数 */
        if (float_valid && node->float_data != NULL) *(node->float_data) = parsed_float;

        /* 回调 */
        if (node->callback != NULL) node->callback();

        return;
    }
}

/**
 * @brief 设置VOFA工作格式
 */
vofa_err_t VOFA_SetFormat(VOFA_Format format)
{
    g_current_format = format;
    return VOFA_OK;
}

/**
 * @brief 通用发送函数 - 根据当前格式自动选择发送方式
 */
vofa_err_t VOFA_Send(void)
{
    if (!g_uart_dev) return VOFA_ERR;

    switch (g_current_format)
    {
    case VOFA_FORMAT_JUSTFLOAT:
        return VOFA_SendJustFloat();
    case VOFA_FORMAT_FIREWATER:
        return VOFA_SendFireWater();
    default:
        return VOFA_ERR;
    }
}

/* ============ 模块入口 ============ */

/* 接收线程入口: 周期调用 VOFA_ReceiveHandler 解析下行数据 */
static void vofa_rx_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        VOFA_ReceiveHandler();
        tx_thread_sleep(2);
    }
}

/* 发送线程入口: 按 VOFA_TX_INTERVAL_MS 周期自动上报全部已注册 Tx 数据 */
static void vofa_tx_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        VOFA_Send();
        tx_thread_sleep(VOFA_TX_INTERVAL_MS);
    }
}

/**
 * @brief 初始化 VOFA 模块 (模块自建 RX/TX 双线程范式)
 *        内部: 初始化 UART + 创建接收线程(解析下行) + 发送线程(周期上报)
 */
void Module_VOFA_Init(void)
{
    if (g_initialized) return;

    /* 1. 初始化 UART6 (DMA 接收, 不定长) */
    UART_Device_init_config uart_cfg = {
        .huart           = &VOFA_UART,
        .rx_buf          = &g_vofa_rx_buf[0][0],
        .rx_buf_size     = sizeof(g_vofa_rx_buf),
        .expected_rx_len = 0,          /* 不定长: 接收任何可用数据 */
        .rx_mode         = UART_MODE_DMA,
        .tx_mode         = UART_MODE_BLOCKING,
    };

    UART_Device *uart_dev = BSP_UART_Device_Init(&uart_cfg);
    if (uart_dev == NULL)
    {
        LOG_E("VOFA UART init failed");
        return;
    }

    /* 2. 初始化 VOFA 协议核心 */
    if (VOFA_Init(uart_dev, (VOFA_Format)VOFA_FORMAT, VOFA_FIREWATER_PREFIX) != VOFA_OK)
    {
        LOG_E("VOFA init failed");
        return;
    }

    /* 3. 创建接收线程 (模块自建任务) */
    UINT status = tx_thread_create(&g_vofa_rx_task, "VOFA Rx", vofa_rx_task_entry, 0,
                                   g_vofa_rx_stack, VOFA_TASK_STACK_SIZE,
                                   VOFA_TASK_PRIORITY, VOFA_TASK_PRIORITY,
                                   TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("VOFA rx task create failed (0x%02x)", status);
        return;
    }

    /* 4. 创建发送线程 (模块自建任务, 周期上报) */
    status = tx_thread_create(&g_vofa_tx_task, "VOFA Tx", vofa_tx_task_entry, 0,
                              g_vofa_tx_stack, VOFA_TASK_STACK_SIZE,
                              VOFA_TASK_PRIORITY, VOFA_TASK_PRIORITY,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("VOFA tx task create failed (0x%02x)", status);
        return;
    }

    g_initialized = true;
    LOG_I("VOFA module initialized (UART, format=%d, tx_interval=%dms)", VOFA_FORMAT, VOFA_TX_INTERVAL_MS);
}
