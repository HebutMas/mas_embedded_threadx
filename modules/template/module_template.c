#include "module_template.h"

/* 根据实际硬件需求包含 BSP 头文件 */
/* 根据需求添加offline相关头文件 */

#include "tx_api.h"
#include "bsp_def.h"

/* ulog 日志配置 */
#define LOG_TAG "module_template"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 静态变量 */

static Module_Template_Device_t   template_device;
static TX_THREAD                  template_thread;
APPS_STACK_SECTION static uint8_t template_thread_stack[TEMPLATE_TASK_STACK_SIZE];

/* 辅助函数 */

/**
 * @brief 模块主任务入口
 */
static void template_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        if (template_device.initialized)
        {
            /* TODO: 在此编写模块主循环逻辑 */
            // 例如: 读取传感器、处理数据、发送控制指令等

            /* 示例: 模拟数据处理 */
            template_device.example_value += 0.1f;

            /* TODO: 如需离线检测, 取消下面注释 */
            // Module_Offline_device_update(template_offline_dev);
        }

        /* 任务周期休眠 */
        tx_thread_sleep(TEMPLATE_TASK_INTERVAL_MS);
    }
}

/**
 * @brief 硬件初始化 (示例)
 * @return 0 成功, 非 0 失败
 */
static int template_hw_init(void)
{
    /* TODO: 在此初始化硬件外设 */
    // 例如: UART / CAN / I2C / SPI / GPIO / DMA 等

    // 示例代码:
    // UART_Device_init_config uart_config = {
    //     .huart           = &huartX,
    //     .expected_rx_len = 0,
    //     .rx_buf          = rx_buffer,
    //     .rx_buf_size     = sizeof(rx_buffer),
    //     .tx_mode         = UART_MODE_DMA,
    //     .rx_mode         = UART_MODE_DMA,
    // };
    // template_device.uart_dev = BSP_UART_Device_Init(&uart_config);
    // if (template_device.uart_dev == NULL) {
    //     LOG_E("hardware init failed");
    //     return -1;
    // }

    return 0;
}

/* 对外函数 */

int Module_Template_Init(void)
{
    if (template_device.initialized)
    {
        LOG_W("already initialized");
        return 0;
    }

    memset(&template_device, 0, sizeof(template_device));

    /* 硬件初始化 */
    if (template_hw_init() != 0)
    {
        LOG_E("hw init failed");
        return -1;
    }

    /* TODO: 如需离线检测, 取消下面注释 */
    // Offline_Init_config_t offline_cfg = {
    //     .name       = "template",
    //     .beep_times = 5,
    //     .enable     = 1,
    //     .timeout_ms = 200,
    // };
    // template_offline_dev = Module_Offline_register(&offline_cfg);
    // if (template_offline_dev == NULL) {
    //     LOG_E("offline register failed");
    //     return -1;
    // }

    /* 创建任务线程 */
    UINT status = tx_thread_create(&template_thread, "template", template_task_entry, 0, template_thread_stack, TEMPLATE_TASK_STACK_SIZE,
                                   TEMPLATE_TASK_PRIORITY, TEMPLATE_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START);

    if (status != TX_SUCCESS)
    {
        LOG_E("thread create failed, status=%d", status);
        return -1;
    }

    template_device.initialized = 1;
    LOG_I("module template init finished");

    return 0;
}

const Module_Template_Device_t *Module_Template_Get(void)
{
    if (template_device.initialized)
    {
        return &template_device;
    }
    return NULL;
}

void Module_Template_SetExample(float value) { template_device.example_value = value; }
