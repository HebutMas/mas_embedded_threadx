#include "module_wt606.h"
#include "arm_math.h"
#include "bsp_def.h"
#include "usart.h"

#define LOG_TAG "module_wt"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

//  帧类型宏
#define WT606_TYPE_TIME    0x50 /* 时间 */
#define WT606_TYPE_ACC     0x51 /* 加速度 + 温度 */
#define WT606_TYPE_GYRO    0x52 /* 角速度 + 电压 */
#define WT606_TYPE_ANGLE   0x53 /* 角度 (欧拉角) + 版本号 */
#define WT606_TYPE_MAG     0x54 /* 磁场 */
#define WT606_TYPE_PORT    0x55 /* 端口状态 */
#define WT606_TYPE_PRESS   0x56 /* 气压高度 */
#define WT606_TYPE_GNSS    0x57 /* 经纬度 */
#define WT606_TYPE_GNDSPD  0x58 /* 地速 */
#define WT606_TYPE_QUAT    0x59 /* 四元数 */
#define WT606_TYPE_GPS_ACC 0x5A /* GPS 定位精度 */
#define WT606_TYPE_READ    0x5F /* 读取 (查询帧) */

/* 传感器量程常数 */
#define ACC_RANGE_G        16.0f                              /* 加速度量程: ±16g */
#define GRAVITY            9.8f                               /* 重力加速度 (m/s²) */
#define ACC_SCALE          (ACC_RANGE_G * GRAVITY / 32768.0f) /* ≈ 0.004785 */
#define GYRO_RANGE_DPS     2000.0f                            /* 角速度量程: ±2000°/s */
#define GYRO_SCALE         (GYRO_RANGE_DPS / 32768.0f)        /* ≈ 0.06104 */
#define ANGLE_RANGE_DEG    180.0f                             /* 欧拉角量程: ±180° */
#define ANGLE_SCALE        (ANGLE_RANGE_DEG / 32768.0f)       /* ≈ 0.005493 */
#define QUAT_SCALE         (1.0f / 32768.0f)                  /* 四元数归一化因子 */

/* 解析辅助宏: 将两个字节组合为有符号 short */
#define WT606_RAW16(ptr)   ((int16_t)((int16_t)(ptr)[1] << 8 | (ptr)[0]))

static Module_WT606_Device_t      wt606_device;
static TX_THREAD                  wt606_thread;
APPS_STACK_SECTION static uint8_t wt606_thread_stack[WT606_TASK_STACK_SIZE];
BUFFER_SECTION static uint8_t     data_buffer[512];

/**
 * @brief 校验和验证 — 累加帧头到 DATA4H 共10字节，取低8位与 SUM 字节比较
 */
static inline bool wt606_checksum_ok(const uint8_t *frame)
{
    uint8_t sum = 0;
    for (uint8_t j = 0; j < 10; j++)
    {
        sum += frame[j];
    }
    return sum == frame[10];
}

/**
 * @brief 解析加速度包 (TYPE=0x51)
 * DATA1=Ax, DATA2=Ay, DATA3=Az, DATA4=温度
 * 加速度 = raw / 32768 * 16 * g  → 单位: m/s²
 * 温度   = raw / 100             → 单位: °C
 */
static void wt606_parse_acc(const uint8_t *data)
{
    // Ax
    wt606_device.acc[0] = WT606_RAW16(&data[0]) * ACC_SCALE;
    // Ay
    wt606_device.acc[1] = WT606_RAW16(&data[2]) * ACC_SCALE;
    // Az
    wt606_device.acc[2] = WT606_RAW16(&data[4]) * ACC_SCALE;
    // 温度
    wt606_device.temperature = WT606_RAW16(&data[6]) * 0.01f;
}

/**
 * @brief 解析角速度包 (TYPE=0x52)
 * DATA1=Wx, DATA2=Wy, DATA3=Wz, DATA4=电压 (非蓝牙产品无效)
 * 角速度 = raw / 32768 * 2000  → 单位: °/s
 * 注: WT9011G4K 需改为 4000°/s 系数
 */
static void wt606_parse_gyro(const uint8_t *data)
{
    // Wx
    wt606_device.gyro_dps[0] = WT606_RAW16(&data[0]) * GYRO_SCALE;
    // Wy
    wt606_device.gyro_dps[1] = WT606_RAW16(&data[2]) * GYRO_SCALE;
    // Wz
    wt606_device.gyro_dps[2] = WT606_RAW16(&data[4]) * GYRO_SCALE;

    // 转换为 rad/s
    const float deg_to_rad   = PI / 180.0f;
    wt606_device.gyro_rps[0] = wt606_device.gyro_dps[0] * deg_to_rad;
    wt606_device.gyro_rps[1] = wt606_device.gyro_dps[1] * deg_to_rad;
    wt606_device.gyro_rps[2] = wt606_device.gyro_dps[2] * deg_to_rad;
}

/**
 * @brief 解析角度包 (TYPE=0x53)
 * DATA1=Roll, DATA2=Pitch, DATA3=Yaw, DATA4=版本号
 * 欧拉角 = raw / 32768 * 180  → 单位: °
 * 处理 yaw 过零累计
 */
static void wt606_parse_angle(const uint8_t *data)
{
    // Roll
    wt606_device.Euler_degree[0] = WT606_RAW16(&data[0]) * ANGLE_SCALE;
    // Pitch
    wt606_device.Euler_degree[1] = WT606_RAW16(&data[2]) * ANGLE_SCALE;
    // Yaw
    wt606_device.Euler_degree[2] = WT606_RAW16(&data[4]) * ANGLE_SCALE;

    // yaw 过零累计
    const float delta_yaw = wt606_device.Euler_degree[2] - wt606_device.last_yaw_degree;
    if (delta_yaw > 180.0f)
    {
        wt606_device.YawRoundCount--;
    }
    else if (delta_yaw < -180.0f)
    {
        wt606_device.YawRoundCount++;
    }

    wt606_device.last_yaw_degree      = wt606_device.Euler_degree[2];
    wt606_device.YawTotalAngle_degree = wt606_device.Euler_degree[2] + 360.0f * wt606_device.YawRoundCount;

    // 转换为 rad
    const float deg_to_rad         = PI / 180.0f;
    wt606_device.Euler_rad[0]      = wt606_device.Euler_degree[0] * deg_to_rad;
    wt606_device.Euler_rad[1]      = wt606_device.Euler_degree[1] * deg_to_rad;
    wt606_device.Euler_rad[2]      = wt606_device.Euler_degree[2] * deg_to_rad;
    wt606_device.YawTotalAngle_rad = wt606_device.YawTotalAngle_degree * deg_to_rad;
}

/**
 * @brief 解析四元数包 (TYPE=0x59)
 * DATA1=Q0(w), DATA2=Q1(x), DATA3=Q2(y), DATA4=Q3(z)
 * 四元数 = raw / 32768  → 范围 [-1, 1]
 */
static void wt606_parse_quat(const uint8_t *data)
{
    wt606_device.quat[0] = WT606_RAW16(&data[0]) * QUAT_SCALE;
    wt606_device.quat[1] = WT606_RAW16(&data[2]) * QUAT_SCALE;
    wt606_device.quat[2] = WT606_RAW16(&data[4]) * QUAT_SCALE;
    wt606_device.quat[3] = WT606_RAW16(&data[6]) * QUAT_SCALE;
}

static void wt606_task_entry(ULONG arg)
{
    while (1)
    {
        if (wt606_device.initialized == 1)
        {
            static uint8_t buf[256];
            uint32_t       rx_len;
            if (BSP_UART_Read(wt606_device.uart_dev, buf, sizeof(buf), &rx_len, TX_WAIT_FOREVER) > 0)
            {
                // 扫描帧头 0x55，需要至少11字节才能组成一帧
                for (uint16_t i = 0; i + 10 < rx_len; i++)
                {
                    if (buf[i] != 0x55) continue;

                    if (!wt606_checksum_ok(&buf[i])) continue;

                    const uint8_t  type = buf[i + 1];
                    const uint8_t *data = &buf[i + 2];

                    switch (type)
                    {
                    case WT606_TYPE_ACC:
                        wt606_parse_acc(data);
                        break;

                    case WT606_TYPE_GYRO:
                        wt606_parse_gyro(data);
                        break;

                    case WT606_TYPE_ANGLE:
                        wt606_parse_angle(data);
                        break;
                    case WT606_TYPE_QUAT:
                        wt606_parse_quat(data);
                        break;
                    default:
                        break;
                    }

                    Module_Offline_device_update(wt606_device.offline_dev);
                }
            }
        }
        else
        {
            tx_thread_sleep(10);
        }
    }
}

int Module_WT606_Init(void)
{
    // 重新初始化 UART
    WT606_UART.Init.BaudRate = 921600;
    if (HAL_UART_Init(&WT606_UART) != HAL_OK)
    {
        LOG_E("uart init error");
        return -1;
    }

    UART_Device_init_config config = {
        .huart           = &WT606_UART,
        .expected_rx_len = 0,
        .rx_buf          = data_buffer,
        .rx_buf_size     = 512,
        .tx_mode         = UART_MODE_DMA,
        .rx_mode         = UART_MODE_DMA,
    };
    wt606_device.uart_dev = BSP_UART_Device_Init(&config);
    if (wt606_device.uart_dev == NULL)
    {
        LOG_E("uart device init error");
        return -1;
    }

    Offline_Init_config_t offlineconfig = {
        .name       = "wt606",
        .beep_times = 10,
        .enable     = WT606_OFFLINE_ENABLE,
        .timeout_ms = 100,
    };
    wt606_device.offline_dev = Module_Offline_register(&offlineconfig);
    if (wt606_device.offline_dev == NULL)
    {
        LOG_E("offline device register error");
        return -1;
    }

    if (tx_thread_create(&wt606_thread, "wt606", wt606_task_entry, 0, wt606_thread_stack, WT606_TASK_STACK_SIZE, WT606_TASK_PRIORITY,
                         WT606_TASK_PRIORITY, TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("wt606 thread create error");
        return -1;
    }

    wt606_device.initialized = 1;
    LOG_I("module wt606 init finished");

    return 0;
}

const Module_WT606_Device_t *Module_WT606_Get(void)
{
    if (wt606_device.initialized)
    {
        return &wt606_device;
    }
    return NULL;
}

uint8_t Module_WT606_Get_offline_state(void)
{
    if (wt606_device.offline_dev == NULL)
    {
        return STATE_OFFLINE;
    }
    return Module_Offline_get_device_status(wt606_device.offline_dev);
}
