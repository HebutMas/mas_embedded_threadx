#include "module_bmi088.h"
#include "BMI088_reg.h"
#include "bsp_dwt.h"
#include "bsp_pwm.h"
#include "bsp_flash.h"
#include "pid.h"
#include "tx_api.h"
#include "spi.h"
#include "gpio.h"
#include "tim.h"
#include <stdint.h>
#include <string.h>

#define LOG_TAG "module_bmi088"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

static Bmi088_device_t bmi088_device;

/* 表索引 */
#define BMI088_REG   0
#define BMI088_DATA  1
#define BMI088_ERROR 2

/* Accel 初始化表: {寄存器, 值, 错误码} */
static const uint8_t BMI088_Accel_Init_Table[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR},
};

/* Gyro 初始化表: {寄存器, 值, 错误码} */
static const uint8_t BMI088_Gyro_Init_Table[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR},
};

/* 内部 SPI 读写 */
static inline uint8_t _bmi088_write_reg(SPI_Device *device, uint8_t reg, const uint8_t *data, uint8_t len, uint32_t timeout)
{
    uint8_t addr = reg & BMI088_SPI_WRITE_CODE;
    return BSP_SPI_TransAndTrans(device, &addr, 1, data, len, timeout);
}

static inline uint8_t _bmi088_read_reg(SPI_Device *device, uint8_t reg, uint8_t *data, uint8_t len, uint8_t is_accel, uint32_t timeout)
{
    if (len > 6) return 1;

    uint8_t tx[8] = {0};
    uint8_t rx[8];
    uint8_t dummy_bytes = is_accel ? 2 : 1;

    tx[0] = BMI088_SPI_READ_CODE | reg;

    uint8_t status = BSP_SPI_TransReceive(device, tx, rx, len + dummy_bytes, timeout);
    memcpy(data, rx + dummy_bytes, len);
    return status;
}

/**
 * @brief 按初始化表写入并回读校验
 */
static uint8_t _bmi088_write_init_table(SPI_Device *device, const uint8_t table[][3], uint8_t rows, uint8_t is_accel)
{
    uint8_t             error = BMI088_NO_ERROR;
    uint8_t             reg, data, readback;
    BMI088_ERORR_CODE_e error_code;

    for (uint8_t i = 0; i < rows; i++)
    {
        reg        = table[i][BMI088_REG];
        data       = table[i][BMI088_DATA];
        error_code = (BMI088_ERORR_CODE_e)table[i][BMI088_ERROR];

        _bmi088_write_reg(device, reg, &data, 1, TX_NO_WAIT);
        BSP_DWT_Delay(0.001f);

        _bmi088_read_reg(device, reg, &readback, 1, is_accel, TX_NO_WAIT);
        BSP_DWT_Delay(0.001f);

        if (readback != data)
        {
            error |= error_code;
            bmi088_device.BMI088_ERORR_CODE |= error_code;
            LOG_E("%s Init failed reg:0x%02X (w:0x%02X r:0x%02X)", is_accel ? "ACCEL" : "GYRO", reg, data, readback);
        }
    }
    return error;
}

static void bmi088_acc_init(void)
{
    uint8_t whoami = 0;
    uint8_t tmp;

    /* SPI 模式切换: 上电默认 I2C, 需要一次读操作触发上升沿切换到 SPI */
    _bmi088_read_reg(bmi088_device.acc_device, BMI088_ACC_CHIP_ID, &whoami, 1, 1, TX_NO_WAIT);
    BSP_DWT_Delay(0.001f);

    /* 软复位 */
    tmp = BMI088_ACC_SOFTRESET_VALUE;
    _bmi088_write_reg(bmi088_device.acc_device, BMI088_ACC_SOFTRESET, &tmp, 1, TX_NO_WAIT);
    BSP_DWT_Delay(BMI088_COM_WAIT_SENSOR_TIME);

    /* 读 WHO AM I */
    _bmi088_read_reg(bmi088_device.acc_device, BMI088_ACC_CHIP_ID, &whoami, 1, 1, TX_NO_WAIT);
    BSP_DWT_Delay(0.001f);
    // if (whoami != BMI088_ACC_CHIP_ID_VALUE)
    // {
    //     LOG_E("Accel WHOAMI mismatch: 0x%02X (expected 0x%02X)", whoami, BMI088_ACC_CHIP_ID_VALUE);
    // }

    /* 写配置表 */
    _bmi088_write_init_table(bmi088_device.acc_device, BMI088_Accel_Init_Table, BMI088_WRITE_ACCEL_REG_NUM, 1);
}

static void bmi088_gyro_init(void)
{
    uint8_t whoami = 0;
    uint8_t tmp;

    /* 软复位 */
    tmp = BMI088_GYRO_SOFTRESET_VALUE;
    _bmi088_write_reg(bmi088_device.gyro_device, BMI088_GYRO_SOFTRESET, &tmp, 1, TX_NO_WAIT);
    BSP_DWT_Delay(BMI088_COM_WAIT_SENSOR_TIME);

    /* 读 WHO AM I */
    _bmi088_read_reg(bmi088_device.gyro_device, BMI088_GYRO_CHIP_ID, &whoami, 1, 0, TX_NO_WAIT);
    BSP_DWT_Delay(0.001f);
    if (whoami != BMI088_GYRO_CHIP_ID_VALUE)
    {
        LOG_E("Gyro WHOAMI mismatch: 0x%02X (expected 0x%02X)", whoami, BMI088_GYRO_CHIP_ID_VALUE);
        bmi088_device.BMI088_ERORR_CODE = BMI088_NO_SENSOR;
        return;
    }

    /* 写配置表 */
    _bmi088_write_init_table(bmi088_device.gyro_device, BMI088_Gyro_Init_Table, BMI088_WRITE_GYRO_REG_NUM, 0);
}

/* 对外函数 */

int8_t Module_BMI088_get_accel(void)
{
    if (!bmi088_device.initialized) return -1;

    uint8_t buf[6];
    int8_t  status = _bmi088_read_reg(bmi088_device.acc_device, BMI088_ACCEL_XOUT_L, buf, 6, 1, TX_WAIT_FOREVER);

    for (uint8_t i = 0; i < 3; i++)
    {
        int16_t raw          = (int16_t)((buf[2 * i + 1] << 8) | buf[2 * i]);
        float   val          = BMI088_ACCEL_6G_SEN * (float)raw;
        bmi088_device.acc[i] = bmi088_device.BMI088_Cali_Offset.Calibrated ? val * bmi088_device.BMI088_Cali_Offset.AccelScale : val;
    }
    return status;
}

int8_t Module_BMI088_get_gyro(void)
{
    if (!bmi088_device.initialized) return -1;

    uint8_t buf[6];
    int8_t  status = _bmi088_read_reg(bmi088_device.gyro_device, BMI088_GYRO_X_L, buf, 6, 0, TX_WAIT_FOREVER);

    for (uint8_t i = 0; i < 3; i++)
    {
        int16_t raw           = (int16_t)((buf[2 * i + 1] << 8) | buf[2 * i]);
        float   val           = BMI088_GYRO_2000_SEN * (float)raw;
        bmi088_device.gyro[i] = bmi088_device.BMI088_Cali_Offset.Calibrated ? val - bmi088_device.BMI088_Cali_Offset.GyroOffset[i] : val;
    }
    return status;
}

int8_t Module_BMI088_get_temp(void)
{
    if (!bmi088_device.initialized) return -1;

    uint8_t buf[2];
    int8_t  status = _bmi088_read_reg(bmi088_device.acc_device, BMI088_TEMP_M, buf, 2, 1, TX_WAIT_FOREVER);

    int16_t raw = (((int16_t)buf[0] << 3) | (buf[1] >> 5));
    if (raw > 1023) raw -= 2048;

    bmi088_device.temperature = (float)raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    return status;
}

void Module_BMI088_temp_ctrl(void)
{
#if BMI088_TEMP_ENABLE
    if (!bmi088_device.bmi088_pwm) return;

    PIDCalculate(&bmi088_device.pid_temp, bmi088_device.temperature, BMI088_TEMP_SET);

    float duty = bmi088_device.pid_temp.Output;
    if (duty < 0.0f)
        duty = 0.0f;
    else if (duty > 1000.0f)
        duty = 1000.0f;

    BSP_PWM_SetDutyCycle(bmi088_device.bmi088_pwm, (uint16_t)duty);
#endif
}

Bmi088_device_t *Module_BMI088_get_device(void) { return bmi088_device.initialized ? &bmi088_device : NULL; }

void Module_BMI088_init(void)
{
    memset(&bmi088_device, 0, sizeof(bmi088_device));

    /* SPI / PWM 设备初始化 */
#if defined(STM32H723xx)
    SPI_Device_Init_Config gyro_cfg = {
        .hspi    = &hspi2,
        .cs_port = GPIOC,
        .cs_pin  = GPIO_PIN_3,
        .tx_mode = SPI_MODE_BLOCKING,
        .rx_mode = SPI_MODE_BLOCKING,
    };
    bmi088_device.gyro_device = BSP_SPI_Device_Init(&gyro_cfg);

    SPI_Device_Init_Config acc_cfg = {
        .hspi    = &hspi2,
        .cs_port = GPIOC,
        .cs_pin  = GPIO_PIN_0,
        .tx_mode = SPI_MODE_BLOCKING,
        .rx_mode = SPI_MODE_BLOCKING,
    };
    bmi088_device.acc_device = BSP_SPI_Device_Init(&acc_cfg);

    PWM_Init_Config pwm_cfg = {
        .Channel = TIM_CHANNEL_4,
        .dutyx10 = 0,
        .htim    = &htim3,
        .Mode    = PWM_MODE_BLOCKING,
    };
    bmi088_device.bmi088_pwm = BSP_PWM_Device_Init(&pwm_cfg);

#elif defined(STM32F407xx)
    SPI_Device_Init_Config gyro_cfg = {
        .hspi    = &hspi1,
        .cs_port = GPIOB,
        .cs_pin  = GPIO_PIN_0,
        .tx_mode = SPI_MODE_BLOCKING,
        .rx_mode = SPI_MODE_BLOCKING,
    };
    bmi088_device.gyro_device = BSP_SPI_Device_Init(&gyro_cfg);

    SPI_Device_Init_Config acc_cfg = {
        .hspi    = &hspi1,
        .cs_port = GPIOA,
        .cs_pin  = GPIO_PIN_4,
        .tx_mode = SPI_MODE_BLOCKING,
        .rx_mode = SPI_MODE_BLOCKING,
    };
    bmi088_device.acc_device = BSP_SPI_Device_Init(&acc_cfg);

    PWM_Init_Config pwm_cfg = {
        .Channel = TIM_CHANNEL_1,
        .dutyx10 = 0,
        .htim    = &htim10,
        .Mode    = PWM_MODE_BLOCKING,
    };
    bmi088_device.bmi088_pwm = BSP_PWM_Device_Init(&pwm_cfg);
#endif

    if (!bmi088_device.acc_device || !bmi088_device.gyro_device || !bmi088_device.bmi088_pwm)
    {
        LOG_E("SPI/PWM device init failed");
        return;
    }

    /* 传感器初始化 */
    bmi088_acc_init();
    bmi088_gyro_init();

    /* 温度 PID 初始化 */
    PID_Init_Config_s pid_cfg = {
        .MaxOut        = 1000,
        .IntegralLimit = 20,
        .DeadBand      = 0,
        .Kp            = 20,
        .Ki            = 0,
        .Kd            = 0,
        .Improve       = 0x01,
    };
    PIDInit(&bmi088_device.pid_temp, &pid_cfg);

#if BMI088_TEMP_ENABLE
    BSP_PWM_Start(bmi088_device.bmi088_pwm, NULL, 0);
#endif

    bmi088_device.initialized = 1;

    /* 加载标定数据 */
    uint8_t flash_buf[sizeof(BMI088_Cali_Offset_t) + 2] = {0};
    BSP_FLASH_Read_Buffer(flash_buf, sizeof(flash_buf));

    //if(1)如果需要重新标定数据
    if (flash_buf[sizeof(BMI088_Cali_Offset_t) + 1] != 0xAA)
    {
        /* 无有效标定数据, 执行标定 */
        Module_BMI088_calibrate_bmi088_offset(&bmi088_device);
    }
    else
    {
        memcpy(&bmi088_device.BMI088_Cali_Offset, flash_buf, sizeof(BMI088_Cali_Offset_t));
    }

    if (bmi088_device.BMI088_ERORR_CODE == BMI088_NO_ERROR)
    {
        LOG_I("BMI088 init success");
        return;
    }

    LOG_W("BMI088 init with errors: 0x%02X", bmi088_device.BMI088_ERORR_CODE);
}
