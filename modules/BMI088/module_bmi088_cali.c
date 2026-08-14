#include "module_bmi088.h"
#include "bsp_dwt.h"
#include "bsp_led.h"
#include "bsp_flash.h"
#include "math.h"
#include "iwdg.h"

#define LOG_TAG "module_bmi088"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* 内部辅助函数 */

static inline bool _is_data_valid(float gnorm, const float gyro[3])
{
    if (isnan(gnorm) || isinf(gnorm)) return false;
    if (gnorm < 5.0f || gnorm > 15.0f) return false;
    for (int i = 0; i < 3; i++)
    {
        if (isnan(gyro[i]) || isinf(gyro[i])) return false;
        if (fabsf(gyro[i]) > 5.0f) return false;
    }
    return true;
}

static inline bool _is_stable(float gnorm_diff, const float gyro_diff[3])
{
    return (gnorm_diff <= 0.5f) && (gyro_diff[0] <= 0.15f) && (gyro_diff[1] <= 0.15f) && (gyro_diff[2] <= 0.15f);
}

static inline bool _is_result_ok(float gnorm, const float gyro_offset[3], float gnorm_diff, const float gyro_diff[3])
{
    return (fabsf(gnorm - 9.81f) <= 0.5f) && (fabsf(gyro_offset[0]) <= 0.01f) && (fabsf(gyro_offset[1]) <= 0.01f) &&
           (fabsf(gyro_offset[2]) <= 0.01f) && _is_stable(gnorm_diff, gyro_diff);
}

static void _apply_default(BMI088_Cali_Offset_t *cali)
{
    cali->GyroOffset[0] = -0.00601393497f;
    cali->GyroOffset[1] = -0.00196841615f;
    cali->GyroOffset[2] = 0.00114696583f;
    cali->gNorm         = 9.67463112f;
    cali->TempWhenCali  = 40.0f;
    cali->Calibrated    = 1;
    cali->AccelScale    = 9.81f / 9.67463112f;
}

/**
 * @brief 采集一轮 6000 组有效数据, 计算 gNorm / GyroOffset
 * @return 有效采样数, 0 表示失败
 */
static uint16_t _collect_samples(Bmi088_device_t *dev, float *gnorm_out, float gyro_offset[3], float *gnorm_diff, float gyro_diff[3])
{
    float gnorm_max = 0.0f, gnorm_min = 100.0f;
    float gyro_max[3] = {-100.0f, -100.0f, -100.0f};
    float gyro_min[3] = {100.0f, 100.0f, 100.0f};

    float    gnorm_sum   = 0.0f;
    float    gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t valid       = 0;

    float    elapsed = 0.0f;
    uint32_t dt_cnt  = 0;

    while (valid < 6000)
    {
        elapsed += BSP_DWT_GetDeltaT(&dt_cnt);

        /* 超时保护: 10s */
        if (elapsed > 10.0f)
        {
            LOG_E("Calibration timeout after %.1fs, collected %u/6000 samples", (double)elapsed, valid);
            return 0;
        }

        /* 读传感器 */
        if (Module_BMI088_get_accel() != 0 || Module_BMI088_get_gyro() != 0)
        {
            BSP_DWT_Delay(0.001f);
            continue;
        }

        float gnorm = sqrtf(dev->acc[0] * dev->acc[0] + dev->acc[1] * dev->acc[1] + dev->acc[2] * dev->acc[2]);

        /* 数据合理性校验 */
        if (!_is_data_valid(gnorm, dev->gyro))
        {
            BSP_DWT_Delay(0.001f);
            continue;
        }

        /* 累加 */
        gnorm_sum += gnorm;
        gyro_sum[0] += dev->gyro[0];
        gyro_sum[1] += dev->gyro[1];
        gyro_sum[2] += dev->gyro[2];
        valid++;

        /* 更新 min/max */
        if (valid == 1)
        {
            gnorm_max = gnorm_min = gnorm;
            gyro_max[0] = gyro_min[0] = dev->gyro[0];
            gyro_max[1] = gyro_min[1] = dev->gyro[1];
            gyro_max[2] = gyro_min[2] = dev->gyro[2];
        }
        else
        {
            if (gnorm > gnorm_max) gnorm_max = gnorm;
            if (gnorm < gnorm_min) gnorm_min = gnorm;
            for (int j = 0; j < 3; j++)
            {
                if (dev->gyro[j] > gyro_max[j]) gyro_max[j] = dev->gyro[j];
                if (dev->gyro[j] < gyro_min[j]) gyro_min[j] = dev->gyro[j];
            }
        }

        /* 波动检测 — 设备移动则丢弃本轮, 重新采集 6000 组 */
        float g_diff  = gnorm_max - gnorm_min;
        float gx_diff = gyro_max[0] - gyro_min[0];
        float gy_diff = gyro_max[1] - gyro_min[1];
        float gz_diff = gyro_max[2] - gyro_min[2];

        if (!_is_stable(g_diff, (const float[3]){gx_diff, gy_diff, gz_diff}))
        {
            LOG_W("Device moved (gNorm diff=%.3f gyro diff=%.3f/%.3f/%.3f), restarting...", (double)g_diff, (double)gx_diff, (double)gy_diff,
                  (double)gz_diff);
            valid       = 0;
            gnorm_sum   = 0.0f;
            gyro_sum[0] = gyro_sum[1] = gyro_sum[2] = 0.0f;
            gnorm_max                               = 0.0f;
            gnorm_min                               = 100.0f;
            for (int j = 0; j < 3; j++)
            {
                gyro_max[j] = -100.0f;
                gyro_min[j] = 100.0f;
            }
            BSP_DWT_Delay(0.5f);
        }

        /* 喂狗，防止独立看门狗复位 */
#if defined(STM32H723xx)
        HAL_IWDG_Refresh(&hiwdg1);
#elif defined(STM32F407xx)
        HAL_IWDG_Refresh(&hiwdg);
#endif

        BSP_DWT_Delay(0.001f);
    }

    /* 计算平均值 */
    *gnorm_out = gnorm_sum / 6000.0f;
    for (int i = 0; i < 3; i++) gyro_offset[i] = gyro_sum[i] / 6000.0f;

    /* 返回最终波动值供外层 do-while 判定 */
    *gnorm_diff  = gnorm_max - gnorm_min;
    gyro_diff[0] = gyro_max[0] - gyro_min[0];
    gyro_diff[1] = gyro_max[1] - gyro_min[1];
    gyro_diff[2] = gyro_max[2] - gyro_min[2];

    return valid;
}

int8_t Module_BMI088_calibrate_bmi088_offset(Bmi088_device_t *dev)
{
    if (!dev || !dev->initialized)
    {
        LOG_E("BMI088 instance is NULL or not initialized");
        return -1;
    }

    BMI088_Cali_Offset_t cali;
    float                gnorm_diff, gyro_diff[3];
    int8_t               status = 0;

    BSP_LED_Show(LED_Yellow);

    do
    {
        memset(&cali, 0, sizeof(cali));

        uint16_t valid = _collect_samples(dev, &cali.gNorm, cali.GyroOffset, &gnorm_diff, gyro_diff);
        if (valid == 0)
        {
            _apply_default(&cali);
            gnorm_diff   = 0.0f;
            gyro_diff[0] = gyro_diff[1] = gyro_diff[2] = 0.0f;
            BSP_LED_Show(LED_Red);
            status = -1;
            break;
        }

        /* 记录标定温度 */
        if (Module_BMI088_get_temp() == 0)
        {
            cali.TempWhenCali = dev->temperature;
            LOG_I("Calibration temperature: %.2f C", (double)cali.TempWhenCali);
        }
        else
        {
            cali.TempWhenCali = 40.0f;
        }

        cali.Calibrated = 1;

        LOG_I("Round result: gNorm=%.4f offsets=%.6f/%.6f/%.6f gDiff=%.4f gyDiff=%.4f/%.4f/%.4f", (double)cali.gNorm, (double)cali.GyroOffset[0],
              (double)cali.GyroOffset[1], (double)cali.GyroOffset[2], (double)gnorm_diff, (double)gyro_diff[0], (double)gyro_diff[1],
              (double)gyro_diff[2]);

    } while (!_is_result_ok(cali.gNorm, cali.GyroOffset, gnorm_diff, gyro_diff));

    /* 计算缩放因子 */
    if (cali.gNorm > 0.1f)
    {
        cali.AccelScale = 9.81f / cali.gNorm;
        LOG_I("AccelScale: %f (gNorm: %f)", (double)cali.AccelScale, (double)cali.gNorm);
    }
    else
    {
        cali.AccelScale = 1.0f;
        LOG_E("Invalid gNorm, using default scale");
    }

    /* Flash 写入耗时较长, 写前喂狗防止复位 */
#if defined(STM32H723xx)
    HAL_IWDG_Refresh(&hiwdg1);
#elif defined(STM32F407xx)
    HAL_IWDG_Refresh(&hiwdg);
#endif

    /* 保存到 Flash */
    uint8_t flash_buf[sizeof(BMI088_Cali_Offset_t) + 2];
    memcpy(flash_buf, &cali, sizeof(cali));
    flash_buf[sizeof(BMI088_Cali_Offset_t) + 1] = 0xAA;
    BSP_FLASH_Write_Buffer(flash_buf, sizeof(flash_buf));

    /* 写入 dev */
    memcpy(&dev->BMI088_Cali_Offset, &cali, sizeof(cali));

    LOG_I("Gyro offsets: X=%f Y=%f Z=%f", (double)cali.GyroOffset[0], (double)cali.GyroOffset[1], (double)cali.GyroOffset[2]);

    /* Flash 写入耗时较长, 写后喂狗防止再次复位 */
#if defined(STM32H723xx)
    HAL_IWDG_Refresh(&hiwdg1);
#elif defined(STM32F407xx)
    HAL_IWDG_Refresh(&hiwdg);
#endif

    BSP_LED_Show(LED_Green);
    BSP_DWT_Delay(0.5f);
    return status;
}
