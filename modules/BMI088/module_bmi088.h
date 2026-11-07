/*
 * @Author: laladuduqq 2807523947@qq.com
 * @Date: 2025-09-11 13:43:14
 * @LastEditors: laladuduqq 2807523947@qq.com
 * @LastEditTime: 2026-01-22 18:46:15
 * @FilePath: /MAS_RM_DAMIAOH7/Mas_User/MODULES/BMI088/module_bmi088.h
 * @Description:
 */
#ifndef _BMI088_H_
#define _BMI088_H_

#include "BMI088_reg.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "pid.h"
#include <stdint.h>

#ifndef BMI088_TEMP_ENABLE
#define BMI088_TEMP_ENABLE 0 /* 启用温度控制 */
#endif
#ifndef BMI088_TEMP_SET
#define BMI088_TEMP_SET 35.0f /* 目标温度 ℃ */
#endif

typedef struct
{
    float   AccelScale;    /* 加速度计缩放因子 */
    float   GyroOffset[3]; /* 陀螺仪零偏 (dps) */
    float   gNorm;         /* 重力模长 (标定获取) */
    float   TempWhenCali;  /* 标定时温度 */
    uint8_t Calibrated;    /* 标定完成标志 */
} BMI088_Cali_Offset_t;

typedef struct
{
    uint8_t              initialized;
    SPI_Device          *gyro_device;
    SPI_Device          *acc_device;
    PWM_Device          *bmi088_pwm;
    PIDInstance          pid_temp;
    float                acc[3];      /* 加速度 (m/s²) */
    float                gyro[3];     /* 角速度 (dps) */
    float                temperature; /* 温度 (℃) */
    BMI088_Cali_Offset_t BMI088_Cali_Offset;
    BMI088_ERORR_CODE_e  BMI088_ERORR_CODE;
} Bmi088_device_t;

/**
 * @brief 初始化 BMI088
 * @return
 */
int Module_BMI088_init(void);

/**
 * @brief 获取设备指针
 * @return 设备指针, 未初始化返回 NULL
 */
Bmi088_device_t *Module_BMI088_get_device(void);

/**
 * @brief 读取加速度数据到 dev->acc[3]
 * @return 0 成功, -1 未初始化, >0 SPI 错误
 */
int8_t Module_BMI088_get_accel(void);

/**
 * @brief 读取陀螺仪数据到 dev->gyro[3]
 * @return 0 成功, -1 未初始化, >0 SPI 错误
 */
int8_t Module_BMI088_get_gyro(void);

/**
 * @brief 读取温度数据到 dev->temperature
 * @return 0 成功, -1 未初始化, >0 SPI 错误
 */
int8_t Module_BMI088_get_temp(void);

/**
 * @brief PID 温度控制 (需 BMI088_TEMP_ENABLE=1)
 */
void Module_BMI088_temp_ctrl(void);

/**
 * @brief 陀螺仪零偏标定并保存到 Flash
 * @param dev 设备指针
 * @return 0 成功, 非0 失败
 */
int8_t Module_BMI088_calibrate_bmi088_offset(Bmi088_device_t *dev);

#endif /* _BMI088_H_ */
