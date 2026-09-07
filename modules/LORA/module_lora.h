/*
 * @Author: shentianao 2755930351@qq.com
 * @Date: 2026-09-04 19:04:15
 * @LastEditors: shentianao 2755930351@qq.com
 * @LastEditTime: 2026-09-06 21:08:56
 * @FilePath: \fork\modules\LORA\module_lora.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * @brief 塔石 L33 LoRa 透传模块(框架版)
 *        两板点对点透传,应用层帧:0xAA 0x55 + LEN + SEQ + 数据 + CRC8
 *        参数已在塔石上位机配好(115200 / 19200 / 透传 / 信道23),本模块只做收发。
 * @note  启用前需在 robot.cmake 配置(见 modules/module_config.cmake 与 LORA 说明):
 *        LORA_UART、LORA_M0_GPIO_PORT/PIN、LORA_M1_GPIO_PORT/PIN、[LORA_AUX_GPIO_PORT/PIN]
 */
#ifndef _MODULE_LORA_H_
#define _MODULE_LORA_H_

#include "bsp_uart.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LORA_TYPE_INT8,
    LORA_TYPE_INT16,
    LORA_TYPE_INT32,
    LORA_TYPE_UINT8,
    LORA_TYPE_UINT16,
    LORA_TYPE_UINT32,
    LORA_TYPE_FLOAT,
    LORA_TYPE_DOUBLE,
} Lora_Data_Type;

/* 注册项只保存用户变量指针，不复制变量内容。 */
int Lora_Register(const char *name, void *value, Lora_Data_Type type);

void Module_Lora_Init(void);
int Lora_Start(void);

#endif /* _MODULE_LORA_H_ */
