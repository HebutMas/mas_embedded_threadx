#include "bsp_led.h"

#define LOG_TAG "bsp_led"
#define LOG_LVL LOG_LVL_WARNING
#include "ulog_def.h"

#if defined(STM32H723xx)

#include "bsp_spi.h"
#include "spi.h"
#include "tx_api.h"

#define WS2812_LowLevel  0xC0 // 0码
#define WS2812_HighLevel 0xF0 // 1码

static SPI_Device *ws2812_dev = NULL;

void BSP_LED_Init(void)
{
    SPI_Device_Init_Config config = {
        .hspi    = &hspi6,
        .cs_port = NULL,
        .cs_pin  = 0,
        .tx_mode = SPI_MODE_IT,
        .rx_mode = SPI_MODE_BLOCKING,
    };

    ws2812_dev = BSP_SPI_Device_Init(&config);

    if (ws2812_dev != NULL)
    {
        BSP_LED_Show(LED_Black);
        LOG_I("RGB init success");
    }
    else
    {
        LOG_E("RGB init failed");
    }
}

void BSP_LED_Show(uint32_t aRGB)
{
    if (ws2812_dev == NULL)
    {
        return;
    }

    static uint8_t alpha;
    static uint8_t red, green, blue;
    static uint8_t buf[24];

    // 提取ARGB值
    alpha = (aRGB & 0xFF000000) >> 24;
    red   = (aRGB & 0x00FF0000) >> 16;
    green = (aRGB & 0x0000FF00) >> 8;
    blue  = (aRGB & 0x000000FF) >> 0;

    // 应用alpha通道
    if (alpha != 0xFF)
    {
        red   = (red * alpha) / 255;
        green = (green * alpha) / 255;
        blue  = (blue * alpha) / 255;
    }

    for (int i = 0; i < 8; i++)
    {
        buf[7 - i]  = (((green >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        buf[15 - i] = (((red >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        buf[23 - i] = (((blue >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }

    BSP_SPI_Transmit(ws2812_dev, buf, 24, TX_WAIT_FOREVER);
}

#elif defined(STM32F407xx)

#include "bsp_pwm.h"
#include "tim.h"

static PWM_Device *pwm_r = NULL;
static PWM_Device *pwm_g = NULL;
static PWM_Device *pwm_b = NULL;

void BSP_LED_Init(void)
{
    PWM_Init_Config config = {.Channel = TIM_CHANNEL_3, .dutyx10 = 0, .htim = &htim5, .Mode = PWM_MODE_BLOCKING};
    pwm_r                  = BSP_PWM_Device_Init(&config);
    config.Channel         = TIM_CHANNEL_2;
    pwm_g                  = BSP_PWM_Device_Init(&config);
    config.Channel         = TIM_CHANNEL_1;
    pwm_b                  = BSP_PWM_Device_Init(&config);

    if (pwm_r == NULL || pwm_g == NULL || pwm_b == NULL)
    {
        LOG_E("RGB init failed");
    }
    else
    {
        BSP_PWM_Start(pwm_r, NULL, 0);
        BSP_PWM_Start(pwm_g, NULL, 0);
        BSP_PWM_Start(pwm_b, NULL, 0);
        LOG_I("BSP LED init success");
    }
}

void BSP_LED_Show(uint32_t aRGB)
{
    if (!pwm_r || !pwm_g || !pwm_b)
    {
        return;
    }

    static uint8_t alpha;
    static uint8_t red, green, blue;

    // 提取ARGB值
    alpha = (aRGB & 0xFF000000) >> 24;
    red   = (aRGB & 0x00FF0000) >> 16;
    green = (aRGB & 0x0000FF00) >> 8;
    blue  = (aRGB & 0x000000FF) >> 0;

    // 应用alpha通道
    if (alpha != 0xFF)
    {
        red   = (red * alpha) / 255;
        green = (green * alpha) / 255;
        blue  = (blue * alpha) / 255;
    }

    // 将0-255的值转换为0-1000的占空比
    uint16_t red_duty   = (red * 1000) / 255;
    uint16_t green_duty = (green * 1000) / 255;
    uint16_t blue_duty  = (blue * 1000) / 255;

    // 设置PWM占空比
    BSP_PWM_SetDutyCycle(pwm_r, red_duty);
    BSP_PWM_SetDutyCycle(pwm_g, green_duty);
    BSP_PWM_SetDutyCycle(pwm_b, blue_duty);
}
#elif defined(STM32F105xC) || defined(STM32F103xB)

#include "gpio.h"

void BSP_LED_Init(void)
{
    LOG_I("BSP LED init (GPIO stub)");
}

void BSP_LED_Show(uint32_t aRGB)
{
    (void)aRGB;
}
#endif
