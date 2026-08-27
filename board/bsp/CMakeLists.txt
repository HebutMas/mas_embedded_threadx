# bsp library
# 外设编译由 BSP_*_ENABLE 变量控制，board_common.cmake 设默认值，板级可覆盖

set(_bsp_sources
    bsp_init.c
)

set(_bsp_links
    stm32cubemx
    azrtos::threadx
    utils
)

if(BSP_DWT_ENABLE)
    list(APPEND _bsp_sources DWT/bsp_dwt.c)
endif()
if(BSP_UART_ENABLE)
    list(APPEND _bsp_sources UART/bsp_uart.c)
endif()
if(BSP_SPI_ENABLE)
    list(APPEND _bsp_sources SPI/bsp_spi.c)
endif()
if(BSP_I2C_ENABLE)
    list(APPEND _bsp_sources I2C/bsp_i2c.c)
endif()
if(BSP_FLASH_ENABLE)
    list(APPEND _bsp_sources FLASH/bsp_flash.c)
endif()
if(BSP_GPIO_ENABLE)
    list(APPEND _bsp_sources GPIO/bsp_gpio.c)
endif()
if(BSP_PWM_ENABLE)
    list(APPEND _bsp_sources PWM/bsp_pwm.c)
endif()
if(BSP_USB_ENABLE)
    list(APPEND _bsp_sources USB/usbd_cdc_acm_user.c)
    list(APPEND _bsp_links cherryusb)
endif()
if(BSP_LED_ENABLE)
    list(APPEND _bsp_sources LED/bsp_led.c)
endif()
if(BSP_BEEP_ENABLE)
    list(APPEND _bsp_sources BEEP/bsp_beep.c)
endif()
if(BSP_CAN_ENABLE)
    list(APPEND _bsp_sources CAN/bsp_can.c CAN/bsp_can_task.c)
endif()

add_library(bsp OBJECT ${_bsp_sources})

target_compile_options(bsp PRIVATE -O2)

# 将 BSP_*_ENABLE 变量传递给 C 代码，供 bsp_init.c 条件编译
foreach(_bsp_mod DWT UART SPI I2C FLASH GPIO PWM LED BEEP CAN USB)
    if(BSP_${_bsp_mod}_ENABLE)
        target_compile_definitions(bsp PRIVATE BSP_${_bsp_mod}_ENABLE=1)
    endif()
endforeach()

target_include_directories(bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/DWT
    ${CMAKE_CURRENT_SOURCE_DIR}/UART
    ${CMAKE_CURRENT_SOURCE_DIR}/SPI
    ${CMAKE_CURRENT_SOURCE_DIR}/I2C
    ${CMAKE_CURRENT_SOURCE_DIR}/FLASH
    ${CMAKE_CURRENT_SOURCE_DIR}/GPIO
    ${CMAKE_CURRENT_SOURCE_DIR}/PWM
    ${CMAKE_CURRENT_SOURCE_DIR}/USB
    ${CMAKE_CURRENT_SOURCE_DIR}/LED
    ${CMAKE_CURRENT_SOURCE_DIR}/BEEP
    ${CMAKE_CURRENT_SOURCE_DIR}/CAN
)

target_link_libraries(bsp PUBLIC ${_bsp_links})