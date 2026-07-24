# dji_c (STM32F407 / Cortex-M4) MCU 差异参数
set(MCU_TARGET_FLAGS  "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(MCU_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../STM32F407XX_FLASH.ld")
# 公共工具链逻辑
include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/gcc-arm-none-eabi-common.cmake)
