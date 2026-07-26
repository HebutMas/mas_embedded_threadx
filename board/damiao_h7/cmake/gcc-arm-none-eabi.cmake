# damiao_h7 (STM32H723 / Cortex-M7) MCU 差异参数
set(MCU_TARGET_FLAGS  "-mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard")
set(MCU_LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../STM32H723XG_FLASH.ld")
# 公共工具链逻辑
include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/gcc-arm-none-eabi-common.cmake)
