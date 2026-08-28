cmake_minimum_required(VERSION 3.22)
project(base)

# MCU flags / 链接脚本见本目录 cmake/gcc-arm-none-eabi.cmake

# 本板差异
# ThreadX configuration for Cortex-M3 / GCC
set(THREADX_ARCH cortex_m3)
# F103C8 无 USB OTG 硬件，关闭 BSP USB 外设，DWC2 端口自动联动关闭
set(BSP_USB_ENABLE OFF)
# 公共构建逻辑
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/board_common.cmake)

# 板级专属 sources / includes / defines / libraries 在此追加
# target_sources(${CMAKE_PROJECT_NAME} PRIVATE ...)