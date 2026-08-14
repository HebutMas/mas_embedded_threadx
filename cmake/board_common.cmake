# 板级公共构建逻辑
# 各板 CMakeLists.txt 只需 project(base) + set(THREADX_ARCH ...) 后 include 本文件；
# MCU flags / 链接脚本差异在各板 cmake/gcc-arm-none-eabi.cmake。
# 本文件中相对路径（如 cmake/stm32cubemx）均相对当前板目录解析。

# 仓库根目录（本文件位于 <root>/cmake/）
get_filename_component(MAS_ROOT ${CMAKE_CURRENT_LIST_DIR}/.. ABSOLUTE)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# 加载配置（ROBOT/BOARD 缓存变量在 apps/config.cmake 中定义）
include(${MAS_ROOT}/apps/config.cmake)

# 生成头文件到 build 目录（configure_file 仅内容变化时更新）
set(_generated_dir ${CMAKE_CURRENT_BINARY_DIR}/generated)
configure_file(${MAS_ROOT}/apps/robot_def.h.in      ${_generated_dir}/robot_def.h      @ONLY)
configure_file(${MAS_ROOT}/apps/module_config.h.in  ${_generated_dir}/module_config.h  @ONLY)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add ThreadX RTOS
add_subdirectory(${MAS_ROOT}/threadx threadx)

# Add shared project libraries
add_subdirectory(${MAS_ROOT}/utils     ${CMAKE_CURRENT_BINARY_DIR}/utils)
add_subdirectory(${MAS_ROOT}/board/bsp ${CMAKE_CURRENT_BINARY_DIR}/bsp)
add_subdirectory(${MAS_ROOT}/robot     ${CMAKE_CURRENT_BINARY_DIR}/robot)
add_subdirectory(${MAS_ROOT}/modules   ${CMAKE_CURRENT_BINARY_DIR}/modules)
add_subdirectory(${MAS_ROOT}/apps      ${CMAKE_CURRENT_BINARY_DIR}/apps)

# CherryUSB Device CDC ACM
set(CONFIG_CHERRYUSB_DEVICE ON CACHE BOOL "Enable CherryUSB device stack" FORCE)
set(CONFIG_CHERRYUSB_DEVICE_CDC_ACM ON CACHE BOOL "Enable CDC ACM class" FORCE)
set(CONFIG_CHERRYUSB_DEVICE_DWC2_ST ON CACHE BOOL "Use DWC2 OTG with STM32 glue" FORCE)
set(CONFIG_CHERRYUSB_OSAL "threadx" CACHE STRING "Use ThreadX OS abstraction layer" FORCE)

include(${MAS_ROOT}/CherryUSB/cherryusb.cmake)
list(REMOVE_DUPLICATES cherryusb_srcs)
list(REMOVE_DUPLICATES cherryusb_incs)

add_library(cherryusb STATIC ${cherryusb_srcs})
target_include_directories(cherryusb PUBLIC
    ${MAS_ROOT}/board/bsp/USB
    ${cherryusb_incs}
)
target_compile_options(cherryusb PRIVATE -O3 -ffast-math -fno-math-errno)
target_link_libraries(cherryusb PUBLIC stm32cubemx azrtos::threadx utils)

# CMSIS-DSP settings (M4/M7 通用)
set(LOOPUNROLL ON CACHE BOOL "Loop unrolling for max performance" FORCE)
set(DISABLEFLOAT16 ON CACHE BOOL "Disable float16 kernels (not needed on M4/M7)" FORCE)
add_subdirectory(${MAS_ROOT}/CMSIS-DSP ${CMAKE_CURRENT_BINARY_DIR}/cmsisdsp)
# Apply max performance compiler flags to CMSIS-DSP
target_compile_options(CMSISDSP PRIVATE -O3 -ffast-math -fno-math-errno -flto)
# Provide CMSIS-Core headers (cmsis_compiler.h) to CMSIS-DSP（板目录下的 Drivers）
target_include_directories(CMSISDSP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${_generated_dir}
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    azrtos::threadx
    utils
    bsp
    robot
    CMSISDSP
    cherryusb
    modules
    app
)

# Enable LTO at link stage (required for -flto compiled objects)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -flto)
