# 基础源文件 (始终编译)
set(_module_sources
    module_init.c
    algorithm/pid.c
    algorithm/user_lib.c
    algorithm/crc_rm.c
    algorithm/lqr.c
)

# 生成的头文件目录 (board binary dir/generated)
get_filename_component(_board_binary_dir ${CMAKE_CURRENT_BINARY_DIR} DIRECTORY)
set(_generated_dir ${_board_binary_dir}/generated)

set(_module_includes
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/algorithm
    ${_generated_dir}
)

# 条件源文件

if(MODULE_OFFLINE)
    list(APPEND _module_sources OFFLINE/module_offline.c)
    list(APPEND _module_includes ${CMAKE_CURRENT_SOURCE_DIR}/OFFLINE)
endif()

if(MODULE_REMOTE)
    list(APPEND _module_sources
        REMOTE/module_remote.c
        REMOTE/SBUS/sbus.c
        REMOTE/DT7/dt7.c
        REMOTE/VT02/vt02.c
        REMOTE/VT03/vt03.c
    )
    list(APPEND _module_includes
        ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE
        ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE/SBUS
        ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE/DT7
        ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE/VT02
        ${CMAKE_CURRENT_SOURCE_DIR}/REMOTE/VT03
    )
endif()

if(MODULE_BMI088)
    list(APPEND _module_sources
        BMI088/module_bmi088.c
        BMI088/module_bmi088_cali.c
    )
    list(APPEND _module_includes ${CMAKE_CURRENT_SOURCE_DIR}/BMI088)
endif()

if(MODULE_INS)
    list(APPEND _module_sources
        INS/module_ins.c
        INS/kalman_filter.c
        INS/QuaternionEKF.c
    )
    list(APPEND _module_includes ${CMAKE_CURRENT_SOURCE_DIR}/INS)
endif()

if(MODULE_REFEREE)
    list(APPEND _module_sources
        REFEREE/module_referee.c
        REFEREE/referee_protocol_ui.c
    )
    list(APPEND _module_includes ${CMAKE_CURRENT_SOURCE_DIR}/REFEREE)
endif()

if(MODULE_WT606)
    list(APPEND _module_sources
        WT606/module_wt606.c
    )
    list(APPEND _module_includes ${CMAKE_CURRENT_SOURCE_DIR}/WT606)
endif()

if(MODULE_SUPERCAP)
    list(APPEND _module_sources
        SuperCap/module_supercap.c
    )
    list(APPEND _module_includes ${CMAKE_CURRENT_SOURCE_DIR}/SuperCap)
endif()

if(MODULE_MOTOR)
    list(APPEND _module_sources
        MOTOR/module_motor.c
        MOTOR/motor_base.c
        MOTOR/DJI/motor_dji.c
        MOTOR/POWER_CONTROL/power_control.c
        MOTOR/DAMIAO/motor_damiao.c
        MOTOR/SERVO/motor_servo.c
        MOTOR/ZDT_STEPMOTOR/motor_zdt.c
    )
    # 底盘运动学：只有单板/底盘板才需要
    if(BOARD STREQUAL "single" OR BOARD STREQUAL "chassis")
        list(APPEND _module_sources algorithm/chassis_type.c)
    endif()
    list(APPEND _module_includes
        ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR
        ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR/DJI
        ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR/DAMIAO
        ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR/SERVO
        ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR/ZDT_STEPMOTOR
        ${CMAKE_CURRENT_SOURCE_DIR}/MOTOR/POWER_CONTROL
    )
endif()

if(MODULE_VISION)
    list(APPEND _module_sources
        VISION/module_vision.c
    )
    list(APPEND _module_includes ${CMAKE_CURRENT_SOURCE_DIR}/VISION)
endif()

if(MODULE_BOARDCOMM)
    list(APPEND _module_sources
        BOARDCOMM/module_boardcomm.c
    )
    list(APPEND _module_includes ${CMAKE_CURRENT_SOURCE_DIR}/BOARDCOMM)
endif()


# 创建库
add_library(modules OBJECT ${_module_sources})

# -include module_config.h: 保证 cmake 生成的宏在所有模块头文件 #ifndef 之前定义
target_compile_options(modules PRIVATE -O2 -include module_config.h)

target_include_directories(modules PUBLIC ${_module_includes})

target_link_libraries(modules PUBLIC stm32cubemx azrtos::threadx m CMSISDSP bsp utils cherryusb)
