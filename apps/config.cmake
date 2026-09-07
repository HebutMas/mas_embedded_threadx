# 机器人 & 板型 配置
# 在这里选择机器人和板型；配置文件是构建配置的唯一来源

# 目标机器人 & 板型
set(ROBOT "test") # hero / engineer / infantry3 / infantry4 / infantry5 / drone / sentry / darts / customcontrol
set(BOARD "single") # single / gimbal / chassis; sentry has no single_board

# 板型校验
if(NOT BOARD MATCHES "^(single|gimbal|chassis)$")
    message(FATAL_ERROR "Unknown BOARD '${BOARD}', expected: single / gimbal / chassis")
endif()

# 加载默认模块配置
include(${CMAKE_CURRENT_LIST_DIR}/../modules/module_config.cmake)

# 加载机器人差异配置
include(${CMAKE_CURRENT_LIST_DIR}/${ROBOT}/robot.cmake)

# 派生变量
string(TOUPPER ${ROBOT} ROBOT_UPPER)
string(TOUPPER ${BOARD} BOARD_UPPER)

message(STATUS "Robot: ${ROBOT}")
message(STATUS "Board: ${BOARD}")

# 板型宏（只有一个为 1）
set(SINGLE_BOARD  0)
set(GIMBAL_BOARD  0)
set(CHASSIS_BOARD 0)
set(${BOARD_UPPER}_BOARD 1)

# 模块开关
foreach(_m OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP WT606 MOTOR VISION BOARDCOMM LORA VOFA)
    set(MODULE_${_m} 0)
endforeach()

set(_enabled ${MODULES_${BOARD_UPPER}})
foreach(_m ${_enabled})
    set(MODULE_${_m} 1)
endforeach()
