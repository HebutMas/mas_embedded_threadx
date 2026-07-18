# 根据 CMake 变量生成 robot_def.h 和 module_config.h

if(NOT DEFINED _generated_dir)
    message(FATAL_ERROR "_generated_dir must be defined before including generate_headers.cmake")
endif()

# robot_def.h 
file(WRITE ${_generated_dir}/robot_def.h
"#ifndef _ROBOT_DEF_H_
#define _ROBOT_DEF_H_

/* 机器人类型定义 */
#define ROBOT_HERO      1
#define ROBOT_ENGINEER  2
#define ROBOT_INFANTRY3 3
#define ROBOT_INFANTRY4 4
#define ROBOT_INFANTRY5 5
#define ROBOT_DRONE     6
#define ROBOT_SENTRY    7
#define ROBOT_DARTS     8

/* 当前编译的机器人 — 由 CMake 根据 ROBOT 选项自动生成 */
#define CURRENT_ROBOT   ROBOT_${ROBOT_UPPER}

#define ROBOT_TYPE      CURRENT_ROBOT

#endif // _ROBOT_DEF_H_
")

# helper: 模拟 #cmakedefine 行为
# 若变量已定义且不为 FALSE/OFF/0/N/IGNORE/NOTFOUND/xxx-NOTFOUND/空字符串，则输出 #define，否则输出 /* #undef */
function(_gen_cmakedefine OUT_VAR NAME VALUE_VAR)
    if(DEFINED ${VALUE_VAR}
       AND NOT "${${VALUE_VAR}}" STREQUAL ""
       AND NOT "${${VALUE_VAR}}" MATCHES "^(FALSE|OFF|0|N|IGNORE|NOTFOUND|.*-NOTFOUND)$")
        set(${OUT_VAR} "#define ${NAME} ${${VALUE_VAR}}" PARENT_SCOPE)
    else()
        set(${OUT_VAR} "/* #undef ${NAME} */" PARENT_SCOPE)
    endif()
endfunction()

# 生成 #cmakedefine 行
_gen_cmakedefine(_REMOTE_UART        REMOTE_UART        REMOTE_UART)
_gen_cmakedefine(_REMOTE_VT_UART     REMOTE_VT_UART     REMOTE_VT_UART)
_gen_cmakedefine(_REFEREE_UART       REFEREE_UART       REFEREE_UART)
_gen_cmakedefine(_SUPERCAP_CAN       SUPERCAP_CAN       SUPERCAP_CAN)
_gen_cmakedefine(_WT606_UART         WT606_UART         WT606_UART)
_gen_cmakedefine(_BOARDCOMM_CAN      BOARDCOMM_CAN      BOARDCOMM_CAN)

file(WRITE ${_generated_dir}/module_config.h
"#ifndef _MODULE_CONFIG_H_
#define _MODULE_CONFIG_H_

/* 
 * 本文件由 CMake 根据 apps/<robot>/robot.cmake 自动生成
 * 请勿手动修改，修改请去对应 robot.cmake
 */

/* 板型(只有一个为 1) */
#define SINGLE_BOARD  ${SINGLE_BOARD}
#define GIMBAL_BOARD  ${GIMBAL_BOARD}
#define CHASSIS_BOARD ${CHASSIS_BOARD}

/* 模块开关(1=使能 0=禁用) */
#define MODULE_OFFLINE   ${MODULE_OFFLINE}
#define MODULE_REMOTE    ${MODULE_REMOTE}
#define MODULE_BMI088    ${MODULE_BMI088}
#define MODULE_INS       ${MODULE_INS}
#define MODULE_REFEREE   ${MODULE_REFEREE}
#define MODULE_SUPERCAP  ${MODULE_SUPERCAP}
#define MODULE_WT606     ${MODULE_WT606}
#define MODULE_MOTOR     ${MODULE_MOTOR}
#define MODULE_VISION    ${MODULE_VISION}
#define MODULE_BOARDCOMM ${MODULE_BOARDCOMM}

/* 参数配置 */

/* OFFLINE 参数 */
#define OFFLINE_WATCHDOG_ENABLE ${OFFLINE_WATCHDOG_ENABLE}
#define OFFLINE_BEEP_ENABLE     ${OFFLINE_BEEP_ENABLE}
#define OFFLINE_TASK_STACK_SIZE ${OFFLINE_TASK_STACK_SIZE}
#define OFFLINE_TASK_PRIORITY   ${OFFLINE_TASK_PRIORITY}

/* REMOTE 参数 */
${_REMOTE_UART}
${_REMOTE_VT_UART}
#define REMOTE_SOURCE           ${REMOTE_SOURCE}
#define REMOTE_VT_SOURCE        ${REMOTE_VT_SOURCE}
#define REMOTE_DEAD_ZONE        ${REMOTE_DEAD_ZONE}
#define REMOTE_TASK_STACK_SIZE  ${REMOTE_TASK_STACK_SIZE}
#define REMOTE_TASK_PRIORITY    ${REMOTE_TASK_PRIORITY}
#define REMOTE_TASK_VT_PRIORITY ${REMOTE_TASK_VT_PRIORITY}

/* BMI088 参数 */
#define BMI088_TEMP_ENABLE      ${BMI088_TEMP_ENABLE}
#define BMI088_TEMP_SET         ${BMI088_TEMP_SET}f

/* INS 参数 */
#define INS_TASK_STACK_SIZE     ${INS_TASK_STACK_SIZE}
#define INS_TASK_PRIORITY       ${INS_TASK_PRIORITY}

/* REFEREE 参数 */
${_REFEREE_UART}
#define REFEREE_TASK_STACK_SIZE ${REFEREE_TASK_STACK_SIZE}
#define REFEREE_TASK_PRIORITY   ${REFEREE_TASK_PRIORITY}

/* SuperCap 参数 */
${_SUPERCAP_CAN}

/* WT606 参数 */
${_WT606_UART}
#define WT606_TASK_STACK_SIZE   ${WT606_TASK_STACK_SIZE}
#define WT606_TASK_PRIORITY     ${WT606_TASK_PRIORITY}

/* MOTOR 参数 */
#define MOTOR_TASK_STACK_SIZE   ${MOTOR_TASK_STACK_SIZE}
#define MOTOR_TASK_PRIORITY     ${MOTOR_TASK_PRIORITY}

/* BOARDCOMM 参数 */
${_BOARDCOMM_CAN}

/* VISION 参数 */
#define VISION_TASK_STACK_SIZE   ${VISION_TASK_STACK_SIZE}
#define VISION_TASK_PRIORITY     ${VISION_TASK_PRIORITY}

#endif /* _MODULE_CONFIG_H_ */
")
