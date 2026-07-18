# 先加载默认模板，再覆盖差异

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关（按板型覆盖默认值）
set(MODULES_SINGLE   OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP VISION MOTOR)


# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)    # 开启蜂鸣器

# REMOTE 参数
set(REMOTE_UART             huart3) # 串口
set(REMOTE_VT_UART          huart6) # 图传串口
set(REMOTE_SOURCE           0)      # 遥控器选择: 0=none, 1=sbus, 2=dt7
set(REMOTE_VT_SOURCE        1)      # 图传选择:   0=none, 1=vt02, 2=vt03

# REFEREE 参数
set(REFEREE_UART            huart6) # 串口选择