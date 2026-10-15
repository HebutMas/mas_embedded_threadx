# 先加载默认模板，再覆盖差异

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关（按板型覆盖默认值）
set(MODULES_GIMBAL   OFFLINE REMOTE BMI088 INS WT606 MOTOR VISION NAV BOARDCOMM)
set(MODULES_CHASSIS  OFFLINE BMI088 INS REFEREE MOTOR BOARDCOMM)

# 6800 rpm摩擦轮实测双峰约114/115 Hz；仅云台板启用。
if(BOARD STREQUAL "gimbal")
    set(INS_NOTCH_FREQ_HZ 115)
endif()


# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)    # 开启蜂鸣器

# REMOTE 参数
set(REMOTE_UART             huart3) # DJI C 串口
set(REMOTE_VT_UART          huart6) # 图传串口
set(REMOTE_SOURCE           1)      # 遥控器选择: 0=none, 1=sbus, 2=dt7
set(REMOTE_VT_SOURCE        0)      # 图传选择:   0=none, 1=vt02, 2=vt03

# REFEREE 参数
set(REFEREE_UART            huart6) # 串口选择

# WT606 参数
set(WT606_UART              huart1) # DJI C 串口

# 达妙 H7 的实际接线
if(CMAKE_SOURCE_DIR MATCHES "/damiao_h7$")
    set(REMOTE_UART         huart5)
    set(WT606_UART          huart10)
endif()

# BOARDCOMM 参数
set(BOARDCOMM_CAN           BSP_CAN_HANDLE2) # CAN 句柄
