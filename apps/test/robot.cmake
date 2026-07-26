# F103C8T6 板级配置
# BMI088/INS 硬件上不存在（C板专用）
# VISION 使用 CherryUSB DWC2，F103 无 OTG
# CMSIS-DSP 无 FPU + Flash/RAM 限制

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关（仅保留 F103 硬件支持的模块）
set(MODULES_SINGLE   OFFLINE REMOTE MOTOR)
set(MODULES_GIMBAL   OFFLINE REMOTE REFEREE MOTOR BOARDCOMM)
set(MODULES_CHASSIS  OFFLINE REFEREE MOTOR BOARDCOMM)

# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)    # 开启蜂鸣器

# REMOTE 参数
set(REMOTE_UART             huart1) # SBUS 串口 (F105 UART4)
set(REMOTE_VT_UART          huart2) # 图传串口
set(REMOTE_SOURCE           1)      # 遥控器选择: 0=none, 1=sbus, 2=dt7
set(REMOTE_VT_SOURCE        0)      # 图传选择:   0=none, 1=vt02, 2=vt03

# (REFEREE 和 WT606 待添加 UART 后再启用)

# SUPERCAP 参数
set(SUPERCAP_CAN            BSP_CAN_HANDLE2)

# BOARDCOMM 参数
set(BOARDCOMM_CAN           BSP_CAN_HANDLE2)



