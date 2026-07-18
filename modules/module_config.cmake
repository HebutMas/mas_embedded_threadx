# 模块默认配置模板
# 各 apps/<robot>/robot.cmake 应先 include 本文件，再覆盖差异项。
# 覆盖方式：直接 set(变量名 新值) 即可，无需前缀。

# 可用模块列表 OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP WT606 MOTOR BOARDCOMM VISION

# 默认模块列表
set(MODULES_SINGLE   OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP MOTOR)
set(MODULES_GIMBAL   OFFLINE REMOTE BMI088 INS MOTOR VISION BOARDCOMM)
set(MODULES_CHASSIS  OFFLINE BMI088 INS REFEREE SUPERCAP MOTOR BOARDCOMM)

# OFFLINE 默认参数
set(OFFLINE_WATCHDOG_ENABLE 1)    # 开启看门狗
set(OFFLINE_BEEP_ENABLE     1)    # 开启蜂鸣器
set(OFFLINE_TASK_STACK_SIZE 1024) # 任务栈大小
set(OFFLINE_TASK_PRIORITY   6)    # 任务优先级

# REMOTE 默认参数
set(REMOTE_UART             huart3) # 串口
set(REMOTE_VT_UART          huart1) # 图传串口
set(REMOTE_SOURCE           1)      # 遥控器选择: 0=none, 1=sbus, 2=dt7
set(REMOTE_VT_SOURCE        1)      # 图传选择:   0=none, 1=vt02, 2=vt03
set(REMOTE_DEAD_ZONE        10)     # 死区
set(REMOTE_TASK_STACK_SIZE  1024)   # 任务栈大小
set(REMOTE_TASK_PRIORITY    9)      # 任务优先级
set(REMOTE_TASK_VT_PRIORITY 8)      # 图传串口任务优先级
set(REMOTE_OFFLINE_ENABLE      1)   # 遥控器离线检测
set(REMOTE_VT_OFFLINE_ENABLE   2)   # 图传离线检测

# BMI088 默认参数
set(BMI088_TEMP_ENABLE      0)      # 温度控制
set(BMI088_TEMP_SET         35.0)   # 温度设置

# INS 默认参数
set(INS_TASK_STACK_SIZE     1024)   # 任务栈大小
set(INS_TASK_PRIORITY       7)      # 任务优先级

# REFEREE 默认参数
set(REFEREE_UART            huart1) # 串口选择
set(REFEREE_TASK_STACK_SIZE 1024)   # 任务栈大小
set(REFEREE_TASK_PRIORITY   10)     # 任务优先级
set(REFEREE_OFFLINE_ENABLE  1)      # 离线检测开启

# SUPERCAP 默认参数
set(SUPERCAP_CAN            BSP_CAN_HANDLE2) # CAN 句柄
set(SUPERCAP_OFFLINE_ENABLE 1)      # 离线检测开启

# WT606 默认参数
set(WT606_UART              huart1) # 串口选择
set(WT606_TASK_STACK_SIZE   1024)   # 任务栈大小
set(WT606_TASK_PRIORITY     8)      # 任务优先级
set(WT606_OFFLINE_ENABLE    1)      # 离线检测开启

# MOTOR 默认参数
set(MOTOR_TASK_STACK_SIZE   1024)   # 任务栈大小 
set(MOTOR_TASK_PRIORITY     12)     # 任务优先级
set(MOTOR_OFFLINE_ENABLE    1)      # 电机离线检测默认值

# BOARDCOMM 默认参数
set(BOARDCOMM_CAN           BSP_CAN_HANDLE2) # CAN 句柄  
set(BOARDCOMM_OFFLINE_ENABLE 1)     # 离线检测开启

# VISION 默认参数
set(VISION_TASK_STACK_SIZE   1024)   # 任务栈大小
set(VISION_TASK_PRIORITY     10)     # 任务优先级
set(VISION_OFFLINE_ENABLE    1)      # 离线检测开启
