# test 兵种 — VOFA 链路测试
# 最小模块集: OFFLINE(掉线检测) + VOFA(协议收发)
# 板型: single

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关(按板型覆盖默认值)
set(MODULES_SINGLE   OFFLINE VOFA)

# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)    # 测试兵种不蜂鸣

# VOFA 参数
set(VOFA_UART               huart6)      # VOFA 走串口6
set(VOFA_FORMAT             0)           # 0 = VOFA_FORMAT_JUSTFLOAT (二进制, 帧尾 00 00 80 7f)
set(VOFA_TASK_STACK_SIZE    1024)
set(VOFA_TASK_PRIORITY      11)
