# m2006test 兵种 — M2006 拨盘电机测试
# 用途: 验证 2006 电机与过弹盘机械结构 (上电即转, 连发转速 -8000 RPM)
# 最小模块集: OFFLINE + MOTOR

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关(按板型覆盖默认值)
set(MODULES_SINGLE   OFFLINE MOTOR)

# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)    # 测试兵种不蜂鸣
