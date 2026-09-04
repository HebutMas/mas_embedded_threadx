# test 兵种 — 新模块接入测试骨架
# 用途: 新模块开发时在此单独验证 (见 robot_control.c 顶部工作流说明)
# 最小模块集: OFFLINE (测具体模块时, 把对应模块加入 MODULES_SINGLE)

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关(按板型覆盖默认值)
set(MODULES_SINGLE   OFFLINE)

# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)    # 测试兵种不蜂鸣