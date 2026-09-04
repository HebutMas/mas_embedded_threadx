# 模块框架文档

## 概述

模块层管理所有功能模块（传感器、电机、算法等），通过编译期开关按机器人类型选择性编译。核心思路：**每个模块一个目录，CMake + 预处理宏双重控制**。

## 架构

```
modules/
├── module_init.c               ← 入口: #if MODULE_XXX 条件初始化
├── module_init.h
├── module_config.cmake         ← 模块默认配置模板 (cmake)
├── CMakeLists.txt              ← 条件编译源文件
├── algorithm/                  ← 基础算法 (始终编译)
├── OFFLINE/                    ← 离线检测模块
│   ├── module_offline.h
│   └── module_offline.c
├── REMOTE/                     ← 遥控器模块
├── BMI088/                     ← IMU 模块
└── ...                         ← 其他模块

apps/<robot>/
└── robot.cmake                 ← 机器人配置: include module_config.cmake 并覆盖差异
```

### 编译文件控制

| 层级 | 位置 | 作用 |
|------|------|------|
| CMake | `modules/CMakeLists.txt` | 决定哪些 `.c` 被编译 |
| C 预处理 | `module_init.c` | 决定哪些模块被初始化 |

机器人 robot.cmake → include modules/module_config.cmake (默认) → 覆盖差异 → CMake 解析 → `generate_headers.cmake` 生成 module_config.h → 条件编译源文件 + 条件初始化，未使能的模块完全不参与编译。

## 如何添加一个新模块

以添加一个 **"激光测距 VL53L0X"** 模块为例：

### 步骤 1：创建模块目录和文件

```
modules/VL53L0X/
├── module_vl53l0x.h
└── module_vl53l0x.c
```

**`module_vl53l0x.h`**（对外接口 + 默认参数）：

```c
#ifndef _MODULE_VL53L0X_H_
#define _MODULE_VL53L0X_H_

#include <stdint.h>

/* 可被子类 module_config.h 覆盖的默认参数 */
#ifndef VL53L0X_TASK_STACK_SIZE
#define VL53L0X_TASK_STACK_SIZE 512
#endif
#ifndef VL53L0X_TASK_PRIORITY
#define VL53L0X_TASK_PRIORITY   10
#endif

/* 对外函数 */
int  Module_VL53L0X_Init(void);
float Module_VL53L0X_GetDistance(void);

#endif
```

> 参数用 `#ifndef ... #define` 包裹，允许 `module_config.h` 提前定义覆盖。

**`module_vl53l0x.c`**（模块实现）：

```c
#include "module_vl53l0x.h"
#include "bsp_i2c.h"
#include "tx_api.h"

#define LOG_TAG "VL53L0X"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD  g_thread;
static uint8_t    g_stack[VL53L0X_TASK_STACK_SIZE];
static float      g_distance;

static void task_entry(ULONG arg)
{
    (void)arg;
    while (1) {
        // 读取传感器...
        tx_thread_sleep(MS_TO_TICKS(20));
    }
}

int Module_VL53L0X_Init(void)
{
    // 硬件初始化...
    tx_thread_create(&g_thread, "vl53l0x", task_entry, 0,
                     g_stack, VL53L0X_TASK_STACK_SIZE,
                     VL53L0X_TASK_PRIORITY, VL53L0X_TASK_PRIORITY,
                     TX_NO_TIME_SLICE, TX_AUTO_START);
    LOG_I("VL53L0X initialized");
    return 0;
}

float Module_VL53L0X_GetDistance(void) { return g_distance; }
```

### 步骤 2：在 CMakeLists.txt 添加条件编译

```modules/CMakeLists.txt
# --- 在文件末尾、add_library 之前添加 ---

if(MODULE_VL53L0X)
    list(APPEND _module_sources
        VL53L0X/module_vl53l0x.c
    )
    list(APPEND _module_includes
        ${CMAKE_CURRENT_SOURCE_DIR}/VL53L0X
    )
endif()
```

### 步骤 3：在 module_init.c 添加条件初始化

```c
// 头文件区
#if MODULE_VL53L0X
#include "module_vl53l0x.h"
#endif

// MODULE_Init() 函数内
void MODULE_Init(void)
{
    // ... 已有模块 ...
#if MODULE_VL53L0X
    Module_VL53L0X_Init();
#endif
    LOG_I("Modules init finished");
}
```

### 步骤 4：在 module_config.cmake 添加默认配置

在 `modules/module_config.cmake` 中添加默认模块列表和参数：

```cmake
# 在 MODULES_SINGLE / MODULES_GIMBAL / MODULES_CHASSIS 列表中添加模块名
set(MODULES_SINGLE   ... VL53L0X)   # 按需添加到对应板型

# 添加模块默认参数
set(VL53L0X_TASK_STACK_SIZE 512)
set(VL53L0X_TASK_PRIORITY   10)
```

### 步骤 5：在 module_config.h.in 模板添加参数宏导出

在 `apps/module_config.h.in` 模板中，追加该模块的参数宏（构建时由 `cmake/board_common.cmake` 的 `configure_file(... @ONLY)` 自动生成 `module_config.h`）：

```c
/* VL53L0X 参数 */
#define VL53L0X_TASK_STACK_SIZE @VL53L0X_TASK_STACK_SIZE@
#define VL53L0X_TASK_PRIORITY   @VL53L0X_TASK_PRIORITY@
```

> 若参数为可选硬件句柄（如 `VL53L0X_I2C`），使用 `#cmakedefine`：未设置时该行不生成，
> 模块代码中可用 `#ifdef VL53L0X_I2C` 判断。
> ```c
> #cmakedefine VL53L0X_I2C @VL53L0X_I2C@
> ```

### 步骤 6：在机器人配置中使能 / 覆盖

在需要该模块的机器人 `apps/<robot>/robot.cmake` 中：

```cmake
# 如需覆盖模块列表：
set(MODULES_SINGLE   OFFLINE REMOTE BMI088 INS REFEREE MOTOR VL53L0X)

# 如需覆盖参数：
set(VL53L0X_TASK_STACK_SIZE 1024)
```

不需要此模块的机器人只需不将其加入 `MODULES_<BOARD>` 列表即可。

## 总结

| # | 文件 | 改动 |
|---|------|------|
| 1 | `modules/<NAME>/module_<name>.h` + `.c` | 新建模块代码 |
| 2 | `modules/CMakeLists.txt` | 加 `if(MODULE_XXX)` 块 |
| 3 | `modules/module_init.c` | 加 `#include` + `Init()` 调用 |
| 4 | `modules/module_config.cmake` | 添加默认列表和参数 |
| 5 | `apps/module_config.h.in` | 添加该模块的参数宏导出（普通宏用 `#define VAR @VAR@`，可选硬件句柄用 `#cmakedefine`） |
| 6 | `apps/<robot>/robot.cmake` | 覆盖模块列表 / 参数 |

## 模块命名规范

| 项目 | 规范 | 示例 |
|------|------|------|
| 目录名 | 大写 | `VL53L0X/` |
| 文件名 | `module_<小写>.h/.c` | `module_vl53l0x.c` |
| 宏前缀 | `MODULE_` | `MODULE_VL53L0X` |
| Init 函数 | `Module_<CamelCase>_Init()` | `Module_VL53L0X_Init()` |
| 参数宏 | `<MODULE>_<PARAM>` | `VL53L0X_TASK_STACK_SIZE` |
| 参数默认值 | `#ifndef` 包裹在 `.h` 中 | 见步骤 1 |

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-11 | 初始版本 |
