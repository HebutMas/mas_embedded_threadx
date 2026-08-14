# 模块模板 (modules/template)

本目录为新增模块的 **复制模板**。按照以下步骤即可快速创建一个新模块。

---

## 快速开始

### 1. 复制模板

```bash
cd modules
cp -r template/ <MODULE_NAME>/   # 例如: cp -r template/ VL53L0X/
```

> 目录名使用 **全大写**。

### 2. 全局替换

将目录内所有文件中的 `TEMPLATE` / `template` / `Template` 替换为你的模块名：

| 原值 | 替换为 | 示例 |
|------|--------|------|
| `TEMPLATE` | `<MODULE_NAME>` | `VL53L0X` |
| `template` | `<module_name>` | `vl53l0x` |
| `Template` | `<Module_Name>` | `VL53L0X` |
| `module_template.h` | `module_<name>.h` | `module_vl53l0x.h` |

文件本身也要重命名：
```bash
mv module_template.h module_vl53l0x.h
mv module_template.c module_vl53l0x.c
```

### 3. 修改 CMakeLists.txt

在 `modules/CMakeLists.txt` 中添加条件编译块：

```cmake
if(MODULE_VL53L0X)
    list(APPEND _module_sources
        VL53L0X/module_vl53l0x.c
    )
    list(APPEND _module_includes
        ${CMAKE_CURRENT_SOURCE_DIR}/VL53L0X
    )
endif()
```

### 4. 修改 module_init.c / module_init.h

在 `modules/module_init.c` 中增加：

```c
// 头文件区
#if MODULE_VL53L0X
#include "module_vl53l0x.h"
#endif

// MODULE_Init() 函数内
#if MODULE_VL53L0X
    Module_VL53L0X_Init();
#endif
```

### 5. 修改 module_config.cmake

在 `modules/module_config.cmake` 中添加默认参数：

```cmake
# 在 MODULES_SINGLE / MODULES_GIMBAL / MODULES_CHASSIS 列表中加入模块名
set(MODULES_SINGLE ... VL53L0X)

# 默认参数
set(VL53L0X_TASK_STACK_SIZE 512)
set(VL53L0X_TASK_PRIORITY   10)
set(VL53L0X_TASK_INTERVAL_MS 20)
```

### 6. 修改 apps/module_config.h.in

在 `apps/module_config.h.in` 模板中添加该模块的参数宏导出：

```c
/* VL53L0X 参数 */
#define VL53L0X_TASK_STACK_SIZE @VL53L0X_TASK_STACK_SIZE@
#define VL53L0X_TASK_PRIORITY   @VL53L0X_TASK_PRIORITY@
```

> 若参数为可选硬件句柄（如 `VL53L0X_UART`），用 CMake 原生的 `#cmakedefine`：
> ```c
> #cmakedefine VL53L0X_UART @VL53L0X_UART@
> ```
> 变量未定义或为空时输出 `/* #undef VL53L0X_UART */`，否则输出 `#define`。

### 7. 在 robot.cmake 中使能/覆盖

在需要该模块的机器人配置中：

```cmake
set(MODULES_SINGLE ... VL53L0X)          # 加入模块列表
set(VL53L0X_TASK_STACK_SIZE 1024)        # (可选) 覆盖默认参数
```

---

## 模板文件说明

| 文件 | 说明 |
|------|------|
| `module_template.h` | 对外接口头文件，包含默认参数宏和数据结构 |
| `module_template.c` | 模块实现，包含 ThreadX 任务创建、硬件初始化示例 |
| `README.md` | 本说明文件，复制后可直接删除或替换为模块文档 |

---

## 命名规范速查

| 项目 | 规范 | 示例 |
|------|------|------|
| 目录名 | 全大写 | `VL53L0X/` |
| 文件名 | `module_<小写>.h/.c` | `module_vl53l0x.c` |
| CMake 宏 | `MODULE_` 前缀 | `MODULE_VL53L0X` |
| Init 函数 | `Module_<CamelCase>_Init()` | `Module_VL53L0X_Init()` |
| 参数宏 | `<MODULE>_<PARAM>` | `VL53L0X_TASK_STACK_SIZE` |
| 默认参数 | `#ifndef` 包裹在 `.h` 中 | 允许 `module_config.h` 提前覆盖 |

---

## 详细文档

更多细节请参考 `modules/MODULES.MD`。
