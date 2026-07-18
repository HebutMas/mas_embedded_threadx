[![zread](https://img.shields.io/badge/Ask_Zread-_.svg?style=for-the-badge&color=00b0aa&labelColor=000000&logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB3aWR0aD0iMTYiIGhlaWdodD0iMTYiIHZpZXdCb3g9IjAgMCAxNiAxNiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTQuOTYxNTYgMS42MDAxSDIuMjQxNTZDMS44ODgxIDEuNjAwMSAxLjYwMTU2IDEuODg2NjQgMS42MDE1NiAyLjI0MDFWNC45NjAxQzEuNjAxNTYgNS4zMTM1NiAxLjg4ODEgNS42MDAxIDIuMjQxNTYgNS42MDAxSDQuOTYxNTZDNS4zMTUwMiA1LjYwMDEgNS42MDE1NiA1LjMxMzU2IDUuNjAxNTYgNC45NjAxVjIuMjQwMUM1LjYwMTU2IDEuODg2NjQgNS4zMTUwMiAxLjYwMDEgNC45NjE1NiAxLjYwMDFaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00Ljk2MTU2IDEwLjM5OTlIMi4yNDE1NkMxLjg4ODEgMTAuMzk5OSAxLjYwMTU2IDEwLjY4NjQgMS42MDE1NiAxMS4wMzk5VjEzLjc1OTlDMS42MDE1NiAxNC4xMTM0IDEuODg4MSAxNC4zOTk5IDIuMjQxNTYgMTQuMzk5OUg0Ljk2MTU2QzUuMzE1MDIgMTQuMzk5OSA1LjYwMTU2IDE0LjExMzQgNS42MDE1NiAxMy43NTk5VjExLjAzOTlDNS42MDE1NiAxMC42ODY0IDUuMzE1MDIgMTAuMzk5OSA0Ljk2MTU2IDEwLjM5OTlaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik0xMy43NTg0IDEuNjAwMUgxMS4wMzg0QzEwLjY4NSAxLjYwMDEgMTAuMzk4NCAxLjg4NjY0IDEwLjM5ODQgMi4yNDAxVjQuOTYwMUMxMC4zOTg0IDUuMzEzNTYgMTAuNjg1IDUuNjAwMSAxMS4wMzg0IDUuNjAwMUgxMy43NTg0QzE0LjExMTkgNS42MDAxIDE0LjM5ODQgNS4zMTM1NiAxNC4zOTg0IDQuOTYwMVYyLjI0MDFDMTQuMzk4NCAxLjg4NjY0IDE0LjExMTkgMS42MDAxIDEzLjc1ODQgMS42MDAxWiIgZmlsbD0iI2ZmZiIvPgo8cGF0aCBkPSJNNCAxMkwxMiA0TDQgMTJaIiBmaWxsPSIjZmZmIi8%2BCjxwYXRoIGQ9Ik00IDEyTDEyIDQiIHN0cm9rZT0iI2ZmZiIgc3Ryb2tlLXdpZHRoPSIxLjUiIHN0cm9rZS1saW5lY2FwPSJyb3VuZCIvPgo8L3N2Zz4K&logoColor=ffffff)](https://zread.ai/HebutMas/mas_embedded_threadx)

# mas_embedded_threadx

基于 **ThreadX RTOS** 的嵌入式机器人控制框架，面向多兵种、多板型的 RoboMaster 设计。

## 开发环境配置

[开发环境配置](docs/开发环境搭建.MD)

## 工程结构

```
mas_embedded_threadx/
│
├── apps/                          # 应用层：机器人业务代码
│   ├── config.cmake               # 全局构建配置（选择 ROBOT / BOARD）
│   ├── app_init.c / app_init.h    # 应用层入口
│   ├── generate_headers.cmake     # CMake 生成 robot_def.h / module_config.h 脚本
│   ├── templates/                 # 机器人配置模板
│   │   ├── robot.cmake            # 各机器人差异配置的参考模板
│   │   ├── single_board/          # 单板模板代码
│   │   ├── gimbal_board/          # 云台板模板代码
│   │   └── chassis_board/         # 底盘板模板代码
│   └──...                         # 各机器人文件夹 
│
├── board/                         # 板级支持包（BSP）
│   ├── bsp/                       # 通用 BSP 库（HAL 封装）
│   │   ├── bsp_init.c             # BSP 初始化入口
│   │   ├── CAN/                   # CAN 总线驱动 + 接收任务
│   │   ├── UART/                  # 串口驱动
│   │   ├── SPI/                   # SPI 驱动
│   │   ├── I2C/                   # I2C 驱动
│   │   ├── PWM/                   # PWM 输出
│   │   ├── GPIO/                  # GPIO 控制
│   │   ├── LED/                   # LED 指示灯
│   │   ├── BEEP/                  # 蜂鸣器
│   │   ├── FLASH/                 # Flash 读写
│   │   ├── DWT/                   # DWT 延时
│   │   └── USB/                   # USB CDC 用户层
│   ├── damiao_h7/                 # H7 开发板（Cortex-M7）
│   │   ├── CMakeLists.txt         
│   │   ├── Core/                  
│   │   ├── Drivers/               
│   │   └── cmake/stm32cubemx/     
│   └── dji_c/                     # DJI C 板（Cortex-M4）
│       ├── CMakeLists.txt         
│       ├── Core/                  
│       ├── Drivers/               
│       └── cmake/stm32cubemx/     
│
├── modules/                       # 模块层：功能模块
│   ├── module_init.c / .h         # 模块初始化入口（条件编译）
│   ├── module_config.cmake        # 模块默认配置模板
│   ├── CMakeLists.txt             # 条件编译：按需收录源文件
│   ├── algorithm/                 # 基础算法库           
│   ├── OFFLINE/                   # 离线检测模块
│   ├── REMOTE/                    # 遥控器模块
│   ├── BMI088/                    # IMU 传感器
│   ├── INS/                       # 惯性导航系统
│   ├── REFEREE/                   # 裁判系统通信
│   ├── MOTOR/                     # 电机控制
│   ├── VISION/                    # 视觉通信模块
│   ├── BOARDCOMM/                 # 板间通信
│   ├── SuperCap/                  # 超级电容管理
│   ├── WT606/                     # WT606 模块
│   └──...                         # 未来增加模块       
│
├── robot/                         # 机器人初始化层
│   └── robot_init.c / .h          
│
├── threadx/                       # Microsoft ThreadX RTOS
│   ├── common/                    
│   ├── ports/                     
│   ├── tx_user.h                  
│   └── CMakeLists.txt             
│
├── utils/                         # 通用工具库
│   ├── ulog/                      # 日志系统（基于 SEGGER RTT）
│   ├── list/                      # 通用链表
│   ├── kfifo/                     # 环形缓冲区
│   └── utils_init.c / .h          # 工具层初始化
│
├── CMSIS-DSP/                     # ARM CMSIS-DSP 数学库
├── CherryUSB/                     # CherryUSB 设备协议栈
│
├── .clang-format                  # 代码格式化配置
├── .clang-tidy                    # 静态检查配置
└── .vscode / .zed                 # IDE 配置与调试任务
```

## 快速开始

### 获取工程
```bash
git clone https://github.com/HebutMas/mas_embedded_threadx.git // http
git clone git@github.com:HebutMas/mas_embedded_threadx.git // ssh
```


### 选择机器人和板型

编辑 `apps/config.cmake`：

```cmake
set(ROBOT "sentry")     # hero / engineer / infantry3 / infantry4 / infantry5 / drone / sentry / darts / customcontrol
set(BOARD "gimbal")     # single / gimbal / chassis
```

### 编译

推荐使用 IDE 内置任务一键编译，也可手动执行：

**damiao_h7（Cortex-M7）：**
```bash
#注意手动执行编译后，elf生成在build/damiao_h7/Debug/base.elf
cmake -S board/damiao_h7 -B build/damiao_h7/Debug --preset Debug -G Ninja
cmake --build build/damiao_h7/Debug --config Debug -j $(nproc)
```

**dji_c（Cortex-M4）：**
```bash
#注意手动执行编译后，elf生成在build/dji_c/Debug/base.elf
cmake -S board/dji_c -B build/dji_c/Debug --preset Debug -G Ninja
cmake --build build/dji_c/Debug --config Debug -j $(nproc)
```

> IDE 内置任务一键编译完成后会自动将 `compile_commands.json` 复制到 `build/`，并将 `.elf` 复制到 `build/<board>.elf`，供 `clangd` 和调试器使用。

### 烧录

项目提供一键烧录脚本，支持交互式选择：

- **VS Code**：运行任务 `Flash board`，在弹出的选项中选择开发板（`damiao_h7` / `dji_c`）和调试器（`stlink` / `daplink` / `jlink`）
- **Zed**：运行任务 `Flash board`

也可直接调用脚本：
```bash
# 交互式选择
.vscode/flash_interactive.sh <board> <probe>
.zed/flash_select.sh
```

### 调试

| 调试器 | 工具 | 配置位置 |
|--------|------|----------|
| ST-Link / DAP-Link | **Cortex-Debug** (OpenOCD) | `.vscode/launch.json` |
| J-Link | **Ozone** | 外部启动 |

## 如何贡献与维护代码

### 代码风格

本项目使用 **`.clang-format`** 统一代码风格，提交前请务必格式化：

```bash
# 格式化所有修改过的文件
clang-format -i $(git diff --name-only HEAD '*.c' '*.h')

# 或格式化整个项目
find . -name '*.c' -o -name '*.h' | xargs clang-format -i
```

### 添加新模块

若需为项目添加新的功能模块（如传感器、执行器），请遵循 **6 步模块化流程**：

1. 在 `modules/<NAME>/` 下创建模块代码（`module_<name>.h` + `module_<name>.c`）
2. 在 `modules/CMakeLists.txt` 中添加 `if(MODULE_XXX)` 条件编译块
3. 在 `modules/module_init.c` 中添加条件初始化代码
4. 在 `modules/module_config.cmake` 中注册默认参数和模块列表
5. 在 `apps/generate_headers.cmake` 中添加该模块的参数宏导出（普通宏用 `#define VAR ${VAR}`，可选硬件句柄用 `_gen_cmakedefine`）
6. 在目标机器人的 `apps/<robot>/robot.cmake` 中使能或覆盖配置

**详细规范与示例请参考 [`modules/MODULES.MD`](modules/MODULES.MD)。**

### 添加新机器人

1. 在 `apps/` 下新建目录 `apps/<robot_name>/`
2. 复制 `apps/templates/robot.cmake` 到 `apps/<robot_name>/robot.cmake` 并修改模块列表和参数
3. 在 `apps/<robot_name>/` 下按板型创建业务代码目录：
   - `single_board/` —— 单板整机
   - `gimbal_board/` —— 云台板
   - `chassis_board/` —— 底盘板
4. 在 `apps/config.cmake` 中将新机器人加入注释说明

### Git 工作流建议

1. **分支管理**：
   - `main`：稳定分支，仅接受经过测试的合并
   - `dev`：日常开发分支
   - `feature/<模块名>` 或 `fix/<问题描述>`：个人开发分支

2. **提交规范**：
   - 提交信息使用中文或英文，清晰描述改动内容
   - 一次提交只做一件事，避免混杂无关改动
   - 涉及模块改动的提交，建议在信息中标注模块名，例如：`[MOTOR] 修复达妙电机 CAN ID 过滤逻辑`

3. **代码审查 checklist**：
   - [ ] 是否通过 `.clang-format` 格式化
   - [ ] 新模块是否按 `MODULES.MD` 规范添加
   - [ ] 是否修改了 `apps/config.cmake` 中的默认 ROBOT/BOARD（不应提交个人调试配置）
   - [ ] 是否在目标板卡上编译通过
   - [ ] 是否在实际硬件上测试通过

## 注意事项

- 不同系统下的git换行符问题
```bash
// 在git bash中运行
linux： git config --global core.autocrlf input
windows：git config --global core.autocrlf true
```


## 相关文档

- [`modules/MODULES.MD`](modules/MODULES.MD) —— 模块框架详细文档（添加模块的完整指南）
- [ThreadX 官方文档](https://github.com/eclipse-threadx/threadx) —— RTOS API 参考
- [CMSIS-DSP 文档](https://arm-software.github.io/CMSIS-DSP/latest/) —— DSP 函数说明
- [CherryUSB 文档](https://github.com/sakumisu/CherryUSB) —— USB 协议栈说明

