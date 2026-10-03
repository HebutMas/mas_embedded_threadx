# modules/LORA

塔石 L33 LoRa 透传模块。默认参数为串口 115200、空中速率 19200、信道 23。

## 工作方式

帧格式为：

```text
0xAA 0x55 LEN SEQ PAYLOAD CRC8
```

`PAYLOAD` 是注册变量按注册顺序拼接后的值，所有数值均按小端序编码。收发两端必须使用完全相同的注册顺序和类型。Payload 最大 59 字节，注册超限返回 `-1`。

模块初始化由框架自动调用，用户只需要注册变量、启动模块，并在自己的线程或主循环中调用处理函数。

```c
#include "module_lora.h"

static uint32_t counter;
static float angle;
static int16_t mode;

void app_init(void)
{
    Lora_Register("counter", &counter, LORA_TYPE_UINT32);
    Lora_Register("angle", &angle, LORA_TYPE_FLOAT);
    Lora_Register("mode", &mode, LORA_TYPE_INT16);
    Lora_Start();
}

void app_loop(void)
{
    Lora_Process();
}
```

## API

对外只提供四个函数：

```c
void Module_Lora_Init(void);                               /* 框架自动调用 */
int  Lora_Register(const char *name, void *value, Lora_Data_Type type);
int  Lora_Start(void);
void Lora_Process(void);
```

支持的数据类型：

```c
LORA_TYPE_INT8
LORA_TYPE_INT16
LORA_TYPE_INT32
LORA_TYPE_UINT8
LORA_TYPE_UINT16
LORA_TYPE_UINT32
LORA_TYPE_FLOAT
LORA_TYPE_DOUBLE
```

注册项只保存变量指针。每次发送时直接读取指针指向的最新值；每次收到合法帧后按注册表顺序写回变量。

发送周期由宏配置，默认 50 ms：

```cmake
set(LORA_TX_INTERVAL_MS 50)
```

## F103C8 示例

```cmake
set(MODULES_SINGLE OFFLINE LORA)
set(LORA_UART huart2)
set(LORA_M0_GPIO_PORT GPIOA)
set(LORA_M0_GPIO_PIN GPIO_PIN_4)
set(LORA_M1_GPIO_PORT GPIOA)
set(LORA_M1_GPIO_PIN GPIO_PIN_5)
set(LORA_AUX_GPIO_PORT GPIOA)
set(LORA_AUX_GPIO_PIN GPIO_PIN_6)
set(LORA_AUX_ENABLE 0)
```

CubeMX 配置：

```text
USART2: PA2 TX, PA3 RX, 115200 8N1
USART2_RX DMA: Circular
PA4: GPIO output, low, M0
PA5: GPIO output, low, M1
PA6: GPIO input, AUX (AUX_ENABLE=1 时使用)
```

接线：

```text
PA2 / USART2_TX -> L33 RXD
PA3 / USART2_RX <- L33 TXD
PA4              -> L33 M0
PA5              -> L33 M1
PA6              <- L33 AUX (可选)
3.3V             -> L33 VCC
GND              -> L33 GND
```

两块板上的注册顺序和类型必须一致。两块 LORA 模块的透传模式、信道、空中速率和天线也必须一致。不要让 LORA 与其他模块共用同一个 UART。
