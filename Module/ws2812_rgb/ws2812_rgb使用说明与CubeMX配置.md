# `ws2812_rgb.hpp` 使用说明与 STM32CubeMX 配置说明

本文档说明 `Module/ws2812_rgb/ws2812_rgb.hpp` 的使用方法，以及驱动 WS2812E/WS2812 系列单总线 RGB 灯珠时，STM32CubeMX 桌面端应该怎样配置 TIM、GPIO、DMA 和 NVIC。

本文档只写 WS2812E 驱动所需配置和接入方式。

## 1. 参考资料

本说明结合了当前工程代码和用户提供的两份规格书：

| 资料 | 路径 | 本文使用的信息 |
|---|---|---|
| WS2812E 灯珠规格书 2025 | `C:\Users\ljf\Desktop\WS2812E灯珠规格书2025.pdf` | 供电范围、输入电平、数据传输频率、数据 1 占空比、reset 时间、RGB 恒流参数 |
| BTF 5V WS2812E 60L-B30 灯条规格书 | `C:\Users\ljf\Desktop\049-20220429-BTF-5V-WS2812E-60L-B30灯条规格书.pdf` | 灯条型号上下文：BTF、5V、WS2812E、60L-B30；灯条功耗、板宽、焊盘定义等细节建议打开 PDF 人工确认 |

PDF 文本层对部分中文和 `µ` 符号抽取不稳定，所以本文对不能完全从文本层确认的灯条级参数标注为“需人工确认”。

## 2. 驱动功能概览

| 项目 | 内容 |
|---|---|
| 驱动文件 | `Module/ws2812_rgb/ws2812_rgb.hpp` |
| 驱动类型 | C++ 模板类，`template<uint16_t LED_num, uint16_t reset_time = 200>` |
| 输出方式 | TIM PWM + DMA |
| 数据顺序 | GRB，不是 RGB |
| bit 顺序 | 每个颜色字节高位先发 |
| 刷新方式 | `show()` 编码整条灯带并启动一次 PWM DMA |
| 完成处理 | 在 `HAL_TIM_PWM_PulseFinishedCallback()` 中调用 `onDma_finished()` |

这个类不负责初始化 CubeMX 外设。它只接收外部传入的 `TIM_HandleTypeDef *`、PWM 通道和 DMA buffer，然后把 RGB 数据编码成 PWM 占空比序列。

## 3. WS2812E 规格要点

从 `WS2812E灯珠规格书2025.pdf` 抽取到的关键参数如下：

| 类别 | 参数 | 规格书信息 | 对本驱动的影响 |
|---|---|---|---|
| 供电 | `VDD` | 工作电压约 `+3.7 V ~ +5.5 V` | 5V 灯条供电是合理的 |
| 输入高电平 | `VIH` | 最小约 `3.1 V` | STM32 3.3V 输出余量较小，比赛环境建议加 74AHCT/74HCT 电平转换 |
| 输入低电平 | `VIL` | 最大约 `1.5 V` | STM32 低电平满足要求 |
| 数据速率 | `fDIN` | `800 KHz` | CubeMX PWM 频率应配置为约 800 kHz |
| 数据 1 占空比 | Data 1 duty | 规格书文本抽取为 `67%` | 驱动中 `code1_` 建议按约 67% 校准，当前 60% 属于偏低设置 |
| 码元周期 | bit period | 最低要求约 `1.2 us` | 800 kHz 对应 `1.25 us`，满足要求 |
| reset 低电平 | reset/latch | 时序图文本抽取约为 `>200 us`，`µ` 符号抽取不稳定，需人工确认 | 默认 `reset_time=200` 在 800 kHz 下约 `250 us`，满足该保守要求 |
| 通道恒流 | `IOUT R/G/B` | 约 `11.5 mA`/通道 | 单颗全白约 `34.5 mA`，灯条电源仍建议留足余量 |
| RGB 波长/亮度 | R/G/B | R `620~625 nm`，G `515~525 nm`，B `465~475 nm`；亮度见规格书 | 不影响 CubeMX，只影响视觉效果和功耗估算 |
| 工作温度 | `Topt` | 约 `-40~+85 C` | 竞赛环境通常满足，但高亮长时间工作要注意散热 |

灯条规格书文件名表明当前参考对象是 `BTF-5V-WS2812E-60L-B30` 灯条。`60L-B30` 的精确定义、灯条总长度、每米灯数、额定功耗、焊盘顺序和防护结构请以原 PDF 页面为准，需人工确认。

## 4. CubeMX 配置是否正确

结论：Markdown 中“PWM 频率约 800 kHz、PWM DMA 输出 CCR 序列、GPIO 复用推挽输出、DMA 普通模式”的方向是对的；需要重点校准的是 PWM 占空比和 reset 时间。

假设 TIM 计数时钟为 `275 MHz`，推荐参数：

```text
Prescaler = 0
Counter Period = 343
ARR + 1 = 344
PWM frequency = 275 MHz / 344 = 799418.6 Hz
PWM period = 1.251 us
```

这个频率与 WS2812E `800 KHz` 要求匹配。

当前 `ws2812_rgb.hpp` 的编码方式：

```cpp
code0_ = period * 25 / 100;
code1_ = period * 60 / 100;
```

在 `ARR + 1 = 344` 时：

| 码值 | 当前占空比 | CCR | 高电平时间 | 评价 |
|---|---:|---:|---:|---|
| `0` | 25% | 86 | 约 `0.313 us` | 通常可用，仍建议示波器确认 |
| `1` | 60% | 206 | 约 `0.749 us` | 低于规格书抽取到的 67% 典型值 |
| reset | 200 个周期 | 0 | 约 `250 us` 低电平 | 满足 `>200 us` 的保守要求 |

如果要更贴近 WS2812E 规格书的 Data 1 `67%` 占空比，建议把驱动里的 `code1_` 改为：

```cpp
code1_ = period * 67 / 100;
```

此时 `ARR + 1 = 344` 时 `code1_ = 230`，高电平约 `0.836 us`，更接近 800 kHz 下 67% 的数据 1 波形。

CubeMX 里的 `Pulse` 初始值保持 `0` 是正确的，因为实际 CCR 值由 DMA 每个 bit 动态写入，不靠固定 Pulse 输出。

## 5. 硬件连接

| 灯条/灯珠端 | STM32 端 | 说明 |
|---|---|---|
| `DIN` | 任意支持 TIMx_CHy PWM 输出的引脚 | 需与 CubeMX 中选择的 TIM 通道一致 |
| `5V` | 外部 5V 电源 | 不建议由开发板 3.3V 直接供电 |
| `GND` | STM32 GND 与外部电源 GND 共地 | 必须共地 |

硬件建议：

1. 数据线靠近第一颗 LED 的 DIN 端串联 `220 Ω ~ 470 Ω` 电阻。
2. 灯条 5V 与 GND 之间并联 `470 uF ~ 1000 uF` 电解电容，容量按灯数和电源线长度调整，需人工确认。
3. 因 WS2812E `VIH` 约 `3.1 V`，STM32 3.3V 直推只有少量余量；线长、噪声、电源跌落都会降低可靠性，建议使用 5V 兼容电平转换器。
4. 电源电流按灯珠规格可估算为单颗全白约 `34.5 mA`，实际灯条功耗以 BTF 灯条规格书为准；工程供电设计建议留 30% 以上余量。

## 6. CubeMX 桌面配置流程

这一章按 STM32CubeMX 的实际操作顺序写。核心目标只有一个：让某个 `TIMx_CHy` 引脚输出约 `800 kHz` 的 PWM，并让 DMA 按 buffer 内容连续改写该通道的 CCR 值。

下面用 `TIMx_CHy` 表示“你最终选择的定时器通道”。如果你实际选择的是 `TIM4_CH3`，就把文中的 `TIMx` 替换为 `TIM4`，把 `CHy` 替换为 `CH3`；如果你选择其它通道，也按同样逻辑替换。

### 6.1 配置前先确定 4 件事

在 CubeMX 里开始点选之前，先在纸上或注释里确定这 4 个量：

| 要确定的内容 | 示例 | 为什么要先确定 |
|---|---|---|
| WS2812 数据引脚 | `WS2812_DIN` | 后面 GPIO、TIM channel、实际接线必须一致 |
| 使用哪个 PWM 通道 | `TIMx_CHy` | DMA request 必须选同一个通道 |
| 目标 PWM 频率 | `800 kHz` | WS2812E 规格书要求 `fDIN = 800 KHz` |
| reset 低电平周期数 | `reset_time = 200` | 800 kHz 下约 `250 us`，满足 reset/latch 要求 |

不要先随便选一个 PWM 后面再猜参数。WS2812 的时序比较窄，配置逻辑应该是“先定时序，再定外设”。

### 6.2 打开 `.ioc` 并进入 Pinout

1. 打开 STM32CubeMX。
2. 选择 `File -> Open Project`。
3. 打开工程根目录下的 `ROBOCON26.ioc`。
4. 进入 `Pinout & Configuration` 页面。
5. 在芯片引脚图上找到你准备接 WS2812 `DIN` 的引脚。
6. 点击该引脚，在下拉功能里选择对应的 `TIMx_CHy`。

如果你不知道某个引脚能不能输出目标 TIM 通道，可以用两种方式找：

1. 在左侧 `Timers -> TIMx` 里启用某个 Channel，CubeMX 会高亮可选引脚。
2. 直接点引脚，看下拉列表里有没有 `TIMx_CHy`。

选完后，建议给这个引脚加用户标签：

```text
System Core -> GPIO -> 选中该引脚 -> User Label = WS2812_DIN
```

标签不是必须，但后续查原理图、查生成代码会清楚很多。

### 6.3 配置 GPIO 参数

路径：

```text
Pinout & Configuration -> System Core -> GPIO
```

找到刚才选的 `TIMx_CHy` 引脚，按下面配置：

| GPIO 项 | 推荐值 | 原因 |
|---|---|---|
| GPIO mode | Alternate Function Push Pull | 定时器复用推挽输出 PWM |
| GPIO Pull-up/Pull-down | No pull-up and no pull-down | WS2812 DIN 不需要 MCU 内部上下拉 |
| Maximum output speed | Very High | 800 kHz 数据线需要比较干净的边沿 |
| User Label | `WS2812_DIN` | 方便维护 |

注意：这里的 `Alternate Function` 不要手动乱选。CubeMX 在你选择 `TIMx_CHy` 后会自动填入正确 AF，例如 `GPIO_AFx_TIMx`。如果 AF 不对，PWM 可能不会从引脚输出。

### 6.4 进入 Clock Configuration，确认 TIM 时钟

路径：

```text
Clock Configuration
```

在时钟树里找到目标定时器所在总线的 timer clock。常见情况：

| 定时器类型 | 通常所在总线 | 在 CubeMX 里要看的位置 |
|---|---|---|
| `TIM1`、`TIM8` 等高级定时器 | APB2 | `APB2 timer clocks` |
| `TIM2`、`TIM3`、`TIM4`、`TIM5` 等通用定时器 | APB1 | `APB1 timer clocks` |

当前工程里文档按 `275 MHz` 的 TIM 计数时钟举例：

```text
timer_clock = 275 MHz
target_pwm = 800 kHz
```

计算公式：

```text
pwm_freq = timer_clock / (Prescaler + 1) / (Period + 1)
```

推荐先让 `Prescaler = 0`，这样定时器分辨率最高。再算 `Period`：

```text
Period + 1 = timer_clock / target_pwm
Period + 1 = 275000000 / 800000 = 343.75
```

CubeMX 的 `Counter Period` 必须是整数，所以取：

```text
Prescaler = 0
Counter Period = 343
实际 PWM = 275000000 / (343 + 1) = 799418.6 Hz
实际周期 = 1.251 us
```

这个误差很小，可以用于 WS2812E。

如果你的 timer clock 不是 `275 MHz`，按下面方式重新算：

| timer clock | 推荐 Prescaler | 推荐 Period | 实际 PWM |
|---:|---:|---:|---:|
| `275 MHz` | `0` | `343` | 约 `799.4 kHz` |
| `240 MHz` | `0` | `299` | `800.0 kHz` |
| `200 MHz` | `0` | `249` | `800.0 kHz` |
| `168 MHz` | `0` | `209` | `800.0 kHz` |
| `84 MHz` | `0` | `104` | `800.0 kHz` |

如果算出来不是整数，优先选择最接近 `800 kHz` 的 `Period`，然后用示波器确认实际 bit 周期接近 `1.25 us`。

### 6.5 配置 TIM 基本模式

路径：

```text
Pinout & Configuration -> Timers -> TIMx
```

先在 `Mode` 区域配置：

| Mode 项 | 选择 |
|---|---|
| Clock Source | Internal Clock |
| Channel y | PWM Generation CHy |

说明：

1. `Clock Source = Internal Clock` 表示 TIM 用 MCU 内部时钟计数。
2. `PWM Generation CHy` 表示该通道输出 PWM。
3. 这里的 `CHy` 必须和你引脚上选择的 `TIMx_CHy` 是同一个通道。

### 6.6 配置 TIM 参数

仍在：

```text
Timers -> TIMx -> Configuration -> Parameter Settings
```

按下面配置：

| 参数 | 推荐值 | 说明 |
|---|---|---|
| Prescaler | `0` | 不分频，保留最高计数分辨率 |
| Counter Mode | Up | 向上计数 |
| Counter Period | `343`，若 timer clock 为 `275 MHz` | 决定 PWM 周期 |
| Internal Clock Division | No Division | 不再额外分频 |
| Auto-reload preload | Disable | WS2812 固定周期输出，不需要 ARR 预装载 |
| Repetition Counter | 默认值 | 普通通用定时器可能没有该项 |

然后展开 `PWM Generation Channel y`，按下面配置：

| PWM 参数 | 推荐值 | 说明 |
|---|---|---|
| Mode | PWM mode 1 | CCR 内为高电平，符合驱动编码逻辑 |
| Pulse | `0` | 初始化时先不输出高电平 |
| Output compare preload | Enable 或 Disable 均可，优先保持 CubeMX 默认 | DMA 会持续写 CCR，最终以示波器验证为准 |
| Fast Mode | Disable | 不需要快速比较模式 |
| CH Polarity | High | 高电平表示 WS2812 的有效高电平 |
| CH Idle State | Reset 或默认值 | 普通通道通常不需要改 |

特别注意：不要在 CubeMX 里把 `Pulse` 设成 `25%`、`60%` 或 `67%`。`Pulse` 只是初始 CCR 值，实际每个 bit 的 CCR 会由 DMA buffer 写入。

### 6.7 配置 DMA

路径：

```text
Timers -> TIMx -> Configuration -> DMA Settings
```

操作步骤：

1. 点击 `Add`。
2. 在新行的 `DMA Request` 中选择对应通道，例如 `TIMx_CHy`。
3. 点选这条 DMA 配置，展开右侧或下方参数。
4. 按下表配置。

| DMA 项 | 推荐值 | 说明 |
|---|---|---|
| DMA Request | `TIMx_CHy` | 必须和 PWM 通道一致 |
| Direction | Memory To Peripheral | 从内存 buffer 写到 TIM CCR |
| Priority | Very High 或 High | 避免其它 DMA 抢占导致波形抖动 |
| Mode | Normal | 一帧发完就停止，不能用 Circular 无限循环 |
| Increment Address - Peripheral | Disable | TIM CCR 地址固定 |
| Increment Address - Memory | Enable | 每个 bit 读取下一个 CCR 值 |
| Data Width - Peripheral | Word | 与 `uint32_t *` DMA buffer 匹配 |
| Data Width - Memory | Word | 与 `uint32_t pwm_buffer[]` 匹配 |
| FIFO Mode | Disable | WS2812 这种短周期 CCR 更新不需要 FIFO |

为什么 `Mode` 要选 `Normal`：

1. 一次 `show()` 只发送一帧：`LED_num * 24 + reset_time` 个 CCR。
2. DMA 完成后进入回调，调用 `onDma_finished()` 停止 PWM。
3. 如果选 `Circular`，DMA 会一直循环发送旧帧，`busy_` 状态和 reset 低电平都不好控制。

为什么数据宽度建议 `Word`：

1. `HAL_TIM_PWM_Start_DMA()` 的数据指针参数是 `uint32_t *`。
2. 当前驱动 buffer 类型也是 `uint32_t[]`。
3. STM32H7 的 TIM CCR 寄存器按 32-bit 写入更直接。

如果 CubeMX 让你手动选择 DMA Stream/Channel，就选择一个空闲项；如果 CubeMX 自动分配，则生成代码后确认 `HAL_DMA_IRQHandler()` 对应的是该 TIM PWM DMA 句柄。

### 6.8 配置 NVIC

路径：

```text
System Core -> NVIC
```

需要确认：

| 中断项 | 配置 | 说明 |
|---|---|---|
| 所选 DMA Stream IRQ | Enable | DMA 完成回调依赖它 |
| Preemption Priority | 可用 `5`，或按工程 FreeRTOS 规则设置 | 当前工程同类中断常用该等级 |
| Sub Priority | `0` | 一般保持 0 |
| TIMx global interrupt | 通常 Disable | 只做 PWM DMA 输出时不需要 TIM 更新中断 |

逻辑关系是：

```text
DMA 传输完成
-> DMA Stream IRQ
-> HAL_DMA_IRQHandler()
-> HAL_TIM_PWM_PulseFinishedCallback()
-> ws2812_rgb::onDma_finished()
-> busy_ = false
```

所以 DMA IRQ 没开，`show()` 第一次可能成功，后面就可能一直忙。

### 6.9 检查 Project Manager

路径：

```text
Project Manager
```

建议检查：

| 页面 | 项目 | 推荐 |
|---|---|---|
| Project | Toolchain / IDE | 保持当前工程的 `CMake` |
| Code Generator | Keep User Code | Enabled |
| Code Generator | Generate peripheral initialization as a pair of `.c/.h` files per peripheral | 保持当前工程风格 |
| Advanced Settings | TIM、DMA 驱动 | 使用 HAL |

用户自己写的 WS2812 初始化函数、测试函数、C++ 回调封装，建议放在独立 `.cpp` 文件里，例如 `APP/led_task/led_task.cpp` 或你自己的模块文件。不要写到 CubeMX 生成区外的普通位置，也不要把 C++ 模板调用直接写进 `.c` 文件。

### 6.10 生成代码

操作：

```text
Project -> Generate Code
```

生成后至少检查这些文件：

| 文件 | 要看到什么 |
|---|---|
| `Core/Inc/tim.h` | 有 `extern TIM_HandleTypeDef htimx;` 和 `void MX_TIMx_Init(void);` |
| `Core/Src/tim.c` | `MX_TIMx_Init()` 中有 PWM init、目标通道 config、`HAL_TIM_MspPostInit()` |
| `Core/Src/tim.c` | `HAL_TIM_PWM_MspInit()` 中有该 TIM 通道的 DMA init 和 `__HAL_LINKDMA(...)` |
| `Core/Src/dma.c` | DMA 控制器时钟已开启，所选 DMA IRQ 已使能 |
| `Core/Src/stm32h7xx_it.c` | 所选 DMA IRQHandler 调用对应 DMA handle 的 `HAL_DMA_IRQHandler()` |
| `Core/Src/main.c` | `MX_DMA_Init()` 和 `MX_TIMx_Init()` 都已在使用前调用 |

这里的 `htimx`、`MX_TIMx_Init()`、IRQ 名称都要替换成你实际选择的 TIM 和 DMA。

### 6.11 配置完成后的波形验收

先不要急着接整条灯带，建议先用示波器或逻辑分析仪看 `WS2812_DIN`：

| 检查项 | 目标 |
|---|---|
| PWM bit 周期 | 约 `1.25 us` |
| bit 频率 | 约 `800 kHz` |
| `0` 码高电平 | 当前驱动约 `0.31 us` |
| `1` 码高电平 | 当前驱动约 `0.75 us`；若改 67%，约 `0.84 us` |
| 帧末 reset 低电平 | 默认约 `250 us` |
| 空闲电平 | 低电平 |

验收顺序建议：

1. 只初始化 TIM/DMA 和 WS2812 驱动，不接灯条，用示波器看 DIN 波形。
2. 接 1 颗灯珠，低亮度测试红、绿、蓝。
3. 接完整灯条，仍然先用低亮度。
4. 确认电源不掉压、第一颗灯不闪烁，再提高亮度。

## 7. DMA buffer 与内存位置

DMA buffer 长度公式：

```text
buffer_length = LED_num * 24 + reset_time
```

原因：

1. 每颗灯 24 bit：`G[7:0] + R[7:0] + B[7:0]`。
2. 每个 bit 对应一个 PWM CCR 值。
3. 末尾追加 `reset_time` 个 0，占用低电平 reset/latch 时间。

示例：8 颗灯，`reset_time=200`：

```text
8 * 24 + 200 = 392 个 uint32_t
392 * 4 = 1568 字节
```

STM32H7 的 DTCM 通常不能被普通 DMA 访问。当前工程已经定义：

```cpp
#define DMA_BUFFER_ATTR __attribute__((section(".dma_buffer"), aligned(32)))
```

因此建议：

```cpp
static constexpr uint16_t kLedNum = 8;
static constexpr uint16_t kResetSlots = 200;

static uint32_t ws2812_pwm_buffer[kLedNum * 24 + kResetSlots]
    DMA_BUFFER_ATTR;

using BoardLedStrip = ws2812_rgb<kLedNum, kResetSlots>;
```

如果未来启用 D-Cache，要么保证 `.dma_buffer` 区域不可缓存，要么在启动 DMA 前清理 cache。否则 DMA 可能读到旧数据。

## 8. 使用示例

### 8.1 初始化

在某个 `.cpp` 文件中：

```cpp
#include "ws2812_rgb.hpp"
#include "tim.h"

static constexpr uint16_t kLedNum = 8;
static constexpr uint16_t kResetSlots = 200;

static uint32_t ws2812_pwm_buffer[kLedNum * 24 + kResetSlots]
    DMA_BUFFER_ATTR;

using BoardLedStrip = ws2812_rgb<kLedNum, kResetSlots>;

extern "C" void Ws2812_Init(void)
{
    BoardLedStrip::instance().init(&htim4, TIM_CHANNEL_3, ws2812_pwm_buffer);
    BoardLedStrip::instance().clear();
    BoardLedStrip::instance().show();
}
```

`&htim4` 和 `TIM_CHANNEL_3` 只是示例。实际使用时必须换成 CubeMX 中选择的 TIM 句柄和通道。

### 8.2 DMA 完成回调

同样建议写在 `.cpp` 文件中：

```cpp
extern "C" void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        BoardLedStrip::instance().onDma_finished(htim);
    }
}
```

如果你实际使用的不是 TIM4，请同步改成对应 `TIMx`。

### 8.3 设置颜色

```cpp
extern "C" void Ws2812_TestOnce(void)
{
    auto &strip = BoardLedStrip::instance();

    if (strip.isBusy()) {
        return;
    }

    strip.clear();
    strip.set_color(0, 32, 0, 0);  // 红
    strip.set_color(1, 0, 32, 0);  // 绿
    strip.set_color(2, 0, 0, 32);  // 蓝
    strip.show();
}
```

建议先用低亮度测试，例如 `32/255`，确认供电和波形正常后再提高亮度。

## 9. 接口说明

### `instance()`

```cpp
static ws2812_rgb& instance();
```

返回当前模板参数对应的单例对象。`ws2812_rgb<8>` 和 `ws2812_rgb<16>` 是两个不同类型，也会有不同单例。

### `init()`

```cpp
void init(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t *pwm_buffer);
```

绑定定时器、PWM 通道和 DMA buffer，并根据 `ARR + 1` 计算 `0` 码和 `1` 码 CCR 值。

注意：

1. 必须在 CubeMX 生成的 `MX_TIMx_Init()` 之后调用。
2. `pwm_buffer` 不能是空指针。
3. buffer 长度必须至少是 `LED_num * 24 + reset_time`。

### `set_color()`

```cpp
void set_color(uint16_t index, uint8_t r, uint8_t g, uint8_t b);
```

设置第 `index` 颗灯的颜色。越界时直接返回。设置后必须调用 `show()` 才会实际刷新。

### `clear()`

```cpp
void clear();
```

清空内部颜色缓存。调用后还要 `show()` 才会熄灭灯条。

### `show()`

```cpp
HAL_StatusTypeDef show();
```

把内部颜色缓存编码成 PWM DMA buffer，并启动一次 DMA 传输。

返回值：

| 返回值 | 含义 |
|---|---|
| `HAL_OK` | DMA 刷新已启动 |
| `HAL_ERROR` | 未初始化或 HAL 启动失败 |
| `HAL_BUSY` | 上一次刷新未完成 |

`show()` 非阻塞。不要在 `HAL_BUSY` 时反复写 buffer。

### `onDma_finished()`

```cpp
void onDma_finished(TIM_HandleTypeDef* htim);
```

在 PWM DMA 完成回调中调用，用于停止 PWM DMA、把 CCR 置 0、释放 `busy_`。

### `isBusy()`

```cpp
bool isBusy() const;
```

查询当前是否仍在 DMA 刷新。

### `size()`

```cpp
uint16_t size() const;
```

返回 LED 数量 `LED_num`。

## 10. 还需要注意的问题

1. **建议把 `code1_` 校准为 67%**：规格书抽取到数据 1 占空比为 `67%`，当前驱动使用 `60%`，建议用示波器实测；若严格按规格书，推荐改成 `period * 67 / 100`。
2. **reset 时间按 `>200 us` 设计更稳**：默认 `reset_time=200` 在 800 kHz 下约 `250 us`，是合理的。若以后提高 PWM 频率或降低 reset slots，要重新计算。
3. **3.3V 直推 5V 灯条余量偏小**：WS2812E `VIH` 最小约 `3.1 V`，STM32 高电平在长线、噪声和压降下可能不够稳，建议加电平转换。
4. **电源别按“能亮”来设计**：规格书通道恒流约 `11.5 mA`，单颗全白约 `34.5 mA`，整条灯带还要看 BTF 灯条规格书额定功耗。电源和线宽要留余量。
5. **DMA buffer 必须在 DMA 可访问 RAM**：H7 上不要把 buffer 放进 DTCM；当前工程可用 `DMA_BUFFER_ATTR`。
6. **DMA 完成回调必须接上**：否则第一次 `show()` 后 `busy_` 可能一直为 true。
7. **多任务访问要加保护**：如果多个 FreeRTOS task 都会设置灯带颜色，建议统一由一个 LED task 控制，其他任务发队列消息。
8. **必须用示波器最终确认**：重点看 bit 周期约 `1.25 us`，数据 0/1 高电平宽度，帧末 reset 低电平是否大于规格要求。

## 11. 快速验收清单

1. CubeMX 中选定 `TIMx_CHy`，PWM 频率约 `800 kHz`。
2. GPIO 为 `Alternate Function Push Pull`，`No Pull`，`Very High`。
3. DMA 为 `TIMx_CHy` request，`Memory To Peripheral`，`Normal`，内存自增。
4. DMA Stream IRQ 已启用。
5. `pwm_buffer` 使用 `DMA_BUFFER_ATTR`。
6. `HAL_TIM_PWM_PulseFinishedCallback()` 中调用 `onDma_finished()`。
7. `reset_time=200` 时，reset 低电平约 `250 us`。
8. 若追求更贴合 WS2812E 规格，把 `code1_` 改为 `67%` 并用示波器确认。
