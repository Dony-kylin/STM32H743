# G题数据处理模块使用说明

## 1. 模块用途

本模块由 Simulink 导出，用于处理一帧 AD9220 采样数据，并输出：

- 基频和各频率分量；
- 各分量的电压峰值；
- 谐波次数；
- 电压峰峰值和真有效值；
- 1个或3个完整周期的显示波形。

处理流程为：

```text
16384点、8 MHz采样
→ V4模式选择：题目1/2走原始路径，题目3走64阶FIR低通
→ 不抽取，保留全部16384点
→ 16384点Hann窗FFT
→ 频率和波形提取
→ 原始8 MHz数据的多频联合最小二乘幅值拟合
→ 前端增益与电压校准，恢复真实输入电压
```

处理器没有 `scale` 输入，也不会读取仿真信号源内部状态。所有结果都在
一帧 ADC 数据采集完成后计算。

频谱同时提供两组幅值：

- ADC端口实际存在的物理幅值；
- 按“合成波形峰值受基波设定Vpeak限制”的发生器规则恢复出的设置幅值。

题目1、2可按采样帧峰值与实测基波幅值之比恢复发生器设置幅值。
题目3不使用原始Vpp或最大采样点计算换算因子，因为200 mVpp、
≥1 MHz干扰会污染这些量。算法先拟合基波和谐波幅值，再按识别到
的谐波波次和发生器默认零相位规则，仅使用≤500 kHz的有效分量
重建一个周期的干净波形，然后计算：

```text
K = 重建干净波形的绝对峰值 / 拟合得到的基波Vpeak
设置幅值 = 实测频谱幅值 × K
```

因此K包含各次谐波峰值叠加的影响，但不包含≥1 MHz干扰，也不需要
测量相位。

包装接口对频率、两组幅值、Vpp和Vrms进行自适应帧间平滑。变化不
超过5%时使用四帧指数平均；超过5%视为真实信号变化并立即更新。
`waveform[600]` 不做帧间平均，避免不同相位的波形互相抵消。

应用层只使用 `task0729_processor.h`，不要直接访问
`G_Export_V4_U` 或 `G_Export_V4_Y`。

## 2. 相关文件

| 文件 | 作用 |
|---|---|
| `Expand/Inc/task0729_processor.h` | 稳定的对外接口 |
| `Expand/Src/task0729_processor.c` | 接口包装与数据复制 |
| `Expand/Generated/Task0729V4/G_Export_V4.c` | Simulink生成的V4算法 |
| `Expand/Generated/Task0729V4/G_Export_V4.h` | V4生成接口 |
| `G_Sim_V4.slx` | 完整实际仿真模型 |
| `G_Export_V4.slx` | 只用于生成C代码的模型 |

## 3. 输入数据要求

每次必须传入恰好 `16384` 个 `int16_t` 数据：

```c
int16_t samples[TASK0729_INPUT_SAMPLES];
```

数据必须满足：

- 已经去除 ADC 中点偏置；
- `-32768～32767` 对应 ADC 输入端约 `-2.5～+2.5 V`；
- 数据顺序连续，采样率为 `8 MHz`；
- 不要传入仍带有 `2.5 V` 偏置的无符号原始码。

当前项目的 `AD9220_CopySignedSamples()` 已经输出符合要求的数据，可以直接传入。

## 4. 初始化

包含头文件：

```c
#include "task0729_processor.h"
```

系统启动后调用一次：

```c
Task0729_Init();
```

不要在每一帧前重复初始化。

## 5. 处理一帧数据

```c
static int16_t adc_samples[TASK0729_INPUT_SAMPLES];
static Task0729_Result result;

void ProcessAdcBlock(void)
{
    uint32_t copied;

    copied = AD9220_CopySignedSamples(
        adc_samples, TASK0729_INPUT_SAMPLES);

    if (copied != TASK0729_INPUT_SAMPLES)
    {
        return;
    }

    if (Task0729_Process(
            adc_samples,
            TASK0729_MODE_QUESTION_3, /* 或按工况选择1/2/3 */
            1U,
            &result) == 0U)
    {
        /* 参数错误 */
        return;
    }

    /* 此处可以刷新LCD或通过串口发送result */
}
```

`Task0729_Process()` 是阻塞函数，不要在 DMA 中断服务函数中调用。应当在采样完成后的主循环或数据处理任务中调用。

## 6. 参数说明

函数定义：

```c
uint8_t Task0729_Process(
    const int16_t samples[TASK0729_INPUT_SAMPLES],
    Task0729_Mode mode,
    uint8_t periods,
    Task0729_Result *result);
```

### mode

| 参数 | 用途 |
|---|---|
| `TASK0729_MODE_QUESTION_1` | 第一问，分析10～200 kHz |
| `TASK0729_MODE_QUESTION_2` | 第二问，分析10～500 kHz |
| `TASK0729_MODE_QUESTION_3` | 第三问，分析10～500 kHz并使用抗干扰链路 |

### periods

只能填写：

- `1U`：提取1个完整周期；
- `3U`：提取3个完整周期。

### 返回值

- `1U`：处理成功；
- `0U`：指针为空、模式错误或周期数不是1/3。

## 7. 输出结果

```c
typedef struct
{
    float frequency_hz[3];
    float amplitude_vpk[3];
    float amplitude_setting_vpk[3];
    uint8_t harmonic_order[3];
    uint8_t component_count;
    float vpp;
    float vrms;
    float fundamental_hz;
    float waveform[600];
    uint16_t waveform_count;
} Task0729_Result;
```

| 字段 | 单位 | 说明 |
|---|---|---|
| `frequency_hz[i]` | Hz | 第i个有效频率分量，按频率从低到高排列 |
| `amplitude_vpk[i]` | Vpeak | ADC实际输入中第i个分量的物理幅值 |
| `amplitude_setting_vpk[i]` | Vpeak | 恢复的信号发生器界面设置幅值 |
| `harmonic_order[i]` | 无 | 1表示基波，2表示二次谐波 |
| `component_count` | 个 | 有效频率分量数，最大为3 |
| `vpp` | V | 被测信号峰峰值 |
| `vrms` | V | 被测信号真有效值 |
| `fundamental_hz` | Hz | 基频 |
| `waveform[i]` | V | 去直流、恢复真实幅值并重采样后的显示波形 |
| `waveform_count` | 点 | 成功时固定为600；根据 `periods` 覆盖1个或3个完整周期 |

幅值拟合使用一个直流项和最多三个正弦/余弦分量同时求解。与逐个
频率独立拟合相比，强基波、直流偏置和其他谐波不容易串入目标分量。
`vpp` 和 `vrms` 由联合拟合后的交流波形计算，因此不会把单个量化
尖峰误认为真实幅值。

电压标定参数在 `task0729_processor.h` 中：

```c
#define TASK0729_FRONTEND_GAIN        4.0F
#define TASK0729_VOLTAGE_CALIBRATION  1.0F
```

`TASK0729_FRONTEND_GAIN` 必须填写信号输入端到 AD9220 输入端的实测
模拟增益，不能只使用原理图标称值。使用已知标准信号校准时：

```text
新 VOLTAGE_CALIBRATION
= 旧 VOLTAGE_CALIBRATION × 标准信号幅值 / 当前测量幅值
```

题目3建议用已知纯基波的Vpeak进行标定，不要用含高频干扰波形的
Vpp。例如标准基波为 `25.000 mVpeak`，当前拟合得到
`24.223 mVpeak`，则校准系数应乘以约 `1.03208`。

仿真模型 `G_Sim_V4.slx` 也提供同名的模型工作区参数
`frontend_gain` 和 `frontend_bias`。用示波器测得前级交流增益和直流
偏置后，在 Model Explorer → Model Workspace 中修改它们，再运行仿真。
建议用纯基波在 10、100、200、300、400、500 kHz 各测一次：

```text
frontend_gain = ADC端交流Vpp / 信号源输入交流Vpp
frontend_bias = ADC端无信号时的直流电压
```

如果各频点增益基本一致，使用平均值即可；若随频率明显变化，当前标量
增益只能作为近似，需进一步增加频率查表校准。不要用题目3含有
≥1 MHz干扰的总Vpp来标定增益。

`Task0729_SampleToInputVolts()` 可以把一个有符号 Q15 采样转换为使用
相同标定系数的外部输入瞬时电压。它只是瞬时值，不能替代整帧的
`amplitude_vpk` 或 `vpp`。

只遍历有效分量：

```c
for (uint8_t i = 0U; i < result.component_count; ++i)
{
    printf("H%u: %.1f Hz, actual %.6f Vpk, setting %.6f Vpk\r\n",
           result.harmonic_order[i],
           result.frequency_hz[i],
           result.amplitude_vpk[i],
           result.amplitude_setting_vpk[i]);
}
```

## 8. 读取最近一次结果

```c
const Task0729_Result *last;

last = Task0729_GetLastResult();
```

返回的是模块内部只读指针：

- 不要修改指针指向的数据；
- 下一次调用 `Task0729_Process()` 后内容会更新；
- 模块当前不是可重入的，不要由两个任务同时调用。

## 9. STM32H743资源说明

- ADC输入复制：约32 KB；
- Simulink输入和FFT工作区：约352 KB，位于 AXI SRAM；
- `.task0729_ram` 实际链接占用：约360480字节；
- 当前完整固件：约166 KB Flash；
- 当前工程总BSS约626 KB，分布在多个STM32H743 RAM区域。

导出模型使用 Compact 文件封装，Hann窗、FIR和FFT常量放在
`G_Export_V4.c` 中，不再单独生成 `G_Export_V4_data.c`。

## 10. 代码生成配置

- 根输入 `adc_block` 的维度为16384，即一次函数调用输入一整帧；
- `mode` 和 `periods` 的维度为1；
- 根输入采样时间为1，表示每个模型步处理一帧，不是1个ADC采样点；
- 求解器为固定步长离散，步长1；
- 目标硬件为ARM Cortex-M，最终工程使用Cortex-M7、硬件FPU和`-O3`；
- 代码封装为Compact、不可重入函数接口；
- 优化目标为执行速度，并启用缓冲复用；
- 大型FFT、FIR和窗函数数据流使用`single`，避免`double`使RAM翻倍；
- FFT点数、窗系数等结构参数内联，不生成可调全局参数；
- 前端增益和最终电压校准由`task0729_processor.h`中的宏统一配置。

## 11. 串口检查

USART1配置为PA9/TX、PA10/RX、2000000波特率、8N1。串口终端发送：

```text
FFT ON
STREAM ON
```

可通过串口命令切换分析模式：

```text
MODE 1
MODE 2
MODE 3
```

模式1/2输出原始路径结果；模式3选择抗干扰FIR路径，并对频域幅值补偿FIR通带增益。
当前模型为静态图，模式1/2虽然不选用FIR结果，但FIR计算仍会随每帧执行；若后续需要
降低CPU占用，可再改成条件执行子系统。

每个有效分量会输出：

```text
#COMP H1 F=50000Hz ACT=48.999mV SET=60.001mV
```

`ACT` 是ADC实际测得幅值，`SET` 是恢复出的信号源设置幅值。

## 11. 重新生成代码

1. 打开 `G_Export_V4.slx`。
2. 进入“APP → Embedded Coder”。
3. 确认目标文件为 `ert.tlc`。
4. 确认硬件为 `ARM Compatible → ARM Cortex-M`。
5. 勾选“仅生成代码”。
6. 按 `Ctrl+B`。

也可以在 MATLAB 命令行执行：

```matlab
slbuild('G_Export_V4')
```

需要复制到项目的文件：

```text
G_Export_V4.c
G_Export_V4.h
rtwtypes.h
```

不要加入 `ert_main.c`。

重新复制 `G_Export_V4.c` 后，必须保留 `TASK0729_AXI_RAM` 属性，使
`G_Export_V4_B` 和 `G_Export_V4_U` 位于 `.task0729_ram`。否则会出现 DTCM 溢出。

## 12. 当前项目中的调用位置

当前已经在以下流程中调用：

```text
ScopeApp_ProcessCompletedCapture()
→ AD9220_CopySignedSamples()
→ Task0729_Process()
→ LCD/串口结果结构
```

具体位置在 `Core/Src/scope_app.c`。
