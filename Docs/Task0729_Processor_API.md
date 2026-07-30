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
→ 64阶FIR低通
→ 4倍抽取至2 MHz
→ 4096点Hann窗FFT
→ 频率、幅值和时域参数提取
→ 除以前端增益4，恢复真实输入电压
```

处理器没有 `scale` 输入，也不会读取仿真信号源内部状态。所有结果都在
一帧 ADC 数据采集完成后计算。

频谱同时提供两组幅值：

- ADC端口实际存在的物理幅值；
- 按“合成波形峰值受基波设定Vpeak限制”的发生器规则恢复出的设置幅值。

恢复系数由采样帧最大绝对峰值与实测基波幅值之比得到。比值不超过
`1.005` 时按1处理，避免FIR启动瞬态对纯基波产生伪校正。

应用层只使用 `task0729_processor.h`，不要直接访问
`G_Export_V3_U` 或 `G_Export_V3_Y`。

## 2. 相关文件

| 文件 | 作用 |
|---|---|
| `Expand/Inc/task0729_processor.h` | 稳定的对外接口 |
| `Expand/Src/task0729_processor.c` | 接口包装与数据复制 |
| `Expand/Generated/Task0729/G_Export_V3.c` | Simulink生成的算法 |
| `Expand/Generated/Task0729/G_Export_V3_data.c` | FIR、Hann窗和FFT常量 |
| `G_Sim_V3.slx` | 完整实际仿真模型 |
| `G_Export_V3.slx` | 只用于生成C代码的模型 |

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
            TASK0729_MODE_QUESTION_3,
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
- Simulink FFT工作区：约160 KB，位于 AXI SRAM；
- FIR、Hann窗和FFT常量：编译后约28 KB Flash；
- 当前完整固件：约83 KB Flash。

`G_Export_V3_data.c` 的文本文件约110 KB，是因为浮点常量以十进制文本保存；它不等于实际Flash占用。

## 10. 重新生成代码

1. 打开 `G_Export_V3.slx`。
2. 进入“APP → Embedded Coder”。
3. 确认目标文件为 `ert.tlc`。
4. 确认硬件为 `ARM Compatible → ARM Cortex-M`。
5. 勾选“仅生成代码”。
6. 按 `Ctrl+B`。

也可以在 MATLAB 命令行执行：

```matlab
slbuild('G_Export_V3')
```

需要复制到项目的文件：

```text
G_Export_V3.c
G_Export_V3.h
G_Export_V3_data.c
G_Export_V3_private.h
G_Export_V3_types.h
rtwtypes.h
```

不要加入 `ert_main.c`。

重新复制 `G_Export_V3.c` 后，必须保留 `TASK0729_AXI_RAM` 属性，使
`G_Export_V3_B` 和 `G_Export_V3_U` 位于 `.task0729_ram`。否则会出现 DTCM 溢出。

## 11. 当前项目中的调用位置

当前已经在以下流程中调用：

```text
ScopeApp_ProcessCompletedCapture()
→ AD9220_CopySignedSamples()
→ Task0729_Process()
→ LCD/串口结果结构
```

具体位置在 `Core/Src/scope_app.c`。
