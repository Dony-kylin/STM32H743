# Simulink 第二版代码导出

## 模型命名

- `G_Sim_V2.slx`：实际仿真模型，包含信号源、Simscape 前端和显示模块。
- `G_Export_V2.slx`：纯数据处理导出模型，只包含可生成 C 代码的算法。

## 在 Simulink 中生成 C 代码

1. 打开 `G_Export_V2.slx`。
2. 打开“APP”选项卡，选择 **Embedded Coder**。
3. 打开“模型设置 → 代码生成”。
4. 系统目标文件选择 `ert.tlc`。
5. 语言选择 `C`。
6. 硬件选择 `ARM Compatible → ARM Cortex-M`。
7. 勾选“仅生成代码”。
8. 按 `Ctrl+B`，或在 MATLAB 命令行执行：

```matlab
slbuild('G_Export_V2')
```

生成目录为 `G_Export_V2_ert_rtw`。嵌入项目需要以下文件：

- `G_Export_V2.c`
- `G_Export_V2.h`
- `G_Export_V2_data.c`
- `G_Export_V2_private.h`
- `G_Export_V2_types.h`
- `rtwtypes.h`

不要把示例文件 `ert_main.c` 加入 STM32 工程。

## 项目对外接口

应用代码只需要包含：

```c
#include "task0729_processor.h"
```

初始化：

```c
Task0729_Init();
```

处理一帧 16384 点、8 MHz 的 AD9220 数据：

```c
Task0729_Result result;

if (Task0729_Process(samples,
                     TASK0729_MODE_QUESTION_3,
                     1U,
                     &result) != 0U)
{
    /* 使用 result.frequency_hz、amplitude_vpk、vpp、vrms 等结果 */
}
```

`task0729_processor.h/.c` 是稳定包装层。以后重新生成模型代码时，应用层接口不需要改变。

## STM32H743 内存说明

生成模型的 FFT 工作区约 160 KB，超过 H743 的 128 KB DTCM。
项目将 `G_Export_V2_B` 和 `G_Export_V2_U` 放在 AXI SRAM 的
`.task0729_ram` 段中。重新复制生成的 `G_Export_V2.c` 后，需要保留
`TASK0729_AXI_RAM` 属性；否则链接器会报告 DTCM 溢出。
