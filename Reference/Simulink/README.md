# G题第二版Simulink参考模型

本文件夹保存与 STM32 数据处理代码对应的 Simulink 第二版模型。

## 文件说明

### G_Sim_V2.slx

完整实际仿真模型，包含：

- G题信号源；
- 2.5 V偏置和4倍前端放大；
- ADC采样与限幅；
- 1 MHz以上干扰；
- FIR低通、抽取、FFT和时域测量；
- 频率、谐波幅值、峰峰值和真有效值显示。

此模型用于验证完整信号链，不直接生成单片机代码。

### G_Export_V2.slx

纯数字处理和代码导出模型，包含：

- `int16[16384]` ADC数据输入；
- 64阶FIR低通；
- 4倍抽取；
- 4096点Hann窗FFT；
- 频率、谐波次数和幅值提取；
- 峰峰值、真有效值和周期波形提取；
- 前端4倍增益的最终补偿。

项目的稳定接口层还会使用原始8 MHz数据，对识别出的最多三个频率
分量进行带直流项的联合最小二乘拟合。最终的分量峰值、Vpp和RMS
来自联合拟合，前端实测增益及电压校准参数位于
`Expand/Inc/task0729_processor.h`，无需修改Simulink生成文件。

此模型通过 Embedded Coder 生成项目中的 `G_Export_V2.c/.h`。

## 对应代码

生成代码：

```text
Expand/Generated/Task0729/
```

稳定调用接口：

```text
Expand/Inc/task0729_processor.h
Expand/Src/task0729_processor.c
```

详细接口说明：

```text
Docs/Task0729_Processor_API.md
```
