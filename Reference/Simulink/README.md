# G题Simulink参考模型

V2 是保留的稳定版本；V3 是当前与 STM32 数据处理代码对应的版本。

## 当前模型

### G_Sim_V3.slx

完整仿真模型，包含：

- G题信号源；
- 2.5 V偏置和4倍前端放大；
- ADC采样；
- 1 MHz以上干扰；
- FIR低通、抽取、FFT和采样后时域测量；
- 频率、谐波幅值、峰峰值和真有效值显示；
- 将选择的1个或3个完整周期重采样为固定600点。

此模型用于验证完整信号链，不直接生成单片机代码。

### G_Export_V3.slx

纯数字处理和代码导出模型，包含：

- `int16[16384]` ADC数据输入；
- 64阶FIR低通和4倍抽取；
- 4096点Hann窗FFT；
- 频率、谐波次数和幅值提取；
- 峰峰值和真有效值测量；
- 1个或3个完整周期固定输出为 `waveform[600]`；
- 前端4倍增益的最终补偿。

模型只接收 `adc_block`、`mode`、`periods`，没有仿真信号源的
`scale` 输入。此模型通过 Embedded Coder 生成项目中的
`G_Export_V3.c/.h`。

## 历史参考

`G_Sim_V2.slx` 和 `G_Export_V2.slx` 是修改 V3 前保留的稳定版本。

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
