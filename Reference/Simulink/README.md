# G题 Simulink 参考模型

V2 是保留的稳定版本；V3 是当前与 STM32 数据处理代码对应的版本。

## G_Sim_V3.slx

完整仿真模型，包含：

- 基波与谐波合成后，按最大绝对峰值归一化到基波设定 Vpeak；
- 归一化后再叠加不受缩放的高频干扰；
- 2.5 V偏置、4倍前端放大和ADC采样；
- FIR低通、4倍抽取和4096点Hann窗FFT；
- ADC实际频谱幅值与恢复的发生器设置幅值；
- 峰峰值、真有效值、基频及固定600点的1/3周期波形。

此模型用于验证完整信号链，不直接生成单片机代码。

## G_Export_V3.slx

纯数字处理和代码导出模型：

- 输入为 `int16[16384]`、`mode`、`periods`；
- 没有信号源 `scale` 输入；
- 同时输出 `amplitude_Vpk[3]` 和 `amplitude_SettingVpk[3]`；
- Vpp、Vrms和 `waveform[600]` 始终表示ADC实际输入；
- 通过 Embedded Coder 生成 `G_Export_V3.c/.h`。

## 历史参考

`G_Sim_V2.slx` 和 `G_Export_V2.slx` 保持不变，作为稳定历史版本。

## 对应代码

```text
Expand/Generated/Task0729/
Expand/Inc/task0729_processor.h
Expand/Src/task0729_processor.c
Docs/Task0729_Processor_API.md
```
