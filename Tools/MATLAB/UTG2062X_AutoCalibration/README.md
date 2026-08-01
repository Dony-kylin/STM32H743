# UTG2062X + STM32H743 自动校准工具

这套脚本让 MATLAB 同时控制：

- UTG2062X：通过 NI-VISA/USBTMC 接收 SCPI 命令；
- STM32H743：通过 USB CDC 虚拟串口持续上报 FFT 分析结果；
- 100 组测试表：`outputs/g_test_100_20260801/G题_100组测试数据.xlsx`。

主入口是 `run_auto_calibration.m`。第一次使用只需要修改
`auto_calibration_config.m` 中的 `VisaResource` 和 `H743Port`。两项留空时，
脚本会在能够可靠识别的情况下自动选择设备。

## 运行前准备

1. 安装 MATLAB Instrument Control Toolbox 和 NI-VISA。
2. UTG2062X 后面板 USB DEVICE 连接电脑。
3. H743 的 PA11/PA12 USB CDC 连接电脑并烧录当前固件。
4. 信号发生器 CH1 输出接被测输入；脚本默认把 UTG 负载设置为高阻
   （`1,000,000 Ω`），适合目前带 68 kΩ 下拉的 AD9220 输入。只有在 ADC
   输入端确实并接了 50 Ω 终端电阻时，才把 `UtgLoadOhms` 改成 `50`。
5. 题目 3 默认使用 UTG 的 CH1 Merge 功能把 CH2 干扰合并到 CH1。
   如果使用外部合路器，把 `UseInternalMerge` 改为 `false`，并自行接好 CH2。

## 运行

```matlab
cd('工程目录/Tools/MATLAB/UTG2062X_AutoCalibration')
selftest_auto_calibration
check_calibration_environment
run = run_auto_calibration;
```

也可以只测试前 3 组：

```matlab
cfg = auto_calibration_config;
cfg.FirstCase = 1;
cfg.LastCase = 3;
run = run_auto_calibration(cfg);
```

## 输出

每次运行会在 `outputs/utg_auto_calibration/<时间戳>/` 生成：

- `raw_and_corrected_results.csv`：每组原始值、校正值、误差和判定；
- `raw_and_corrected_results.xlsx`：同一份结果的 Excel 版本；
- `calibration_run.mat`：完整 MATLAB 数据和拟合结果；
- `calibration_report.md`：汇总误差、通过率和推荐参数；
- `task0729_auto_calibration_data.h`：可供固件使用的分段线性标定表；
- `amplitude_calibration.png`：校准前后幅值误差图。

脚本不会自动修改或烧录 STM32 固件，也不会自动写 Flash。先检查报告，确认
标定数据合理后，再决定是否把生成的头文件接入固件。

## 安全和防卡死设计

- 所有串口等待都有明确超时；
- 每个测试点失败后只重试有限次数，然后记录失败并继续；
- H743 帧必须同时通过帧头、长度、版本和 CRC16 校验；
- 切换信号后要求频率与当前测试点匹配，避免把上一组结果记入下一组；
- 多帧采用中位数，幅值波动过大时标记为不稳定；
- 无论正常结束还是 MATLAB 报错，清理函数都会尝试关闭 UTG 两路输出。
