function generate_calibration_report(run, outputPath)
%GENERATE_CALIBRATION_REPORT Write a compact Markdown calibration report.

[fileId, message] = fopen(outputPath, 'w');
if fileId < 0
    error('Calibration:ReportWrite', ...
        'Cannot create %s: %s', outputPath, message);
end
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>

results = run.Results;
fit = run.Calibration;
fprintf(fileId, '# UTG2062X + H743 自动校准报告\n\n');
fprintf(fileId, '- 时间：%s\n', char(run.Timestamp));
fprintf(fileId, '- UTG：`%s`\n', char(run.UtgIdentity));
fprintf(fileId, '- VISA：`%s`\n', char(run.UtgResource));
fprintf(fileId, '- H743：`%s`\n', char(run.H743Port));
fprintf(fileId, '- 测试点：%d，成功：%d，稳定：%d\n\n', ...
    height(results), nnz(results.Success), nnz(results.Stable));

fprintf(fileId, '## 幅值拟合\n\n');
fprintf(fileId, '- 有效分量数：%d\n', fit.PointCount);
fprintf(fileId, '- 仅增益系数：%.9g\n', fit.GainOnly);
fprintf(fileId, '- 等效建议前端增益：%.9g（当前 %.9g）\n', ...
    fit.RecommendedFrontendGain, run.Config.CurrentFrontendGain);
fprintf(fileId, '- 分段线性节点数：%d\n\n', ...
    numel(fit.KnotMeasuredVpk_mV));

fprintf(fileId, '| 模型 | MAE/mV | RMSE/mV | 最大误差/mV | ±5mV通过率 |\n');
fprintf(fileId, '|---|---:|---:|---:|---:|\n');
writeMetric(fileId, '未校准', fit.Identity);
writeMetric(fileId, '单增益', fit.Gain);
writeMetric(fileId, '仿射直线', fit.Linear);
writeMetric(fileId, '分段线性', fit.Piecewise);

fprintf(fileId, '\n## 整组判定\n\n');
rawRate = mean(results.RawPass) * 100;
correctedRate = mean(results.CorrectedPass) * 100;
fprintf(fileId, '- 原始整组通过率：%.1f%%\n', rawRate);
fprintf(fileId, '- 校正后整组通过率：%.1f%%\n', correctedRate);
fprintf(fileId, '- 校正头文件只描述“测量峰值 → 信号源设置峰值”的映射。\n');
fprintf(fileId, '- 在烧入固件前，应先检查异常点和校正曲线是否单调合理。\n');
end

function writeMetric(fileId, name, metric)
fprintf(fileId, '| %s | %.4f | %.4f | %.4f | %.1f%% |\n', ...
    name, metric.MeanAbsoluteError_mV, metric.Rmse_mV, ...
    metric.MaximumAbsoluteError_mV, metric.PassRate5mV * 100);
end
