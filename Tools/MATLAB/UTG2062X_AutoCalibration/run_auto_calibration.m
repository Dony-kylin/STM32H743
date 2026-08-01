function run = run_auto_calibration(cfg)
%RUN_AUTO_CALIBRATION Control UTG2062X, collect H743 FFT results and fit.

toolDir = fileparts(mfilename('fullpath'));
addpath(toolDir);
if nargin < 1 || isempty(cfg)
    cfg = auto_calibration_config;
end
validateConfiguration(cfg);

allCases = load_g_test_cases(cfg);
selected = allCases.CaseId >= cfg.FirstCase & ...
    allCases.CaseId <= cfg.LastCase;
cases = allCases(selected, :);
if isempty(cases)
    error('Calibration:NoSelectedCases', ...
        'No cases are in the requested range %d through %d.', ...
        cfg.FirstCase, cfg.LastCase);
end

timestamp = string(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
outputDir = fullfile(cfg.OutputRoot, char(timestamp));
if ~isfolder(outputDir)
    mkdir(outputDir);
end

utg = [];
h743 = [];
try
    [utg, utgIdentity, utgResource] = utg_open(cfg);
    [h743, h743Port] = h743_open(cfg);
catch exception
    if ~isempty(utg)
        utg_shutdown(utg, cfg);
    end
    rethrow(exception);
end
deviceCleanup = onCleanup(@() cleanupDevices(utg, cfg));

if cfg.Verbose
    fprintf('UTG:  %s\n', utgIdentity);
    fprintf('VISA: %s\n', utgResource);
    fprintf('H743: %s\n', h743Port);
    fprintf('Cases: %d\n\n', height(cases));
end

controlSequence = h743_start_stream(h743, uint16(1)); %#ok<NASGU>
pause(0.05);
flush(h743, 'input');

records = repmat(emptyRecord(), height(cases), 1);
for row = 1:height(cases)
    testCase = cases(row, :);
    record = initializeRecord(testCase);
    if cfg.Verbose
        fprintf('[%3d/%3d] Case %d: f0=%.3f kHz, Vpp=%.1f mV', ...
            row, height(cases), testCase.CaseId, ...
            testCase.Fundamental_kHz, testCase.TargetVpp_mV);
    end

    caseTimer = tic;
    lastException = [];
    for attempt = 1:(cfg.RetryCount + 1)
        record.Attempts = attempt;
        try
            verification = utg_apply_case(utg, testCase, cfg);
            pause(cfg.SettleTimeSeconds);

            % Discard all old data and any partial old frame after a signal
            % change. The next accepted frame must match this case's f0.
            flush(h743, 'input');
            parser = h743_parser_init;
            [measurement, parser] = h743_collect_stable( ...
                h743, parser, testCase, cfg);
            record = fillMeasurement(record, measurement, verification, ...
                parser, toc(caseTimer) * 1000, cfg);
            lastException = [];
            break;
        catch exception
            lastException = exception;
            record.Note = string(exception.message);
            if attempt <= cfg.RetryCount
                pause(0.05);
            end
        end
    end

    if ~isempty(lastException)
        record.Success = false;
        record.Stable = false;
        record.CollectionTime_ms = toc(caseTimer) * 1000;
        if cfg.Verbose
            fprintf(' -> FAILED: %s\n', record.Note);
        end
    elseif cfg.Verbose
        fprintf(' -> %.3f kHz, %.2f mVpp, %s\n', ...
            record.MeasuredF0_kHz, record.MeasuredVpp_mV, ...
            stableWord(record.Stable));
    end
    records(row) = record;
    pause(cfg.InterCasePauseSeconds);
end

% Closing outputs before fitting prevents an analysis or plotting failure
% from leaving the generator energized.
clear deviceCleanup
clear h743 utg

results = struct2table(records);
[fitMeasured, fitTarget] = collectFitPairs(results);
calibration = fit_amplitude_calibration(fitMeasured, fitTarget, cfg);
results = attach_corrected_results(results, calibration, cfg);

run.Timestamp = timestamp;
run.Config = cfg;
run.UtgIdentity = string(utgIdentity);
run.UtgResource = string(utgResource);
run.H743Port = string(h743Port);
run.Cases = cases;
run.Results = results;
run.Calibration = calibration;
run.OutputDirectory = string(outputDir);

writetable(results, fullfile(outputDir, ...
    'raw_and_corrected_results.csv'));
writetable(results, fullfile(outputDir, ...
    'raw_and_corrected_results.xlsx'));
save(fullfile(outputDir, 'calibration_run.mat'), 'run');
write_calibration_header(calibration, fullfile(outputDir, ...
    'task0729_auto_calibration_data.h'));
generate_calibration_report(run, fullfile(outputDir, ...
    'calibration_report.md'));
createCalibrationPlot(calibration, fullfile(outputDir, ...
    'amplitude_calibration.png'));

if cfg.Verbose
    fprintf('\nCalibration complete.\n');
    fprintf('Output: %s\n', outputDir);
    fprintf('Raw component MAE:       %.3f mV\n', ...
        calibration.Identity.MeanAbsoluteError_mV);
    fprintf('Corrected component MAE: %.3f mV\n', ...
        calibration.Piecewise.MeanAbsoluteError_mV);
    fprintf('Corrected +/-5mV rate:   %.1f%%\n', ...
        calibration.Piecewise.PassRate5mV * 100);
end
end

function validateConfiguration(cfg)
required = {'TestWorkbook', 'VisaResource', 'H743Port', 'FirstCase', ...
    'LastCase', 'CaseTimeoutSeconds', 'RetryCount', 'OutputRoot'};
for k = 1:numel(required)
    if ~isfield(cfg, required{k})
        error('Calibration:ConfigField', ...
            'Configuration field %s is missing.', required{k});
    end
end
if cfg.CaseTimeoutSeconds <= 0 || cfg.RetryCount < 0
    error('Calibration:ConfigValue', ...
        'Timeout must be positive and RetryCount cannot be negative.');
end
end

function cleanupDevices(utg, cfg)
if ~isempty(utg)
    utg_shutdown(utg, cfg);
end
end

function output = stableWord(stable)
if stable
    output = 'stable';
else
    output = 'UNSTABLE';
end
end

function record = emptyRecord()
record = struct( ...
    'CaseId', 0, 'Question', 0, 'Signal', "", ...
    'TargetVpp_mV', NaN, 'TargetRms_mV', NaN, ...
    'TargetF0_kHz', NaN, 'OrderA', 0, 'RatioA', 0, ...
    'PhaseA_deg', 0, 'OrderB', 0, 'RatioB', 0, ...
    'PhaseB_deg', 0, 'TargetH1Vpk_mV', NaN, ...
    'TargetHAVpk_mV', NaN, 'TargetHBVpk_mV', NaN, ...
    'InterferenceVpp_mV', 0, 'Interference_MHz', 0, ...
    'Success', false, 'Stable', false, 'Attempts', 0, 'Note', "", ...
    'FrameCount', 0, 'AnalysisSequence', NaN, ...
    'ParserCrcErrors', 0, 'ParserFormatErrors', 0, ...
    'MaxAmplitudeCv', NaN, 'CollectionTime_ms', NaN, ...
    'UtgFrequencyQuery_Hz', NaN, 'UtgAmplitudeQuery_Vpp', NaN, ...
    'MeasuredF0_kHz', NaN, 'FrequencyError_kHz', NaN, ...
    'MeasuredVpp_mV', NaN, 'VppError_mV', NaN, ...
    'MeasuredRms_mV', NaN, 'RmsError_mV', NaN, ...
    'ThdPercent', NaN, 'MeasuredH1Vpk_mV', NaN, ...
    'MeasuredHAVpk_mV', NaN, 'MeasuredHBVpk_mV', NaN, ...
    'SettingH1Vpk_mV', NaN, 'SettingHAVpk_mV', NaN, ...
    'SettingHBVpk_mV', NaN, 'H1Error_mV', NaN, ...
    'HAError_mV', NaN, 'HBError_mV', NaN, 'RawPass', false);
end

function record = initializeRecord(testCase)
record = emptyRecord();
record.CaseId = testCase.CaseId;
record.Question = testCase.Question;
record.Signal = testCase.Signal;
record.TargetVpp_mV = testCase.TargetVpp_mV;
record.TargetRms_mV = testCase.TheoreticalRms_mV;
record.TargetF0_kHz = testCase.Fundamental_kHz;
record.OrderA = testCase.OrderA;
record.RatioA = testCase.RatioA;
record.PhaseA_deg = testCase.PhaseA_deg;
record.OrderB = testCase.OrderB;
record.RatioB = testCase.RatioB;
record.PhaseB_deg = testCase.PhaseB_deg;
record.TargetH1Vpk_mV = testCase.BaseTargetVpk_mV;
record.TargetHAVpk_mV = testCase.HarmonicATargetVpk_mV;
record.TargetHBVpk_mV = testCase.HarmonicBTargetVpk_mV;
record.InterferenceVpp_mV = testCase.InterferenceVpp_mV;
record.Interference_MHz = testCase.Interference_MHz;
end

function record = fillMeasurement(record, measurement, verification, ...
    parser, elapsed_ms, cfg)
record.Success = measurement.Valid;
record.Stable = measurement.Stable;
record.Note = measurement.Note;
record.FrameCount = measurement.FrameCount;
record.AnalysisSequence = measurement.AnalysisSequence;
record.ParserCrcErrors = double(parser.CrcErrors);
record.ParserFormatErrors = double(parser.FormatErrors);
record.MaxAmplitudeCv = measurement.MaxAmplitudeCv;
record.CollectionTime_ms = elapsed_ms;
record.UtgFrequencyQuery_Hz = verification.FrequencyHz;
record.UtgAmplitudeQuery_Vpp = verification.AmplitudeVpp;
record.MeasuredF0_kHz = measurement.FundamentalHz / 1000;
record.FrequencyError_kHz = ...
    record.MeasuredF0_kHz - record.TargetF0_kHz;
record.MeasuredVpp_mV = measurement.VppV * 1000;
record.VppError_mV = record.MeasuredVpp_mV - record.TargetVpp_mV;
record.MeasuredRms_mV = measurement.VrmsV * 1000;
record.RmsError_mV = record.MeasuredRms_mV - record.TargetRms_mV;
record.ThdPercent = measurement.ThdPercent;
record.MeasuredH1Vpk_mV = measurement.MeasuredVpk(1) * 1000;
record.MeasuredHAVpk_mV = measurement.MeasuredVpk(2) * 1000;
record.SettingH1Vpk_mV = measurement.SettingVpk(1) * 1000;
record.SettingHAVpk_mV = measurement.SettingVpk(2) * 1000;
if numel(measurement.ExpectedOrders) >= 3
    record.MeasuredHBVpk_mV = measurement.MeasuredVpk(3) * 1000;
    record.SettingHBVpk_mV = measurement.SettingVpk(3) * 1000;
end
record.H1Error_mV = record.MeasuredH1Vpk_mV - record.TargetH1Vpk_mV;
record.HAError_mV = record.MeasuredHAVpk_mV - record.TargetHAVpk_mV;
record.HBError_mV = record.MeasuredHBVpk_mV - record.TargetHBVpk_mV;

limit = cfg.RequiredVoltageError_mV;
record.RawPass = record.Success && ...
    abs(record.VppError_mV) <= limit && ...
    abs(record.RmsError_mV) <= limit && ...
    abs(record.FrequencyError_kHz * 1000) <= ...
        cfg.RequiredFrequencyErrorHz && ...
    abs(record.H1Error_mV) <= limit && ...
    abs(record.HAError_mV) <= limit && ...
    (record.OrderB == 0 || abs(record.HBError_mV) <= limit) && ...
    record.CollectionTime_ms <= cfg.RequiredCompletionTime_ms;
end

function [measured, target] = collectFitPairs(results)
validRows = results.Success;
measured = results.MeasuredH1Vpk_mV(validRows);
target = results.TargetH1Vpk_mV(validRows);
measured = [measured; results.MeasuredHAVpk_mV(validRows)];
target = [target; results.TargetHAVpk_mV(validRows)];
withB = validRows & results.OrderB >= 2;
measured = [measured; results.MeasuredHBVpk_mV(withB)];
target = [target; results.TargetHBVpk_mV(withB)];
valid = isfinite(measured) & isfinite(target) & measured > 0 & target > 0;
measured = measured(valid);
target = target(valid);
end

function createCalibrationPlot(calibration, outputPath)
figureHandle = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [100, 100, 1100, 440]);
cleanup = onCleanup(@() close(figureHandle)); %#ok<NASGU>
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
scatter(calibration.MeasuredInput_mV, calibration.TargetOutput_mV, ...
    18, 'filled', 'MarkerFaceAlpha', 0.45);
hold on;
plot(calibration.KnotMeasuredVpk_mV, ...
    calibration.KnotTargetVpk_mV, 'r-o', 'LineWidth', 1.5);
axis equal;
grid on;
xlabel('H743 measured Vpk (mV)');
ylabel('UTG nominal Vpk (mV)');
title('Amplitude calibration map');

nexttile;
rawError = calibration.MeasuredInput_mV - calibration.TargetOutput_mV;
correctedError = calibration.CorrectedOutput_mV - ...
    calibration.TargetOutput_mV;
plot(rawError, '.', 'DisplayName', 'raw');
hold on;
plot(correctedError, '.', 'DisplayName', 'corrected');
yline(5, '--k');
yline(-5, '--k');
grid on;
xlabel('Sorted component sample');
ylabel('Amplitude error (mV)');
title('Before and after calibration');
legend('Location', 'best');

exportgraphics(figureHandle, outputPath, 'Resolution', 160);
end

