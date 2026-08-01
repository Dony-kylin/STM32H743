function cfg = auto_calibration_config()
%AUTO_CALIBRATION_CONFIG User-editable configuration.

toolDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(fileparts(toolDir)));

cfg.ProjectRoot = projectRoot;
cfg.TestWorkbook = fullfile(projectRoot, 'outputs', ...
    'g_test_100_20260801', 'G题_100组测试数据.xlsx');
cfg.TestSheetIndex = 2;

% Leave empty to scan VISA devices and select the one whose *IDN? contains
% UTG2062.  Supplying the resource explicitly is faster and safer.
cfg.VisaResource = "";

% Example: "COM7".  When empty, automatic selection is allowed only if
% exactly one serial port is available.
cfg.H743Port = "";
cfg.H743BaudRate = 115200;

cfg.FirstCase = 1;
cfg.LastCase = 100;

cfg.UtgMainChannel = 1;
cfg.UtgInterferenceChannel = 2;
% AD9220 VINA is normally high impedance (the 68 kOhm pull-down is still
% high-Z relative to the generator's 50 Ohm source). UTG uses 1,000,000 Ohm
% to represent high impedance. Change this to 50 only when a real 50 Ohm
% termination is physically installed at the ADC input.
cfg.UtgLoadOhms = 1000000;
cfg.UseInternalMerge = true;
cfg.KeepOutputOnAfterRun = false;
cfg.VerifyGeneratorSettings = true;

cfg.SettleTimeSeconds = 0.45;
cfg.FramesPerCase = 7;
cfg.MinimumFramesPerCase = 4;
cfg.CaseTimeoutSeconds = 4.0;
cfg.RetryCount = 2;
cfg.InterCasePauseSeconds = 0.03;

cfg.MaxFrequencyErrorHzForAcceptance = 2500;
cfg.MaxRelativeFrequencyErrorForAcceptance = 0.03;
cfg.MaxAmplitudeCoefficientOfVariation = 0.025;
cfg.MaxAmplitudeStdV = 0.0015;

cfg.RequiredFrequencyErrorHz = 1000;
cfg.RequiredVoltageError_mV = 5;
cfg.RequiredCompletionTime_ms = 2000;

cfg.CurrentFrontendGain = 3.9446;
cfg.MaxCalibrationKnots = 12;

cfg.OutputRoot = fullfile(projectRoot, 'outputs', ...
    'utg_auto_calibration');
cfg.Verbose = true;
end
