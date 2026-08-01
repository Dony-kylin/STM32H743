function report = check_calibration_environment(cfg)
%CHECK_CALIBRATION_ENVIRONMENT Check prerequisites without changing outputs.

if nargin < 1 || isempty(cfg)
    cfg = auto_calibration_config;
end

report.MatlabVersion = string(version);
report.InstrumentControlToolbox = ~isempty(ver('instrument'));
report.HasVisaApi = exist('visadev', 'file') == 2;
report.VisaDevices = table;
report.SerialPorts = strings(0, 1);
report.DetectedH743Ports = strings(0, 1);
report.WorkbookExists = isfile(cfg.TestWorkbook);
report.HighImpedanceOhms = cfg.UtgLoadOhms;

fprintf('MATLAB: %s\n', report.MatlabVersion);
fprintf('Test workbook: %s\n', passFail(report.WorkbookExists));
fprintf('UTG load setting: %g Ohm (high impedance)\n', ...
    report.HighImpedanceOhms);

if report.HasVisaApi
    try
        report.VisaDevices = visadevlist;
        fprintf('VISA devices: %d\n', height(report.VisaDevices));
        disp(report.VisaDevices);
    catch exception
        fprintf('VISA enumeration failed: %s\n', exception.message);
    end
else
    fprintf(['VISA API: MISSING. Install Instrument Control Toolbox and ', ...
        'NI-VISA before running calibration.\n']);
end

try
    report.SerialPorts = string(serialportlist("available"));
    fprintf('Available serial ports: %s\n', ...
        strjoin(report.SerialPorts, ', '));
    report.DetectedH743Ports = h743_detect_ports;
    fprintf('Detected STM32 H743 ports: %s\n', ...
        strjoin(report.DetectedH743Ports, ', '));
catch exception
    fprintf('Serial port enumeration failed: %s\n', exception.message);
end
end

function value = passFail(condition)
if condition
    value = 'OK';
else
    value = 'MISSING';
end
end
