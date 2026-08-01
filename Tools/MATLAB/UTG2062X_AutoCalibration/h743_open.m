function [port, portName] = h743_open(cfg)
%H743_OPEN Open the H743 USB CDC port with bounded settings.

requested = string(cfg.H743Port);
if strlength(requested) == 0
    available = string(serialportlist("available"));
    if isempty(available)
        error('Calibration:NoSerialPort', ...
            'No available serial port was found for the H743.');
    end
    detected = h743_detect_ports;
    if numel(detected) == 1
        requested = detected(1);
    elseif numel(available) ~= 1
        error('Calibration:SerialPortAmbiguous', ...
            ['H743 USB VID_0483/PID_5740 could not be selected uniquely. ', ...
             'Set cfg.H743Port explicitly. Available ports: %s'], ...
             strjoin(available, ', '));
    else
        requested = available(1);
    end
end

port = serialport(requested, cfg.H743BaudRate, 'Timeout', 0.1);
flush(port);
portName = requested;
end
