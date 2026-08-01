function ports = h743_detect_ports()
%H743_DETECT_PORTS Find STM32 USB CDC VID_0483/PID_5740 COM ports.

ports = strings(0, 1);
if ~ispc
    return;
end

command = ['reg query ', ...
    '"HKLM\SYSTEM\CurrentControlSet\Enum\USB\VID_0483&PID_5740" ', ...
    '/s /v PortName'];
[status, output] = system(command);
if status ~= 0
    return;
end
matches = regexp(output, 'COM\d+', 'match');
if isempty(matches)
    return;
end
ports = unique(string(matches(:)), 'stable');
available = string(serialportlist("available"));
ports = ports(ismember(upper(ports), upper(available)));
end

