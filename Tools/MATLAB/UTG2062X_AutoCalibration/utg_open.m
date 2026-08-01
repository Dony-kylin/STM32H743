function [utg, identity, resource] = utg_open(cfg)
%UTG_OPEN Open and verify a UTG2062X VISA/USBTMC instrument.

if ~exist('visadev', 'file')
    error('Calibration:NoInstrumentToolbox', ...
        ['MATLAB visadev is unavailable. Install Instrument Control ', ...
         'Toolbox and NI-VISA.']);
end

requested = string(cfg.VisaResource);
if strlength(requested) > 0
    candidates = requested;
else
    list = visadevlist;
    if isempty(list)
        error('Calibration:NoVisaDevice', ...
            'No VISA device was found. Check NI-VISA and the USB cable.');
    end
    candidates = string(list.ResourceName);
end

lastMessage = "";
for k = 1:numel(candidates)
    candidate = candidates(k);
    device = [];
    try
        device = visadev(candidate);
        device.Timeout = 3;
        configureTerminator(device, "LF");
        flush(device);
        reply = strtrim(string(writeread(device, "*IDN?")));
        if contains(upper(reply), "UTG2062")
            utg = device;
            identity = reply;
            resource = candidate;
            return;
        end
        lastMessage = "Unexpected *IDN? reply from " + candidate + ...
            ": " + reply;
        clear device
    catch exception
        lastMessage = candidate + ": " + string(exception.message);
        if ~isempty(device)
            clear device
        end
    end
end

error('Calibration:UTGNotFound', ...
    'No UTG2062X was identified. Last VISA result: %s', lastMessage);
end

