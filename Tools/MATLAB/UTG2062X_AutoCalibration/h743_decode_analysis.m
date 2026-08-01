function result = h743_decode_analysis(frame)
%H743_DECODE_ANALYSIS Decode the 53-byte APP_USB_MSG_ANALYSIS payload.

if frame.Type ~= hex2dec('80') || numel(frame.Payload) ~= 53
    error('Calibration:AnalysisFrameFormat', ...
        'Frame is not a 53-byte H743 analysis message.');
end

p = uint8(frame.Payload);
result.FrameSequence = double(frame.Sequence);
result.Mode = double(p(1));
result.ComponentCount = min(double(p(2)), 3);
result.StatusFlags = double(readU32(p, 3));
result.AnalysisSequence = double(readU32(p, 7));
result.FundamentalHz = double(readU32(p, 11)) / 1000;
result.VppV = double(readU32(p, 15)) / 1e6;
result.VrmsV = double(readU32(p, 19)) / 1e6;
result.ThdPercent = double(readU32(p, 23)) / 1e4;
result.Valid = bitand(uint32(result.StatusFlags), uint32(1)) ~= 0;
result.Overrange = bitand(uint32(result.StatusFlags), uint32(2)) ~= 0;

blank = struct('Order', 0, 'MeasuredVpk', NaN, 'SettingVpk', NaN);
result.Components = repmat(blank, 1, 3);
offsets = [27, 36, 45];
for k = 1:3
    first = offsets(k);
    result.Components(k).Order = double(p(first));
    result.Components(k).MeasuredVpk = ...
        double(readU32(p, first + 1)) / 1e6;
    result.Components(k).SettingVpk = ...
        double(readU32(p, first + 5)) / 1e6;
end
end

function value = readU32(bytes, first)
value = uint32(bytes(first));
value = bitor(value, bitshift(uint32(bytes(first + 1)), 8));
value = bitor(value, bitshift(uint32(bytes(first + 2)), 16));
value = bitor(value, bitshift(uint32(bytes(first + 3)), 24));
end

