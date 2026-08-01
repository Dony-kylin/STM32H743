function frame = h743_encode_frame(messageType, sequence, payload)
%H743_ENCODE_FRAME Encode one command using the firmware USB protocol.

payload = uint8(payload(:).');
if numel(payload) > 128
    error('Calibration:PayloadTooLarge', ...
        'H743 payload exceeds the protocol limit of 128 bytes.');
end

lengthBytes = littleEndianU16(uint16(numel(payload)));
sequenceBytes = littleEndianU16(uint16(sequence));
body = [uint8(1), uint8(messageType), sequenceBytes, ...
    lengthBytes, payload];
crc = h743_crc16(body);
frame = [uint8([hex2dec('A5'), hex2dec('5A')]), body, ...
    littleEndianU16(crc)];
end

function bytes = littleEndianU16(value)
bytes = uint8([bitand(value, 255), bitshift(value, -8)]);
end

