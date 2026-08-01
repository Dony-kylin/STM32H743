function crc = h743_crc16(bytes)
%H743_CRC16 CRC-16/CCITT-FALSE, polynomial 0x1021, initial value 0xFFFF.

bytes = uint8(bytes(:).');
crc = uint16(hex2dec('FFFF'));
polynomial = uint16(hex2dec('1021'));
for index = 1:numel(bytes)
    crc = bitxor(crc, bitshift(uint16(bytes(index)), 8));
    for bit = 1:8
        if bitand(crc, uint16(hex2dec('8000'))) ~= 0
            crc = bitxor(bitshift(crc, 1), polynomial);
        else
            crc = bitshift(crc, 1);
        end
    end
end
end

