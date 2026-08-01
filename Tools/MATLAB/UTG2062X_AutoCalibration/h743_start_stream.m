function sequence = h743_start_stream(port, sequence)
%H743_START_STREAM Request continuous latest-result streaming.

if nargin < 2
    sequence = uint16(1);
end
frame = h743_encode_frame(hex2dec('01'), sequence, uint8([]));
write(port, frame, 'uint8');
sequence = uint16(sequence + 1);
end

