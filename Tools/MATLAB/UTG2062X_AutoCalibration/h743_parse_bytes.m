function [state, frames] = h743_parse_bytes(state, newBytes)
%H743_PARSE_BYTES Incrementally parse arbitrary USB CDC byte chunks.

if ~isfield(state, 'Buffer')
    state = h743_parser_init;
end
state.Buffer = [state.Buffer, uint8(newBytes(:).')]; %#ok<AGROW>
frames = cell(0, 1);

while true
    if numel(state.Buffer) < 2
        return;
    end

    starts = find(state.Buffer(1:end-1) == hex2dec('A5') & ...
        state.Buffer(2:end) == hex2dec('5A'), 1, 'first');
    if isempty(starts)
        if state.Buffer(end) == hex2dec('A5')
            state.Buffer = state.Buffer(end);
        else
            state.Buffer = uint8([]);
        end
        return;
    end
    if starts > 1
        state.Buffer(1:starts-1) = [];
    end
    if numel(state.Buffer) < 8
        return;
    end

    version = state.Buffer(3);
    payloadLength = double(readU16(state.Buffer, 7));
    if version ~= 1 || payloadLength > 128
        state.FormatErrors = state.FormatErrors + 1;
        state.Buffer(1) = [];
        continue;
    end

    frameLength = 8 + payloadLength + 2;
    if numel(state.Buffer) < frameLength
        return;
    end
    candidate = state.Buffer(1:frameLength);
    receivedCrc = readU16(candidate, 9 + payloadLength);
    calculatedCrc = h743_crc16(candidate(3:8 + payloadLength));
    if receivedCrc ~= calculatedCrc
        state.CrcErrors = state.CrcErrors + 1;
        state.Buffer(1) = [];
        continue;
    end

    frame.Type = candidate(4);
    frame.Sequence = readU16(candidate, 5);
    frame.Payload = candidate(9:8 + payloadLength);
    frames{end + 1, 1} = frame; %#ok<AGROW>
    state.FramesDecoded = state.FramesDecoded + 1;
    state.Buffer(1:frameLength) = [];
end
end

function value = readU16(bytes, first)
value = bitor(uint16(bytes(first)), ...
    bitshift(uint16(bytes(first + 1)), 8));
end

