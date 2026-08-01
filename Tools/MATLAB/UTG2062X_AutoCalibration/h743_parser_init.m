function state = h743_parser_init()
%H743_PARSER_INIT Initialize persistent stream parser state.

state.Buffer = uint8([]);
state.CrcErrors = uint32(0);
state.FormatErrors = uint32(0);
state.FramesDecoded = uint32(0);
end

