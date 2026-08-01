function selftest_auto_calibration()
%SELFTEST_AUTO_CALIBRATION Offline test; no hardware is accessed.

toolDir = fileparts(mfilename('fullpath'));
addpath(toolDir);
cfg = auto_calibration_config;
cases = load_g_test_cases(cfg);
assert(height(cases) == 100, 'Expected exactly 100 test cases.');
assert(nnz(cases.Question == 1) == 34);
assert(nnz(cases.Question == 2) == 33);
assert(nnz(cases.Question == 3) == 33);

commands = utg_build_case_commands(cases(1, :), cfg);
assert(any(contains(commands, ':BASE:FREQuency 10000')));
assert(any(contains(commands, ':BASE:AMPLitude 0.1')));
assert(any(contains(commands, ':HARMonic:USER:TYPe #H0003')));

payload = zeros(1, 53, 'uint8');
payload(1) = 3;
payload(2) = 2;
payload = putU32(payload, 3, 1);
payload = putU32(payload, 7, 12345);
payload = putU32(payload, 11, 50000000);
payload = putU32(payload, 15, 400000);
payload = putU32(payload, 19, 141421);
payload = putU32(payload, 23, 1234);
payload(27) = 1;
payload = putU32(payload, 28, 200000);
payload = putU32(payload, 32, 210000);
payload(36) = 2;
payload = putU32(payload, 37, 100000);
payload = putU32(payload, 41, 105000);

encoded = h743_encode_frame(hex2dec('80'), hex2dec('1234'), payload);
parser = h743_parser_init;
[parser, firstFrames] = h743_parse_bytes(parser, ...
    [uint8([1, 2, 3]), encoded(1:17)]);
assert(isempty(firstFrames));
[parser, frames] = h743_parse_bytes(parser, encoded(18:end));
assert(numel(frames) == 1);
decoded = h743_decode_analysis(frames{1});
assert(decoded.Valid);
assert(decoded.AnalysisSequence == 12345);
assert(abs(decoded.FundamentalHz - 50000) < 1e-9);
assert(abs(decoded.VppV - 0.4) < 1e-9);
assert(decoded.Components(2).Order == 2);
assert(abs(decoded.Components(2).MeasuredVpk - 0.1) < 1e-9);
assert(parser.CrcErrors == 0);

bad = encoded;
bad(20) = bitxor(bad(20), uint8(1));
[parser, badFrames] = h743_parse_bytes(parser, bad);
assert(isempty(badFrames));
assert(parser.CrcErrors == 1);

target = (5:5:200).';
measured = target * 0.9;
fit = fit_amplitude_calibration(measured, target, cfg);
corrected = apply_amplitude_calibration(measured, fit);
assert(max(abs(corrected - target)) < 1e-9);
assert(abs(fit.GainOnly - 1 / 0.9) < 1e-12);

[vpp, rmsValue] = compose_wave_metrics([200, 100], [1, 2], [0, 0]);
assert(vpp > 400 && vpp < 600);
assert(abs(rmsValue - sqrt((200^2 + 100^2) / 2)) < 1e-9);

fprintf('selftest_auto_calibration: PASS\n');
end

function bytes = putU32(bytes, first, value)
value = uint32(value);
bytes(first) = uint8(bitand(value, 255));
bytes(first + 1) = uint8(bitand(bitshift(value, -8), 255));
bytes(first + 2) = uint8(bitand(bitshift(value, -16), 255));
bytes(first + 3) = uint8(bitand(bitshift(value, -24), 255));
end
