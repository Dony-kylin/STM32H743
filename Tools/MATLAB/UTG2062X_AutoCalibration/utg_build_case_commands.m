function commands = utg_build_case_commands(testCase, cfg)
%UTG_BUILD_CASE_COMMANDS Build all SCPI commands for one test case.

main = cfg.UtgMainChannel;
interference = cfg.UtgInterferenceChannel;
totalVpp = testCase.TargetVpp_mV / 1000;
f0 = testCase.Fundamental_kHz * 1000;

orders = testCase.OrderA;
ratios = testCase.RatioA;
phases = testCase.PhaseA_deg;
if testCase.OrderB >= 2 && testCase.RatioB > 0
    orders(end + 1) = testCase.OrderB; %#ok<AGROW>
    ratios(end + 1) = testCase.RatioB; %#ok<AGROW>
    phases(end + 1) = testCase.PhaseB_deg; %#ok<AGROW>
end
valid = orders >= 2 & orders <= 16 & ratios > 0;
orders = orders(valid);
ratios = ratios(valid);
phases = phases(valid);
if isempty(orders)
    error('Calibration:NoHarmonic', ...
        'Case %d has no valid harmonic.', testCase.CaseId);
end

mask = uint16(0);
for k = 1:numel(orders)
    mask = bitor(mask, bitshift(uint16(1), orders(k) - 2));
end

commands = strings(0, 1);
commands(end + 1) = sprintf(':CHANnel%d:OUTPut OFF', main);
commands(end + 1) = sprintf(':CHANnel%d:MERGe OFF', main);
commands(end + 1) = sprintf(':CHANnel%d:OUTPut OFF', interference);
commands(end + 1) = sprintf(':CHANnel%d:AMPLitude:UNIT VPP', main);
commands(end + 1) = sprintf(':CHANnel%d:LOAD %d', main, cfg.UtgLoadOhms);
commands(end + 1) = sprintf(':CHANnel%d:MODe CONTinue', main);
commands(end + 1) = sprintf(':CHANnel%d:BASE:WAVe HARMonic', main);
commands(end + 1) = sprintf( ...
    ':CHANnel%d:BASE:FREQuency %.12g', main, f0);

% UTG harmonic mode normalizes the combined waveform to BASE:AMPLitude.
% Harmonic amplitudes below preserve the requested ratios before that
% normalization, matching the 100-case workbook model.
commands(end + 1) = sprintf( ...
    ':CHANnel%d:BASE:AMPLitude %.12g', main, totalVpp);
commands(end + 1) = sprintf(':CHANnel%d:BASE:OFFSet 0', main);
commands(end + 1) = sprintf(':CHANnel%d:BASE:PHASe 0', main);
commands(end + 1) = sprintf( ...
    ':CHANnel%d:HARMonic:TOTal:ORDer %d', main, max(orders));
commands(end + 1) = sprintf(':CHANnel%d:HARMonic:TYPe USER', main);
commands(end + 1) = sprintf( ...
    ':CHANnel%d:HARMonic:USER:TYPe #H%04X', main, mask);

for k = 1:numel(orders)
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:HARMonic:ORDer%d:AMPLitude %.12g', ...
        main, orders(k), totalVpp * ratios(k));
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:HARMonic:ORDer%d:PHASe %.12g', ...
        main, orders(k), phases(k));
end

if testCase.Question == 3 && testCase.InterferenceVpp_mV > 0
    intVpp = testCase.InterferenceVpp_mV / 1000;
    intHz = testCase.Interference_MHz * 1e6;
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:AMPLitude:UNIT VPP', interference);
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:LOAD %d', interference, cfg.UtgLoadOhms);
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:MODe CONTinue', interference);
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:BASE:WAVe SINusoid', interference);
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:BASE:FREQuency %.12g', interference, intHz);
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:BASE:AMPLitude %.12g', interference, intVpp);
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:BASE:OFFSet 0', interference);
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:BASE:PHASe 0', interference);
    commands(end + 1) = sprintf( ...
        ':CHANnel%d:OUTPut ON', interference);
    if cfg.UseInternalMerge
        commands(end + 1) = sprintf(':CHANnel%d:MERGe ON', main);
    end
end

commands(end + 1) = sprintf(':CHANnel%d:OUTPut ON', main);
end

