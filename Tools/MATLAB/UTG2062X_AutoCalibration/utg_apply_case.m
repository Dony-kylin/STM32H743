function verification = utg_apply_case(utg, testCase, cfg)
%UTG_APPLY_CASE Configure one generated test case.

commands = utg_build_case_commands(testCase, cfg);
for k = 1:numel(commands)
    writeline(utg, commands(k));
end

verification = struct('FrequencyHz', NaN, 'AmplitudeVpp', NaN);
if cfg.VerifyGeneratorSettings
    main = cfg.UtgMainChannel;
    verification.FrequencyHz = str2double(strtrim(string(writeread(utg, ...
        sprintf(':CHANnel%d:BASE:FREQuency?', main)))));
    verification.AmplitudeVpp = str2double(strtrim(string(writeread(utg, ...
        sprintf(':CHANnel%d:BASE:AMPLitude?', main)))));
    if ~isfinite(verification.FrequencyHz) || ...
            ~isfinite(verification.AmplitudeVpp)
        error('Calibration:UTGQuery', ...
            'UTG returned an invalid frequency or amplitude query result.');
    end
end
end

