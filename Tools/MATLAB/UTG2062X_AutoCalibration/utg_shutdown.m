function utg_shutdown(utg, cfg)
%UTG_SHUTDOWN Best-effort output shutdown; never masks the original error.

if isempty(utg) || cfg.KeepOutputOnAfterRun
    return;
end
try
    writeline(utg, sprintf(':CHANnel%d:MERGe OFF', ...
        cfg.UtgMainChannel));
    writeline(utg, sprintf(':CHANnel%d:OUTPut OFF', ...
        cfg.UtgMainChannel));
    writeline(utg, sprintf(':CHANnel%d:OUTPut OFF', ...
        cfg.UtgInterferenceChannel));
catch
    % Cleanup must remain non-throwing.
end
end
