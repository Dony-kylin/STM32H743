function [vpp_mV, rms_mV] = compose_wave_metrics( ...
    amplitudesVpk_mV, orders, phasesDeg)
%COMPOSE_WAVE_METRICS Reconstruct Vpp and RMS from fitted tone peaks.

amplitudes = double(amplitudesVpk_mV(:).');
orders = double(orders(:).');
phases = double(phasesDeg(:).');
valid = isfinite(amplitudes) & amplitudes >= 0 & orders >= 1;
amplitudes = amplitudes(valid);
orders = orders(valid);
phases = phases(valid);
if isempty(amplitudes)
    vpp_mV = NaN;
    rms_mV = NaN;
    return;
end

theta = (0:32767).' * (2 * pi / 32768);
y = zeros(size(theta));
for k = 1:numel(amplitudes)
    y = y + amplitudes(k) * ...
        sin(orders(k) * theta + deg2rad(phases(k)));
end
vpp_mV = max(y) - min(y);
rms_mV = sqrt(sum(amplitudes .^ 2) / 2);
end

