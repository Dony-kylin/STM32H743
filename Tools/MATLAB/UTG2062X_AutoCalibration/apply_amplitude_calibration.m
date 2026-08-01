function corrected_mV = apply_amplitude_calibration(measured_mV, calibration)
%APPLY_AMPLITUDE_CALIBRATION Apply the monotonic piecewise-linear map.

shape = size(measured_mV);
x = double(measured_mV(:));
corrected = nan(size(x));
valid = isfinite(x) & x >= 0;
if any(valid)
    corrected(valid) = interp1( ...
        calibration.KnotMeasuredVpk_mV, ...
        calibration.KnotTargetVpk_mV, x(valid), 'linear', 'extrap');
    corrected(valid) = max(corrected(valid), 0);
end
corrected_mV = reshape(corrected, shape);
end

