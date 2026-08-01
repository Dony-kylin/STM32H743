function calibration = fit_amplitude_calibration( ...
    measuredVpk_mV, targetVpk_mV, cfg)
%FIT_AMPLITUDE_CALIBRATION Fit measured peak amplitude to nominal peak.

x = double(measuredVpk_mV(:));
y = double(targetVpk_mV(:));
valid = isfinite(x) & isfinite(y) & x > 0 & y > 0;
x = x(valid);
y = y(valid);
if numel(x) < 5
    error('Calibration:InsufficientFitData', ...
        'At least five valid amplitude pairs are required; got %d.', ...
        numel(x));
end

[x, order] = sort(x);
y = y(order);

calibration.PointCount = numel(x);
calibration.GainOnly = dot(x, y) / dot(x, x);
linearCoefficient = [ones(size(x)), x] \ y;
calibration.LinearOffset_mV = linearCoefficient(1);
calibration.LinearGain = linearCoefficient(2);

knotCount = min(cfg.MaxCalibrationKnots, ...
    max(3, floor(sqrt(numel(x)))));
boundaries = round(linspace(1, numel(x) + 1, knotCount + 1));
knotX = zeros(knotCount, 1);
knotY = zeros(knotCount, 1);
used = 0;
for k = 1:knotCount
    first = boundaries(k);
    last = boundaries(k + 1) - 1;
    if last < first
        continue;
    end
    used = used + 1;
    knotX(used) = median(x(first:last));
    knotY(used) = median(y(first:last));
end
knotX = knotX(1:used);
knotY = knotY(1:used);

% Merge duplicate measured knots and force a physically monotonic map.
[uniqueX, ~, groups] = unique(knotX, 'stable');
uniqueY = zeros(size(uniqueX));
for k = 1:numel(uniqueX)
    uniqueY(k) = median(knotY(groups == k));
end
knotX = uniqueX;
knotY = uniqueY;
for k = 2:numel(knotY)
    knotY(k) = max(knotY(k), knotY(k - 1));
end
if knotX(1) > 1e-9
    knotX = [0; knotX];
    knotY = [0; knotY];
end

calibration.KnotMeasuredVpk_mV = knotX;
calibration.KnotTargetVpk_mV = knotY;
calibration.Model = "piecewise-linear";
calibration.InputMinimum_mV = min(x);
calibration.InputMaximum_mV = max(x);

identityPrediction = x;
gainPrediction = x * calibration.GainOnly;
linearPrediction = calibration.LinearOffset_mV + ...
    calibration.LinearGain * x;
piecewisePrediction = apply_amplitude_calibration(x, calibration);

calibration.Identity = errorMetrics(identityPrediction - y);
calibration.Gain = errorMetrics(gainPrediction - y);
calibration.Linear = errorMetrics(linearPrediction - y);
calibration.Piecewise = errorMetrics(piecewisePrediction - y);
calibration.MeasuredInput_mV = x;
calibration.TargetOutput_mV = y;
calibration.CorrectedOutput_mV = piecewisePrediction;
calibration.RecommendedFrontendGain = ...
    cfg.CurrentFrontendGain / calibration.GainOnly;
end

function metrics = errorMetrics(errorValues)
absolute = abs(errorValues);
metrics.MeanError_mV = mean(errorValues);
metrics.MeanAbsoluteError_mV = mean(absolute);
metrics.Rmse_mV = sqrt(mean(errorValues .^ 2));
metrics.MaximumAbsoluteError_mV = max(absolute);
metrics.PassRate5mV = mean(absolute <= 5);
end

