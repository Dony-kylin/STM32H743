function [measurement, parser] = h743_collect_stable( ...
    port, parser, testCase, cfg)
%H743_COLLECT_STABLE Collect bounded, frequency-matched analysis frames.

expectedOrders = [1, testCase.OrderA, testCase.OrderB];
expectedOrders = expectedOrders(expectedOrders >= 1);
expectedF0 = testCase.Fundamental_kHz * 1000;
frequencyLimit = max(cfg.MaxFrequencyErrorHzForAcceptance, ...
    cfg.MaxRelativeFrequencyErrorForAcceptance * expectedF0);

capacity = max(cfg.FramesPerCase * 4, 16);
f0 = nan(capacity, 1);
vpp = nan(capacity, 1);
vrms = nan(capacity, 1);
thd = nan(capacity, 1);
measured = nan(capacity, 3);
setting = nan(capacity, 3);
analysisSequence = nan(capacity, 1);
count = 0;
lastSequence = NaN;
timer = tic;

while toc(timer) < cfg.CaseTimeoutSeconds
    available = port.NumBytesAvailable;
    if available == 0
        pause(0.002);
        continue;
    end
    bytes = read(port, available, 'uint8');
    [parser, frames] = h743_parse_bytes(parser, bytes);

    for frameIndex = 1:numel(frames)
        frame = frames{frameIndex};
        if frame.Type ~= hex2dec('80') || numel(frame.Payload) ~= 53
            continue;
        end
        decoded = h743_decode_analysis(frame);
        if ~decoded.Valid || decoded.Overrange || ...
                decoded.AnalysisSequence == lastSequence || ...
                abs(decoded.FundamentalHz - expectedF0) > frequencyLimit
            continue;
        end

        oneMeasured = nan(1, 3);
        oneSetting = nan(1, 3);
        allExpectedPresent = true;
        for orderIndex = 1:numel(expectedOrders)
            order = expectedOrders(orderIndex);
            componentIndex = find([decoded.Components.Order] == order, ...
                1, 'first');
            if isempty(componentIndex)
                allExpectedPresent = false;
                break;
            end
            oneMeasured(orderIndex) = ...
                decoded.Components(componentIndex).MeasuredVpk;
            oneSetting(orderIndex) = ...
                decoded.Components(componentIndex).SettingVpk;
        end
        if ~allExpectedPresent
            continue;
        end

        lastSequence = decoded.AnalysisSequence;
        count = count + 1;
        if count > capacity
            f0(1:end-1) = f0(2:end);
            vpp(1:end-1) = vpp(2:end);
            vrms(1:end-1) = vrms(2:end);
            thd(1:end-1) = thd(2:end);
            measured(1:end-1, :) = measured(2:end, :);
            setting(1:end-1, :) = setting(2:end, :);
            analysisSequence(1:end-1) = analysisSequence(2:end);
            count = capacity;
        end
        f0(count) = decoded.FundamentalHz;
        vpp(count) = decoded.VppV;
        vrms(count) = decoded.VrmsV;
        thd(count) = decoded.ThdPercent;
        measured(count, :) = oneMeasured;
        setting(count, :) = oneSetting;
        analysisSequence(count) = decoded.AnalysisSequence;

        if count >= cfg.FramesPerCase
            selected = (count - cfg.FramesPerCase + 1):count;
            [stable, maxCv] = amplitudesStable( ...
                measured(selected, 1:numel(expectedOrders)), cfg);
            if stable
                measurement = summarize(selected, f0, vpp, vrms, thd, ...
                    measured, setting, analysisSequence, true, maxCv, ...
                    toc(timer), expectedOrders);
                return;
            end
        end
    end
end

if count >= cfg.MinimumFramesPerCase
    selected = max(1, count - cfg.FramesPerCase + 1):count;
    [~, maxCv] = amplitudesStable( ...
        measured(selected, 1:numel(expectedOrders)), cfg);
    measurement = summarize(selected, f0, vpp, vrms, thd, ...
        measured, setting, analysisSequence, false, maxCv, ...
        toc(timer), expectedOrders);
    measurement.Note = "Timed out before the amplitude became stable";
    return;
end

error('Calibration:H743Timeout', ...
    ['No matching H743 result was collected within %.2f seconds. ', ...
     'Expected f0 %.3f Hz; decoded frames=%d, CRC errors=%d.'], ...
    cfg.CaseTimeoutSeconds, expectedF0, parser.FramesDecoded, ...
    parser.CrcErrors);
end

function [stable, maxCv] = amplitudesStable(values, cfg)
centers = median(values, 1, 'omitnan');
spread = std(values, 0, 1, 'omitnan');
limits = max(cfg.MaxAmplitudeStdV * ones(size(centers)), ...
    cfg.MaxAmplitudeCoefficientOfVariation .* abs(centers));
valid = isfinite(centers) & centers > 0 & isfinite(spread);
stable = all(valid) && all(spread(valid) <= limits(valid));
cv = spread ./ max(abs(centers), 1e-12);
maxCv = max(cv, [], 'omitnan');
end

function output = summarize(selected, f0, vpp, vrms, thd, measured, ...
    setting, sequence, stable, maxCv, elapsed, expectedOrders)
output.Valid = true;
output.Stable = stable;
output.Note = "";
output.FrameCount = numel(selected);
output.FundamentalHz = median(f0(selected), 'omitnan');
output.VppV = median(vpp(selected), 'omitnan');
output.VrmsV = median(vrms(selected), 'omitnan');
output.ThdPercent = median(thd(selected), 'omitnan');
output.MeasuredVpk = median(measured(selected, :), 1, 'omitnan');
output.SettingVpk = median(setting(selected, :), 1, 'omitnan');
output.ExpectedOrders = expectedOrders;
output.AnalysisSequence = max(sequence(selected), [], 'omitnan');
output.MaxAmplitudeCv = maxCv;
output.CollectionTimeSeconds = elapsed;
end
