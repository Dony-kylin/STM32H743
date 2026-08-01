function cases = load_g_test_cases(cfg)
%LOAD_G_TEST_CASES Read the fixed 100-case workbook without relying on its
%formula cache or localized column names.

if ~isfile(cfg.TestWorkbook)
    error('Calibration:WorkbookMissing', ...
        'Test workbook does not exist: %s', cfg.TestWorkbook);
end

raw = readcell(cfg.TestWorkbook, 'Sheet', cfg.TestSheetIndex);
if size(raw, 1) < 5 || size(raw, 2) < 25
    error('Calibration:WorkbookFormat', ...
        'The test workbook does not contain the expected 25 columns.');
end

rows = 5:size(raw, 1);
caseId = nan(numel(rows), 1);
for k = 1:numel(rows)
    caseId(k) = numericCell(raw{rows(k), 1}, NaN);
end
rows = rows(isfinite(caseId));
count = numel(rows);

CaseId = zeros(count, 1);
Question = zeros(count, 1);
Signal = strings(count, 1);
TargetVpp_mV = zeros(count, 1);
Fundamental_kHz = zeros(count, 1);
OrderA = zeros(count, 1);
RatioA = zeros(count, 1);
PhaseA_deg = zeros(count, 1);
OrderB = zeros(count, 1);
RatioB = zeros(count, 1);
PhaseB_deg = zeros(count, 1);
InterferenceVpp_mV = zeros(count, 1);
Interference_MHz = zeros(count, 1);
ShapeFactor = zeros(count, 1);
BaseTargetVpk_mV = zeros(count, 1);
HarmonicATargetVpk_mV = zeros(count, 1);
HarmonicBTargetVpk_mV = zeros(count, 1);
TheoreticalRms_mV = zeros(count, 1);

for k = 1:count
    r = rows(k);
    CaseId(k) = numericCell(raw{r, 1}, k);
    Question(k) = numericCell(raw{r, 2}, 0);
    Signal(k) = stringCell(raw{r, 3});
    TargetVpp_mV(k) = numericCell(raw{r, 5}, NaN);
    Fundamental_kHz(k) = numericCell(raw{r, 6}, NaN);
    OrderA(k) = numericCell(raw{r, 8}, 0);
    RatioA(k) = numericCell(raw{r, 10}, 0);
    PhaseA_deg(k) = numericCell(raw{r, 11}, 0);
    OrderB(k) = numericCell(raw{r, 12}, 0);
    RatioB(k) = numericCell(raw{r, 14}, 0);
    PhaseB_deg(k) = numericCell(raw{r, 15}, 0);
    InterferenceVpp_mV(k) = numericCell(raw{r, 24}, 0);
    Interference_MHz(k) = numericCell(raw{r, 25}, 0);

    ShapeFactor(k) = waveformShapeFactor( ...
        OrderA(k), RatioA(k), PhaseA_deg(k), ...
        OrderB(k), RatioB(k), PhaseB_deg(k));
    BaseTargetVpk_mV(k) = TargetVpp_mV(k) / ShapeFactor(k);
    HarmonicATargetVpk_mV(k) = BaseTargetVpk_mV(k) * RatioA(k);
    HarmonicBTargetVpk_mV(k) = BaseTargetVpk_mV(k) * RatioB(k);
    TheoreticalRms_mV(k) = BaseTargetVpk_mV(k) * ...
        sqrt((1 + RatioA(k)^2 + RatioB(k)^2) / 2);
end

if any(~isfinite(TargetVpp_mV)) || any(~isfinite(Fundamental_kHz))
    error('Calibration:WorkbookData', ...
        'The test workbook contains invalid target amplitude or frequency.');
end

cases = table(CaseId, Question, Signal, TargetVpp_mV, ...
    Fundamental_kHz, OrderA, RatioA, PhaseA_deg, OrderB, RatioB, ...
    PhaseB_deg, InterferenceVpp_mV, Interference_MHz, ShapeFactor, ...
    BaseTargetVpk_mV, HarmonicATargetVpk_mV, ...
    HarmonicBTargetVpk_mV, TheoreticalRms_mV);
end

function value = numericCell(input, fallback)
if isnumeric(input) && isscalar(input) && isfinite(input)
    value = double(input);
elseif islogical(input) && isscalar(input)
    value = double(input);
else
    value = fallback;
end
end

function value = stringCell(input)
if isstring(input) || ischar(input)
    value = string(input);
else
    value = "";
end
end

function factor = waveformShapeFactor(orderA, ratioA, phaseA, ...
    orderB, ratioB, phaseB)
% Dense deterministic grid; this is performed only once before acquisition.
theta = (0:65535).' * (2 * pi / 65536);
y = sin(theta);
if orderA >= 2 && ratioA > 0
    y = y + ratioA * sin(orderA * theta + deg2rad(phaseA));
end
if orderB >= 2 && ratioB > 0
    y = y + ratioB * sin(orderB * theta + deg2rad(phaseB));
end
factor = max(y) - min(y);
if ~(factor > 0)
    error('Calibration:InvalidWaveform', ...
        'Computed waveform peak-to-peak factor is invalid.');
end
end

