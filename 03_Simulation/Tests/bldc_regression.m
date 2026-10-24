function T = bldc_regression(varargin)
%BLDC_REGRESSION  Regression test of all controllers against stored reference metrics.
%   T = BLDC_REGRESSION()                    run scenario S2 for every controller
%   T = BLDC_REGRESSION('Scenarios', {'S1','S2','S3','S4'})
%   T = BLDC_REGRESSION('Controllers', {'comp'})
%   T = BLDC_REGRESSION('Update', true)      store the current metrics as the new reference
%   T = BLDC_REGRESSION('RelTol', 0.01)      relative tolerance (default 1 %)
%
%   Each controller in BLDC_CONTROLLERS is simulated with BLDC_RUN_SCENARIO, the metrics of
%   BLDC_METRICS are compared with regression_reference.mat, and a table is printed.
%   A metric passes if |new - ref| <= RelTol*|ref| + AbsTol (AbsTol = 1e-3 in its own unit).
%   The function errors if any metric fails, so it can be used as a quick check after
%   editing Common/ or a model:   >> bldc_regression
%
%   See also BLDC_CONTROLLERS, BLDC_METRICS, BLDC_RUN_SCENARIO.

ip = inputParser;
ip.addParameter('Scenarios', {'S2'});
ip.addParameter('Controllers', {});
ip.addParameter('Update', false);
ip.addParameter('RelTol', 0.01);
ip.addParameter('AbsTol', 1e-3);
ip.parse(varargin{:});
opt = ip.Results;

here    = fileparts(mfilename('fullpath'));
simRoot = fileparts(here);
addpath(fullfile(simRoot, 'Common'));
refFile = fullfile(here, 'regression_reference.mat');

C = bldc_controllers();
if ~isempty(opt.Controllers), C = C(ismember({C.tag}, opt.Controllers)); end
if exist(refFile, 'file'), REF = load(refFile, 'REF'); REF = REF.REF; else, REF = struct(); end

rows = {};
for c = 1:numel(C)
    folder = fullfile(simRoot, C(c).folder);
    addpath(folder);
    load_system(fullfile(folder, [C(c).model '.slx']));
    for s = 1:numel(opt.Scenarios)
        SCEN = opt.Scenarios{s};
        W = runInit(C(c).init, SCEN);
        vars = cellfun(@(n) {n, W.(n)}, C(c).vars, 'UniformOutput', false);
        vars = vertcat(vars{:});
        [R, ~, tSim] = bldc_run_scenario(C(c).model, W.SC, vars);
        M = bldc_metrics(R, W.SC, W.P);
        key = sprintf('%s_%s', SCEN, C(c).tag);
        if opt.Update
            REF.(key) = M;
        end
        names = fieldnames(M);
        for k = 1:numel(names)
            v = M.(names{k});
            if isfield(REF, key) && isfield(REF.(key), names{k})
                r = REF.(key).(names{k});
                ok = (isnan(v) && isnan(r)) || abs(v - r) <= opt.RelTol*abs(r) + opt.AbsTol;
            else
                r = NaN; ok = false;
            end
            rows(end+1, :) = {C(c).tag, SCEN, names{k}, v, r, ok}; %#ok<AGROW>
        end
        fprintf('  %-5s %-3s simulated in %4.1f s\n', C(c).tag, SCEN, tSim);
    end
end

T = cell2table(rows, 'VariableNames', {'Controller', 'Scenario', 'Metric', 'Value', 'Reference', 'Pass'});
if opt.Update
    meta = struct('date', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), 'matlab', version);
    save(refFile, 'REF', 'meta');
    fprintf('Reference metrics updated: %s\n', refFile);
    T.Pass(:) = true;
end
disp(T);
nFail = sum(~T.Pass);
if nFail > 0
    error('bldc_regression: %d of %d metrics outside tolerance.', nFail, height(T));
end
fprintf('bldc_regression: all %d metrics within tolerance (RelTol = %g).\n', height(T), opt.RelTol);
end

function W = runInit(initName, SCEN) %#ok<INUSD>
% Run a controller init script in this function's workspace and return its variables.
eval(initName);
v = who;
W = struct();
for k = 1:numel(v), W.(v{k}) = eval(v{k}); end
end
