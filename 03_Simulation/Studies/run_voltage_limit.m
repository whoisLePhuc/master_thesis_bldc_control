function T = run_voltage_limit(varargin)
%RUN_VOLTAGE_LIMIT  Compensated FOC with different voltage limiters vs PVC + feedforward.
%   T = RUN_VOLTAGE_LIMIT() checks whether the advantage of PVC + feedforward at
%   100 rpm, Vdc = 36 V with dead time and device drops (Theory doc 07, Sec. 9.7.4)
%   comes from the voltage limiter of Compensated FOC. Compensated FOC is run with
%   every limiter of BLDC_VLIM (P.vlim = 0..3), PVC + feedforward with its phase
%   limiter, at each (speed, load, Vdc, inverter case), in parallel with parsim.
%   Results: 04_Results/Tables/voltage_limit.csv (one row per run).
%
%   Each run: speed step 0 -> n at 0.02 s, load step 0 -> TL at 0.12 s,
%   evaluation window [0.25, 0.25 + k/fe] (k whole electrical periods, at least 0.1 s).
%
%   Options (name-value):
%     'Modes'   limiters of Compensated FOC (default 0:3), see BLDC_VLIM
%     'Speeds'  rpm  (default [80 100])
%     'Loads'   N.m  (default [0.25 0.5 1])
%     'Vdc'     V    (default [36 48])
%     'Cases'   inverter cases (default {'ideal','dt2vf'}), as RUN_SWEEP_SPEED_LOAD
%     'Workers' parallel pool size (default 8)
%
%   See also BLDC_VLIM, CHECK_VOLTAGE_SATURATION, RUN_SWEEP_SPEED_LOAD.

ip = inputParser;
ip.addParameter('Modes', 0:3);
ip.addParameter('Speeds', [80 100]);
ip.addParameter('Loads', [0.25 0.5 1]);
ip.addParameter('Vdc', [36 48]);
ip.addParameter('Cases', {'ideal', 'dt2vf'});
ip.addParameter('Workers', 8);
ip.parse(varargin{:}); opt = ip.Results;

here = fileparts(mfilename('fullpath')); simRoot = fileparts(here); repo = fileparts(simRoot);
common = fullfile(simRoot, 'Common');
addpath(common, fullfile(simRoot, 'Tests'));
CASES = struct('name', {'ideal', 'dt1', 'dt2', 'dt2vf'}, 'Td', {0, 1e-6, 2e-6, 2e-6}, ...
               'Vf', {[0 0], [0 0], [0 0], [0.8 0.8]});
CASES = CASES(ismember({CASES.name}, opt.Cases));
C = bldc_controllers(); C = C(ismember({C.tag}, {'comp', 'pvcff'}));
if isempty(gcp('nocreate')), parpool(opt.Workers); end
RPM = 2*pi/60;
oldDir = pwd; cleanupDir = onCleanup(@() cd(oldDir));
rows = {};
for c = 1:numel(C)
    folder = fullfile(simRoot, C(c).folder); addpath(folder); cd(folder);
    load_system(fullfile(folder, [C(c).model '.slx']));
    W0 = runInit(C(c).init, 'S2');
    if strcmp(C(c).tag, 'comp'), modes = opt.Modes; else, modes = NaN; end
    in = Simulink.SimulationInput.empty; meta = {};
    for n = opt.Speeds
        fe = W0.P.p*n/60; Tw = max(1, ceil(0.1*fe))/fe;
        SC = struct('T', 0.25 + Tw, 't_ref', [0 0.02], 'w_ref', [0 n]*RPM, ...
                    't_load', [0 0.12], 'TL', [0 0], 'win', [0.25 0.25 + Tw]);
        for TL = opt.Loads
            SC.TL = [0 TL];
            for vd = opt.Vdc
                for k = 1:numel(CASES)
                    for md = modes
                        W = W0; W.SC = SC;
                        W.P = bldc_set_inverter(W.P, CASES(k).Td, CASES(k).Vf);
                        W.P.Vdc = vd; W.P.Vm = vd/sqrt(3);
                        if isfield(W, 'PVC'), W.PVC.wb = W.P.Vm/(W.P.p*W.P.lam1); end
                        if ~isnan(md), W.P.vlim = md; end
                        vars = cellfun(@(v) {v, W.(v)}, C(c).vars, 'UniformOutput', false);
                        in(end+1) = bldc_sim_input(C(c).model, SC, vertcat(vars{:})); %#ok<AGROW>
                        meta(end+1, :) = {n, TL, vd, CASES(k).name, md, SC, W.P}; %#ok<AGROW>
                    end
                end
            end
        end
    end
    fprintf('%s: %d runs\n', C(c).tag, numel(in));
    so = parsim(in, 'ShowProgress', 'off', 'TransferBaseWorkspaceVariables', 'off', ...
                'SetupFcn', @() addpath(folder, common), 'UseFastRestart', 'off');
    for r = 1:numel(so)
        if ~isempty(so(r).ErrorMessage)
            warning('%s run %d: %s', C(c).tag, r, so(r).ErrorMessage); continue
        end
        R = bldc_collect(so(r), C(c).model, meta{r, 6});
        M = bldc_ss_indicators(R, meta{r, 7}, meta{r, 6}.win);
        if isnan(meta{r, 5}), lbl = 'pvcff'; else, lbl = sprintf('comp_v%d', meta{r, 5}); end
        rows(end+1, :) = [{lbl, meta{r, 1}, meta{r, 2}, meta{r, 3}, meta{r, 4}}, struct2cell(M)']; %#ok<AGROW>
    end
    rmpath(folder);
end
T = cell2table(rows, 'VariableNames', [{'Controller', 'Speed_rpm', 'Load_Nm', 'Vdc', 'Case'}, fieldnames(M)']);
writetable(T, fullfile(repo, '04_Results', 'Tables', 'voltage_limit.csv'));
fprintf('run_voltage_limit: %d runs -> voltage_limit.csv\n', height(T));
end

function W = runInit(initName, SCEN) %#ok<INUSD>
% Run a controller init script in this function's workspace and return its variables.
eval(initName);
v = who;
W = struct();
for k = 1:numel(v), W.(v{k}) = eval(v{k}); end
end
