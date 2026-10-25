function T = run_sweep_speed_load(varargin)
%RUN_SWEEP_SPEED_LOAD  Steady-state sweep over speed and load for all controllers.
%   T = RUN_SWEEP_SPEED_LOAD() simulates every controller registered in
%   Tests/bldc_controllers.m at each (speed, load, inverter case), in parallel
%   with parsim, and computes the steady-state indicators of BLDC_SS_INDICATORS.
%   Results: 04_Results/Tables/sweep_speed_load.csv (one row per run).
%
%   Each run: speed step 0 -> n at 0.02 s, load step 0 -> TL at 0.12 s,
%   evaluation window [0.25, 0.25 + k/fe] (k whole electrical periods, at least 0.1 s).
%
%   Options (name-value):
%     'Vlim'         voltage limiter of Compensated FOC, P.vlim (default 0 = original, as in Sec. 9.7-9.9;
%                    the model default is 3 since Sec. 10)
%     'Controllers'  tags (default: all registered)
%     'Speeds'       rpm   (default [20 40 60 80 100])
%     'Loads'        N.m   (default [0.25 0.5 1])
%     'Cases'        inverter cases (default {'ideal','dt2vf'}):
%                    ideal (Td = 0, Vf = 0), dt2 (Td = 2 us), dt2vf (Td = 2 us, Vf = 0.8 V)
%     'Workers'      parallel pool size (default: current pool or 8)
%     'Vdc'          DC-link voltage (V, default 36); results of other values are written to
%                    sweep_speed_load_Vdc<value>.csv
%
%   See also BLDC_SS_INDICATORS, RUN_INVERTER_NONIDEAL, BLDC_SET_INVERTER.

ip = inputParser;
ip.addParameter('Controllers', {});
ip.addParameter('Vlim', 0);   % Compensated FOC limiter; 0 (original) reproduces the published tables
ip.addParameter('Speeds', [20 40 60 80 100]);
ip.addParameter('Loads', [0.25 0.5 1]);
ip.addParameter('Cases', {'ideal', 'dt2vf'});
ip.addParameter('Workers', 8);
ip.addParameter('Vdc', 36);
ip.parse(varargin{:}); opt = ip.Results;

here    = fileparts(mfilename('fullpath'));
simRoot = fileparts(here);
repo    = fileparts(simRoot);
common  = fullfile(simRoot, 'Common');
addpath(common, fullfile(simRoot, 'Tests'));
tabDir = fullfile(repo, '04_Results', 'Tables');

CASES = struct('name', {'ideal', 'dt1', 'dt2', 'dt2vf'}, 'Td', {0, 1e-6, 2e-6, 2e-6}, ...
               'Vf', {[0 0], [0 0], [0 0], [0.8 0.8]});
CASES = CASES(ismember({CASES.name}, opt.Cases));
C = bldc_controllers();
if ~isempty(opt.Controllers), C = C(ismember({C.tag}, opt.Controllers)); end
if isempty(gcp('nocreate')), parpool(opt.Workers); end

RPM = 2*pi/60;
oldDir = pwd; cleanupDir = onCleanup(@() cd(oldDir));
rows = {};
for c = 1:numel(C)
    folder = fullfile(simRoot, C(c).folder);
    addpath(folder); cd(folder);
    load_system(fullfile(folder, [C(c).model '.slx']));
    W0 = runInit(C(c).init, 'S2');
    in = Simulink.SimulationInput.empty; meta = {};
    for n = opt.Speeds
        fe = C2fe(n, W0.P);
        Tw = max(1, ceil(0.1*fe))/fe;                       % whole electrical periods, >= 0.1 s
        SC = struct('T', 0.25 + Tw, 't_ref', [0 0.02], 'w_ref', [0 n]*RPM, ...
                    't_load', [0 0.12], 'TL', [0 0], 'win', [0.25 0.25 + Tw]);
        for TL = opt.Loads
            SC.TL = [0 TL];
            for k = 1:numel(CASES)
                W = W0; W.SC = SC;
                W.P = bldc_set_inverter(W.P, CASES(k).Td, CASES(k).Vf);
                W.P.vlim = opt.Vlim;                                % limiter of Compensated FOC (BLDC_VLIM)
                if opt.Vdc ~= W.P.Vdc                          % DC-link voltage and derived limits
                    W.P.Vdc = opt.Vdc; W.P.Vm = opt.Vdc/sqrt(3);
                    if isfield(W, 'PVC'), W.PVC.wb = W.P.Vm/(W.P.p*W.P.lam1); end
                end
                vars = cellfun(@(v) {v, W.(v)}, C(c).vars, 'UniformOutput', false);
                in(end+1) = bldc_sim_input(C(c).model, SC, vertcat(vars{:})); %#ok<AGROW>
                meta(end+1, :) = {n, TL, CASES(k).name, SC, W.P}; %#ok<AGROW>
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
        R = bldc_collect(so(r), C(c).model, meta{r, 4});
        M = bldc_ss_indicators(R, meta{r, 5}, meta{r, 4}.win);
        rows(end+1, :) = [{C(c).tag, C(c).name, meta{r, 1}, meta{r, 2}, meta{r, 3}}, struct2cell(M)']; %#ok<AGROW>
    end
    rmpath(folder);
end
T = cell2table(rows, 'VariableNames', [{'Controller', 'Name', 'Speed_rpm', 'Load_Nm', 'Case'}, fieldnames(M)']);
fname = 'sweep_speed_load.csv';
if opt.Vdc ~= 36, fname = sprintf('sweep_speed_load_Vdc%g.csv', opt.Vdc); end
writetable(T, fullfile(tabDir, fname));
fprintf('run_sweep_speed_load: %d runs -> %s\n', height(T), fullfile(tabDir, fname));
end

% ---------------- local functions ----------------
function fe = C2fe(n_rpm, P)
fe = P.p*n_rpm/60;                 % electrical frequency (Hz)
end

function W = runInit(initName, SCEN) %#ok<INUSD>
% Run a controller init script in this function's workspace and return its variables.
eval(initName);
v = who;
W = struct();
for k = 1:numel(v), W.(v{k}) = eval(v{k}); end
end
