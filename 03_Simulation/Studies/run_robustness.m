function T = run_robustness(varargin)
%RUN_ROBUSTNESS  Robustness of all controllers to motor mismatch and position-sensor errors.
%   T = RUN_ROBUSTNESS() simulates every controller registered in Tests/bldc_controllers.m
%   (plus Compensated FOC with the hexagon limiter, P.vlim = 3, tag 'comp_v3') with each
%   case below at each operating point, in parallel with parsim. The controllers keep
%   their nominal parameters; only the motor block (BLDC_SET_PLANT) or the position
%   sensor (BLDC_SET_ENCODER) changes.
%     nominal                       no deviation
%     flat100 ... flat140           motor back-EMF flat top 100/110/130/140 deg (tables: 120 deg)
%     lam0.9, lam1.1                motor flux linkage -10 %, +10 %
%     L0.8, L1.2                    motor inductance -20 %, +20 %
%     R1.3                          motor resistance +30 % (hot winding)
%     enc256, enc1024, enc4096      encoder counts per mechanical revolution
%     off+5, off-5, off+10, off-10  encoder mounting offset (electrical deg)
%   Results: 04_Results/Tables/robustness.csv (one row per run, indicators of
%   BLDC_SS_INDICATORS plus T1 = fe torque component, i_peak).
%
%   Each run: speed step 0 -> n at 0.02 s, load step 0 -> TL at 0.12 s, evaluation window
%   [0.25, 0.25 + k/fe] (k whole electrical periods, at least 0.1 s).
%
%   Options (name-value):
%     'Controllers'  tags (default: all registered + 'comp_v3')
%     'Cases'        case names (default: all)
%     'Points'       [rpm N.m] rows (default [100 1; 30 0.5])
%     'Inverter'     'ideal' (default) or 'dt2vf' (2 us dead time + 0.8 V device drops)
%     'Workers'      parallel pool size (default 8)
%     'File'         output file (default 'robustness.csv')
%
%   See also BLDC_SET_PLANT, BLDC_SET_ENCODER, RUN_SENSOR_ERROR.

ip = inputParser;
ip.addParameter('Controllers', {});
ip.addParameter('Cases', {});
ip.addParameter('Points', [100 1; 30 0.5]);
ip.addParameter('Inverter', 'ideal');
ip.addParameter('Workers', 8);
ip.addParameter('File', 'robustness.csv');
ip.parse(varargin{:}); opt = ip.Results;

here = fileparts(mfilename('fullpath')); simRoot = fileparts(here); repo = fileparts(simRoot);
common = fullfile(simRoot, 'Common');
addpath(common, fullfile(simRoot, 'Tests'));

% ---------------- cases: {name, plant name-value pairs, [N_enc, offset_e (deg)]} ----------------
K = {'nominal', {}, [0 0]
     'flat100', {'flat', 100}, [0 0]
     'flat110', {'flat', 110}, [0 0]
     'flat130', {'flat', 130}, [0 0]
     'flat140', {'flat', 140}, [0 0]
     'lam0.9',  {'lam', 0.9},  [0 0]
     'lam1.1',  {'lam', 1.1},  [0 0]
     'L0.8',    {'L', 0.8},    [0 0]
     'L1.2',    {'L', 1.2},    [0 0]
     'R1.3',    {'R', 1.3},    [0 0]
     'enc256',  {}, [256 0]
     'enc1024', {}, [1024 0]
     'enc4096', {}, [4096 0]
     'off+5',   {}, [0 5]
     'off-5',   {}, [0 -5]
     'off+10',  {}, [0 10]
     'off-10',  {}, [0 -10]};
if ~isempty(opt.Cases), K = K(ismember(K(:, 1), opt.Cases), :); end
switch opt.Inverter
    case 'ideal', Td = 0;    Vf = [0 0];
    case 'dt2vf', Td = 2e-6; Vf = [0.8 0.8];
    otherwise, error('run_robustness: unknown inverter case %s', opt.Inverter);
end

C = bldc_controllers();
V = struct('name', {C.tag}, 'tag', {C.tag}, 'vlim', 0);
V(end+1) = struct('name', 'comp_v3', 'tag', 'comp', 'vlim', 3);
if ~isempty(opt.Controllers), V = V(ismember({V.name}, opt.Controllers)); end
if isempty(gcp('nocreate')), parpool(opt.Workers); end
RPM = 2*pi/60;
oldDir = pwd; cleanupDir = onCleanup(@() cd(oldDir));
rows = {};
for tag = unique({V.tag}, 'stable')
    c = find(strcmp({C.tag}, tag{1}));
    folder = fullfile(simRoot, C(c).folder); addpath(folder); cd(folder);
    load_system(fullfile(folder, [C(c).model '.slx']));
    W0 = runInit(C(c).init, 'S2');
    Vk = V(strcmp({V.tag}, tag{1}));
    in = Simulink.SimulationInput.empty; meta = {};
    for v = 1:numel(Vk)
        for ip_ = 1:size(opt.Points, 1)
            n = opt.Points(ip_, 1); TL = opt.Points(ip_, 2);
            fe = W0.P.p*n/60; Tw = max(1, ceil(0.1*fe))/fe;
            SC = struct('T', 0.25 + Tw, 't_ref', [0 0.02], 'w_ref', [0 n]*RPM, ...
                        't_load', [0 0.12], 'TL', [0 TL], 'win', [0.25 0.25 + Tw]);
            for k = 1:size(K, 1)
                W = W0; W.SC = SC;
                W.P = bldc_set_inverter(W.P, Td, Vf);
                W.P.vlim = Vk(v).vlim;
                W.P = bldc_set_plant(W.P, K{k, 2}{:});
                W.P = bldc_set_encoder(W.P, K{k, 3}(1), K{k, 3}(2)*pi/180);
                vars = cellfun(@(x) {x, W.(x)}, C(c).vars, 'UniformOutput', false);
                in(end+1) = bldc_sim_input(C(c).model, SC, vertcat(vars{:})); %#ok<AGROW>
                meta(end+1, :) = {Vk(v).name, n, TL, K{k, 1}, SC, W.P}; %#ok<AGROW>
            end
        end
    end
    fprintf('%s: %d runs\n', tag{1}, numel(in));
    so = parsim(in, 'ShowProgress', 'off', 'TransferBaseWorkspaceVariables', 'off', ...
                'SetupFcn', @() addpath(folder, common), 'UseFastRestart', 'off');
    for r = 1:numel(so)
        if ~isempty(so(r).ErrorMessage)
            warning('%s run %d: %s', tag{1}, r, so(r).ErrorMessage); continue
        end
        SC = meta{r, 5}; P = meta{r, 6};
        R = bldc_collect(so(r), C(c).model, SC);
        M = bldc_ss_indicators(R, P, SC.win);
        k = R.t >= SC.win(1) & R.t < SC.win(2);
        Tm = movmean(R.Te, round(P.Ts/(R.t(2) - R.t(1))));
        fe = P.p*abs(mean(R.w(k)))/(2*pi);
        M.T1 = harmAmp(R.t(k), Tm(k), fe);
        M.i_peak = max(abs(R.i_abc(k, :)), [], 'all');
        rows(end+1, :) = [meta(r, 1:4), struct2cell(M)']; %#ok<AGROW>
    end
    rmpath(folder);
end
T = cell2table(rows, 'VariableNames', [{'Controller', 'Speed_rpm', 'Load_Nm', 'Case'}, fieldnames(M)']);
writetable(T, fullfile(repo, '04_Results', 'Tables', opt.File));
fprintf('run_robustness: %d runs -> %s\n', height(T), opt.File);
end

function W = runInit(initName, SCEN) %#ok<INUSD>
% Run a controller init script in this function's workspace and return its variables.
eval(initName);
v = who;
W = struct();
for k = 1:numel(v), W.(v{k}) = eval(v{k}); end
end

function A = harmAmp(t, x, f)
x = x - mean(x);
A = 2/numel(x)*abs(sum(x.*exp(-1j*2*pi*f*t)));
end
