function T = run_high_speed(varargin)
%RUN_HIGH_SPEED  Scenario S5 (above base speed) for all controllers and field-weakening variants.
%   T = RUN_HIGH_SPEED() runs scenario S5 (100 -> 140 rpm at 0.3 s, 0.25 N.m; base speed of
%   the FOC models 119 rpm at Vdc = 36 V) for the variants below, in parallel with parsim:
%     conv          Conventional FOC
%     comp          Compensated FOC (limiter P.vlim = 0, no field weakening)
%     comp_fw       Compensated FOC + voltage-feedback field weakening (P.FW.on = 1)
%     comp_v3       Compensated FOC, hexagon limiter (P.vlim = 3)
%     comp_v3fw     Compensated FOC, hexagon limiter + field weakening
%     six           Six-step
%     pvc, pvcff    PVC, PVC + feedforward, with the field weakening of the patent (Eq. 9.4)
%     pvcff_nofw    PVC + feedforward without field weakening (PVC.fw = 0)
%   Results: 04_Results/Tables/high_speed.csv (steady-state indicators in SC.win plus
%   i_peak = largest phase current over the whole run) and 04_Results/Raw/S5_high_speed.mat
%   (time series decimated to 0.1 ms for figures).
%
%   Options (name-value): 'Variants' (default all), 'Vdc' (default 36), 'Inverter' (default
%   'ideal'; 'dt2vf' = 2 us dead time + 0.8 V device drops), 'Workers' (default 8).
%
%   See also BLDC_SCENARIO, BLDC_VLIM, RUN_VOLTAGE_LIMIT.

ip = inputParser;
ip.addParameter('Variants', {});
ip.addParameter('Vdc', 36);
ip.addParameter('Inverter', 'ideal');
ip.addParameter('Workers', 8);
ip.parse(varargin{:}); opt = ip.Results;

here = fileparts(mfilename('fullpath')); simRoot = fileparts(here); repo = fileparts(simRoot);
common = fullfile(simRoot, 'Common');
addpath(common, fullfile(simRoot, 'Tests'));
V = struct('name', {'conv', 'comp', 'comp_fw', 'comp_v3', 'comp_v3fw', 'six', 'pvc', 'pvcff', 'pvcff_nofw'}, ...
           'tag',  {'conv', 'comp', 'comp', 'comp', 'comp', 'six', 'pvc', 'pvcff', 'pvcff'}, ...
           'vlim', {0, 0, 0, 3, 3, 0, 0, 0, 0}, ...
           'fw',   {0, 0, 1, 0, 1, 0, 1, 1, 0});
if ~isempty(opt.Variants), V = V(ismember({V.name}, opt.Variants)); end
switch opt.Inverter
    case 'ideal', Td = 0;    Vf = [0 0];
    case 'dt2vf', Td = 2e-6; Vf = [0.8 0.8];
    otherwise, error('run_high_speed: unknown inverter case %s', opt.Inverter);
end
C = bldc_controllers();
if isempty(gcp('nocreate')), parpool(opt.Workers); end
oldDir = pwd; cleanupDir = onCleanup(@() cd(oldDir));
rows = {}; S = struct();
for tag = unique({V.tag}, 'stable')
    c = find(strcmp({C.tag}, tag{1}));
    folder = fullfile(simRoot, C(c).folder); addpath(folder); cd(folder);
    load_system(fullfile(folder, [C(c).model '.slx']));
    W0 = runInit(C(c).init, 'S5');
    Vk = V(strcmp({V.tag}, tag{1}));
    in = Simulink.SimulationInput.empty; Pk = {};
    for k = 1:numel(Vk)
        W = W0;
        W.P = bldc_set_inverter(W.P, Td, Vf);
        W.P.Vdc = opt.Vdc; W.P.Vm = opt.Vdc/sqrt(3);
        W.P.vlim = Vk(k).vlim; W.P.FW.on = Vk(k).fw;
        if isfield(W, 'PVC'), W.PVC.fw = Vk(k).fw; W.PVC.wb = W.P.Vm/(W.P.p*W.P.lam1); end
        vars = cellfun(@(v) {v, W.(v)}, C(c).vars, 'UniformOutput', false);
        in(end+1) = bldc_sim_input(C(c).model, W.SC, vertcat(vars{:})); %#ok<AGROW>
        Pk{end+1} = W.P; %#ok<AGROW>
    end
    so = parsim(in, 'ShowProgress', 'off', 'TransferBaseWorkspaceVariables', 'off', ...
                'SetupFcn', @() addpath(folder, common), 'UseFastRestart', 'off');
    for k = 1:numel(so)
        if ~isempty(so(k).ErrorMessage)
            warning('%s: %s', Vk(k).name, so(k).ErrorMessage); continue
        end
        R = bldc_collect(so(k), C(c).model, W0.SC);
        M = bldc_ss_indicators(R, Pk{k}, W0.SC.win);
        M.i_peak = max(abs(R.i_abc), [], 'all');
        rows(end+1, :) = [{Vk(k).name}, struct2cell(M)']; %#ok<AGROW>
        d = 1:10:numel(R.t);                                   % 0.1 ms
        S.(Vk(k).name) = struct('t', R.t(d), 'w', R.w(d), 'w_ref', R.w_ref(d), 'Te', movmean(R.Te(d), 1), ...
                                'id', R.id(d), 'iq', R.iq(d), 'i_abc', R.i_abc(d, :));
    end
    rmpath(folder);
end
T = cell2table(rows, 'VariableNames', [{'Variant'}, fieldnames(M)']);
sfx = ''; if opt.Vdc ~= 36 || ~strcmp(opt.Inverter, 'ideal'), sfx = sprintf('_Vdc%g_%s', opt.Vdc, opt.Inverter); end
writetable(T, fullfile(repo, '04_Results', 'Tables', ['high_speed' sfx '.csv']));
save(fullfile(repo, '04_Results', 'Raw', ['S5_high_speed' sfx '.mat']), 'S', 'T', 'opt');
disp(T(:, {'Variant', 'w_mean', 'Te_mean', 'Te_rms', 'id_mean', 'Irms', 'Pcu', 'i_peak'}));
end

function W = runInit(initName, SCEN) %#ok<INUSD>
% Run a controller init script in this function's workspace and return its variables.
eval(initName);
v = who;
W = struct();
for k = 1:numel(v), W.(v{k}) = eval(v{k}); end
end
