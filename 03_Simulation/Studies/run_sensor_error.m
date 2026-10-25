function T = run_sensor_error(varargin)
%RUN_SENSOR_ERROR  All controllers with phase-current measurement errors (offset, gain).
%   T = RUN_SENSOR_ERROR() tests the second argument of the PVC patent (US 7,339,346):
%   errors of the current-measurement circuit of one phase should not be spread to
%   the other phases when the currents are controlled phase by phase. Every
%   controller registered in Tests/bldc_controllers.m is simulated with each sensor
%   case (BLDC_SET_SENSOR) at each operating point, in parallel with parsim.
%   Results: 04_Results/Tables/sensor_error.csv (one row per run).
%
%   Each run: speed step 0 -> n at 0.02 s, load step 0 -> TL at 0.12 s, evaluation window
%   [Tstart, Tstart + k/fe] (k whole electrical periods, at least 0.1 s).
%   Extra indicators (torque averaged over one PWM period): T1, T2 = amplitudes of the
%   fe and 2*fe torque components (N.m), Ia_dc = mean of the true phase-a current (A).
%
%   Options (name-value):
%     'Controllers'  tags (default: all registered)
%     'Points'       [rpm N.m] rows (default [100 1; 30 0.5])
%     'Cases'        sensor cases (default all): ideal, offA3, offA2, gainB3, gainB2
%                    offA*: phase-a offset 'Offset' A, gainB*: phase-b gain 1 + 'GainErr',
%                    *3: three phase sensors, *2: sensors on a and b, i_c = -(i_a + i_b)
%     'Offset'       A (default 0.03, 1 % of Imax)    'GainErr'  (default 0.03)
%     'Tstart'       start of the evaluation window (s, default 0.25)
%     'Inverter'     inverter case (default 'ideal'), as RUN_SWEEP_SPEED_LOAD
%     'Workers'      parallel pool size (default 8)
%     'File'         output file name (default 'sensor_error.csv')
%
%   See also BLDC_SET_SENSOR, RUN_INVERTER_NONIDEAL, RUN_SWEEP_SPEED_LOAD.

ip = inputParser;
ip.addParameter('Controllers', {});
ip.addParameter('Points', [100 1; 30 0.5]);
ip.addParameter('Cases', {'ideal', 'offA3', 'offA2', 'gainB3', 'gainB2'});
ip.addParameter('Offset', 0.03);
ip.addParameter('GainErr', 0.03);
ip.addParameter('Tstart', 0.25);
ip.addParameter('Inverter', 'ideal');
ip.addParameter('Workers', 8);
ip.addParameter('File', 'sensor_error.csv');
ip.parse(varargin{:}); opt = ip.Results;

here = fileparts(mfilename('fullpath')); simRoot = fileparts(here); repo = fileparts(simRoot);
common = fullfile(simRoot, 'Common');
addpath(common, fullfile(simRoot, 'Tests'));
o = opt.Offset; g = 1 + opt.GainErr;
SENS = struct('name', {'ideal', 'offA3', 'offA2', 'gainB3', 'gainB2'}, ...
              'off',  {[0 0 0], [o 0 0], [o 0 0], [0 0 0], [0 0 0]}, ...
              'gain', {[1 1 1], [1 1 1], [1 1 1], [1 g 1], [1 g 1]}, ...
              'n',    {3, 3, 2, 3, 2});
SENS = SENS(ismember({SENS.name}, opt.Cases));
INV = struct('name', {'ideal', 'dt2', 'dt2vf'}, 'Td', {0, 2e-6, 2e-6}, 'Vf', {[0 0], [0 0], [0.8 0.8]});
INV = INV(strcmp({INV.name}, opt.Inverter));
C = bldc_controllers();
if ~isempty(opt.Controllers), C = C(ismember({C.tag}, opt.Controllers)); end
if isempty(gcp('nocreate')), parpool(opt.Workers); end
RPM = 2*pi/60;
oldDir = pwd; cleanupDir = onCleanup(@() cd(oldDir));
rows = {};
for c = 1:numel(C)
    folder = fullfile(simRoot, C(c).folder); addpath(folder); cd(folder);
    load_system(fullfile(folder, [C(c).model '.slx']));
    W0 = runInit(C(c).init, 'S2');
    in = Simulink.SimulationInput.empty; meta = {};
    for ip_ = 1:size(opt.Points, 1)
        n = opt.Points(ip_, 1); TL = opt.Points(ip_, 2);
        fe = W0.P.p*n/60; Tw = max(1, ceil(0.1*fe))/fe;
        SC = struct('T', opt.Tstart + Tw, 't_ref', [0 0.02], 'w_ref', [0 n]*RPM, ...
                    't_load', [0 0.12], 'TL', [0 TL], 'win', [opt.Tstart opt.Tstart + Tw]);
        for k = 1:numel(SENS)
            W = W0; W.SC = SC;
            W.P = bldc_set_inverter(W.P, INV.Td, INV.Vf);
            W.P = bldc_set_sensor(W.P, SENS(k).off, SENS(k).gain, SENS(k).n);
            vars = cellfun(@(v) {v, W.(v)}, C(c).vars, 'UniformOutput', false);
            in(end+1) = bldc_sim_input(C(c).model, SC, vertcat(vars{:})); %#ok<AGROW>
            meta(end+1, :) = {n, TL, SENS(k).name, SC, W.P}; %#ok<AGROW>
        end
    end
    fprintf('%s: %d runs\n', C(c).tag, numel(in));
    so = parsim(in, 'ShowProgress', 'off', 'TransferBaseWorkspaceVariables', 'off', ...
                'SetupFcn', @() addpath(folder, common), 'UseFastRestart', 'off');
    for r = 1:numel(so)
        if ~isempty(so(r).ErrorMessage)
            warning('%s run %d: %s', C(c).tag, r, so(r).ErrorMessage); continue
        end
        SC = meta{r, 4}; P = meta{r, 5};
        R = bldc_collect(so(r), C(c).model, SC);
        M = bldc_ss_indicators(R, P, SC.win);
        k = R.t >= SC.win(1) & R.t < SC.win(2);
        Tm = movmean(R.Te, round(P.Ts/(R.t(2) - R.t(1))));
        fe = P.p*abs(mean(R.w(k)))/(2*pi);
        M.T1 = harmAmp(R.t(k), Tm(k), fe);
        M.T2 = harmAmp(R.t(k), Tm(k), 2*fe);
        M.Ia_dc = mean(R.i_abc(k, 1));
        rows(end+1, :) = [{C(c).tag, meta{r, 1}, meta{r, 2}, meta{r, 3}}, struct2cell(M)']; %#ok<AGROW>
    end
    rmpath(folder);
end
T = cell2table(rows, 'VariableNames', [{'Controller', 'Speed_rpm', 'Load_Nm', 'Sensor'}, fieldnames(M)']);
writetable(T, fullfile(repo, '04_Results', 'Tables', opt.File));
fprintf('run_sensor_error: %d runs -> %s\n', height(T), opt.File);
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
