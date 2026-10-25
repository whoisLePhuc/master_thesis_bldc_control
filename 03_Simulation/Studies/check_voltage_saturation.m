function T = check_voltage_saturation(varargin)
%CHECK_VOLTAGE_SATURATION  Voltage-limit activity of Compensated FOC and PVC + feedforward.
%   T = CHECK_VOLTAGE_SATURATION() runs both controllers at 100 rpm, 1 N.m, with
%   2 us dead time and 0.8 V device drops, for Vdc = 36 V and 48 V, logs the
%   voltage limiters and the duty cycles (models are not saved), and reports in
%   the window 0.25-0.40 s:
%     clip_pct   share of samples in which the controller voltage limiter is active
%                (Compensated FOC: VLim block, either axis; PVC: Sat_v of any phase)
%     duty_pct   share of samples with a duty cycle clamped at 0 or 1
%     u_max      largest requested voltage before the limiter (V), limit P.Vm = Vdc/sqrt(3)
%     Te_rms     torque ripple RMS (N.m)
%   Results: 04_Results/Tables/voltage_saturation.csv
%
%   Options: 'Speed' (rpm, default 100), 'Load' (N.m, default 1), 'Vdc' (default [36 48]).

ip = inputParser;
ip.addParameter('Speed', 100); ip.addParameter('Load', 1); ip.addParameter('Vdc', [36 48]);
ip.parse(varargin{:}); opt = ip.Results;
here = fileparts(mfilename('fullpath')); simRoot = fileparts(here); repo = fileparts(simRoot);
addpath(fullfile(simRoot, 'Common'));
RPM = 2*pi/60;
SC = struct('T', 0.40, 't_ref', [0 0.02], 'w_ref', [0 opt.Speed]*RPM, ...
            't_load', [0 0.12], 'TL', [0 opt.Load], 'win', [0.25 0.40]);
oldDir = pwd; cleanupDir = onCleanup(@() cd(oldDir));
rows = {};

% ---------------- Compensated FOC ----------------
m = 'Compensated_FOC'; cd(fullfile(simRoot, m)); addpath(pwd);
load_system(m); comp_foc_init;                        % init after loading (PreLoadFcn runs the default)
logOut([m '/SVPWM'], 'duty');
logOut([m '/Current PI/VLim'], 'ud_sat', 1); logIn([m '/Current PI/VLim'], 'ud_raw', 1);   % limiter P.vlim (BLDC_VLIM)
logOut([m '/Current PI/VLim'], 'uq_sat', 2); logIn([m '/Current PI/VLim'], 'uq_raw', 2);
for vd = opt.Vdc
    Pv = setVdc(P, vd);
    in = bldc_sim_input(m, SC, {'P', Pv; 'TAB', TAB; 'SC', SC; 'THETA_OFF', THETA_OFF; 'TAB_SHIFT', TAB_SHIFT; 'DECIM', DECIM});
    so = sim(in.setModelParameter('SignalLogging', 'on', 'SignalLoggingName', 'logsout'));
    [uq, kq] = sig(so, 'uq_raw', SC.win); uqs = sig(so, 'uq_sat', SC.win);
    ud = sig(so, 'ud_raw', SC.win); uds = sig(so, 'ud_sat', SC.win); D = sig(so, 'duty', SC.win);
    clip = mean(abs(uq - uqs) > 1e-9 | abs(ud - uds) > 1e-9);
    M = bldc_ss_indicators(bldc_collect(so, m, SC), Pv, SC.win);
    rows(end+1, :) = {'comp', vd, 100*clip, 100*mean(any(D <= 1e-6 | D >= 1 - 1e-6, 2)), max(abs([uq; ud])), Pv.Vm, M.Te_rms}; %#ok<AGROW>
    clear kq
end
close_system(m, 0); rmpath(pwd);

% ---------------- PVC + feedforward ----------------
m = 'Pseudo_Vector_Control'; cd(fullfile(simRoot, 'PVC')); addpath(pwd);
load_system(m); PVC_FF = 1; pvc_init; %#ok<NASGU>
logOut([m '/PVC Ctrl'], 'duty');
logOut([m '/PVC Ctrl/Sat_v'], 'v_sat'); logIn([m '/PVC Ctrl/Sat_v'], 'v_raw');
for vd = opt.Vdc
    Pv = setVdc(P, vd); PV = PVC; PV.wb = Pv.Vm/(Pv.p*Pv.lam1);
    in = bldc_sim_input(m, SC, {'P', Pv; 'TAB', TAB; 'PVC', PV; 'SC', SC; 'THETA_OFF', THETA_OFF; 'DECIM', DECIM});
    so = sim(in.setModelParameter('SignalLogging', 'on', 'SignalLoggingName', 'logsout'));
    A = sig(so, 'v_raw', SC.win); B = sig(so, 'v_sat', SC.win); D = sig(so, 'duty', SC.win);
    M = bldc_ss_indicators(bldc_collect(so, m, SC), Pv, SC.win);
    rows(end+1, :) = {'pvcff', vd, 100*mean(any(abs(A - B) > 1e-9, 2)), 100*mean(any(D <= 1e-6 | D >= 1 - 1e-6, 2)), max(abs(A), [], 'all'), Pv.Vm, M.Te_rms}; %#ok<AGROW>
end
close_system(m, 0); rmpath(pwd);

T = cell2table(rows, 'VariableNames', {'Controller', 'Vdc', 'clip_pct', 'duty_pct', 'u_max', 'Vm', 'Te_rms'});
writetable(T, fullfile(repo, '04_Results', 'Tables', 'voltage_saturation.csv'));
disp(T);
end

% ---------------- local functions ----------------
function P = setVdc(P, vd)
P.Vdc = vd; P.Vm = vd/sqrt(3);
P = bldc_set_inverter(P, 2e-6, [0.8 0.8]);
end
function logOut(b, name, k)
if nargin < 3, k = 1; end
ph = get_param(b, 'PortHandles');
set_param(ph.Outport(k), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end
function logIn(b, name, k)
if nargin < 3, k = 1; end
lh = get_param(b, 'LineHandles'); src = get_param(lh.Inport(k), 'SrcPortHandle');
set_param(src, 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end
function [x, k] = sig(so, name, win)
v = so.logsout.getElement(name).Values;
x = squeeze(v.Data); if size(x, 1) ~= numel(v.Time), x = x.'; end
k = v.Time >= win(1) & v.Time < win(2); x = x(k, :);
end
