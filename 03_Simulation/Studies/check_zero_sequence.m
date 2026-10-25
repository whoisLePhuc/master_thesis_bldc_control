function D = check_zero_sequence(varargin)
%CHECK_ZERO_SEQUENCE  Zero-sequence wind-up of the PVC phase PI controllers with a sensor offset.
%   D = CHECK_ZERO_SEQUENCE() runs PVC + feedforward with a phase-a offset (three phase
%   sensors, BLDC_SET_SENSOR) and logs the outputs of the three phase PI controllers
%   (PVC Ctrl/PI_abc) and the phase-voltage limiter (PVC Ctrl/Sat_v). With three sensors
%   the measured currents do not sum to zero, the sum of the three PI errors is
%   -(off_a + off_b + off_c) and the zero-sequence part of the integrators ramps at
%       dv0/dt = -Ki_i*(off_a + off_b + off_c)/3
%   until the PI outputs reach +/-Vm. The common mode does not reach the motor (min-max
%   modulation removes it) but it uses up the voltage range of Sat_v and the PI limits.
%   Results: 04_Results/Raw/zero_sequence_drift.mat (decimated to 0.1 ms).
%
%   Options: 'Points' [rpm N.m] rows (default [100 1; 30 0.5]), 'Offset' (A, default 0.03),
%            'Sensors' (3 or 2, default 3), 'T' stop time (s, default 3).
%
%   See also RUN_SENSOR_ERROR, BLDC_SET_SENSOR.

ip = inputParser;
ip.addParameter('Points', [100 1; 30 0.5]);
ip.addParameter('Offset', 0.03);
ip.addParameter('Sensors', 3);
ip.addParameter('T', 3);
ip.parse(varargin{:}); opt = ip.Results;
here = fileparts(mfilename('fullpath')); simRoot = fileparts(here); repo = fileparts(simRoot);
addpath(fullfile(simRoot, 'Common'));
oldDir = pwd; cleanupDir = onCleanup(@() cd(oldDir));
m = 'Pseudo_Vector_Control'; cd(fullfile(simRoot, 'PVC')); addpath(pwd);
load_system(m); PVC_FF = 1; pvc_init; %#ok<NASGU>
logOut([m '/PVC Ctrl/PI_abc'], 'v_pi');
logOut([m '/PVC Ctrl/Sat_v'], 'v_sat');
RPM = 2*pi/60;
D = struct('n', {}, 'TL', {}, 't', {}, 'v_pi', {}, 'v_sat', {}, 'w', {}, 'Te', {}, 'rate_theory', {});
for i = 1:size(opt.Points, 1)
    n = opt.Points(i, 1); TL = opt.Points(i, 2);
    SC = struct('T', opt.T, 't_ref', [0 0.02], 'w_ref', [0 n]*RPM, 't_load', [0 0.12], 'TL', [0 TL], ...
                'win', [opt.T - 0.2 opt.T]);
    Pk = bldc_set_sensor(P, [opt.Offset 0 0], [1 1 1], opt.Sensors);
    in = bldc_sim_input(m, SC, {'P', Pk; 'TAB', TAB; 'PVC', PVC; 'SC', SC; 'THETA_OFF', THETA_OFF; 'DECIM', DECIM});
    so = sim(in.setModelParameter('SignalLogging', 'on', 'SignalLoggingName', 'logsout'));
    a = sig(so, 'v_pi'); b = sig(so, 'v_sat');
    R = bldc_collect(so, m, SC);
    d = 1:round(1e-4/(R.t(2) - R.t(1))):numel(R.t);
    Tm = movmean(R.Te, round(P.Ts/(R.t(2) - R.t(1))));
    D(i) = struct('n', n, 'TL', TL, 't', a.t, 'v_pi', a.x, 'v_sat', b.x, 'w', [R.t(d) R.w(d)], ...
                  'Te', [R.t(d) Tm(d)], 'rate_theory', -P.Ki_i*opt.Offset/3);
    fprintf('  %g rpm, %g N.m: v0 at end = %.2f V (theory rate %.1f V/s)\n', n, TL, mean(a.x(end, :)), -P.Ki_i*opt.Offset/3);
end
save(fullfile(repo, '04_Results', 'Raw', 'zero_sequence_drift.mat'), 'D', 'opt');
close_system(m, 0); rmpath(pwd);
end

function logOut(b, name)
ph = get_param(b, 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', name);
end
function s = sig(so, name)
v = so.logsout.getElement(name).Values;
x = squeeze(v.Data); if size(x, 1) ~= numel(v.Time), x = x.'; end
d = 1:max(1, round(1e-4/median(diff(v.Time)))):numel(v.Time);
s = struct('t', v.Time(d), 'x', x(d, :));
end
