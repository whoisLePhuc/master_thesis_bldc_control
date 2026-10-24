%% RUN_COMP_FOC  Run Compensated_FOC.slx for scenarios S1-S4 and save the results
%  Results: <repo>/04_Results/Raw/<SCEN>_comp.mat, one file per scenario, containing
%     R    : struct with time series (t, w_ref, TL, w, Te, i_abc, id, iq, iq_ref, iq_cmd)
%            iq_ref = speed-PI output i_q*, iq_cmd = shaped current reference i_q** = k_s*i_q*
%     P, TAB, SC, sc_name, THETA_OFF, TAB_SHIFT, meta
%
%  Usage:
%     >> run_comp_foc                       % all scenarios
%     >> SCEN_LIST = {'S2'}; run_comp_foc   % selected scenarios only
%     >> DO_CALIB = true; run_comp_foc      % first tune TAB_SHIFT (7 short runs, saved to
%                                           % comp_foc_calib.mat), then run the scenarios

here = fileparts(mfilename('fullpath'));
mdl  = 'Compensated_FOC';
outDir = fullfile(here, '..', '..', '04_Results', 'Raw');
if ~exist(outDir, 'dir'), mkdir(outDir); end

if ~exist('SCEN_LIST', 'var'), SCEN_LIST = {'S1', 'S2', 'S3', 'S4'}; end
if ~exist('DO_CALIB', 'var'),  DO_CALIB = false; end
oldDir = cd(here);                 % so that comp_foc_init and bldc_trap are found
cleanupDir = onCleanup(@() cd(oldDir));
load_system(fullfile(here, [mdl '.slx']));

% ---------------- optional calibration of TAB_SHIFT ----------------
%  Sweep the phase shift of the back-EMF tables in +/-15 deg (CAL scenario: 100 rpm,
%  1 N.m from 0.1 s) and keep the value giving the smallest peak-to-peak torque ripple.
if DO_CALIB
    SCEN = 'CAL'; TAB_SHIFT = 0; comp_foc_init;
    shv = (-15:5:15)*pi/180; rp = inf(size(shv));
    fprintf('--- TAB_SHIFT calibration (%d runs) ---\n', numel(shv));
    for c = 1:numel(shv)
        so = simOnce(mdl, P, TAB, SC, THETA_OFF, shv(c), DECIM);
        Te = getOut(so.yout, [mdl '/Te']); k = Te.Time >= 0.18;
        Tm = movmean(Te.Data(:), round(P.Ts/(DECIM*P.dt)));     % average over one PWM period
        rp(c) = max(Tm(k)) - min(Tm(k));
        fprintf('  TAB_SHIFT = %+5.1f deg -> torque ripple %.3f N.m p-p\n', shv(c)*180/pi, rp(c));
    end
    [~, bs] = min(rp); TAB_SHIFT = shv(bs);
    if bs == 1 || bs == numel(shv)
        warning('run_comp_foc: optimum TAB_SHIFT is at the edge of the sweep (%.0f deg).', TAB_SHIFT*180/pi);
    end
    save(fullfile(here, 'comp_foc_calib.mat'), 'THETA_OFF', 'TAB_SHIFT');
    fprintf('=> TAB_SHIFT = %.4f rad (%.1f deg), saved to comp_foc_calib.mat\n', TAB_SHIFT, TAB_SHIFT*180/pi);
end

% ---------------- scenarios ----------------
for iSc = 1:numel(SCEN_LIST)
    SCEN = SCEN_LIST{iSc};
    comp_foc_init;                 % creates P, TAB, SC, sc_name, THETA_OFF, TAB_SHIFT, DECIM

    tic; so = simOnce(mdl, P, TAB, SC, THETA_OFF, TAB_SHIFT, DECIM); tSim = toc;

    % ---- collect outputs (root Outports, found by block path) ----
    %  w, Te, i_abc are logged every DECIM*P.dt (10 us); idq, iq_cmd at P.Ts; iq_ref at P.Tsw.
    %  Slower signals are held (zero-order) onto the 10 us time base.
    y  = so.yout;
    gv = @(name) getOut(y, [mdl '/' name]);
    w  = gv('w');
    R = struct();
    R.t      = w.Time;
    R.w      = w.Data;
    R.Te     = holdOn(gv('Te'),    R.t);
    R.i_abc  = holdOn(gv('i_abc'), R.t);
    idq      = holdOn(gv('idq'),   R.t);
    R.id     = idq(:, 1);
    R.iq     = idq(:, 2);
    R.iq_ref = holdOn(gv('iq_ref'), R.t);
    R.iq_cmd = holdOn(gv('iq_cmd'), R.t);
    R.w_ref  = interp1(SC.t_ref,  SC.w_ref, R.t, 'previous', 'extrap');
    R.TL     = interp1(SC.t_load, SC.TL,    R.t, 'previous', 'extrap');

    meta = struct('model', mdl, 'controller', 'Compensated FOC', 'scenario', SCEN, ...
                  'date', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), 'matlab', version, ...
                  'sim_time_s', tSim, 'log_step_s', DECIM*P.dt);
    f = fullfile(outDir, sprintf('%s_comp.mat', SCEN));
    f = char(java.io.File(f).getCanonicalPath());
    save(f, 'R', 'P', 'TAB', 'SC', 'sc_name', 'THETA_OFF', 'TAB_SHIFT', 'meta');
    fprintf('  %s (%s) done in %.0f s -> %s\n', SCEN, sc_name, tSim, f);
end
clear SCEN_LIST DO_CALIB

%% ---------------- local functions ----------------
function so = simOnce(mdl, P, TAB, SC, THETA_OFF, TAB_SHIFT, DECIM)
% One simulation run with the given parameters (reference and load as step inputs).
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(stepSignal(SC.t_ref,  SC.w_ref, SC.T, 'w_ref'), 'w_ref');
ds = ds.addElement(stepSignal(SC.t_load, SC.TL,    SC.T, 'TL'),    'TL');
in = Simulink.SimulationInput(mdl);
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime', num2str(SC.T));
vars = {'P', P; 'TAB', TAB; 'SC', SC; 'THETA_OFF', THETA_OFF; 'TAB_SHIFT', TAB_SHIFT; 'DECIM', DECIM};
for k = 1:size(vars, 1), in = in.setVariable(vars{k,1}, vars{k,2}); end
so = sim(in);
if ~isempty(so.ErrorMessage), error('run_comp_foc: %s', so.ErrorMessage); end
end

function ts = getOut(y, blockPath)
% Return the logged timeseries of the root Outport with the given block path.
for k = 1:y.numElements
    e = y.getElement(k);
    if strcmp(e.BlockPath.getBlock(1), blockPath), ts = e.Values; return; end
end
error('run_comp_foc: output "%s" not found in yout', blockPath);
end

function ts = stepSignal(t, v, T, name)
% Piecewise-constant input for a root Inport with Interpolate = off.
% The last value is repeated after the stop time, otherwise the Inport
% outputs 0 once the final data point has passed.
t = t(:); v = v(:);
keep = t <= T;                        % drop padding points beyond the stop time
t = [t(keep); T + 1];  v = [v(keep); v(find(keep, 1, 'last'))];
ts = timeseries(v, t, 'Name', name);
end

function x = holdOn(ts, t)
% Zero-order hold of timeseries ts onto time vector t (columns = channels).
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
x = interp1(ts.Time, d, t, 'previous', 'extrap');
end
