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
addpath(fullfile(here, '..', 'Common'));

if ~exist('SCEN_LIST', 'var'), SCEN_LIST = {'S1', 'S2', 'S3', 'S4'}; end
if ~exist('DO_CALIB', 'var'),  DO_CALIB = false; end
oldDir = cd(here);                 % so that comp_foc_init is found
cleanupDir = onCleanup(@() cd(oldDir));
load_system(fullfile(here, [mdl '.slx']));

% ---------------- optional calibration of TAB_SHIFT ----------------
%  Sweep the phase shift of the back-EMF tables in +/-15 deg (CAL scenario: 100 rpm,
%  1 N.m from 0.1 s) and keep the value giving the smallest peak-to-peak torque ripple.
if DO_CALIB
    SCEN = 'CAL'; TAB_SHIFT = 0; comp_foc_init;  %#ok<UNRCH> (DO_CALIB may be set before running the script)
    shv = (-15:5:15)*pi/180; rp = inf(size(shv));
    fprintf('--- TAB_SHIFT calibration (%d runs) ---\n', numel(shv));
    for c = 1:numel(shv)
        vars = {'P', P; 'TAB', TAB; 'SC', SC; 'THETA_OFF', THETA_OFF; 'TAB_SHIFT', shv(c); 'DECIM', DECIM};
        Rc = bldc_run_scenario(mdl, SC, vars);
        k  = Rc.t >= 0.18;
        Tm = movmean(Rc.Te, round(P.Ts/(DECIM*P.dt)));          % average over one PWM period
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

    vars = {'P', P; 'TAB', TAB; 'SC', SC; 'THETA_OFF', THETA_OFF; 'TAB_SHIFT', TAB_SHIFT; 'DECIM', DECIM};
    [R, ~, tSim] = bldc_run_scenario(mdl, SC, vars);

    meta = struct('model', mdl, 'controller', 'Compensated FOC', 'scenario', SCEN, ...
                  'date', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), 'matlab', version, ...
                  'sim_time_s', tSim, 'log_step_s', DECIM*P.dt);
    f = char(java.io.File(fullfile(outDir, sprintf('%s_comp.mat', SCEN))).getCanonicalPath());
    save(f, 'R', 'P', 'TAB', 'SC', 'sc_name', 'THETA_OFF', 'TAB_SHIFT', 'meta');
    fprintf('  %s (%s) done in %.0f s -> %s\n', SCEN, sc_name, tSim, f);
end
clear SCEN_LIST DO_CALIB
