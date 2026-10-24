%% RUN_CONV_FOC  Run Conventional_FOC.slx for scenarios S1-S4 and save the results
%  Results: <repo>/04_Results/Raw/<SCEN>_conv.mat, one file per scenario, containing
%     R    : struct with time series (t, w_ref, TL, w, Te, i_abc, id, iq, iq_ref, iq_cmd)
%     P, SC, sc_name, THETA_OFF, meta
%
%  Usage:
%     >> run_conv_foc                       % all scenarios
%     >> SCEN_LIST = {'S2'}; run_conv_foc   % selected scenarios only

here = fileparts(mfilename('fullpath'));
mdl  = 'Conventional_FOC';
outDir = fullfile(here, '..', '..', '04_Results', 'Raw');
if ~exist(outDir, 'dir'), mkdir(outDir); end
addpath(fullfile(here, '..', 'Common'));

if ~exist('SCEN_LIST', 'var'), SCEN_LIST = {'S1', 'S2', 'S3', 'S4'}; end
oldDir = cd(here);                 % so that conv_foc_init is found
cleanupDir = onCleanup(@() cd(oldDir));
load_system(fullfile(here, [mdl '.slx']));

for iSc = 1:numel(SCEN_LIST)
    SCEN = SCEN_LIST{iSc};
    conv_foc_init;                 % creates P, SC, sc_name, THETA_OFF, DECIM in this workspace

    vars = {'P', P; 'SC', SC; 'THETA_OFF', THETA_OFF; 'DECIM', DECIM};
    [R, ~, tSim] = bldc_run_scenario(mdl, SC, vars);

    meta = struct('model', mdl, 'controller', 'Conventional FOC', 'scenario', SCEN, ...
                  'date', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), 'matlab', version, ...
                  'sim_time_s', tSim, 'log_step_s', DECIM*P.dt);
    f = char(java.io.File(fullfile(outDir, sprintf('%s_conv.mat', SCEN))).getCanonicalPath());
    save(f, 'R', 'P', 'SC', 'sc_name', 'THETA_OFF', 'meta');
    fprintf('  %s (%s) done in %.0f s -> %s\n', SCEN, sc_name, tSim, f);
end
clear SCEN_LIST
