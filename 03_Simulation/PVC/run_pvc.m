%% RUN_PVC  Run Pseudo_Vector_Control.slx for scenarios S1-S4 and save the results
%  Two variants are simulated:
%     PVC_FF = 0  pseudo vector control as in US 7,339,346 (phase PI only)  -> <SCEN>_pvc.mat
%     PVC_FF = 1  with phase back-EMF feedforward                           -> <SCEN>_pvcff.mat
%  Each file contains R (t, w_ref, TL, w, Te, i_abc, id, iq, iq_ref, iq_cmd),
%  P, TAB, PVC, SC, sc_name, THETA_OFF, meta.
%
%  Usage:
%     >> run_pvc                                   % both variants, all scenarios
%     >> SCEN_LIST = {'S2'}; FF_LIST = 0; run_pvc  % selected scenarios / variants

here = fileparts(mfilename('fullpath'));
mdl  = 'Pseudo_Vector_Control';
outDir = fullfile(here, '..', '..', '04_Results', 'Raw');
if ~exist(outDir, 'dir'), mkdir(outDir); end
addpath(fullfile(here, '..', 'Common'));

if ~exist('SCEN_LIST', 'var'), SCEN_LIST = {'S1', 'S2', 'S3', 'S4'}; end
if ~exist('FF_LIST', 'var'),   FF_LIST = [0 1]; end
oldDir = cd(here);                 % so that pvc_init is found
cleanupDir = onCleanup(@() cd(oldDir));
load_system(fullfile(here, [mdl '.slx']));

for PVC_FF = FF_LIST
    tag = 'pvc'; name = 'PVC'; if PVC_FF, tag = 'pvcff'; name = 'PVC + feedforward'; end
    for iSc = 1:numel(SCEN_LIST)
        SCEN = SCEN_LIST{iSc};
        pvc_init;                  % creates P, TAB, PVC, SC, sc_name, THETA_OFF, DECIM in this workspace

        vars = {'P', P; 'TAB', TAB; 'PVC', PVC; 'SC', SC; 'THETA_OFF', THETA_OFF; 'DECIM', DECIM};
        [R, ~, tSim] = bldc_run_scenario(mdl, SC, vars);

        meta = struct('model', mdl, 'controller', name, 'scenario', SCEN, ...
                      'date', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), 'matlab', version, ...
                      'sim_time_s', tSim, 'log_step_s', DECIM*P.dt);
        f = char(java.io.File(fullfile(outDir, sprintf('%s_%s.mat', SCEN, tag))).getCanonicalPath());
        save(f, 'R', 'P', 'TAB', 'PVC', 'SC', 'sc_name', 'THETA_OFF', 'meta');
        fprintf('  %s %s (%s) done in %.0f s -> %s\n', name, SCEN, sc_name, tSim, f);
    end
end
clear SCEN_LIST FF_LIST PVC_FF
