%% MAKE_SIMULINK_SCREENSHOTS  Screenshots of the Simulink models used in the theory documents
%  Opens every controller model and exports the diagrams shown in documents 03, 04, 06 and 07
%  to 01_Documents/Thesis/figures (PNG, 150 dpi):
%    fig5_sim_conv_{root,speed_pi,current_pi,abc_to_dq,svpwm,pwm,power_stage}.png  (Figs. 5.2-5.8)
%    fig6_sim_comp_{root,current_pi}.png                                            (Figs. 6.3, 6.4)
%    fig8_sim_six_{root,ctrl}.png                                                   (Figs. 8.3, 8.4)
%    fig9_sim_pvc_{root,ctrl}.png                                                   (Figs. 9.4, 9.5)
%  Needs no simulation results. The models are not modified.
%
%  Usage:  >> make_simulink_screenshots

here    = fileparts(mfilename('fullpath'));
repo    = fileparts(here);
simRoot = fullfile(repo, '03_Simulation');
figDir  = fullfile(repo, '01_Documents', 'Thesis', 'figures');
addpath(fullfile(simRoot, 'Common'));

% {folder, model, subsystem ('' = root), file}
S = {'Conventional_FOC', 'Conventional_FOC', '',              'fig5_sim_conv_root'
     'Conventional_FOC', 'Conventional_FOC', 'Speed PI',      'fig5_sim_conv_speed_pi'
     'Conventional_FOC', 'Conventional_FOC', 'Current PI',    'fig5_sim_conv_current_pi'
     'Conventional_FOC', 'Conventional_FOC', 'abc_to_dq',     'fig5_sim_conv_abc_to_dq'
     'Conventional_FOC', 'Conventional_FOC', 'SVPWM',         'fig5_sim_conv_svpwm'
     'Conventional_FOC', 'Conventional_FOC', 'PWM',           'fig5_sim_conv_pwm'
     'Conventional_FOC', 'Conventional_FOC', 'Power Stage',   'fig5_sim_conv_power_stage'
     'Compensated_FOC',  'Compensated_FOC',  '',              'fig6_sim_comp_root'
     'Compensated_FOC',  'Compensated_FOC',  'Current PI',    'fig6_sim_comp_current_pi'
     'Six_Step',         'Six_Step',         '',              'fig8_sim_six_root'
     'Six_Step',         'Six_Step',         'Six-Step Ctrl', 'fig8_sim_six_ctrl'
     'PVC',              'Pseudo_Vector_Control', '',         'fig9_sim_pvc_root'
     'PVC',              'Pseudo_Vector_Control', 'PVC Ctrl', 'fig9_sim_pvc_ctrl'};

for k = 1:size(S, 1)
    folder = fullfile(simRoot, S{k, 1}); addpath(folder);
    mdl = S{k, 2};
    if ~bdIsLoaded(mdl), load_system(fullfile(folder, [mdl '.slx'])); end
    sys = mdl; if ~isempty(S{k, 3}), sys = [mdl '/' S{k, 3}]; end
    open_system(sys);
    print(['-s' sys], '-dpng', '-r150', fullfile(figDir, [S{k, 4} '.png']));
    fprintf('  %-28s <- %s\n', [S{k, 4} '.png'], sys);
end
for m = unique(S(:, 2))', close_system(m{1}, 0); end
fprintf('make_simulink_screenshots: %d figures in %s\n', size(S, 1), figDir);
