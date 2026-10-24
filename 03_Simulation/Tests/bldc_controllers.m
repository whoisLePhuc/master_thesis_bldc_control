function C = bldc_controllers()
%BLDC_CONTROLLERS  Registry of the implemented controllers (used by the regression test).
%   Each entry: tag (result-file suffix), name, model, folder (relative to 03_Simulation),
%   init (init script name) and vars (workspace variables passed to the model).
%   Add one entry here when a new controller is implemented.

C = struct('tag', {}, 'name', {}, 'model', {}, 'folder', {}, 'init', {}, 'vars', {});
C(end+1) = struct('tag', 'conv', 'name', 'Conventional FOC', 'model', 'Conventional_FOC', ...
    'folder', 'Conventional_FOC', 'init', 'conv_foc_init', ...
    'vars', {{'P', 'SC', 'THETA_OFF', 'DECIM'}});
C(end+1) = struct('tag', 'comp', 'name', 'Compensated FOC', 'model', 'Compensated_FOC', ...
    'folder', 'Compensated_FOC', 'init', 'comp_foc_init', ...
    'vars', {{'P', 'TAB', 'SC', 'THETA_OFF', 'TAB_SHIFT', 'DECIM'}});
end
