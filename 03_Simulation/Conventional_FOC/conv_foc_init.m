%% CONV_FOC_INIT  Parameters and test scenario for Conventional_FOC.slx
%  Conventional FOC: two PI current controllers, cross-coupling and constant
%  fundamental back-EMF compensation (Theory doc 03, Sec. 5, Fig. 5.1).
%
%  Motor, inverter, PI gains and scenarios come from ../Common (bldc_params,
%  bldc_scenario) and are shared with all other controllers.
%
%  Usage:
%     >> SCEN = 'S3'; conv_foc_init;     % then run the model
%     >> run_conv_foc                    % run S1-S4 and save results
%
%  Variables created in the workspace: P, SC, SCEN, sc_name, THETA_OFF, DECIM, RPM

addpath(fullfile(fileparts(which('conv_foc_init')), '..', 'Common'));
if ~exist('SCEN', 'var'), SCEN = 'S2'; end
DECIM = 10;                     % log every 10 solver steps (10 us)
RPM   = 2*pi/60;

% THETA_OFF: FOC angle offset for the SPS PMSM block (calibrated: -pi/2)
if ~exist('THETA_OFF', 'var'), THETA_OFF = -pi/2; end

P = bldc_params();
[SC, sc_name] = bldc_scenario(SCEN);

fprintf('[conv_foc_init] %s: %s (T = %.2f s) | THETA_OFF = %.4f rad\n', ...
        SCEN, sc_name, SC.T, THETA_OFF);
