%% SIX_STEP_INIT  Parameters and test scenario for Six_Step.slx
%  Six-step (120-degree, trapezoidal) current control of the BLDC motor:
%  rotor-position sectors (Hall signals, here derived from the encoder angle),
%  one PI controller for the DC-link-equivalent current I of the two conducting
%  phases, back-EMF feedforward and H_PWM-L_ON modulation.
%
%  Motor, inverter, sampling and scenarios come from ../Common (bldc_params,
%  bldc_scenario) and are shared with all other controllers.
%
%  Usage:
%     >> SCEN = 'S3'; six_step_init;     % then run the model
%     >> run_six_step                    % run S1-S4 and save results
%
%  Variables created in the workspace: P, SS, SC, SCEN, sc_name, THETA_OFF, DECIM, RPM

addpath(fullfile(fileparts(which('six_step_init')), '..', 'Common'));
bldc_cache_setup();             % Simulink caches in <repo>/.cache
if ~exist('SCEN', 'var'), SCEN = 'S2'; end
DECIM = 10;                     % log every 10 solver steps (10 us)
RPM   = 2*pi/60;

% THETA_OFF: electrical-angle offset of the SPS PMSM block (same as the FOC models)
if ~exist('THETA_OFF', 'var'), THETA_OFF = -pi/2; end

P = bldc_params();
[SC, sc_name] = bldc_scenario(SCEN);

% ---------------- six-step controller gains ----------------
% Two phases in series:  V = 2R*I + 2L*dI/dt + 2*lam*we,   Te = 2*p*lam*I
P.Kt_6s   = 2*P.p*P.lam;                         % torque constant (N.m/A)
P.Kp_i6   = 2*P.L/P.tau_c;  P.Ki_i6 = 2*P.R/P.tau_c;   % current PI, IMC (same tau_c as FOC)
wn = 2*pi*20; zeta = 1;                          % speed loop: 20 Hz, critically damped
P.Kp_w_6s = (2*zeta*wn*P.J - P.B)/P.Kt_6s;
P.Ki_w_6s = P.J*wn^2/P.Kt_6s;
clear wn zeta

% ---------------- commutation table (Hall sectors) ----------------
%  e_x = we*lam*f_x(theta),  f_a = -F(theta), f_b = -F(theta - 2pi/3), f_c = -F(theta + 2pi/3)
%  Sector k = 0..5 covers theta in [pi/6 + k*pi/3, pi/6 + (k+1)*pi/3) (+ SS.shift):
%  in each sector two phases are on the flat top (+1 / -1), the third is in transition (0).
SS = struct();
if ~exist('COMM_SHIFT', 'var'), COMM_SHIFT = 0; end
SS.shift = COMM_SHIFT;                           % commutation advance (rad, electrical)
SS.th0   = pi/6;                                 % first sector boundary
thc_ = SS.th0 + ((0:5) + 0.5)*pi/3;              % sector centres
SS.tab = round([-bldc_trap(thc_); -bldc_trap(thc_ - 2*pi/3); -bldc_trap(thc_ + 2*pi/3)]);  % 3x6: s_abc per sector
clear thc_

fprintf('[six_step_init] %s: %s (T = %.2f s) | Kt = %.3f N.m/A | COMM_SHIFT = %.4f rad\n', ...
        SCEN, sc_name, SC.T, P.Kt_6s, SS.shift);
