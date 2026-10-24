%% CONV_FOC_INIT  Parameters and test scenarios for Conventional_FOC.slx
%  Conventional FOC: two PI current controllers, cross-coupling and constant
%  fundamental back-EMF compensation (Theory.pdf, Sec. 5, Fig. 6).
%
%  Motor, inverter, controller parameters and scenarios S1-S4 are the common
%  part of bldc_comp_init.m (FOC_Controller), so that Conventional and
%  Compensated FOC are compared under identical conditions.
%
%  Usage:
%     >> SCEN = 'S3'; conv_foc_init;     % then run the model
%     >> run_conv_foc                    % run S1-S4 and save results
%
%  Variables created in the workspace: P, SC, SCEN, sc_name, THETA_OFF, DECIM, RPM

if ~exist('SCEN', 'var'), SCEN = 'S2'; end
DECIM = 10;                     % log every 10 solver steps (10 us)

% ---------------- angle offset ----------------
% THETA_OFF: FOC angle offset for the SPS PMSM block (calibrated: -pi/2)
if ~exist('THETA_OFF', 'var'), THETA_OFF = -pi/2; end

% ---------------- motor / inverter ----------------
P = struct();
P.p   = 8;          % pole pairs
P.R   = 0.4578;     % stator resistance (ohm)
P.L   = 3.33e-3;    % stator inductance (H)
P.lam = 0.171;      % PM flux linkage (V.s)
P.J   = 1.469e-3;   % inertia (kg.m^2)
P.B   = 3.035e-4;   % viscous friction (N.m.s)
P.Vdc = 36;         % DC-link voltage (V)

P.fpwm = 10e3;  P.Ts = 1/P.fpwm;    % PWM frequency, current-loop sampling
P.Tsw  = 1e-3;                      % speed-loop sampling
P.dt   = 1e-6;                      % solver / powergui step
P.Imax = 3;                         % current limit (A)

P.b1     = 4/pi*sin(pi/6)/(pi/6);   % fundamental Fourier coefficient = 1.2158
P.lam1   = P.lam*P.b1;              % fundamental flux linkage
P.Kt_foc = 1.5*P.p*P.lam1;          % torque constant
P.Vm     = P.Vdc/sqrt(3);           % phase-voltage limit (SVPWM)

% ---------------- PI controllers ----------------
P.tau_c = 2*10/(2*pi*P.fpwm);                  % current-loop time constant (0.318 ms)
P.Kp_i  = P.L/P.tau_c;   P.Ki_i = P.R/P.tau_c; % IMC tuning
wn = 2*pi*20; zeta = 1;                        % speed loop: 20 Hz, critically damped
P.Kp_w_foc = (2*zeta*wn*P.J - P.B)/P.Kt_foc;
P.Ki_w_foc = P.J*wn^2/P.Kt_foc;
clear wn zeta

% ---------------- matrices used in Gain blocks ----------------
P.Kc  = 2/3*[1 -1/2 -1/2; 0 sqrt(3)/2 -sqrt(3)/2];  % Clarke abc -> alpha-beta
P.Kic = [1 0; -1/2 sqrt(3)/2; -1/2 -sqrt(3)/2];     % inverse Clarke
E6 = eye(6); P.Pg = E6([1 4 2 5 3 6], :);           % [up_abc; low_abc] -> Universal Bridge gate order
clear E6

% ---------------- test scenarios ----------------
RPM = 2*pi/60;
switch SCEN
    case 'S1'
        sc_name = 'No-load start-up';
        SC = struct('T',0.4, 't_ref',[0 0.02], 'w_ref',[0 100]*RPM, 't_load',0, 'TL',0);
    case 'S2'
        sc_name = 'Load step';
        SC = struct('T',0.8, 't_ref',[0 0.02], 'w_ref',[0 100]*RPM, ...
                    't_load',[0 0.3 0.6], 'TL',[0 1 0]);
    case 'S3'
        sc_name = 'Speed reversal';
        SC = struct('T',0.7, 't_ref',[0 0.02 0.3], 'w_ref',[0 100 -100]*RPM, 't_load',0, 'TL',0);
    case 'S4'
        sc_name = 'Staircase speed reference';
        SC = struct('T',1.25, 't_ref',[0 0.25 0.5 0.75 1.0], ...
                    'w_ref',[30 60 100 60 30]*RPM, 't_load',0, 'TL',0.5);
    otherwise
        error('conv_foc_init: unknown scenario "%s"', SCEN);
end
if numel(SC.t_ref)  < 2, SC.t_ref  = [SC.t_ref 1e3];  SC.w_ref = SC.w_ref([1 1]); end
if numel(SC.t_load) < 2, SC.t_load = [SC.t_load 1e3]; SC.TL    = SC.TL([1 1]);    end

fprintf('[conv_foc_init] %s: %s (T = %.2f s) | THETA_OFF = %.4f rad\n', ...
        SCEN, sc_name, SC.T, THETA_OFF);
