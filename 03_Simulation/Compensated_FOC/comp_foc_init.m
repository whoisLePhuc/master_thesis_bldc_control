%% COMP_FOC_INIT  Parameters, look-up tables and test scenarios for Compensated_FOC.slx
%  Compensated FOC: conventional FOC + angle-based trapezoidal back-EMF feedforward
%  (LUT g_d, g_q), digital-delay compensation (theta_p, theta_a) and iq current
%  shaping (k_s) (Theory, Sec. 6, Fig. 6.2).
%
%  Motor, inverter, PI gains and scenarios S1-S4 are identical to conv_foc_init.m
%  (Conventional_FOC), so that both controllers are compared under the same conditions.
%
%  Usage:
%     >> SCEN = 'S3'; comp_foc_init;     % then run the model
%     >> run_comp_foc                    % run S1-S4 and save results
%
%  Variables created in the workspace: P, TAB, SC, SCEN, sc_name, THETA_OFF, TAB_SHIFT, DECIM, RPM

if ~exist('SCEN', 'var'), SCEN = 'S2'; end
DECIM = 10;                     % log every 10 solver steps (10 us)

% ---------------- angle calibration ----------------
% THETA_OFF : FOC angle offset for the SPS PMSM block (calibrated: -pi/2)
% TAB_SHIFT : extra phase shift of the back-EMF look-up tables (default 0),
%             tuned by run_comp_foc with DO_CALIB = true and stored in comp_foc_calib.mat
if ~exist('THETA_OFF', 'var') || ~exist('TAB_SHIFT', 'var')
    calibFile_ = fullfile(fileparts(which('comp_foc_init')), 'comp_foc_calib.mat');
    if exist(calibFile_, 'file')
        load(calibFile_, 'THETA_OFF', 'TAB_SHIFT');
    else
        THETA_OFF = -pi/2; TAB_SHIFT = 0;
    end
    clear calibFile_
end

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

% ---------------- digital-delay compensation ----------------
P.Td_ff = 1.5*P.Ts;            % back-EMF feedforward: theta_p = theta_e + 1.5*Ts*we
P.Td_sh = 1.5*P.Ts + P.tau_c;  % current shaping:      theta_a = theta_e + (1.5*Ts + tau_c)*we

% ---------------- matrices used in Gain blocks ----------------
P.Kc  = 2/3*[1 -1/2 -1/2; 0 sqrt(3)/2 -sqrt(3)/2];  % Clarke abc -> alpha-beta
P.Kic = [1 0; -1/2 sqrt(3)/2; -1/2 -sqrt(3)/2];     % inverse Clarke
E6 = eye(6); P.Pg = E6([1 4 2 5 3 6], :);           % [up_abc; low_abc] -> Universal Bridge gate order
clear E6

% ---------------- back-EMF look-up tables on the d-q frame ----------------
%  f_x(theta) = -F(theta - phi_x),  e_x = we*lam*f_x
%  g_d = cos*f_al + sin*f_be ; g_q = -sin*f_al + cos*f_be   (period pi/3)
%  k_s = (lam1/lam)/g_q  ->  Te = 1.5*p*lam1*iq_ref (constant)
TAB = struct();
TAB.N      = 361;
TAB.period = pi/3;
TAB.th     = linspace(0, TAB.period, TAB.N);
fa_ = -bldc_trap(TAB.th); fb_ = -bldc_trap(TAB.th - 2*pi/3); fc_ = -bldc_trap(TAB.th + 2*pi/3);
fal_ = 2/3*(fa_ - fb_/2 - fc_/2); fbe_ = (fb_ - fc_)/sqrt(3);   % Clarke
TAB.gd = cos(TAB.th).*fal_ + sin(TAB.th).*fbe_;                 % Park
TAB.gq = -sin(TAB.th).*fal_ + cos(TAB.th).*fbe_;
TAB.ks = (P.lam1/P.lam)./TAB.gq;                                % current shaping
clear fa_ fb_ fc_ fal_ fbe_

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
    case 'CAL'   % only for TAB_SHIFT calibration: 100 rpm, 1 N.m load from 0.1 s
        sc_name = 'Look-up table calibration';
        SC = struct('T',0.26, 't_ref',[0 0.01], 'w_ref',[0 100]*RPM, 't_load',[0 0.1], 'TL',[0 1]);
    otherwise
        error('comp_foc_init: unknown scenario "%s"', SCEN);
end
if numel(SC.t_ref)  < 2, SC.t_ref  = [SC.t_ref 1e3];  SC.w_ref = SC.w_ref([1 1]); end
if numel(SC.t_load) < 2, SC.t_load = [SC.t_load 1e3]; SC.TL    = SC.TL([1 1]);    end

fprintf('[comp_foc_init] %s: %s (T = %.2f s) | THETA_OFF = %.4f rad, TAB_SHIFT = %.4f rad\n', ...
        SCEN, sc_name, SC.T, THETA_OFF, TAB_SHIFT);
