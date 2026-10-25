%% COMP_FOC_INIT  Parameters, look-up tables and test scenario for Compensated_FOC.slx
%  Compensated FOC: conventional FOC + angle-based trapezoidal back-EMF feedforward
%  (LUT g_d, g_q), digital-delay compensation (theta_p, theta_a) and iq current
%  shaping (k_s) (Theory doc 04, Sec. 6, Fig. 6.2).
%
%  Motor, inverter, PI gains and scenarios come from ../Common (bldc_params,
%  bldc_scenario) and are shared with all other controllers.
%
%  Usage:
%     >> SCEN = 'S3'; comp_foc_init;     % then run the model
%     >> run_comp_foc                    % run S1-S4 and save results
%
%  Optional input: VLIM (voltage limiter, see Common/bldc_vlim.m; default P.vlim of bldc_params = 3)
%  Variables created in the workspace: P, TAB, SC, SCEN, sc_name, THETA_OFF, TAB_SHIFT, DECIM, RPM

addpath(fullfile(fileparts(which('comp_foc_init')), '..', 'Common'));
bldc_cache_setup();             % Simulink caches in <repo>/.cache
if ~exist('SCEN', 'var'), SCEN = 'S2'; end
DECIM = 10;                     % log every 10 solver steps (10 us)
RPM   = 2*pi/60;

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

% ---------------- common parameters ----------------
P = bldc_params();
% voltage limiter: P.vlim = 3 (inverter hexagon) by default since Sec. 10; the results of
% Sec. 7-9 were computed with the original limiter: VLIM = 0; run_comp_foc reproduces them
if exist('VLIM', 'var'), P.vlim = VLIM; end

% ---------------- digital-delay compensation ----------------
P.Td_ff = 1.5*P.Ts;            % back-EMF feedforward: theta_p = theta_e + 1.5*Ts*we
P.Td_sh = 1.5*P.Ts + P.tau_c;  % current shaping:      theta_a = theta_e + (1.5*Ts + tau_c)*we

% ---------------- back-EMF look-up tables on the d-q frame ----------------
%  f_x(theta) = -F(theta - phi_x),  e_x = we*lam*f_x       (F = bldc_trap, Common/)
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

% ---------------- scenario ----------------
[SC, sc_name] = bldc_scenario(SCEN);

fprintf('[comp_foc_init] %s: %s (T = %.2f s) | THETA_OFF = %.4f rad, TAB_SHIFT = %.4f rad | vlim = %d\n', ...
        SCEN, sc_name, SC.T, THETA_OFF, TAB_SHIFT, P.vlim);
