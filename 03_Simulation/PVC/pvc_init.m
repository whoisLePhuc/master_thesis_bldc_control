%% PVC_INIT  Parameters, look-up tables and test scenario for Pseudo_Vector_Control.slx
%  Pseudo vector control (PVC), after US 7,339,346 B2 (Ta, Endo, NSK, 2008):
%  the current references are computed with vector control (d-q back-EMF,
%  power-balance i_q reference, field weakening i_d), then transformed to
%  phase-current references i_abc* that are tracked by three independent
%  phase PI controllers (no d-q current feedback).
%
%  Motor, inverter, PI gains and scenarios come from ../Common (bldc_params,
%  bldc_scenario) and are shared with all other controllers.
%
%  Usage:
%     >> SCEN = 'S3'; pvc_init;          % then run the model
%     >> PVC_FF = 1; pvc_init;           % variant with phase back-EMF feedforward
%     >> run_pvc                         % run S1-S4 (both variants) and save results
%
%  Variables created in the workspace: P, TAB, PVC, SC, SCEN, sc_name, THETA_OFF, DECIM, RPM

addpath(fullfile(fileparts(which('pvc_init')), '..', 'Common'));
bldc_cache_setup();             % Simulink caches in <repo>/.cache
if ~exist('SCEN', 'var'), SCEN = 'S2'; end
DECIM = 10;                     % log every 10 solver steps (10 us)
RPM   = 2*pi/60;

% THETA_OFF: electrical-angle offset of the SPS PMSM block (same as the FOC models)
if ~exist('THETA_OFF', 'var'), THETA_OFF = -pi/2; end

P = bldc_params();
[SC, sc_name] = bldc_scenario(SCEN);

% ---------------- back-EMF look-up tables on the d-q frame (as Compensated FOC) ----------------
%  e_x = we*lam*f_x(theta),  e_dq = we*lam*g_dq(theta)   (period pi/3)
TAB = struct();
TAB.N      = 361;
TAB.period = pi/3;
TAB.th     = linspace(0, TAB.period, TAB.N);
fa_ = -bldc_trap(TAB.th); fb_ = -bldc_trap(TAB.th - 2*pi/3); fc_ = -bldc_trap(TAB.th + 2*pi/3);
fal_ = 2/3*(fa_ - fb_/2 - fc_/2); fbe_ = (fb_ - fc_)/sqrt(3);   % Clarke
TAB.gd = cos(TAB.th).*fal_ + sin(TAB.th).*fbe_;                 % Park
TAB.gq = -sin(TAB.th).*fal_ + cos(TAB.th).*fbe_;
clear fa_ fb_ fc_ fal_ fbe_

% ---------------- PVC options ----------------
%  Torque command  T* = Kt_foc * iq*            (iq* = speed-PI output)
%  i_d* = -|T*/Kt| * sin(acos(wb/|wm|)) for |wm| > wb, else 0          (patent Eq. 6)
%  i_q* = (T*/(1.5*p*lam) - g_d*i_d*)/g_q  (power balance, patent Eq. 10, divided by we)
if ~exist('PVC_FF', 'var'), PVC_FF = 0; end
PVC = struct();
PVC.ff = PVC_FF;                      % 1: add phase back-EMF feedforward e_abc to the PI outputs
PVC.fw = 1;                           % 1: field weakening i_d* (Eq. 6) enabled
PVC.wb = P.Vm/(P.p*P.lam1);           % base speed (rad/s mech.): back-EMF fundamental = Vdc/sqrt(3)

fprintf('[pvc_init] %s: %s (T = %.2f s) | feedforward = %d | base speed = %.1f rpm\n', ...
        SCEN, sc_name, SC.T, PVC.ff, PVC.wb/RPM);
