function P = bldc_params()
%BLDC_PARAMS  Common parameters of the BLDC test bench used by all controllers.
%   P = BLDC_PARAMS() returns a struct with the motor, inverter and sampling
%   parameters, the matrices used in Gain blocks of the shared plant library
%   (bldc_plant_lib.slx) and the FOC controller gains.
%
%   Every controller init script starts from this struct, so that all control
%   strategies are evaluated on exactly the same motor and inverter.
%
%   See also BLDC_SCENARIO, BLDC_RUN_SCENARIO, BLDC_SET_INVERTER.

% ---------------- motor ----------------
P = struct();
P.p   = 8;          % pole pairs
P.R   = 0.4578;     % stator resistance (ohm)
P.L   = 3.33e-3;    % synchronous inductance L = Ls - M (H)
P.lam = 0.171;      % PM flux linkage, flat-top amplitude (V.s)
P.J   = 1.469e-3;   % inertia (kg.m^2)
P.B   = 3.035e-4;   % viscous friction (N.m.s/rad)

% ---------------- inverter / sampling ----------------
P.Vdc  = 36;                        % DC-link voltage (V)
P.fpwm = 10e3;  P.Ts = 1/P.fpwm;    % PWM frequency, current-loop sampling (s)
P.Tsw  = 1e-3;                      % speed-loop sampling (s)
P.dt   = 1e-6;                      % solver / powergui step (s)
P.Imax = 3;                         % current limit (A)
P.Vm   = P.Vdc/sqrt(3);             % phase-voltage limit, linear SVPWM range (V)

% ---------------- inverter non-idealities (off by default, see BLDC_SET_INVERTER) ----------------
P = bldc_set_inverter(P, 0, [0 0]);   % dead time P.Td (s), forward voltages P.Vf = [IGBT diode] (V)

% ---------------- trapezoidal back-EMF ----------------
P.b1   = 4/pi*sin(pi/6)/(pi/6);     % fundamental Fourier coefficient = 12/pi^2 = 1.2158
P.lam1 = P.lam*P.b1;                % fundamental flux linkage (V.s)

% ---------------- FOC controller gains (Theory doc 03, Sec. 4.2-4.3) ----------------
P.Kt_foc = 1.5*P.p*P.lam1;                      % FOC torque constant (N.m/A)
P.tau_c  = 2*10/(2*pi*P.fpwm);                  % current-loop time constant (0.318 ms)
P.Kp_i   = P.L/P.tau_c;   P.Ki_i = P.R/P.tau_c; % current PI, IMC tuning
wn = 2*pi*20; zeta = 1;                         % speed loop: 20 Hz, critically damped
P.Kp_w_foc = (2*zeta*wn*P.J - P.B)/P.Kt_foc;
P.Ki_w_foc = P.J*wn^2/P.Kt_foc;

% ---------------- matrices used in Gain blocks of bldc_plant_lib ----------------
P.Kc  = 2/3*[1 -1/2 -1/2; 0 sqrt(3)/2 -sqrt(3)/2];  % Clarke abc -> alpha-beta
P.Kic = [1 0; -1/2 sqrt(3)/2; -1/2 -sqrt(3)/2];     % inverse Clarke
E6 = eye(6); P.Pg = E6([1 4 2 5 3 6], :);           % [up_abc; low_abc] -> Universal Bridge gate order
end
