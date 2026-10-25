function P = bldc_set_inverter(P, Td, Vf)
%BLDC_SET_INVERTER  Set the inverter non-idealities used by bldc_plant_lib.
%   P = BLDC_SET_INVERTER(P, TD, VF)
%     TD  dead time (s): every gate signal of the PWM block turns on TD later
%         (rounded to whole solver steps P.dt) and turns off immediately.
%         TD = 0 bypasses the dead-time path (ideal inverter, default).
%     VF  forward voltages [V_IGBT V_diode] (V) of the Universal Bridge
%         (default [0 0]).
%   Fields set: P.Td, P.Vf, P.DT.N (delay in solver steps), P.DT.bypass (1 if TD = 0).
%
%   Example (2 us dead time, 0.8 V device drops):
%     P = bldc_params(); P = bldc_set_inverter(P, 2e-6, [0.8 0.8]);
%
%   See also BLDC_PARAMS.

if nargin < 3, Vf = [0 0]; end
P.Td = Td;
P.Vf = Vf;
P.DT.N      = max(1, round(Td/P.dt));   % Delay block length (>= 1, unused when bypassed)
P.DT.bypass = double(Td == 0);          % max(delayed gate, 1) = 1 -> gate unchanged
end
