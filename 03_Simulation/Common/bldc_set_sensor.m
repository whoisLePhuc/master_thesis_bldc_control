function P = bldc_set_sensor(P, off, gain, nsens)
%BLDC_SET_SENSOR  Set the phase-current measurement errors used by bldc_plant_lib/Current Sensor.
%   P = BLDC_SET_SENSOR(P, OFF, GAIN, NSENS)
%     OFF    offset of each phase sensor [a b c] (A), default [0 0 0]
%     GAIN   gain of each phase sensor [a b c] (-), default [1 1 1]
%     NSENS  3: three phase sensors (default)
%            2: sensors on phases a and b only, i_c = -(i_a + i_b) is computed
%   The measured currents are  i_meas = M * (GAIN .* i_abc + OFF),
%   M = eye(3) for NSENS = 3 and M = [1 0 0; 0 1 0; -1 -1 0] for NSENS = 2
%   (with NSENS = 2 the phase-c entries of OFF and GAIN are not used).
%   Fields set: P.CS.off, P.CS.gain (3x1), P.CS.M (3x3), P.CS.n.
%   The defaults give i_meas = i_abc exactly, so results without measurement
%   errors do not change.
%
%   Example (phase-a offset 30 mA, phase-b gain +3 %, two sensors):
%     P = bldc_params(); P = bldc_set_sensor(P, [0.03 0 0], [1 1.03 1], 2);
%
%   See also BLDC_PARAMS, BLDC_SET_INVERTER.

if nargin < 2 || isempty(off),   off = [0 0 0];  end
if nargin < 3 || isempty(gain),  gain = [1 1 1]; end
if nargin < 4 || isempty(nsens), nsens = 3;      end
P.CS.off  = off(:);
P.CS.gain = gain(:);
P.CS.n    = nsens;
switch nsens
    case 3, P.CS.M = eye(3);
    case 2, P.CS.M = [1 0 0; 0 1 0; -1 -1 0];
        P.CS.off(3) = 0; P.CS.gain(3) = 1;
    otherwise, error('bldc_set_sensor: NSENS must be 2 or 3');
end
end
