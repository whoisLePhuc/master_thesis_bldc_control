function P = bldc_set_encoder(P, N, off_e)
%BLDC_SET_ENCODER  Position-sensor model used by bldc_plant_lib/Position Sensor.
%   P = BLDC_SET_ENCODER(P, N, OFF_E)
%     N      encoder resolution (counts per mechanical revolution); 0 = ideal (default)
%     OFF_E  mounting offset (electrical rad, default 0); positive = measured angle leads
%   The measured mechanical angle is
%     theta_meas = Q*floor(theta/Q) + OFF_E/p,   Q = 2*pi/N   (N > 0)
%     theta_meas = theta + OFF_E/p                              (N = 0)
%   It feeds every use of the angle in the controllers (Park transforms, look-up
%   tables, six-step sectors). The speed measurement stays ideal.
%   Fields set: P.ENC.on, P.ENC.q, P.ENC.off (mechanical rad). Defaults reproduce the
%   original model exactly.
%
%   Example (1024 counts/rev, 5 deg electrical offset):
%     P = bldc_params(); P = bldc_set_encoder(P, 1024, 5*pi/180);
%
%   See also BLDC_PARAMS, BLDC_SET_PLANT.

if nargin < 2 || isempty(N), N = 0; end
if nargin < 3 || isempty(off_e), off_e = 0; end
P.ENC.on  = double(N > 0);
P.ENC.q   = 2*pi/max(N, 1);
P.ENC.off = off_e/P.p;
end
