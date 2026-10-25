function P = bldc_set_plant(P, varargin)
%BLDC_SET_PLANT  Deviation of the simulated motor from the nominal (controller) parameters.
%   P = BLDC_SET_PLANT(P, 'R', kR, 'L', kL, 'lam', kLam, 'flat', deg)
%     R, L, lam  factors applied to P.R, P.L, P.lam in the motor block only (default 1)
%     flat       width of the flat top of the motor back-EMF (electrical degrees,
%                default 120 = the shape assumed by every controller, see BLDC_TRAP)
%   The controllers keep using P.R, P.L, P.lam and the 120 deg tables, so these
%   fields model a parameter / back-EMF-shape mismatch. Only the Permanent Magnet
%   Synchronous Machine block of bldc_plant_lib/Power Stage reads P.MIS.
%   Fields set: P.MIS.R, P.MIS.L, P.MIS.lam, P.MIS.flat. Defaults reproduce the
%   original model exactly.
%
%   Example (motor with 10 % less flux and a 140 deg flat top):
%     P = bldc_params(); P = bldc_set_plant(P, 'lam', 0.9, 'flat', 140);
%
%   See also BLDC_PARAMS, BLDC_SET_ENCODER, BLDC_SET_SENSOR.

if ~isfield(P, 'MIS'), P.MIS = struct('R', 1, 'L', 1, 'lam', 1, 'flat', 120); end
for k = 1:2:numel(varargin)
    f = varargin{k};
    if ~isfield(P.MIS, f), error('bldc_set_plant: unknown field "%s"', f); end
    P.MIS.(f) = varargin{k + 1};
end
end
