function R = bldc_collect(so, mdl, SC)
%BLDC_COLLECT  Collect the outputs of one simulation on a common time base.
%   R = BLDC_COLLECT(SO, MDL, SC)
%     SO   Simulink.SimulationOutput of model MDL (common controller interface)
%     SC   scenario struct used for the run
%   R     struct with t, w_ref, TL, w, Te, i_abc, id, iq, iq_ref, iq_cmd
%         (step DECIM*P.dt; slower signals are held, zero-order).
%
%   See also BLDC_RUN_SCENARIO, BLDC_SIM_INPUT.

y = so.yout;
gv = @(name) getOut(y, [mdl '/' name]);
w = gv('w');
R = struct();
R.t      = w.Time;
R.w      = w.Data;
R.Te     = holdOn(gv('Te'),    R.t);
R.i_abc  = holdOn(gv('i_abc'), R.t);
idq      = holdOn(gv('idq'),   R.t);
R.id     = idq(:, 1);
R.iq     = idq(:, 2);
R.iq_ref = holdOn(gv('iq_ref'), R.t);
R.iq_cmd = holdOn(gv('iq_cmd'), R.t);
R.w_ref  = interp1(SC.t_ref,  SC.w_ref, R.t, 'previous', 'extrap');
R.TL     = interp1(SC.t_load, SC.TL,    R.t, 'previous', 'extrap');
end

function ts = getOut(y, blockPath)
% Logged timeseries of the root Outport with the given block path.
for k = 1:y.numElements
    e = y.getElement(k);
    if strcmp(e.BlockPath.getBlock(1), blockPath), ts = e.Values; return; end
end
error('bldc_collect: output "%s" not found in yout', blockPath);
end

function x = holdOn(ts, t)
% Zero-order hold of timeseries ts onto time vector t (columns = channels).
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
x = interp1(ts.Time, d, t, 'previous', 'extrap');
end
