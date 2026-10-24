function [R, so, tSim] = bldc_run_scenario(mdl, SC, vars)
%BLDC_RUN_SCENARIO  Simulate one scenario of a controller model and collect the outputs.
%   [R, SO, TSIM] = BLDC_RUN_SCENARIO(MDL, SC, VARS)
%     MDL   model name (already loaded), with root Inports w_ref, TL and root Outports
%           w, Te, i_abc, idq, iq_ref, iq_cmd (the common interface of all controllers)
%     SC    scenario struct from BLDC_SCENARIO
%     VARS  N-by-2 cell {name, value} of variables passed to the model (P, SC, DECIM, ...);
%           must contain 'P' and 'DECIM'
%   R     struct of time series on a common time base (step DECIM*P.dt):
%         t, w_ref, TL, w, Te, i_abc, id, iq, iq_ref, iq_cmd
%         Signals logged at a slower rate (idq, iq_ref, iq_cmd) are held (zero-order).
%   SO    Simulink.SimulationOutput,   TSIM  wall-clock simulation time (s)
%
%   See also BLDC_SCENARIO, BLDC_PARAMS.

ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(stepSignal(SC.t_ref,  SC.w_ref, SC.T, 'w_ref'), 'w_ref');
ds = ds.addElement(stepSignal(SC.t_load, SC.TL,    SC.T, 'TL'),    'TL');
in = Simulink.SimulationInput(mdl);
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime', num2str(SC.T));
for k = 1:size(vars, 1), in = in.setVariable(vars{k,1}, vars{k,2}); end

tic; so = sim(in); tSim = toc;
if ~isempty(so.ErrorMessage), error('bldc_run_scenario: %s', so.ErrorMessage); end

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

%% ---------------- local functions ----------------
function ts = getOut(y, blockPath)
% Logged timeseries of the root Outport with the given block path.
for k = 1:y.numElements
    e = y.getElement(k);
    if strcmp(e.BlockPath.getBlock(1), blockPath), ts = e.Values; return; end
end
error('bldc_run_scenario: output "%s" not found in yout', blockPath);
end

function ts = stepSignal(t, v, T, name)
% Piecewise-constant input for a root Inport with Interpolate = off.
% The last value is repeated after the stop time, otherwise the Inport
% outputs 0 once the final data point has passed.
t = t(:); v = v(:);
keep = t <= T;
t = [t(keep); T + 1];  v = [v(keep); v(find(keep, 1, 'last'))];
ts = timeseries(v, t, 'Name', name);
end

function x = holdOn(ts, t)
% Zero-order hold of timeseries ts onto time vector t (columns = channels).
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
x = interp1(ts.Time, d, t, 'previous', 'extrap');
end
