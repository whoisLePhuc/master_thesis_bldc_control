function in = bldc_sim_input(mdl, SC, vars)
%BLDC_SIM_INPUT  Simulink.SimulationInput for one scenario of a controller model.
%   IN = BLDC_SIM_INPUT(MDL, SC, VARS)
%     MDL   model name, SC scenario struct (BLDC_SCENARIO), VARS N-by-2 cell {name, value}
%   The speed reference and load torque are applied through the root Inports w_ref, TL.
%   Used by BLDC_RUN_SCENARIO (single run) and by parsim-based studies.
%
%   See also BLDC_RUN_SCENARIO, BLDC_COLLECT.

ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(stepSignal(SC.t_ref,  SC.w_ref, SC.T, 'w_ref'), 'w_ref');
ds = ds.addElement(stepSignal(SC.t_load, SC.TL,    SC.T, 'TL'),    'TL');
in = Simulink.SimulationInput(mdl);
in = in.setExternalInput(ds);
in = in.setModelParameter('StopTime', num2str(SC.T));
for k = 1:size(vars, 1), in = in.setVariable(vars{k,1}, vars{k,2}); end
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
