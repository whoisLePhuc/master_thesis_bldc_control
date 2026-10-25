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
%   See also BLDC_SCENARIO, BLDC_PARAMS, BLDC_SIM_INPUT, BLDC_COLLECT.

in = bldc_sim_input(mdl, SC, vars);
tic; so = sim(in); tSim = toc;
if ~isempty(so.ErrorMessage), error('bldc_run_scenario: %s', so.ErrorMessage); end
R = bldc_collect(so, mdl, SC);
end
