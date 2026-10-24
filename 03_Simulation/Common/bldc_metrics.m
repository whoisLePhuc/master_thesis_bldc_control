function M = bldc_metrics(R, SC, P)
%BLDC_METRICS  Performance metrics of one simulation run.
%   M = BLDC_METRICS(R, SC, P) returns a struct of scalar metrics computed from the
%   result struct R (see BLDC_RUN_SCENARIO), the scenario SC and the parameters P.
%
%   Steady-state metrics are evaluated in the window SC.win (see BLDC_SCENARIO);
%   torque is averaged over one PWM period (moving mean) before computing ripple.
%     w_mean   mean speed (rpm)              w_pp    speed ripple, peak-to-peak (rpm)
%     Te_mean  mean torque (N.m)             Te_pp   torque ripple, peak-to-peak (N.m)
%     Te_rms   torque ripple, RMS (N.m)      iq_pp   q-current ripple, peak-to-peak (A)
%   Whole-run metrics:
%     w_peak   max |speed| (rpm)             i_peak  max |phase current| (A)
%     w_dip    speed dip after the first load step (rpm), NaN if no load step
%
%   See also BLDC_SCENARIO, BLDC_RUN_SCENARIO.

rpm = 60/(2*pi);
if isfield(SC, 'win'), win = SC.win; else, win = [NaN NaN]; end
k  = R.t >= win(1) & R.t <= win(2);
nAvg = max(1, round(P.Ts / (R.t(2) - R.t(1))));          % samples per PWM period
Tav  = movmean(R.Te, nAvg);

M = struct();
M.w_mean  = mean(R.w(k)) * rpm;
M.w_pp    = (max(R.w(k)) - min(R.w(k))) * rpm;
M.Te_mean = mean(R.Te(k));
M.Te_pp   = max(Tav(k)) - min(Tav(k));
M.Te_rms  = std(Tav(k));
M.iq_pp   = max(R.iq(k)) - min(R.iq(k));
M.w_peak  = max(abs(R.w)) * rpm;
M.i_peak  = max(abs(R.i_abc(:)));

% speed dip after the first load step (within 0.15 s)
iStep = find(diff(SC.TL(:)) > 0, 1);
if isempty(iStep) || SC.t_load(iStep+1) > R.t(end)
    M.w_dip = NaN;
else
    t0 = SC.t_load(iStep+1);
    kk = R.t >= t0 & R.t <= t0 + 0.15;
    w0 = interp1(SC.t_ref, SC.w_ref, t0, 'previous', 'extrap');
    M.w_dip = (w0 - min(R.w(kk))) * rpm;
end
end
