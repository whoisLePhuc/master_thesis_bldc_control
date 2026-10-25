function M = bldc_ss_indicators(R, P, win)
%BLDC_SS_INDICATORS  Steady-state indicators of one run in the window WIN = [t1 t2].
%   M = BLDC_SS_INDICATORS(R, P, WIN)  with R from BLDC_RUN_SCENARIO / BLDC_COLLECT.
%   The window should contain an integer number of electrical periods.
%   Fields of M:
%     w_mean (rpm), w_pp (rpm)          mean speed, speed ripple peak-to-peak
%     Te_mean, Te_pp, Te_rms (N.m)      torque averaged over one PWM period: mean, p-p, RMS ripple
%     T6 (N.m)                          amplitude of the 6*fe torque component (one-point DFT)
%     thd (%)                           THD of i_a, harmonics 2..41 of fe
%     Irms (A), Pcu (W)                 RMS phase current, copper loss 3*R*Irms^2
%     id_mean, iq_mean (A)              mean measured d-q currents
%
%   See also BLDC_METRICS, BLDC_KPI.

k = R.t >= win(1) & R.t < win(2);
nAvg = round(P.Ts/(R.t(2) - R.t(1)));
Tm = movmean(R.Te, nAvg);
rpm = 60/(2*pi);
M.w_mean  = mean(R.w(k))*rpm;
M.w_pp    = (max(R.w(k)) - min(R.w(k)))*rpm;
M.Te_mean = mean(Tm(k));
M.Te_pp   = max(Tm(k)) - min(Tm(k));
M.Te_rms  = std(Tm(k), 1);
fe = P.p*abs(mean(R.w(k)))/(2*pi);
Tw = numel(find(k))*(R.t(2) - R.t(1));
M.T6 = harmAmp(R.t(k), Tm(k), 6*fe);
A1 = harmAmp(R.t(k), R.i_abc(k, 1), fe);
Ah = arrayfun(@(h) harmAmp(R.t(k), R.i_abc(k, 1), h*fe), 2:41);
M.thd = 100*sqrt(sum(Ah.^2))/A1;
if fe*Tw < 0.999, M.thd = NaN; end                % less than one electrical period
M.Irms = sqrt(mean(R.i_abc(k, :).^2, 'all'));
M.Pcu  = 3*P.R*M.Irms^2;
M.id_mean = mean(R.id(k));
M.iq_mean = mean(R.iq(k));
end

function A = harmAmp(t, x, f)
x = x - mean(x);
A = 2/numel(x)*abs(sum(x.*exp(-1j*2*pi*f*t)));
end
