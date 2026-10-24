function K = bldc_kpi(R, SC, P, SCEN)
%BLDC_KPI  Scenario-specific performance indicators used in the result tables (Theory doc 05).
%   K = BLDC_KPI(R, SC, P, SCEN) returns a struct of scalar indicators. NaN means "not
%   reached" (e.g. the speed never stays inside the +/-2 rpm band).
%
%   S1  rise_ms       10-90 % rise time of the 0 -> 100 rpm step (ms)
%       os_pct        overshoot (%)
%       settle_ms     settling time into +/-2 rpm, measured from the step (ms)
%       w_pp, Te_pp   no-load speed ripple (rpm) and torque ripple p-p (N.m) in SC.win
%   S2  dip_rpm       speed dip after the load step (rpm)
%       recov_ms      recovery time into +/-2 rpm after the load step (ms)
%       Te_pp, Te_rms torque ripple p-p / RMS (N.m),   T6  6th-harmonic torque amplitude (N.m)
%       w_pp          speed ripple p-p (rpm),         thd_pct  phase-current THD, orders 2-41 (%)
%   S3  zc_ms         time from the reversal command to zero speed (ms)
%       os_pct        overshoot beyond -100 rpm (%),  i_peak   peak phase current (A)
%   S4  settle_ms     mean settling time into +/-2 rpm over the four steps (ms)
%       w_pp          speed ripple p-p at 30 rpm (rpm)
%
%   Torque is averaged over one PWM period before computing ripple and spectra.
%   See also BLDC_METRICS, BLDC_SCENARIO.

rpm  = 60/(2*pi);
band = 2/rpm;                                        % +/-2 rpm in rad/s
M    = bldc_metrics(R, SC, P);
dtL  = R.t(2) - R.t(1);
Tav  = movmean(R.Te, max(1, round(P.Ts/dtL)));
fe   = @(w) P.p*abs(w)/(2*pi);                       % electrical frequency (Hz)
K = struct();
switch SCEN
    case 'S1'
        t0 = SC.t_ref(2); wr = SC.w_ref(2);
        K.rise_ms   = 1e3*(tcross(R.t, R.w, 0.9*wr, t0) - tcross(R.t, R.w, 0.1*wr, t0));
        K.os_pct    = 100*(max(R.w) - wr)/wr;
        K.settle_ms = 1e3*(tsettle(R.t, R.w, wr, band, t0, R.t(end)) - t0);
        K.w_pp      = M.w_pp;
        K.Te_pp     = M.Te_pp;
    case 'S2'
        t0 = SC.t_load(2); wr = SC.w_ref(end);
        K.dip_rpm  = M.w_dip;
        K.recov_ms = 1e3*(tsettle(R.t, R.w, wr, band, t0, SC.t_load(3)) - t0);
        K.Te_pp    = M.Te_pp;
        K.Te_rms   = M.Te_rms;
        f6 = 6*fe(wr);                               % 80 Hz at 100 rpm
        K.T6       = harmAmp(R.t, Tav, f6, SC.win(1), 10/f6);
        K.w_pp     = M.w_pp;
        K.thd_pct  = thd(R.t, R.i_abc(:,1), fe(wr), SC.win(1), 2/fe(wr), 41);
    case 'S3'
        t0 = SC.t_ref(3);
        K.zc_ms  = 1e3*(tcross(R.t, -R.w, 0, t0) - t0);
        K.os_pct = 100*(abs(min(R.w)) - abs(SC.w_ref(3)))/abs(SC.w_ref(3));
        K.i_peak = M.i_peak;
    case 'S4'
        ts = zeros(1, numel(SC.t_ref) - 1);
        for k = 2:numel(SC.t_ref)
            tEnd = min([SC.t_ref(k+1:end) R.t(end)]);
            ts(k-1) = tsettle(R.t, R.w, SC.w_ref(k), band, SC.t_ref(k), tEnd) - SC.t_ref(k);
        end
        K.settle_ms = 1e3*mean(ts);                  % NaN if any step does not settle
        K.w_pp      = M.w_pp;
    otherwise
        error('bldc_kpi: unknown scenario "%s"', SCEN);
end
end

%% ---------------- local functions ----------------
function tc = tcross(t, x, level, t0)
% First time after t0 at which x reaches level (linear interpolation).
k = find(t >= t0 & x >= level, 1);
if isempty(k), tc = NaN; return; end
if k > 1 && x(k-1) < level
    tc = t(k-1) + (level - x(k-1))*(t(k) - t(k-1))/(x(k) - x(k-1));
else
    tc = t(k);
end
end

function ts = tsettle(t, w, wr, band, t0, t1)
% Time after which |w - wr| <= band until t1; NaN if the band is left again before t1.
k = find(t >= t0 & t <= t1);
out = abs(w(k) - wr) > band;
if ~any(out), ts = t0; return; end
last = find(out, 1, 'last');
if last == numel(k) || t(k(end)) - t(k(last)) < 0.02   % must stay inside >= 20 ms
    ts = NaN;
else
    ts = t(k(last + 1));
end
end

function A = harmAmp(t, x, f, t1, Tw)
% Amplitude of the component at frequency f in the window [t1, t1+Tw] (single-bin DFT).
k = t >= t1 & t < t1 + Tw;
xk = x(k) - mean(x(k));
A = 2*abs(mean(xk .* exp(-1j*2*pi*f*t(k))));
end

function T = thd(t, x, f1, t1, Tw, nMax)
% Total harmonic distortion (orders 2..nMax) over [t1, t1+Tw] (integer number of periods).
A = arrayfun(@(n) harmAmp(t, x, n*f1, t1, Tw), 1:nMax);
T = 100*sqrt(sum(A(2:end).^2))/A(1);
end
