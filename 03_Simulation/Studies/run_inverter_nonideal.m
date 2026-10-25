function T = run_inverter_nonideal(varargin)
%RUN_INVERTER_NONIDEAL  Effect of inverter dead time and device drops on all controllers.
%   T = RUN_INVERTER_NONIDEAL() simulates every controller registered in
%   Tests/bldc_controllers.m with the inverter cases below, saves the raw results
%   as 04_Results/Raw/<SCEN>_<tag>_<case>.mat and writes the indicator table
%   04_Results/Tables/inverter_nonideal.{csv,md}.
%
%   Cases (dead time Td, forward voltages Vf = [IGBT diode]):
%     ideal   Td = 0,     Vf = [0 0]      (same as the main results)
%     dt1     Td = 1 us,  Vf = [0 0]
%     dt2     Td = 2 us,  Vf = [0 0]
%     dt2vf   Td = 2 us,  Vf = [0.8 0.8] V
%
%   Options (name-value): 'Controllers' (default: all), 'Scenarios' (default {'S2','S4'}),
%     'Vlim'         voltage limiter of Compensated FOC, P.vlim (default 0 = original, as in Sec. 9.7-9.9;
%                    the model default is 3 since Sec. 10)
%   'Cases' (default all), 'Run' (default true; false = only rebuild the table from saved files).
%
%   See also BLDC_SET_INVERTER, BLDC_CONTROLLERS.

ip = inputParser;
ip.addParameter('Controllers', {});
ip.addParameter('Vlim', 0);   % Compensated FOC limiter; 0 (original) reproduces the published tables
ip.addParameter('Scenarios', {'S2', 'S4'});
ip.addParameter('Cases', {});
ip.addParameter('Run', true);
ip.parse(varargin{:}); opt = ip.Results;

here    = fileparts(mfilename('fullpath'));
simRoot = fileparts(here);
repo    = fileparts(simRoot);
addpath(fullfile(simRoot, 'Common'), fullfile(simRoot, 'Tests'));
rawDir = fullfile(repo, '04_Results', 'Raw');
tabDir = fullfile(repo, '04_Results', 'Tables');
if ~exist(rawDir, 'dir'), mkdir(rawDir); end

CASES = struct('name', {'ideal', 'dt1', 'dt2', 'dt2vf'}, ...
               'Td',   {0, 1e-6, 2e-6, 2e-6}, ...
               'Vf',   {[0 0], [0 0], [0 0], [0.8 0.8]});
if ~isempty(opt.Cases), CASES = CASES(ismember({CASES.name}, opt.Cases)); end
C = bldc_controllers();
if ~isempty(opt.Controllers), C = C(ismember({C.tag}, opt.Controllers)); end

oldDir = pwd; cleanupDir = onCleanup(@() cd(oldDir));
rows = {};
for c = 1:numel(C)
    folder = fullfile(simRoot, C(c).folder);
    addpath(folder); cd(folder);
    if opt.Run, load_system(fullfile(folder, [C(c).model '.slx'])); end
    for s = 1:numel(opt.Scenarios)
        SCEN = opt.Scenarios{s};
        for k = 1:numel(CASES)
            f = fullfile(rawDir, sprintf('%s_%s_%s.mat', SCEN, C(c).tag, CASES(k).name));
            if opt.Run
                W = runInit(C(c).init, SCEN);
                W.P = bldc_set_inverter(W.P, CASES(k).Td, CASES(k).Vf);
                W.P.vlim = opt.Vlim;                                % limiter of Compensated FOC (BLDC_VLIM)
                vars = cellfun(@(n) {n, W.(n)}, C(c).vars, 'UniformOutput', false);
                vars = vertcat(vars{:});
                [R, ~, tSim] = bldc_run_scenario(C(c).model, W.SC, vars);
                P = W.P; SC = W.SC; inv = CASES(k);
                save(f, 'R', 'P', 'SC', 'inv');
                fprintf('  %-6s %s %-6s simulated in %4.1f s\n', C(c).tag, SCEN, CASES(k).name, tSim);
            else
                d = load(f, 'R', 'P', 'SC'); R = d.R; P = d.P; SC = d.SC;
            end
            rows(end+1, :) = [{C(c).tag, C(c).name, SCEN, CASES(k).name}, indicators(R, SC, P, SCEN)]; %#ok<AGROW>
        end
    end
    rmpath(folder);
end

names = {'Te_pp', 'Te_rms', 'T6', 'w_pp', 'thd_pct', 'Irms', 'Pcu'};
T = cell2table(rows, 'VariableNames', [{'Controller', 'Name', 'Scenario', 'Case'}, names]);
writetable(T, fullfile(tabDir, 'inverter_nonideal.csv'));
writeMarkdown(T, fullfile(tabDir, 'inverter_nonideal.md'));
disp(T);
end

% ---------------- local functions ----------------
function v = indicators(R, SC, P, SCEN)
% Steady-state indicators in the window SC.win: torque ripple p-p / RMS (1-PWM-period
% average), 6th-harmonic torque amplitude, speed ripple, phase-current THD, RMS current, copper loss.
k = R.t >= SC.win(1) & R.t < SC.win(2);
nAvg = round(P.Ts/(R.t(2) - R.t(1)));
Tm = movmean(R.Te, nAvg);
w  = R.w(k)*60/(2*pi);
fe = P.p*abs(mean(R.w(k)))/(2*pi); f6 = 6*fe;
t1 = SC.win(1); Tw = floor((SC.win(2) - t1)*f6)/f6;
T6  = harmAmp(R.t, Tm, f6, t1, Tw);
Tw1 = floor((SC.win(2) - t1)*fe)/fe;
if Tw1 > 0
    A1 = harmAmp(R.t, R.i_abc(:, 1), fe, t1, Tw1);
    Ah = arrayfun(@(h) harmAmp(R.t, R.i_abc(:, 1), h*fe, t1, Tw1), 2:41);
    thd = 100*sqrt(sum(Ah.^2))/A1;
else
    thd = NaN;
end
Irms = sqrt(mean(R.i_abc(k, :).^2, 'all'));
v = {max(Tm(k)) - min(Tm(k)), std(Tm(k), 1), T6, max(w) - min(w), thd, Irms, 3*P.R*Irms^2};
if strcmp(SCEN, 'S2')      % use the definitions of the main KPI table (bldc_kpi)
    K = bldc_kpi(R, SC, P, 'S2');
    v(1:5) = {K.Te_pp, K.Te_rms, K.T6, K.w_pp, K.thd_pct};
end
end

function A = harmAmp(t, x, f, t1, Tw)
k = t >= t1 & t < t1 + Tw;
xk = x(k) - mean(x(k));
A = 2/nnz(k)*abs(sum(xk.*exp(-1j*2*pi*f*t(k))));
end

function writeMarkdown(T, f)
vn = @(x, fmt) strrep(sprintf(fmt, x), '.', ',');
L = {'| Bộ điều khiển | KB | Trường hợp | Gợn T_e đ–đ (N·m) | Gợn T_e RMS (N·m) | T6 (N·m) | Gợn tốc độ (vòng/phút) | THD i_a (%) | I_rms (A) | P_Cu (W) |', ...
     '|---|---|---|---:|---:|---:|---:|---:|---:|---:|'};
for r = 1:height(T)
    L{end+1} = sprintf('| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |', T.Name{r}, T.Scenario{r}, T.Case{r}, ...
        vn(T.Te_pp(r), '%.3f'), vn(T.Te_rms(r), '%.3f'), vn(T.T6(r), '%.3f'), vn(T.w_pp(r), '%.2f'), ...
        vn(T.thd_pct(r), '%.1f'), vn(T.Irms(r), '%.3f'), vn(T.Pcu(r), '%.3f')); %#ok<AGROW>
end
writelines(L, f);
end

function W = runInit(initName, SCEN) %#ok<INUSD>
% Run a controller init script in this function's workspace and return its variables.
eval(initName);
v = who;
W = struct();
for k = 1:numel(v), W.(v{k}) = eval(v{k}); end
end
