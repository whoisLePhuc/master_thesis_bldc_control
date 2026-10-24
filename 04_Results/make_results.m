%% MAKE_RESULTS  Comparison tables and figures from the raw simulation results
%  Reads 04_Results/Raw/<SCEN>_<tag>.mat of the controllers listed in
%  03_Simulation/Tests/bldc_controllers.m and writes
%    04_Results/Tables/kpi_comparison.{csv,md,tex}   indicators of BLDC_KPI per scenario
%    04_Results/Tables/metrics_all.csv               all metrics of BLDC_METRICS
%    01_Documents/Thesis/figures/fig7_*.png          figures of Theory doc 05
%  Run the controllers' run_*.m scripts first.
%
%  Usage:  >> make_results

here    = fileparts(mfilename('fullpath'));
repo    = fileparts(here);
simRoot = fullfile(repo, '03_Simulation');
addpath(fullfile(simRoot, 'Common'), fullfile(simRoot, 'Tests'));
rawDir = fullfile(here, 'Raw');
tabDir = fullfile(here, 'Tables');
figDir = fullfile(repo, '01_Documents', 'Thesis', 'figures');
if ~exist(tabDir, 'dir'), mkdir(tabDir); end

C     = bldc_controllers();
tags  = {C.tag};
SCENS = {'S1', 'S2', 'S3', 'S4'};

% ---------------- load results ----------------
D = struct();
for s = 1:numel(SCENS)
    for c = 1:numel(C)
        f = fullfile(rawDir, sprintf('%s_%s.mat', SCENS{s}, tags{c}));
        if ~exist(f, 'file'), error('make_results: missing %s (run run_*.m first)', f); end
        d = load(f, 'R', 'P');
        d.SC = bldc_scenario(SCENS{s});
        D.(SCENS{s}).(tags{c}) = d;
    end
end

% ---------------- indicator table ----------------
%  scenario, field, label (Vietnamese, used in the documents), unit, number format
IND = {
 'S1','rise_ms',  'Thời gian lên',                          'ms',       '%.2f'
 'S1','os_pct',   'Độ quá điều chỉnh',                      '%',        '%.1f'
 'S1','settle_ms','Thời gian xác lập ±2 vòng/phút',         'ms',       '%.1f'
 'S1','w_pp',     'Gợn tốc độ không tải',                   'vòng/phút','%.2f'
 'S1','Te_pp',    'Gợn mô-men không tải đ–đ',               'N·m',      '%.3f'
 'S2','dip_rpm',  'Độ sụt tốc khi đóng tải',                'vòng/phút','%.1f'
 'S2','recov_ms', 'Thời gian phục hồi ±2 vòng/phút',        'ms',       '%.1f'
 'S2','Te_pp',    'Gợn mô-men đỉnh–đỉnh',                   'N·m',      '%.3f'
 'S2','Te_rms',   'Gợn mô-men RMS',                         'N·m',      '%.3f'
 'S2','T6',       'Biên độ gợn bậc 6 (80 Hz)',              'N·m',      '%.3f'
 'S2','w_pp',     'Gợn tốc độ đỉnh–đỉnh',                   'vòng/phút','%.2f'
 'S2','thd_pct',  'THD dòng pha (bậc 2–41)',                '%',        '%.1f'
 'S3','zc_ms',    'Thời gian qua tốc độ 0',                 'ms',       '%.2f'
 'S3','os_pct',   'Độ vọt lố khi đảo chiều',                '%',        '%.1f'
 'S3','i_peak',   'Dòng pha đỉnh',                          'A',        '%.2f'
 'S4','settle_ms','Thời gian xác lập TB các bậc',           'ms',       '%.1f'
 'S4','w_pp',     'Gợn tốc độ ở 30 vòng/phút',              'vòng/phút','%.2f'
};
KPI = struct();
for s = 1:numel(SCENS)
    for c = 1:numel(C)
        d = D.(SCENS{s}).(tags{c});
        KPI.(SCENS{s}).(tags{c}) = bldc_kpi(d.R, d.SC, d.P, SCENS{s});
    end
end
nI = size(IND, 1);
V  = nan(nI, numel(C));
for i = 1:nI
    for c = 1:numel(C), V(i, c) = KPI.(IND{i,1}).(tags{c}).(IND{i,2}); end
end
chg = 100*(V(:, end) - V(:, 1))./abs(V(:, 1));          % last controller vs first (comp vs conv)

fmt = @(x, f) ifelse(isnan(x), '–', sprintf(f, x));
% CSV (machine readable)
T = cell2table([IND(:,1) IND(:,2) IND(:,3) IND(:,4) num2cell(V) num2cell(chg)], ...
    'VariableNames', [{'Scenario', 'Key', 'Indicator', 'Unit'}, tags, {'Change_pct'}]);
writetable(T, fullfile(tabDir, 'kpi_comparison.csv'), 'Encoding', 'UTF-8');
% Markdown and LaTeX (Vietnamese decimal comma)
vn = @(s) strrep(s, '.', ',');
md  = {sprintf('| KB | Chỉ tiêu | %s | Thay đổi |', strjoin({C.name}, ' | ')), ...
       ['|---|---|' repmat('---:|', 1, numel(C)) '---:|']};
tex = {};
for i = 1:nI
    vals = arrayfun(@(c) vn(fmt(V(i,c), IND{i,5})), 1:numel(C), 'UniformOutput', false);
    if isnan(chg(i)), cs = ''; else, cs = vn(sprintf('%+.0f %%', chg(i))); cs = strrep(cs, '-', '−'); end
    md{end+1}  = sprintf('| %s | %s (%s) | %s | %s |', IND{i,1}, IND{i,3}, IND{i,4}, strjoin(vals, ' | '), cs); %#ok<SAGROW>
    texc = strrep(strrep(cs, '%', '\%'), '−', '$-$'); texc = strrep(texc, '+', '$+$');
    unitTex = strrep(strrep(IND{i,4}, '%', '\%'), '·', '$\cdot$');
    line = sprintf('%s & %s (%s) & %s & %s \\\\', IND{i,1}, strrep(IND{i,3}, '±', '$\pm$'), unitTex, strjoin(vals, ' & '), texc);
    if i > 1 && ~strcmp(IND{i,1}, IND{i-1,1}), tex{end+1} = '\midrule'; end %#ok<SAGROW>
    tex{end+1} = line; %#ok<SAGROW>
end
writelines(md,  fullfile(tabDir, 'kpi_comparison.md'));
writelines(tex, fullfile(tabDir, 'kpi_comparison.tex'));
% all generic metrics
rows = {};
for s = 1:numel(SCENS)
    for c = 1:numel(C)
        d = D.(SCENS{s}).(tags{c}); M = bldc_metrics(d.R, d.SC, d.P);
        rows(end+1, :) = [SCENS(s), tags(c), struct2cell(M)']; %#ok<SAGROW>
    end
end
writetable(cell2table(rows, 'VariableNames', [{'Scenario', 'Controller'}, fieldnames(M)']), ...
    fullfile(tabDir, 'metrics_all.csv'));
disp(T);

% ---------------- figures (English labels) ----------------
rpm = 60/(2*pi);
col = struct('conv', [0.16 0.47 0.84], 'comp', [0.11 0.69 0.48]);   % cB, cC of the documents
lw  = 1.2;
A = D.S2.conv.R; B = D.S2.comp.R; P = D.S2.conv.P;
nAvg = round(P.Ts/(A.t(2) - A.t(1)));
Ta = movmean(A.Te, nAvg); Tb = movmean(B.Te, nAvg);

% fig7_1: steady-state torque, S2
f = figure('Visible', 'off', 'Position', [100 100 900 330], 'Color', 'w');
k = A.t >= 0.45 & A.t <= 0.55;
plot(A.t(k), Ta(k), 'Color', col.conv, 'LineWidth', lw); hold on;
plot(B.t(k), Tb(k), 'Color', col.comp, 'LineWidth', lw); yline(1, 'k--');
grid on; xlabel('Time (s)'); ylabel('Torque (N\cdotm)');
legend('conventional FOC', 'compensated FOC', 'T_L', 'Location', 'northoutside', 'Orientation', 'horizontal');
exportgraphics(f, fullfile(figDir, 'fig7_1_S2_torque.png'), 'Resolution', 150); close(f);

% fig7_2: d-q currents, S2
f = figure('Visible', 'off', 'Position', [100 100 900 520], 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; plot(A.t(k), A.iq(k), 'Color', col.conv, 'LineWidth', lw); hold on;
plot(A.t(k), A.iq_cmd(k), 'k--', 'LineWidth', 1); plot(A.t(k), A.id(k), 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
grid on; ylabel('Current (A)'); ylim([-0.15 0.65]); title('(a) Conventional FOC');
legend('i_q', 'i_q^* (reference)', 'i_d', 'Location', 'eastoutside');
nexttile; plot(B.t(k), B.iq(k), 'Color', col.comp, 'LineWidth', lw); hold on;
plot(B.t(k), B.iq_cmd(k), 'k--', 'LineWidth', 1); plot(B.t(k), B.id(k), 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
grid on; xlabel('Time (s)'); ylabel('Current (A)'); ylim([-0.15 0.65]); title('(b) Compensated FOC');
legend('i_q', 'i_q^{**} = k_s i_q^*', 'i_d', 'Location', 'eastoutside');
exportgraphics(f, fullfile(figDir, 'fig7_2_S2_idq.png'), 'Resolution', 150); close(f);

% fig7_3: speed and torque spectrum, S2
f = figure('Visible', 'off', 'Position', [100 100 900 360], 'Color', 'w');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; plot(A.t, A.w_ref*rpm, 'k--'); hold on;
plot(A.t, A.w*rpm, 'Color', col.conv, 'LineWidth', 1); plot(B.t, B.w*rpm, 'Color', col.comp, 'LineWidth', 1);
grid on; xlim([0 0.8]); xlabel('Time (s)'); ylabel('Speed (rpm)'); title('(a) Speed');
legend('\omega_m^*', 'conventional', 'compensated', 'Location', 'south');
nexttile; kk = A.t >= 0.45 & A.t < 0.575;                 % 10 periods of 80 Hz
[fa, Sa] = spec(A.t(kk), Ta(kk)); [~, Sb] = spec(B.t(kk), Tb(kk));
stem(fa, Sa, 'Color', col.conv, 'Marker', 'none', 'LineWidth', 2); hold on;
stem(fa + 2, Sb, 'Color', col.comp, 'Marker', 'none', 'LineWidth', 2);
grid on; xlim([0 500]); xlabel('Frequency (Hz)'); ylabel('Amplitude (N\cdotm)'); title('(b) Torque ripple spectrum');
legend('conventional', 'compensated');
exportgraphics(f, fullfile(figDir, 'fig7_3_S2_speed_spectrum.png'), 'Resolution', 150); close(f);

% fig7_4: phase currents, S2
f = figure('Visible', 'off', 'Position', [100 100 900 480], 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
kc = A.t >= 0.45 & A.t <= 0.525;
nexttile; plot(A.t(kc), A.i_abc(kc, :), 'LineWidth', 1); grid on; ylabel('i_{abc} (A)'); title('(a) Conventional FOC');
nexttile; plot(B.t(kc), B.i_abc(kc, :), 'LineWidth', 1); grid on; ylabel('i_{abc} (A)'); xlabel('Time (s)'); title('(b) Compensated FOC');
legend('i_a', 'i_b', 'i_c', 'Location', 'eastoutside');
exportgraphics(f, fullfile(figDir, 'fig7_4_S2_phase_currents.png'), 'Resolution', 150); close(f);

% fig7_5: speed responses S1, S3, S4
f = figure('Visible', 'off', 'Position', [100 100 900 700], 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ttl = {'S1', '(a) S1 – no-load start-up'; 'S3', '(b) S3 – speed reversal'; 'S4', '(c) S4 – staircase reference, 0.5 N\cdotm'};
for s = 1:3
    a = D.(ttl{s,1}).conv.R; b = D.(ttl{s,1}).comp.R;
    nexttile; plot(a.t, a.w_ref*rpm, 'k--'); hold on;
    plot(a.t, a.w*rpm, 'Color', col.conv, 'LineWidth', 1); plot(b.t, b.w*rpm, 'Color', col.comp, 'LineWidth', 1);
    grid on; ylabel('Speed (rpm)'); title(ttl{s,2}); xlim([0 a.t(end)]);
end
xlabel('Time (s)'); legend('\omega_m^*', 'conventional FOC', 'compensated FOC', 'Location', 'southoutside', 'Orientation', 'horizontal');
exportgraphics(f, fullfile(figDir, 'fig7_5_speed_S1_S3_S4.png'), 'Resolution', 150); close(f);

% fig7_6: summary of steady-state indicators (S2), compensated relative to conventional
sel = {'Te_pp', 'Te_rms', 'T6', 'w_pp', 'thd_pct'};
lab = {'torque p-p', 'torque RMS', '6th harmonic', 'speed p-p', 'current THD'};
v = cellfun(@(x) KPI.S2.comp.(x)/KPI.S2.conv.(x)*100, sel);
f = figure('Visible', 'off', 'Position', [100 100 900 340], 'Color', 'w');
bh = bar(categorical(lab, lab), [100*ones(1, numel(sel)); v]', 'grouped');
bh(1).FaceColor = col.conv; bh(2).FaceColor = col.comp;
grid on; ylabel('Relative to conventional FOC (%)'); ylim([0 115]);
legend('conventional FOC', 'compensated FOC', 'Location', 'northoutside', 'Orientation', 'horizontal');
text(bh(2).XEndPoints, bh(2).YEndPoints + 3, compose('%.0f %%', v), 'HorizontalAlignment', 'center');
exportgraphics(f, fullfile(figDir, 'fig7_6_S2_summary.png'), 'Resolution', 150); close(f);
fprintf('make_results: tables in %s, figures fig7_*.png in %s\n', tabDir, figDir);

%% ---------------- local functions ----------------
function out = ifelse(c, a, b)
if c, out = a; else, out = b; end
end

function [f, A] = spec(t, x)
% Single-sided amplitude spectrum of x (mean removed).
x = x(:) - mean(x); N = numel(x); dt = t(2) - t(1);
X = fft(x)/N; A = 2*abs(X(1:floor(N/2))); f = (0:floor(N/2)-1)'/(N*dt);
end
