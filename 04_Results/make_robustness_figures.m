%% MAKE_ROBUSTNESS_FIGURES  Figures of the robustness study (Theory doc 08, Chapter 10)
%  Reads 04_Results/Tables/robustness.csv (and robustness_dt2vf.csv if present), written by
%  03_Simulation/Studies/run_robustness.m, and writes 01_Documents/Thesis/figures/
%    fig10_1_bemf_shape.png   torque ripple vs back-EMF flat-top width
%    fig10_2_encoder.png      torque ripple and copper loss vs encoder offset
%    fig10_3_heatmap.png      torque ripple of every controller and case (ideal inverter and dt2vf)
%
%  Usage:  >> make_robustness_figures

here   = fileparts(mfilename('fullpath'));
repo   = fileparts(here);
tabDir = fullfile(here, 'Tables');
figDir = fullfile(repo, '01_Documents', 'Thesis', 'figures');
T = readtable(fullfile(tabDir, 'robustness.csv'), 'TextType', 'string');
ctl = ["conv" "comp" "comp_v3" "six" "pvc" "pvcff"];
col = struct('conv', [0.16 0.47 0.84], 'comp', [0.11 0.69 0.48], 'comp_v3', [0.05 0.40 0.25], ...
             'six', [0.85 0.37 0.01], 'pvc', [0.55 0.27 0.68], 'pvcff', [0.85 0.15 0.45]);
nm  = struct('conv', 'conventional FOC', 'comp', 'compensated FOC', 'comp_v3', 'compensated FOC (V3)', ...
             'six', 'six-step', 'pvc', 'PVC', 'pvcff', 'PVC + feedforward');
val = @(c, cs, n, f) T.(f)(T.Controller == c & T.Case == cs & T.Speed_rpm == n);
pts = [100 30];

% ---------------- back-EMF shape ----------------
f2 = fullfile(tabDir, 'robustness_dt2vf.csv');
T2 = []; if exist(f2, 'file'), T2 = readtable(f2, 'TextType', 'string'); end
P3 = {T, 100, 'ideal inverter'; T, 30, 'ideal inverter'; T2, 100, '2 \mus dead time + 0.8 V'};
if isempty(T2), P3 = P3(1:2, :); end
val2 = @(TT, c, cs, n, f) TT.(f)(TT.Controller == c & TT.Case == cs & TT.Speed_rpm == n);
flats = [100 110 120 130 140];
cs = ["flat100" "flat110" "nominal" "flat130" "flat140"];
sel = ctl([1 2 3 4 6]);
f = figure('Visible', 'off', 'Position', [100 100 1200 420], 'Color', 'w');
tiledlayout(1, size(P3, 1), 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:size(P3, 1)
    nexttile; hold on; TT = P3{i, 1};
    for c = sel
        y = arrayfun(@(k) val2(TT, c, cs(k), P3{i, 2}, 'Te_rms'), 1:numel(cs));
        ls = '-'; if c == "comp_v3", ls = '--'; end
        plot(flats, y, [ls 'o'], 'Color', col.(c), 'LineWidth', 1.4, 'MarkerFaceColor', col.(c));
    end
    grid on; xticks(flats); xlabel('Motor back-EMF flat top (el. deg)'); ylabel('T_e ripple RMS (N\cdotm)');
    set(gca, 'YScale', 'log');
    title(sprintf('(%c) %d rpm, %g N\\cdotm, %s', 'a' + i - 1, P3{i, 2}, ...
          TT.Load_Nm(find(TT.Speed_rpm == P3{i, 2}, 1)), P3{i, 3}));
    if i == 1, legend(arrayfun(@(c) nm.(c), sel, 'UniformOutput', false), 'Location', 'northwest'); end
end
exportgraphics(f, fullfile(figDir, 'fig10_1_bemf_shape.png'), 'Resolution', 150); close(f);

% ---------------- heat map (absolute ripple) ----------------
cases = ["nominal" "flat100" "flat110" "flat130" "flat140" "lam0.9" "lam1.1" "L0.8" "L1.2" "R1.3" ...
         "enc256" "enc1024" "enc4096" "off-10" "off-5" "off+5" "off+10"];
clab = {'nominal', 'flat 100°', 'flat 110°', 'flat 130°', 'flat 140°', '\lambda -10%', '\lambda +10%', ...
        'L -20%', 'L +20%', 'R +30%', '256 cpr', '1024 cpr', '4096 cpr', '-10°', '-5°', '+5°', '+10°'};
f = figure('Visible', 'off', 'Position', [100 100 1100 330*size(P3, 1)], 'Color', 'w');
tiledlayout(size(P3, 1), 1, 'TileSpacing', 'loose', 'Padding', 'compact');
for i = 1:size(P3, 1)
    nexttile; TT = P3{i, 1};
    A = zeros(numel(ctl), numel(cases));
    for c = 1:numel(ctl)
        for k = 1:numel(cases), A(c, k) = val2(TT, ctl(c), cases(k), P3{i, 2}, 'Te_rms'); end
    end
    imagesc(log10(A)); clim(log10([0.015 0.4]));
    colormap(gca, interp1([0 0.5 1], [0.15 0.6 0.35; 1 0.95 0.6; 0.8 0.15 0.15], linspace(0, 1, 64)));
    for c = 1:numel(ctl)
        for k = 1:numel(cases)
            text(k, c, sprintf('%.3f', A(c, k)), 'HorizontalAlignment', 'center', 'FontSize', 7.5);
        end
    end
    xline(1.5, 'k-', 'LineWidth', 1); xline(5.5, 'k:'); xline(10.5, 'k:'); xline(13.5, 'k:');
    set(gca, 'XTick', 1:numel(cases), 'XTickLabel', clab, 'XTickLabelRotation', 0, 'FontSize', 8, ...
        'YTick', 1:numel(ctl), 'YTickLabel', arrayfun(@(c) nm.(c), ctl, 'UniformOutput', false), 'TickLength', [0 0]);
    title(sprintf('(%c) T_e ripple RMS (N\\cdotm), %d rpm, %g N\\cdotm, %s   [back-EMF shape | motor parameters | encoder resolution | encoder offset (el.)]', ...
          'a' + i - 1, P3{i, 2}, TT.Load_Nm(find(TT.Speed_rpm == P3{i, 2}, 1)), P3{i, 3}), 'FontSize', 9);
end
exportgraphics(f, fullfile(figDir, 'fig10_3_heatmap.png'), 'Resolution', 150); close(f);

% ---------------- encoder offset ----------------
offs = [-10 -5 0 5 10]; cs = ["off-10" "off-5" "nominal" "off+5" "off+10"];
f = figure('Visible', 'off', 'Position', [100 100 900 420], 'Color', 'w');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
fl = {'Te_rms', 'Pcu'}; yl = {'T_e ripple RMS (N\cdotm)', 'Copper loss (W)'};
for i = 1:2
    nexttile; hold on;
    for c = ctl([1 2 4 5 6])
        y = arrayfun(@(k) val(c, cs(k), 100, fl{i}), 1:numel(cs));
        plot(offs, y, '-o', 'Color', col.(c), 'LineWidth', 1.4, 'MarkerFaceColor', col.(c));
    end
    grid on; xticks(offs); xlabel('Encoder offset (el. deg)'); ylabel(yl{i});
    if i == 1, set(gca, 'YScale', 'log'); end
    title(sprintf('(%c) 100 rpm, 1 N\\cdotm', 'a' + i - 1));
    if i == 2, legend(arrayfun(@(c) nm.(c), ctl([1 2 4 5 6]), 'UniformOutput', false), 'Location', 'best'); end
end
exportgraphics(f, fullfile(figDir, 'fig10_2_encoder.png'), 'Resolution', 150); close(f);
fprintf('make_robustness_figures: figures in %s\n', figDir);
