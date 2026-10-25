%% MAKE_SWEEP_FIGURES  Figures of the speed-load sweep and inverter non-ideality study
%  Reads 04_Results/Tables/sweep_speed_load.csv (and sweep_speed_load_Vdc48.csv if present),
%  written by 03_Simulation/Studies/run_sweep_speed_load.m, and writes
%    01_Documents/Thesis/figures/fig9_sweep_ripple.png    torque ripple vs speed and load
%    01_Documents/Thesis/figures/fig9_sweep_pvc.png       PVC d-axis current error and copper loss vs speed
%    01_Documents/Thesis/figures/fig9_sweep_vdc.png       effect of the DC-link voltage at 100 rpm
%
%  Usage:  >> make_sweep_figures

here   = fileparts(mfilename('fullpath'));
repo   = fileparts(here);
addpath(fullfile(repo, '03_Simulation', 'Common'));
tabDir = fullfile(here, 'Tables');
figDir = fullfile(repo, '01_Documents', 'Thesis', 'figures');
T = readtable(fullfile(tabDir, 'sweep_speed_load.csv'), 'TextType', 'string');
P = bldc_params();

col = struct('conv', [0.16 0.47 0.84], 'comp', [0.11 0.69 0.48], 'six', [0.85 0.37 0.01], ...
             'pvc', [0.55 0.27 0.68], 'pvcff', [0.85 0.15 0.45]);
nm  = struct('conv', 'conventional FOC', 'comp', 'compensated FOC', 'six', 'six-step', ...
             'pvc', 'PVC', 'pvcff', 'PVC + feedforward');
loads = unique(T.Load_Nm)'; speeds = unique(T.Speed_rpm)';
pick = @(tag, cs, L) sortrows(T(T.Controller == tag & T.Case == cs & T.Load_Nm == L, :), 'Speed_rpm');

% ---------------- torque ripple vs speed ----------------
sel = {'comp', 'pvcff', 'six'};
f = figure('Visible', 'off', 'Position', [100 100 900 760], 'Color', 'w');
tl = tiledlayout(numel(loads), 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for i = 1:numel(loads)
    nexttile; hold on; h = gobjects(0); lab = {};
    for c = 1:numel(sel)
        a = pick(sel{c}, "ideal", loads(i)); b = pick(sel{c}, "dt2vf", loads(i));
        h(end+1) = plot(a.Speed_rpm, a.Te_rms, '-o', 'Color', col.(sel{c}), 'LineWidth', 1.4, 'MarkerFaceColor', col.(sel{c})); %#ok<SAGROW>
        h(end+1) = plot(b.Speed_rpm, b.Te_rms, '--s', 'Color', col.(sel{c}), 'LineWidth', 1.4); %#ok<SAGROW>
        lab = [lab, {[nm.(sel{c}) ', ideal'], [nm.(sel{c}) ', 2 \mus + 0.8 V']}]; %#ok<AGROW>
    end
    grid on; xlim([15 105]); xticks(speeds); ylabel('T_e ripple RMS (N\cdotm)');
    title(sprintf('(%c) Load %.2g N\\cdotm', 'a' + i - 1, loads(i)));
end
xlabel('Speed (rpm)');
lg = legend(h, lab, 'NumColumns', 3); lg.Layout.Tile = 'south';
exportgraphics(f, fullfile(figDir, 'fig9_sweep_ripple.png'), 'Resolution', 150); close(f);

% ---------------- PVC d-axis current error and copper loss ----------------
w = linspace(1, max(speeds)*1.1, 200)*2*pi/60*P.p;
ie = -(P.lam1*w)./((P.R + 1j*w*P.L) + (P.Kp_i + P.Ki_i./(1j*w)));
f = figure('Visible', 'off', 'Position', [100 100 900 380], 'Color', 'w');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; hold on; plot(w/P.p*60/(2*pi), -imag(ie), 'k-', 'LineWidth', 1.2); mk = {'o', 's', '^'};
for i = 1:numel(loads)
    a = pick("pvc", "ideal", loads(i));
    plot(a.Speed_rpm, a.id_mean, mk{i}, 'Color', col.pvc, 'MarkerFaceColor', col.pvc*(i == 1) + [1 1 1]*(i ~= 1));
end
grid on; xlabel('Speed (rpm)'); ylabel('i_d (A)'); title('(a) PVC: d-axis current');
legend(['Eq. (9.9)', compose('simulation, %.2g N\\cdotm', loads)], 'Location', 'northwest');
nexttile; hold on;
for c = {'conv', 'comp', 'six', 'pvc', 'pvcff'}
    a = pick(c{1}, "ideal", 1);
    plot(a.Speed_rpm, a.Pcu, '-o', 'Color', col.(c{1}), 'LineWidth', 1.4, 'MarkerFaceColor', col.(c{1}));
end
grid on; xlabel('Speed (rpm)'); ylabel('Copper loss (W)'); title('(b) Copper loss at 1 N\cdotm');
legend(cellfun(@(c) nm.(c), {'conv', 'comp', 'six', 'pvc', 'pvcff'}, 'UniformOutput', false), 'Location', 'northwest');
exportgraphics(f, fullfile(figDir, 'fig9_sweep_pvc.png'), 'Resolution', 150); close(f);

% ---------------- DC-link voltage check at 100 rpm ----------------
f48 = fullfile(tabDir, 'sweep_speed_load_Vdc48.csv');
if exist(f48, 'file')
    T48 = readtable(f48, 'TextType', 'string');
    V = zeros(numel(loads), 4);
    for i = 1:numel(loads)
        g = @(TT, tag) TT.Te_rms(TT.Controller == tag & TT.Case == "dt2vf" & TT.Load_Nm == loads(i) & TT.Speed_rpm == 100);
        V(i, :) = [g(T, "comp"), g(T, "pvcff"), g(T48, "comp"), g(T48, "pvcff")];
    end
    f = figure('Visible', 'off', 'Position', [100 100 900 340], 'Color', 'w');
    bh = bar(categorical(compose('%.2g N\\cdotm', loads)), V, 'grouped');
    bh(1).FaceColor = col.comp; bh(2).FaceColor = col.pvcff;
    bh(3).FaceColor = col.comp*0.5 + 0.5; bh(4).FaceColor = col.pvcff*0.5 + 0.5;
    grid on; ylabel('T_e ripple RMS (N\cdotm)'); title('100 rpm, 2 \mus dead time + 0.8 V device drops');
    legend('compensated FOC, V_{dc} = 36 V', 'PVC + feedforward, V_{dc} = 36 V', ...
           'compensated FOC, V_{dc} = 48 V', 'PVC + feedforward, V_{dc} = 48 V', 'Location', 'northwest');
    exportgraphics(f, fullfile(figDir, 'fig9_sweep_vdc.png'), 'Resolution', 150); close(f);
end
fprintf('make_sweep_figures: figures in %s\n', figDir);
