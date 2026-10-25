%% MAKE_STEP1_FIGURES  Figures of the follow-up studies of PVC (Theory doc 07, Sec. 9.8-9.10)
%  Reads (written by 03_Simulation/Studies/...)
%    04_Results/Tables/voltage_limit.csv      run_voltage_limit
%    04_Results/Tables/sensor_error.csv       run_sensor_error
%    04_Results/Raw/zero_sequence_drift.mat   check_zero_sequence
%    04_Results/Raw/S5_high_speed.mat         run_high_speed
%  and writes 01_Documents/Thesis/figures/
%    fig9_14_vlim.png         torque ripple of Compensated FOC with each voltage limiter vs PVC + ff
%    fig9_15_sensor.png       torque and speed ripple with current-sensor offset / gain errors
%    fig9_16_zero_seq.png     zero-sequence wind-up of the PVC phase PI controllers
%    fig9_17_high_speed.png   scenario S5: speed, d-axis current, speed reached vs copper loss
%
%  Usage:  >> make_step1_figures

here   = fileparts(mfilename('fullpath'));
repo   = fileparts(here);
addpath(fullfile(repo, '03_Simulation', 'Common'));
tabDir = fullfile(here, 'Tables'); rawDir = fullfile(here, 'Raw');
figDir = fullfile(repo, '01_Documents', 'Thesis', 'figures');
col = struct('conv', [0.16 0.47 0.84], 'comp', [0.11 0.69 0.48], 'six', [0.85 0.37 0.01], ...
             'pvc', [0.55 0.27 0.68], 'pvcff', [0.85 0.15 0.45]);
nm  = struct('conv', 'conventional FOC', 'comp', 'compensated FOC', 'six', 'six-step', ...
             'pvc', 'PVC', 'pvcff', 'PVC + feedforward');

% ---------------- voltage limiters ----------------
T = readtable(fullfile(tabDir, 'voltage_limit.csv'), 'TextType', 'string');
tags = ["comp_v0" "comp_v1" "comp_v2" "comp_v3" "pvcff"];
lab  = {'comp. FOC, clamp u_d, u_q (original)', 'comp. FOC, |u_{dq}| \leq V_m', 'comp. FOC, phase clamp (as PVC)', ...
        'comp. FOC, inverter hexagon', 'PVC + feedforward'};
cl = [col.comp; col.comp*0.75 + 0.25; col.comp*0.5 + 0.5; [0.05 0.35 0.25]; col.pvcff];
loads = unique(T.Load_Nm)';
f = figure('Visible', 'off', 'Position', [100 100 900 640], 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
vds = [36 48];
for i = 1:2
    nexttile;
    V = zeros(numel(loads), numel(tags));
    for j = 1:numel(loads)
        for c = 1:numel(tags)
            V(j, c) = T.Te_rms(T.Controller == tags(c) & T.Vdc == vds(i) & T.Case == "dt2vf" & ...
                               T.Speed_rpm == 100 & T.Load_Nm == loads(j));
        end
    end
    bh = bar(categorical(compose('%.2g N\\cdotm', loads)), V, 'grouped');
    for c = 1:numel(tags), bh(c).FaceColor = cl(c, :); end
    grid on; ylabel('T_e ripple RMS (N\cdotm)'); ylim([0 0.14]);
    title(sprintf('(%c) 100 rpm, V_{dc} = %d V, 2 \\mus dead time + 0.8 V device drops', 'a' + i - 1, vds(i)));
end
lg = legend(bh, lab, 'NumColumns', 3); lg.Layout.Tile = 'south';
exportgraphics(f, fullfile(figDir, 'fig9_14_vlim.png'), 'Resolution', 150); close(f);

% ---------------- current-sensor errors ----------------
T = readtable(fullfile(tabDir, 'sensor_error.csv'), 'TextType', 'string');
ctl = ["conv" "comp" "six" "pvc" "pvcff"]; cases = ["ideal" "offA3" "offA2" "gainB3" "gainB2"];
clab = {'ideal', 'phase-a offset 30 mA, 3 sensors', 'phase-a offset 30 mA, 2 sensors', ...
        'phase-b gain +3 %, 3 sensors', 'phase-b gain +3 %, 2 sensors'};
gc = [0.75 0.75 0.75; 0.85 0.33 0.10; 0.95 0.65 0.45; 0.20 0.40 0.75; 0.55 0.70 0.90];
f = figure('Visible', 'off', 'Position', [100 100 900 620], 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
vars = ["Te_rms" "w_pp"]; yl = {'T_e ripple RMS (N\cdotm)', 'Speed ripple p-p (rpm)'};
for i = 1:2
    nexttile; V = zeros(numel(ctl), numel(cases));
    for c = 1:numel(ctl)
        for k = 1:numel(cases)
            V(c, k) = T.(vars(i))(T.Controller == ctl(c) & T.Sensor == cases(k) & T.Speed_rpm == 100);
        end
    end
    bh = bar(categorical(cellfun(@(t) nm.(t), cellstr(ctl), 'UniformOutput', false), ...
             cellfun(@(t) nm.(t), cellstr(ctl), 'UniformOutput', false)), V, 'grouped');
    for k = 1:numel(cases), bh(k).FaceColor = gc(k, :); end
    grid on; ylabel(yl{i}); title(sprintf('(%c) 100 rpm, 1 N\\cdotm, ideal inverter', 'a' + i - 1));
end
lg = legend(bh, clab, 'NumColumns', 3); lg.Layout.Tile = 'south';
exportgraphics(f, fullfile(figDir, 'fig9_15_sensor.png'), 'Resolution', 150); close(f);

% ---------------- zero-sequence wind-up ----------------
fz = fullfile(rawDir, 'zero_sequence_drift.mat');
if exist(fz, 'file')
    Z = load(fz); D = Z.D; P = bldc_params();
    f = figure('Visible', 'off', 'Position', [100 100 900 600], 'Color', 'w');
    tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
    nexttile; hold on; cz = [col.pvcff; 0.4 0.4 0.4];
    for i = 1:numel(D)
        v0 = mean(D(i).v_pi, 2);
        plot(D(i).t, v0, 'Color', cz(i, :), 'LineWidth', 1.4);
    end
    t = D(1).t; plot(t, max(-P.Vm, D(1).rate_theory*t), 'k:', 'LineWidth', 1.2);
    yline(-P.Vm, 'k--');
    grid on; ylabel('v_0 of the PI outputs (V)');
    legend([compose('PVC + feedforward, %g rpm, %g N\\cdotm', [[D.n]' [D.TL]']); {'-K_{i,i}\cdot\delta_a t/3'; '-V_m'}], ...
           'Location', 'southwest');
    title('(a) Zero-sequence component of the three phase PI outputs (phase-a offset 30 mA, 3 sensors)');
    nexttile; hold on;
    for i = 1:numel(D)
        plot(D(i).Te(:, 1), D(i).Te(:, 2)/D(i).TL, 'Color', cz(i, :), 'LineWidth', 1);
    end
    grid on; ylim([0 2]); xlabel('Time (s)'); ylabel('T_e / T_L');
    title('(b) Torque (averaged over one PWM period) / load torque');
    exportgraphics(f, fullfile(figDir, 'fig9_16_zero_seq.png'), 'Resolution', 150); close(f);
end

% ---------------- scenario S5 ----------------
H = load(fullfile(rawDir, 'S5_high_speed.mat')); S = H.S; HT = H.T;
sel = {'comp', 'comp_fw', 'comp_v3fw', 'pvcff', 'six', 'comp_v3'};          % smooth ones drawn last
lb  = struct('conv', 'conventional FOC', 'comp', 'comp. FOC (clamp u_d, u_q)', 'comp_fw', 'comp. FOC + FW', ...
             'comp_v3', 'comp. FOC, hexagon', 'comp_v3fw', 'comp. FOC, hexagon + FW', ...
             'six', 'six-step', 'pvc', 'PVC (FW of Eq. 9.4)', 'pvcff', 'PVC + ff (FW of Eq. 9.4)', ...
             'pvcff_nofw', 'PVC + ff, no FW');
cs = struct('conv', col.conv, 'comp', [0.55 0.85 0.70], 'comp_fw', [0.95 0.75 0.10], 'comp_v3', [0.05 0.40 0.25], ...
            'comp_v3fw', [0.10 0.10 0.10], 'six', col.six, 'pvc', col.pvc, 'pvcff', col.pvcff, 'pvcff_nofw', col.pvcff*0.5 + 0.5);
f = figure('Visible', 'off', 'Position', [100 100 900 820], 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; hold on; rpm = 60/(2*pi);
for k = 1:numel(sel), s = S.(sel{k}); lw = 1 + 0.8*any(strcmp(sel{k}, {'six', 'comp_v3'})); plot(s.t, movmean(s.w*rpm, 20), 'Color', cs.(sel{k}), 'LineWidth', lw); end
plot(s.t, s.w_ref*rpm, 'k:'); yline(119.3, 'k--', 'base speed (9.5)', 'LabelHorizontalAlignment', 'left');
grid on; xlim([0.25 0.8]); ylim([100 145]); ylabel('Speed (rpm)'); title('(a) Speed (2 ms moving average)');
legend([cellfun(@(c) lb.(c), sel, 'UniformOutput', false), {'reference'}], 'Location', 'eastoutside');
nexttile; hold on;
for k = 1:numel(sel), s = S.(sel{k}); lw = 1 + 0.8*any(strcmp(sel{k}, {'six', 'comp_v3'})); plot(s.t, movmean(s.id, 50), 'Color', cs.(sel{k}), 'LineWidth', lw); end
grid on; xlim([0.25 0.8]); ylabel('i_d (A)'); xlabel('Time (s)'); title('(b) d-axis current (5 ms moving average)');
nexttile; hold on;
for k = 1:height(HT)
    v = char(HT.Variant(k)); scatter(HT.Pcu(k), HT.w_mean(k), 60, cs.(v), 'filled');
    dy = 0; if any(strcmp(v, {'six', 'conv'})), dy = -0.9; end
    text(HT.Pcu(k)*1.15, HT.w_mean(k) + dy, sprintf('%s (%.2f N\\cdotm)', lb.(v), HT.Te_rms(k)), 'FontSize', 8);
end
set(gca, 'XScale', 'log'); grid on; xlabel('Copper loss (W, log scale)'); ylabel('Speed reached (rpm)');
title('(c) Steady-state speed vs copper loss at 0.25 N\cdotm (torque ripple RMS in brackets)'); xlim([5e-3 100]); ylim([116 133]);
exportgraphics(f, fullfile(figDir, 'fig9_17_high_speed.png'), 'Resolution', 150); close(f);
fprintf('make_step1_figures: figures in %s\n', figDir);
