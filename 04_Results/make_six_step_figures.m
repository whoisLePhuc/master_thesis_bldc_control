%% MAKE_SIX_STEP_FIGURES  Figures and numbers of Theory doc 06 (six-step, Chapter 8)
%  Reads 04_Results/Raw/<SCEN>_{conv,comp,six}.mat and writes
%    01_Documents/Thesis/figures/fig8_1, fig8_5 ... fig8_9 (*.png, English labels)
%    04_Results/Tables/six_step_numbers.csv   (values quoted in the document)
%  Fig. 8.2 is drawn in foc_block_diagram.drawio; the Simulink screenshots
%  fig8_3, fig8_4 (fig8_sim_six_*.png) are exported with SIMULINK_SCREENSHOTS = true.
%  Run run_conv_foc, run_comp_foc and run_six_step first.
%
%  Usage:  >> make_six_step_figures

here    = fileparts(mfilename('fullpath'));
repo    = fileparts(here);
simRoot = fullfile(repo, '03_Simulation');
addpath(fullfile(simRoot, 'Common'));
rawDir = fullfile(here, 'Raw');
tabDir = fullfile(here, 'Tables');
figDir = fullfile(repo, '01_Documents', 'Thesis', 'figures');

tags = {'conv', 'comp', 'six'};
names = {'conventional FOC', 'compensated FOC', 'six-step'};
col = struct('conv', [0.16 0.47 0.84], 'comp', [0.11 0.69 0.48], 'six', [0.85 0.37 0.01]);
lw = 1.2; rpm = 60/(2*pi);
SCENS = {'S1', 'S2', 'S3', 'S4'};
D = struct();
for s = 1:numel(SCENS)
    for c = 1:numel(tags)
        d = load(fullfile(rawDir, sprintf('%s_%s.mat', SCENS{s}, tags{c})), 'R', 'P');
        d.SC = bldc_scenario(SCENS{s});
        D.(SCENS{s}).(tags{c}) = d;
    end
end
P = bldc_params();
nAvg = round(P.Ts/(D.S2.six.R.t(2) - D.S2.six.R.t(1)));     % 1 PWM period

% ---------------- fig8_1: back-EMF, Hall signals and ideal currents ----------------
th = linspace(0, 2*pi, 3601);
fabc = [-bldc_trap(th); -bldc_trap(th - 2*pi/3); -bldc_trap(th + 2*pi/3)];
H    = [-bldc_trap(th - pi/6) > 0; -bldc_trap(th - pi/6 - 2*pi/3) > 0; -bldc_trap(th - pi/6 + 2*pi/3) > 0];
sec  = floor(mod(th - pi/6, 2*pi)/(pi/3));
thc  = pi/6 + ((0:5) + 0.5)*pi/3;
tab  = round([-bldc_trap(thc); -bldc_trap(thc - 2*pi/3); -bldc_trap(thc + 2*pi/3)]);
sabc = tab(:, sec + 1);
deg = th*180/pi; bnd = 30:60:330; pc = lines(3);
f = figure('Visible', 'off', 'Position', [100 100 900 720], 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; hold on;
for x = 1:3, plot(deg, fabc(x, :), 'Color', pc(x, :), 'LineWidth', 1.5); end
xline(bnd, ':', 'Color', [0.4 0.4 0.4]); ylim([-1.3 1.65]); xlim([0 360]); xticks(0:30:360); grid on;
ylabel('e_x / (\omega_e\lambda)'); title('(a) Normalized back-EMF');
for k = 0:5
    xc = mod(thc(k+1)*180/pi, 360); if k == 5, xc = [15 345]; end
    text(xc, 1.4*ones(size(xc)), sprintf('S%d', k), 'HorizontalAlignment', 'center', 'FontSize', 9);
end
legend('e_a', 'e_b', 'e_c', 'Location', 'eastoutside');
nexttile; hold on;
for x = 1:3, stairs(deg, H(x, :) + 1.5*(3 - x), 'Color', pc(x, :), 'LineWidth', 1.5); end
xline(bnd, ':', 'Color', [0.4 0.4 0.4]); xlim([0 360]); ylim([-0.3 4.3]); xticks(0:30:360); grid on;
yticks([0 1 1.5 2.5 3 4]); yticklabels({'0', '1', '0', '1', '0', '1'});
ylabel('Hall signals'); title('(b) Hall signals H_a, H_b, H_c (top to bottom)');
legend('H_a', 'H_b', 'H_c', 'Location', 'eastoutside');
nexttile; hold on;
for x = 1:3, stairs(deg, sabc(x, :)*(1 - 0.06*(x - 2)), 'Color', pc(x, :), 'LineWidth', 1.5); end
xline(bnd, ':', 'Color', [0.4 0.4 0.4]); xlim([0 360]); ylim([-1.4 1.4]); grid on;
xlabel('\theta_e (deg)'); ylabel('i_x / I'); title('(c) Ideal phase currents (120° conduction)');
legend('i_a', 'i_b', 'i_c', 'Location', 'eastoutside');
xticks(0:30:360);
exportgraphics(f, fullfile(figDir, 'fig8_1_hall_commutation.png'), 'Resolution', 150); close(f);

% ---------------- fig8_5: steady-state torque of the three controllers, S2 ----------------
f = figure('Visible', 'off', 'Position', [100 100 900 640], 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:3
    R = D.S2.(tags{c}).R; k = R.t >= 0.45 & R.t <= 0.50; Tm = movmean(R.Te, nAvg);
    nexttile; plot(R.t(k), Tm(k), 'Color', col.(tags{c}), 'LineWidth', lw); hold on; yline(1, 'k--');
    grid on; ylim([0.5 1.35]); ylabel('T_e (N\cdotm)');
    title(sprintf('(%c) %s', 'a' + c - 1, names{c}));
end
xlabel('Time (s)');
exportgraphics(f, fullfile(figDir, 'fig8_5_S2_torque_3ctrl.png'), 'Resolution', 150); close(f);

% ---------------- fig8_6: one commutation (six-step, S2) ----------------
R = D.S2.six.R; Tm = movmean(R.Te, nAvg);
k = R.t >= 0.48 & R.t <= 0.495; [~, i0] = min(Tm(k)); t0s = R.t(k); t0 = t0s(i0);
k = R.t >= t0 - 1.5e-3 & R.t <= t0 + 2.5e-3; tt = (R.t(k) - t0)*1e3;
f = figure('Visible', 'off', 'Position', [100 100 900 520], 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; plot(tt, R.i_abc(k, :), 'LineWidth', 1.2); hold on; plot(tt, R.iq_ref(k), 'k--');
grid on; ylabel('Current (A)'); title('(a) Phase currents and current reference I^*');
legend('i_a', 'i_b', 'i_c', 'I^*', 'Location', 'eastoutside');
nexttile; plot(tt, R.Te(k), 'Color', [0.75 0.75 0.75]); hold on;
plot(tt, Tm(k), 'Color', col.six, 'LineWidth', 1.5); yline(1, 'k--');
grid on; ylabel('T_e (N\cdotm)'); xlabel('Time relative to the torque minimum (ms)');
title('(b) Electromagnetic torque'); legend('instantaneous', '1-PWM-period average', 'T_L', 'Location', 'eastoutside');
exportgraphics(f, fullfile(figDir, 'fig8_6_commutation_zoom.png'), 'Resolution', 150); close(f);

% ---------------- fig8_7: phase current i_a of the three controllers, S2 ----------------
f = figure('Visible', 'off', 'Position', [100 100 900 330], 'Color', 'w'); hold on;
for c = 1:3
    R = D.S2.(tags{c}).R; k = R.t >= 0.45 & R.t <= 0.525; ia = movmean(R.i_abc(:, 1), nAvg);
    plot(R.t(k), ia(k), 'Color', col.(tags{c}), 'LineWidth', lw);
end
grid on; xlabel('Time (s)'); ylabel('i_a (A)');
legend(names, 'Location', 'northoutside', 'Orientation', 'horizontal');
exportgraphics(f, fullfile(figDir, 'fig8_7_S2_phase_current_3ctrl.png'), 'Resolution', 150); close(f);

% ---------------- fig8_8: speed responses S1, S3, S4 ----------------
f = figure('Visible', 'off', 'Position', [100 100 900 700], 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ttl = {'S1', '(a) S1 – no-load start-up'; 'S3', '(b) S3 – speed reversal'; 'S4', '(c) S4 – staircase reference, 0.5 N\cdotm'};
for s = 1:3
    nexttile; a = D.(ttl{s,1}).conv.R; plot(a.t, a.w_ref*rpm, 'k--'); hold on;
    for c = 1:3
        R = D.(ttl{s,1}).(tags{c}).R; plot(R.t, R.w*rpm, 'Color', col.(tags{c}), 'LineWidth', 1);
    end
    grid on; ylabel('Speed (rpm)'); title(ttl{s,2}); xlim([0 a.t(end)]);
end
xlabel('Time (s)'); legend([{'\omega_m^*'}, names], 'Location', 'southoutside', 'Orientation', 'horizontal');
exportgraphics(f, fullfile(figDir, 'fig8_8_speed_S1_S3_S4.png'), 'Resolution', 150); close(f);

% ---------------- indicators, copper loss, commutation dip ----------------
K = struct(); Pcu = zeros(1, 3); Irms = zeros(1, 3); Tmin = zeros(1, 3);
for c = 1:3
    d = D.S2.(tags{c}); R = d.R; K.(tags{c}) = bldc_kpi(R, d.SC, d.P, 'S2');
    k = R.t >= d.SC.win(1) & R.t < d.SC.win(2);
    Irms(c) = sqrt(mean(R.i_abc(k, :).^2, 'all')); Pcu(c) = 3*P.R*Irms(c)^2;
    Tm = movmean(R.Te, nAvg); Tmin(c) = min(Tm(k));
end
% fig8_9: summary relative to conventional FOC
sel = {'Te_pp', 'Te_rms', 'T6', 'w_pp', 'thd_pct'};
lab = {'torque p-p', 'torque RMS', '6th harmonic', 'speed p-p', 'current THD', 'copper loss'};
V = zeros(3, numel(lab));
for c = 1:3
    V(c, :) = [cellfun(@(x) K.(tags{c}).(x)/K.conv.(x)*100, sel), Pcu(c)/Pcu(1)*100];
end
f = figure('Visible', 'off', 'Position', [100 100 900 360], 'Color', 'w');
bh = bar(categorical(lab, lab), V', 'grouped');
for c = 1:3, bh(c).FaceColor = col.(tags{c}); end
grid on; ylabel('Relative to conventional FOC (%)'); ylim([0 160]);
legend(names, 'Location', 'northoutside', 'Orientation', 'horizontal');
for c = 2:3
    text(bh(c).XEndPoints, bh(c).YEndPoints + 4, compose('%.0f', V(c, :)), 'HorizontalAlignment', 'center', 'FontSize', 8);
end
exportgraphics(f, fullfile(figDir, 'fig8_9_S2_summary_3ctrl.png'), 'Resolution', 150); close(f);

% numbers quoted in the document
E   = P.lam*P.p*D.S2.six.SC.w_ref(end);                     % phase back-EMF at 100 rpm (V)
dip = (P.Vdc - 4*E)/(P.Vdc + 2*E);                          % Eq. (8.10), d = 1
wcr = P.Vdc/(4*P.lam*P.p)*rpm;                              % speed where 4E = Vdc (rpm)
N = table(string(tags'), Irms', Pcu', Tmin', 1 - Tmin', 'VariableNames', ...
    {'Controller', 'Irms_A', 'Pcu_W', 'Te_min_Nm', 'rel_dip'});
writetable(N, fullfile(tabDir, 'six_step_numbers.csv'));
disp(N);
fprintf('E = %.2f V, 4E = %.2f V, predicted dip (d = 1) = %.1f %%, 4E = Vdc at %.1f rpm\n', E, 4*E, -100*dip, wcr);

% ---------------- Simulink screenshots (optional) ----------------
if exist('SIMULINK_SCREENSHOTS', 'var') && SIMULINK_SCREENSHOTS
    mdl = 'Six_Step'; load_system(fullfile(simRoot, 'Six_Step', [mdl '.slx'])); open_system(mdl);
    print(['-s' mdl], '-dpng', '-r150', fullfile(figDir, 'fig8_sim_six_root.png'));
    open_system([mdl '/Six-Step Ctrl']);
    print(['-s' mdl '/Six-Step Ctrl'], '-dpng', '-r150', fullfile(figDir, 'fig8_sim_six_ctrl.png'));
end
fprintf('make_six_step_figures: fig8_*.png in %s\n', figDir);
