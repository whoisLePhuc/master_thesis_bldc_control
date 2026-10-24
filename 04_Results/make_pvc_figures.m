%% MAKE_PVC_FIGURES  Figures and numbers of Theory doc 07 (pseudo vector control, Chapter 9)
%  Reads 04_Results/Raw/<SCEN>_{conv,comp,six,pvc,pvcff}.mat and writes
%    01_Documents/Thesis/figures/fig9_2, fig9_3, fig9_6 ... fig9_9 (*.png, English labels)
%    04_Results/Tables/pvc_numbers.csv   (values quoted in the document)
%  Fig. 9.1 is drawn in foc_block_diagram.drawio; the Simulink screenshots
%  fig9_sim_pvc_*.png (Figs. 9.4, 9.5) are exported with SIMULINK_SCREENSHOTS = true.
%  Run the run_*.m scripts of all controllers first.
%
%  Usage:  >> make_pvc_figures

here    = fileparts(mfilename('fullpath'));
repo    = fileparts(here);
simRoot = fullfile(repo, '03_Simulation');
addpath(fullfile(simRoot, 'Common'));
rawDir = fullfile(here, 'Raw');
tabDir = fullfile(here, 'Tables');
figDir = fullfile(repo, '01_Documents', 'Thesis', 'figures');

tags  = {'conv', 'comp', 'six', 'pvc', 'pvcff'};
names = {'conventional FOC', 'compensated FOC', 'six-step', 'PVC', 'PVC + feedforward'};
col = struct('conv', [0.16 0.47 0.84], 'comp', [0.11 0.69 0.48], 'six', [0.85 0.37 0.01], ...
             'pvc', [0.55 0.27 0.68], 'pvcff', [0.85 0.15 0.45]);
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
nAvg = round(P.Ts/(D.S2.pvc.R.t(2) - D.S2.pvc.R.t(1)));     % 1 PWM period

% ---------------- fig9_2: current references of PVC ----------------
th = linspace(0, 2*pi, 3601);
fa = -bldc_trap(th); fb = -bldc_trap(th - 2*pi/3); fc = -bldc_trap(th + 2*pi/3);
fal = 2/3*(fa - fb/2 - fc/2); fbe = (fb - fc)/sqrt(3);
gd = cos(th).*fal + sin(th).*fbe; gq = -sin(th).*fal + cos(th).*fbe;
Tref = 1; iqF = Tref/P.Kt_foc;                         % FOC (sinusoidal) reference for 1 N.m
iq = (Tref/(1.5*P.p*P.lam))./gq;                       % Eq. (9.3), i_d* = 0
ia = -sin(th).*iq;                                     % inverse Park, i_d* = 0: i_a = cos*id - sin*iq
iaF = -sin(th)*iqF;
deg = th*180/pi;
wm = linspace(0, 2.5*P.Vm/(P.p*P.lam1), 500);          % mechanical speed (rad/s)
wb = P.Vm/(P.p*P.lam1);
kfw = sqrt(max(0, 1 - min(1, wb./max(wm, 1e-9)).^2));
f = figure('Visible', 'off', 'Position', [100 100 900 720], 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile; plot(deg, iq, 'Color', col.pvc, 'LineWidth', 1.5); hold on;
yline(iqF, 'k--'); plot(deg, zeros(size(deg)), 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
grid on; xlim([0 360]); xticks(0:60:360); ylabel('Current (A)');
title('(a) d-q current references for T^* = 1 N\cdotm, below base speed');
legend('i_q^* (PVC, Eq. 9.3)', 'i_q^* (FOC, sinusoidal)', 'i_d^*', 'Location', 'eastoutside');
nexttile; plot(deg, ia, 'Color', col.pvc, 'LineWidth', 1.5); hold on; plot(deg, iaF, 'k--');
grid on; xlim([0 360]); xticks(0:60:360); ylabel('i_a^* (A)'); xlabel('\theta_e (deg)');
title('(b) Phase-current reference'); legend('PVC', 'FOC (sinusoidal)', 'Location', 'eastoutside');
nexttile; plot(wm*rpm, -kfw, 'Color', col.pvc, 'LineWidth', 1.5); hold on; xline(wb*rpm, 'k:');
grid on; ylim([-1.05 0.1]); xlabel('\omega_m (rpm)'); ylabel('i_d^* / |T^*/K_t|');
title(sprintf('(c) Field-weakening d-axis reference, base speed %.0f rpm (Eq. 9.4)', wb*rpm));
legend('i_d^*', '\omega_b', 'Location', 'eastoutside');
exportgraphics(f, fullfile(figDir, 'fig9_2_pvc_references.png'), 'Resolution', 150); close(f);

% ---------------- theory: back-EMF disturbance of the phase PI loop ----------------
%  i_err = -e/(Z + C) at the fundamental frequency (phasor, rotating with the q axis)
ierr = @(w) -(P.lam1*w)./((P.R + 1j*w*P.L) + (P.Kp_i + P.Ki_i./(1j*w)));   % w: electrical rad/s
wv = linspace(1e-3, 150/rpm*P.p, 400);
Ie = ierr(wv);                            % phasor relative to the q axis: q = real, d = -imag
iq_th = real(Ie); id_th = -imag(Ie);
% simulated points: steady-state windows of S2 (100 rpm, 1 N.m), S1 (100 rpm, no load), S4 (30 rpm, 0.5 N.m)
pts = {'S1', 'S2', 'S4'}; Sp = zeros(numel(pts), 4);
for s = 1:numel(pts)
    d = D.(pts{s}).pvc; R = d.R; k = R.t >= d.SC.win(1) & R.t < d.SC.win(2);
    Sp(s, :) = [mean(R.w(k))*rpm, mean(R.id(k)), mean(R.iq(k)) - mean(R.iq_cmd(k)), mean(R.iq_cmd(k))];
end
f = figure('Visible', 'off', 'Position', [100 100 900 360], 'Color', 'w'); hold on;
plot(wv/P.p*rpm, id_th, 'Color', col.pvc, 'LineWidth', 1.5);
plot(wv/P.p*rpm, iq_th, '--', 'Color', col.pvc, 'LineWidth', 1.5);
plot(Sp(:, 1), Sp(:, 2), 'o', 'Color', col.pvc, 'MarkerFaceColor', col.pvc);
plot(Sp(:, 1), Sp(:, 3), 's', 'Color', col.pvc, 'MarkerFaceColor', 'w');
grid on; xlabel('\omega_m (rpm)'); ylabel('Current error (A)');
legend('i_d error, Eq. (9.9)', 'i_q error, Eq. (9.9)', 'i_d, simulation (PVC)', 'i_q - i_q^*, simulation (PVC)', 'Location', 'northwest');
exportgraphics(f, fullfile(figDir, 'fig9_3_backemf_error.png'), 'Resolution', 150); close(f);

% ---------------- fig9_6: d-q currents of PVC with and without feedforward, S2 ----------------
f = figure('Visible', 'off', 'Position', [100 100 900 520], 'Color', 'w');
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ttl = {'pvc', '(a) PVC'; 'pvcff', '(b) PVC + feedforward'};
for c = 1:2
    R = D.S2.(ttl{c,1}).R; k = R.t >= 0.45 & R.t <= 0.55;
    nexttile; plot(R.t(k), R.iq(k), 'Color', col.(ttl{c,1}), 'LineWidth', lw); hold on;
    plot(R.t(k), R.iq_cmd(k), 'k--', 'LineWidth', 1); plot(R.t(k), R.id(k), 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    grid on; ylabel('Current (A)'); ylim([-0.2 1.1]); title(ttl{c,2});
    legend('i_q', 'i_q^* (Eq. 9.3)', 'i_d', 'Location', 'eastoutside');
end
xlabel('Time (s)');
exportgraphics(f, fullfile(figDir, 'fig9_6_S2_idq_pvc.png'), 'Resolution', 150); close(f);

% ---------------- fig9_7: steady-state torque, S2 ----------------
sel3 = {'comp', 'pvc', 'pvcff'};
f = figure('Visible', 'off', 'Position', [100 100 900 640], 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:3
    R = D.S2.(sel3{c}).R; k = R.t >= 0.45 & R.t <= 0.50; Tm = movmean(R.Te, nAvg);
    nexttile; plot(R.t(k), Tm(k), 'Color', col.(sel3{c}), 'LineWidth', lw); hold on; yline(1, 'k--');
    grid on; ylim([0.5 1.5]); ylabel('T_e (N\cdotm)');
    title(sprintf('(%c) %s', 'a' + c - 1, names{strcmp(tags, sel3{c})}));
end
xlabel('Time (s)');
exportgraphics(f, fullfile(figDir, 'fig9_7_S2_torque_pvc.png'), 'Resolution', 150); close(f);

% ---------------- fig9_8: speed responses S1, S3, S4 ----------------
sel4 = {'conv', 'comp', 'pvc', 'pvcff'};
f = figure('Visible', 'off', 'Position', [100 100 900 700], 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
ttl = {'S1', '(a) S1 – no-load start-up'; 'S3', '(b) S3 – speed reversal'; 'S4', '(c) S4 – staircase reference, 0.5 N\cdotm'};
for s = 1:3
    nexttile; a = D.(ttl{s,1}).conv.R; plot(a.t, a.w_ref*rpm, 'k--'); hold on;
    for c = 1:numel(sel4)
        R = D.(ttl{s,1}).(sel4{c}).R; plot(R.t, R.w*rpm, 'Color', col.(sel4{c}), 'LineWidth', 1);
    end
    grid on; ylabel('Speed (rpm)'); title(ttl{s,2}); xlim([0 a.t(end)]);
end
xlabel('Time (s)');
legend([{'\omega_m^*'}, names(ismember(tags, sel4))], 'Location', 'southoutside', 'Orientation', 'horizontal');
exportgraphics(f, fullfile(figDir, 'fig9_8_speed_S1_S3_S4.png'), 'Resolution', 150); close(f);

% ---------------- indicators, copper loss ----------------
K = struct(); Irms = zeros(1, 5); Pcu = Irms; IdM = Irms;
for c = 1:5
    d = D.S2.(tags{c}); R = d.R; K.(tags{c}) = bldc_kpi(R, d.SC, d.P, 'S2');
    k = R.t >= d.SC.win(1) & R.t < d.SC.win(2);
    Irms(c) = sqrt(mean(R.i_abc(k, :).^2, 'all')); Pcu(c) = 3*P.R*Irms(c)^2; IdM(c) = mean(R.id(k));
end
sel = {'Te_pp', 'Te_rms', 'T6', 'w_pp', 'thd_pct'};
lab = {'torque p-p', 'torque RMS', '6th harmonic', 'speed p-p', 'current THD', 'copper loss'};
V = zeros(5, numel(lab));
for c = 1:5
    V(c, :) = [cellfun(@(x) K.(tags{c}).(x)/K.conv.(x)*100, sel), Pcu(c)/Pcu(1)*100];
end
f = figure('Visible', 'off', 'Position', [100 100 900 380], 'Color', 'w');
bh = bar(categorical(lab, lab), V', 'grouped');
for c = 1:5, bh(c).FaceColor = col.(tags{c}); end
grid on; ylabel('Relative to conventional FOC (%)'); ylim([0 460]);
legend(names, 'Location', 'northoutside', 'Orientation', 'horizontal', 'NumColumns', 5);
for c = 2:5
    text(bh(c).XEndPoints, bh(c).YEndPoints + 8, compose('%.0f', V(c, :)), 'HorizontalAlignment', 'center', 'FontSize', 7);
end
exportgraphics(f, fullfile(figDir, 'fig9_9_S2_summary_5ctrl.png'), 'Resolution', 150); close(f);

% numbers quoted in the document
w100 = 100/rpm*P.p; e100 = ierr(w100); e30 = ierr(30/rpm*P.p);
N = table(string(tags'), Irms', Pcu', IdM', 'VariableNames', {'Controller', 'Irms_A', 'Pcu_W', 'id_mean_A'});
writetable(N, fullfile(tabDir, 'pvc_numbers.csv'));
disp(N);
fprintf('Eq. (9.9) at 100 rpm: |i_err| = %.3f A, i_d = %.3f A, i_q = %.3f A\n', abs(e100), -imag(e100), real(e100));
fprintf('Eq. (9.9) at  30 rpm: |i_err| = %.3f A, i_d = %.3f A, i_q = %.3f A\n', abs(e30), -imag(e30), real(e30));
fprintf('Simulated PVC (S1, S2, S4): speed %s rpm, i_d %s A, i_q - i_q* %s A\n', ...
        mat2str(Sp(:,1)', 4), mat2str(Sp(:,2)', 3), mat2str(Sp(:,3)', 3));
for n = [1 5 7 11 13]
    wn = n*w100; Pz = 1/(P.R + 1j*wn*P.L); Cn = P.Kp_i + P.Ki_i/(1j*wn); T = Cn*Pz/(1 + Cn*Pz);
    fprintf('harmonic %2d (%5.1f Hz): |T| = %.3f, phase = %5.1f deg, |1-T| = %.3f\n', n, wn/2/pi, abs(T), angle(T)*180/pi, abs(1 - T));
end

% ---------------- Simulink screenshots (optional) ----------------
if exist('SIMULINK_SCREENSHOTS', 'var') && SIMULINK_SCREENSHOTS
    mdl = 'Pseudo_Vector_Control'; load_system(fullfile(simRoot, 'PVC', [mdl '.slx'])); open_system(mdl);
    print(['-s' mdl], '-dpng', '-r150', fullfile(figDir, 'fig9_sim_pvc_root.png'));
    open_system([mdl '/PVC Ctrl']);
    print(['-s' mdl '/PVC Ctrl'], '-dpng', '-r150', fullfile(figDir, 'fig9_sim_pvc_ctrl.png'));
end
fprintf('make_pvc_figures: fig9_*.png in %s\n', figDir);
