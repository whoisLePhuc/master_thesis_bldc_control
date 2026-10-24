%% RUN_CONV_FOC  Run Conventional_FOC.slx for scenarios S1-S4 and save the results
%  Results: <repo>/04_Results/Raw/<SCEN>_conv.mat, one file per scenario, containing
%     R    : struct with time series (t, w_ref, TL, w, Te, i_abc, id, iq, iq_ref, iq_cmd)
%     P, SC, sc_name, THETA_OFF, meta
%
%  Usage:
%     >> run_conv_foc                       % all scenarios
%     >> SCEN_LIST = {'S2'}; run_conv_foc   % selected scenarios only

here = fileparts(mfilename('fullpath'));
mdl  = 'Conventional_FOC';
outDir = fullfile(here, '..', '..', '04_Results', 'Raw');
if ~exist(outDir, 'dir'), mkdir(outDir); end

if ~exist('SCEN_LIST', 'var'), SCEN_LIST = {'S1', 'S2', 'S3', 'S4'}; end
oldDir = cd(here);                 % so that conv_foc_init is found
cleanupDir = onCleanup(@() cd(oldDir));
load_system(fullfile(here, [mdl '.slx']));

for iSc = 1:numel(SCEN_LIST)
    SCEN = SCEN_LIST{iSc};
    conv_foc_init;                 % creates P, SC, sc_name, THETA_OFF, DECIM in this workspace

    % ---- reference and load torque as piecewise-constant external inputs ----
    ds = Simulink.SimulationData.Dataset;
    ds = ds.addElement(stepSignal(SC.t_ref,  SC.w_ref, SC.T, 'w_ref'), 'w_ref');
    ds = ds.addElement(stepSignal(SC.t_load, SC.TL,    SC.T, 'TL'),    'TL');

    in = Simulink.SimulationInput(mdl);
    in = in.setExternalInput(ds);
    in = in.setModelParameter('StopTime', num2str(SC.T));
    vars = {'P', P; 'SC', SC; 'THETA_OFF', THETA_OFF; 'DECIM', DECIM};
    for k = 1:size(vars, 1), in = in.setVariable(vars{k,1}, vars{k,2}); end

    tic; so = sim(in); tSim = toc;
    if ~isempty(so.ErrorMessage), error('%s: %s', SCEN, so.ErrorMessage); end

    % ---- collect outputs (root Outports, found by block path) ----
    %  w, Te, i_abc are logged every DECIM*P.dt (10 us); idq at P.Ts; iq_ref, iq_cmd at P.Tsw.
    %  Slower signals are held (zero-order) onto the 10 us time base.
    y  = so.yout;
    gv = @(name) getOut(y, [mdl '/' name]);
    w  = gv('w');
    R = struct();
    R.t      = w.Time;
    R.w      = w.Data;
    R.Te     = holdOn(gv('Te'),    R.t);
    R.i_abc  = holdOn(gv('i_abc'), R.t);
    idq      = holdOn(gv('idq'),   R.t);
    R.id     = idq(:, 1);
    R.iq     = idq(:, 2);
    R.iq_ref = holdOn(gv('iq_ref'), R.t);
    R.iq_cmd = holdOn(gv('iq_cmd'), R.t);
    R.w_ref  = interp1(SC.t_ref,  SC.w_ref, R.t, 'previous', 'extrap');
    R.TL     = interp1(SC.t_load, SC.TL,    R.t, 'previous', 'extrap');

    meta = struct('model', mdl, 'controller', 'Conventional FOC', 'scenario', SCEN, ...
                  'date', char(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), 'matlab', version, ...
                  'sim_time_s', tSim, 'log_step_s', DECIM*P.dt);
    f = fullfile(outDir, sprintf('%s_conv.mat', SCEN));
    f = char(java.io.File(f).getCanonicalPath());
    save(f, 'R', 'P', 'SC', 'sc_name', 'THETA_OFF', 'meta');
    fprintf('  %s (%s) done in %.0f s -> %s\n', SCEN, sc_name, tSim, f);
end
clear SCEN_LIST

%% ---------------- local functions ----------------
function ts = getOut(y, blockPath)
% Return the logged timeseries of the root Outport with the given block path.
for k = 1:y.numElements
    e = y.getElement(k);
    if strcmp(e.BlockPath.getBlock(1), blockPath), ts = e.Values; return; end
end
error('run_conv_foc: output "%s" not found in yout', blockPath);
end

function ts = stepSignal(t, v, T, name)
% Piecewise-constant input for a root Inport with Interpolate = off.
% The last value is repeated after the stop time, otherwise the Inport
% outputs 0 once the final data point has passed.
t = t(:); v = v(:);
keep = t <= T;                        % drop padding points beyond the stop time
t = [t(keep); T + 1];  v = [v(keep); v(find(keep, 1, 'last'))];
ts = timeseries(v, t, 'Name', name);
end

function x = holdOn(ts, t)
% Zero-order hold of timeseries ts onto time vector t (columns = channels).
d = squeeze(ts.Data);
if size(d, 1) ~= numel(ts.Time), d = d.'; end
x = interp1(ts.Time, d, t, 'previous', 'extrap');
end
