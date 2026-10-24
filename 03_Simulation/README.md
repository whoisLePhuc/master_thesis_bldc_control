# 03_Simulation

Simulation models of BLDC control strategies (MATLAB/Simulink).
Each strategy lives in its own folder; all strategies use the same motor/inverter parameters
and the same test scenarios S1–S4, so that results are directly comparable.

## Current controllers

| Folder | Controller | Theory |
|---|---|---|
| `Conventional_FOC/` | Two PI current controllers + cross-coupling + constant fundamental back-EMF compensation (ω_e λ_1) | Sec. 5, Fig. 5.1 |
| `Compensated_FOC/` | Conventional FOC + angle-based back-EMF compensation (LUT g_d, g_q), digital-delay compensation (θ_p, θ_a) and i_q current shaping (k_s) | Sec. 6, Fig. 6.2 |
| `Six_Step/` | 120° six-step: Hall sectors (from the encoder angle), PI on the two-phase current I = ½·sᵀi_abc, back-EMF feed-forward 2λω_e, H_PWM–L_ON | Sec. 8, Fig. 8.2 |
| `PVC/` | Pseudo vector control (US 7,339,346): vector-control current references (power-balance i_q*, field-weakening i_d*) → i_abc*, three phase PI controllers; variant with phase back-EMF feedforward (`PVC_FF = 1`) | Sec. 9, Fig. 9.1 |

## Adding a new controller

1. Create a new folder named after the strategy (e.g. `Six_Step/`, `MPC/`).
2. Build the model from the library blocks in `Common/bldc_plant_lib.slx` (open the library with
   `open_system('bldc_plant_lib')` and copy the blocks into the model), add a discrete `powergui` (1 us) and keep the
   controller interface below.
3. Write `<tag>_init.m` starting from `P = bldc_params(); [SC, sc_name] = bldc_scenario(SCEN);`
   and `run_<tag>.m` calling `bldc_run_scenario`; save results as `<SCEN>_<tag>.mat`.
4. Register it in `Tests/bldc_controllers.m` and store its reference metrics (see *Regression test*).
5. Add a row to the table above and to the main `README.md`.

## Folder layout

```
Common/                    # shared by every controller -- edit here only
├── bldc_params.m          # P = bldc_params(): motor, inverter, sampling, FOC gains, Gain-block matrices
├── bldc_scenario.m        # [SC, sc_name] = bldc_scenario('S1'..'S4' | 'CAL')
├── bldc_run_scenario.m    # [R, so, tSim] = bldc_run_scenario(mdl, SC, vars): simulate + collect outputs
├── bldc_metrics.m         # M = bldc_metrics(R, SC, P): speed/torque/current metrics in SC.win
├── bldc_trap.m            # normalized trapezoidal back-EMF shape F(theta)
└── bldc_plant_lib.slx     # Simulink library: Power Stage, PWM, SVPWM, abc_to_dq

Conventional_FOC/
├── Conventional_FOC.slx   # model (powergui discrete 1 us); plant blocks are links to bldc_plant_lib
├── conv_foc_init.m        # P, SC, THETA_OFF, DECIM; run by PreLoadFcn
└── run_conv_foc.m         # run S1-S4, save 04_Results/Raw/<SCEN>_conv.mat

Compensated_FOC/
├── Compensated_FOC.slx    # model (powergui discrete 1 us); plant blocks are links to bldc_plant_lib
├── comp_foc_init.m        # common P + look-up tables TAB (g_d, g_q, k_s), P.Td_ff, P.Td_sh
├── comp_foc_calib.mat     # calibrated THETA_OFF, TAB_SHIFT
└── run_comp_foc.m         # run S1-S4, save 04_Results/Raw/<SCEN>_comp.mat
                           # (DO_CALIB = true: re-tune TAB_SHIFT first)

Six_Step/
├── Six_Step.slx           # model (powergui discrete 1 us); plant blocks are links to bldc_plant_lib
├── six_step_init.m        # common P + six-step gains (Kt = 2*p*lam, current/speed PI) and Hall table SS
└── run_six_step.m         # run S1-S4, save 04_Results/Raw/<SCEN>_six.mat

PVC/
├── Pseudo_Vector_Control.slx  # model (powergui discrete 1 us); plant blocks are links to bldc_plant_lib
├── pvc_init.m             # common P + look-up tables TAB (g_d, g_q) + options PVC (ff, fw, wb)
└── run_pvc.m              # run S1-S4 for PVC_FF = 0 and 1, save <SCEN>_pvc.mat / <SCEN>_pvcff.mat

Tests/
├── bldc_controllers.m     # registry of implemented controllers (model, folder, init script, variables)
├── bldc_regression.m      # regression test against stored reference metrics
└── regression_reference.mat
```

Each model's `PreLoadFcn` adds its own folder and `Common/` to the MATLAB path and calls its
init script, so a model can be opened from any working folder.

Usage (MATLAB, from the controller folder):

```matlab
run_conv_foc                         % Conventional FOC, S1-S4
SCEN_LIST = {'S2'}; run_comp_foc     % Compensated FOC, S2 only
DO_CALIB = true; run_comp_foc        % re-calibrate TAB_SHIFT, then S1-S4
```

## Regression test

`Tests/bldc_regression.m` re-simulates every controller registered in `Tests/bldc_controllers.m`,
computes the metrics of `Common/bldc_metrics.m` (mean speed and torque, speed/torque/i_q ripple,
peak speed and current, speed dip) and compares them with `Tests/regression_reference.mat`
(tolerance 1 % + 1e-3). It errors if any metric is outside tolerance.

```matlab
addpath('03_Simulation/Tests')
bldc_regression                                   % S2, all controllers (~40 s)
bldc_regression('Scenarios', {'S1','S2','S3','S4'})
bldc_regression('Controllers', {'comp'})
bldc_regression('Update', true)                   % accept the current results as new reference
```

Run it after any change in `Common/` (parameters, plant library) or in a controller model.
When a new controller is added, register it in `bldc_controllers.m` and create its reference with
`bldc_regression('Controllers', {'<tag>'}, 'Scenarios', {'S1','S2','S3','S4'}, 'Update', true)`.

## Controller interface

`bldc_run_scenario` works with any model that exposes the common interface:

| Root port | Signal |
|---|---|
| Inport 1 `w_ref`, Inport 2 `TL` | speed reference (rad/s), load torque (N·m); `Interpolate = off` |
| Outports `w`, `Te`, `i_abc` | speed, torque, phase currents (sample time `DECIM*P.dt`) |
| Outports `idq`, `iq_ref`, `iq_cmd` | d-q currents, speed-loop output, final current reference |

Six-step uses the same interface: `iq_ref` = `iq_cmd` = current reference I* of the two conducting phases
(speed-loop output), `idq` = Park transform of the measured currents (for comparison only);
an extra Outport `i_dc` (port 7) logs the measured current I.

## Test scenarios

| ID | Description |
|---|---|
| S1 | No-load start-up, 0 -> 100 rpm at 0.02 s |
| S2 | 100 rpm, 1 N·m load step at 0.3 s, removed at 0.6 s |
| S3 | Speed reversal +100 -> -100 rpm at 0.3 s |
| S4 | Staircase 30-60-100-60-30 rpm, 0.5 N·m load |

## Output naming

Results are saved to `04_Results/Raw/` as `<scenario>_<controller>.mat`,
e.g. `S2_conv.mat`, `S2_comp.mat`, `S2_six.mat`, `S2_pvc.mat`, `S2_pvcff.mat` (use a short, unique tag for each new controller).

## Requirements

- MATLAB/Simulink R2023a, Simscape Electrical (Specialized Power Systems)
