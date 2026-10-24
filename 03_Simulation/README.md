# 03_Simulation

Simulation models of BLDC control strategies (MATLAB/Simulink).
Each strategy lives in its own folder; all strategies use the same motor/inverter parameters
and the same test scenarios S1–S4, so that results are directly comparable.

## Current controllers

| Folder | Controller | Theory |
|---|---|---|
| `Conventional_FOC/` | Two PI current controllers + cross-coupling + constant fundamental back-EMF compensation (ω_e λ_1) | Sec. 5, Fig. 5.1 |
| `Compensated_FOC/` | Conventional FOC + angle-based back-EMF compensation (LUT g_d, g_q), digital-delay compensation (θ_p, θ_a) and i_q current shaping (k_s) | Sec. 6, Fig. 6.2 |

## Adding a new controller

1. Create a new folder named after the strategy (e.g. `Six-step/`, `MPC/`).
2. Use the same motor parameters and scenarios S1–S4 as the existing models.
3. Follow the folder layout below and save results with the same naming convention.
4. Add a row to the table above and to the main `README.md`.

## Folder layout

```
Conventional_FOC/
├── Conventional_FOC.slx   # Simulink model (powergui discrete, 1 us)
├── conv_foc_init.m        # parameters P, scenarios SC (S1-S4), THETA_OFF; run by PreLoadFcn
└── run_conv_foc.m         # run S1-S4, save 04_Results/Raw/<SCEN>_conv.mat

Compensated_FOC/
├── Compensated_FOC.slx    # Simulink model (powergui discrete, 1 us)
├── comp_foc_init.m        # same parameters/scenarios + look-up tables TAB (g_d, g_q, k_s),
│                          # delay compensation P.Td_ff, P.Td_sh; run by PreLoadFcn
├── bldc_trap.m            # normalized trapezoidal back-EMF shape (used to build the tables)
├── comp_foc_calib.mat     # calibrated THETA_OFF, TAB_SHIFT
└── run_comp_foc.m         # run S1-S4, save 04_Results/Raw/<SCEN>_comp.mat
                           # (DO_CALIB = true: re-tune TAB_SHIFT first)
```

Usage (MATLAB, from the controller folder):

```matlab
run_conv_foc                         % Conventional FOC, S1-S4
SCEN_LIST = {'S2'}; run_comp_foc     % Compensated FOC, S2 only
DO_CALIB = true; run_comp_foc        % re-calibrate TAB_SHIFT, then S1-S4
```

## Test scenarios

| ID | Description |
|---|---|
| S1 | No-load start-up, 0 -> 100 rpm at 0.02 s |
| S2 | 100 rpm, 1 N·m load step at 0.3 s, removed at 0.6 s |
| S3 | Speed reversal +100 -> -100 rpm at 0.3 s |
| S4 | Staircase 30-60-100-60-30 rpm, 0.5 N·m load |

## Output naming

Results are saved to `04_Results/Raw/` as `<scenario>_<controller>.mat`,
e.g. `S2_conv.mat`, `S2_comp.mat` (use a short, unique tag for each new controller).

## Requirements

- MATLAB/Simulink R2023a, Simscape Electrical (Specialized Power Systems)
