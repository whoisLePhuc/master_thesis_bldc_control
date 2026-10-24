# master_thesis_bldc_control

Master's thesis: modeling and control of trapezoidal back-EMF BLDC motors —
comparing six-step, conventional FOC, and FOC with back-EMF compensation in MATLAB/Simulink.

## Repository structure

| Folder | Content |
|---|---|
| `01_Documents/` | Proposal, thesis manuscript, theory derivations, presentations, meeting notes |
| `02_References/` | Papers (by topic), books, datasheets, standards, `references.bib` |
| `03_Simulation/` | Shared parameters & scenarios (S1–S4), Simulink models, MATLAB scripts, Python post-processing |
| `04_Results/` | Simulation outputs: raw data (not tracked), exported figures and comparison tables |
| `05_Admin/` | Forms, decisions, deadlines |

## Workflow

`03_Simulation/Parameters` → run models in `03_Simulation/Models` → `04_Results/Raw`
→ post-process with `03_Simulation/Python` → `04_Results/Figures`, `04_Results/Tables`
→ used in `01_Documents/Thesis`.

## Controllers compared

1. Six-step (120° commutation)
2. Conventional FOC
3. FOC with trapezoidal back-EMF compensation

All controllers share the same motor/inverter parameters and test scenarios S1–S4.

## Requirements

- MATLAB/Simulink (version: _TBD_)
- Python 3.10+ (numpy, scipy, matplotlib, pandas)

## Notes

- Copyrighted PDFs in `02_References/` are kept locally and ignored by git; only `references.bib` is tracked.
- Raw simulation data in `04_Results/Raw/` is ignored; it can be regenerated from the models.
