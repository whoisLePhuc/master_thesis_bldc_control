# Theory documents – LaTeX sources

LaTeX version of the theory documents (same content as `../Markdown/`).
Figures are shared with the Markdown version and taken from `../../Thesis/figures/`
(draw.io exports, Simulink screenshots, MATLAB plots).

```
Latex/
├── style/foc_style.sty          # common preamble: fonts, colours, headings, "Ghi chú" box, \seriesbox
├── chapters/
│   ├── ch2_bldc_model.tex       # Chapter 2
│   ├── ch3_dq_model.tex         # Chapter 3
│   ├── ch4_foc_design.tex       # Chapter 4
│   ├── ch5_conventional_foc.tex # Chapter 5 (incl. 5.3 Simulink model)
│   ├── ch6_compensated_foc.tex  # Chapter 6 (incl. 6.7 Simulink model)
│   ├── ch7_simulation_results.tex # Chapter 7 (results, tables/figures from 04_Results/make_results.m)
│   ├── ch8_six_step.tex         # Chapter 8 (six-step, comparison of three controllers)
│   ├── ch9_pseudo_vector_control.tex # Chapter 9 (pseudo vector control)
│   └── ch10_robustness.tex      # Chapter 10 (robustness to motor mismatch and position-sensor errors)
├── 01_BLDC_model.tex            # main files: title, abstract, TOC, symbols, chapters, references
├── 02_dq_model.tex
├── 03_FOC_conventional.tex
├── 04_FOC_compensated.tex
├── 05_simulation_results.tex
├── 06_six_step.tex
├── 07_pseudo_vector_control.tex
├── 08_robustness.tex
├── latexmkrc                    # XeLaTeX, auxiliary files in build/
├── build.sh / build.bat         # build all documents and copy PDFs to ../PDF/
└── build/                       # (generated, not tracked)
```

## Build

Requires XeLaTeX (TeX Live or MiKTeX) with Latin Modern fonts and babel-vietnamese.

```
./build.sh          # Linux / macOS / WSL
build.bat           # Windows
latexmk 03_FOC_conventional.tex   # a single document
```

## Conventions

- Numbering by chapter: equations (3.3), figures `Hình 5.2`, tables `Bảng 6.1`.
  Equations keep a fixed `\tag{x.y}` so numbers stay identical to the Markdown version.
- Labels: `eq:x.y`, `fig:x.y`, `tab:x.y`, `sec:x.y`. References to another document
  (e.g. (3.4) cited in document 03) are written as plain text.
- Each chapter file is a pure body (no preamble); a future full report can `\input` all of them.
