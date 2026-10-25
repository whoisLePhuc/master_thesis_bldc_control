#!/usr/bin/env bash
# Build the theory documents and copy the PDFs to ../PDF
set -e
cd "$(dirname "$0")"
mkdir -p ../PDF
for d in 01_BLDC_model 02_dq_model 03_FOC_conventional 04_FOC_compensated 05_simulation_results 06_six_step 07_pseudo_vector_control 08_robustness; do
  latexmk -xelatex -interaction=nonstopmode -outdir=build "$d.tex"
  cp "build/$d.pdf" "../PDF/$d.pdf"
done
echo "PDFs copied to $(cd ../PDF && pwd)"
