@echo off
rem Build the four theory documents (XeLaTeX) and copy the PDFs to ..\PDF
cd /d "%~dp0"
if not exist ..\PDF mkdir ..\PDF
for %%d in (01_BLDC_model 02_dq_model 03_FOC_conventional 04_FOC_compensated 05_simulation_results 06_six_step 07_pseudo_vector_control) do (
  latexmk -xelatex -interaction=nonstopmode -outdir=build %%d.tex
  copy /Y build\%%d.pdf ..\PDF\%%d.pdf >nul
)
echo PDFs copied to ..\PDF
