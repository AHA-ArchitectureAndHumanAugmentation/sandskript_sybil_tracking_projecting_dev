@echo off
REM -- Scheduler launcher -----------------------------------------------------
REM A read-only ledger of the toolpaths saved under paths\: which path, when.
REM CONTAINED tool, and it touches no hardware and writes nothing, so unlike the
REM replay and Multi-Cam tools it is safe to leave running beside the main app.

set PYTHONUTF8=1
chcp 65001 >nul
cd /d "%~dp0"

echo Starting Scheduler...
echo (A browser tab will open at http://localhost:5008)
echo.

REM Hardcoded conda environment (see run.bat / environment.yml).
set "CONDA_PY=C:\Users\linfo\miniconda3\envs\sandskript\python.exe"
if exist "%CONDA_PY%" (
    "%CONDA_PY%" scheduler_main.py
) else (
    echo ERROR: conda env python not found at %CONDA_PY%
    echo Create it with:  conda env create -f environment.yml
    echo then update the CONDA_PY path at the top of this file.
)

echo.
echo Program stopped. Press any key to close this window.
pause >nul
