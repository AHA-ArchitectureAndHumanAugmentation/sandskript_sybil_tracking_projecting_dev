@echo off
REM -- Dual-Cam Vision prototype launcher ------------------------------------
REM Merges the feeds of TWO RealSense D435i cameras into one heightmap.
REM CONTAINED prototype: close the main app first (one process per RealSense).
REM With fewer than two cameras it runs on a synthetic scene.

set PYTHONUTF8=1
chcp 65001 >nul
cd /d "%~dp0"

echo Starting Dual-Cam Vision prototype...
echo (A browser tab will open at http://localhost:5006)
echo.

REM Hardcoded conda environment (see run.bat / environment.yml).
REM RealSense USB driver = OS-level install, not part of the env.
set "CONDA_PY=C:\Users\linfo\miniconda3\envs\sandskript\python.exe"
if exist "%CONDA_PY%" (
    "%CONDA_PY%" stitch_main.py
) else (
    echo ERROR: conda env python not found at %CONDA_PY%
    echo Create it with:  conda env create -f environment.yml
    echo then update the CONDA_PY path at the top of this file.
)

echo.
echo Program stopped. Press any key to close this window.
pause >nul
