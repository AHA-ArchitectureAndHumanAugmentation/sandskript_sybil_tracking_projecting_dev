@echo off
REM -- Depth Camera to Robot launcher ---------------------------------------
REM Double-click this file to start the program.

REM Use UTF-8 so Unicode log output does not crash the program.
set PYTHONUTF8=1
chcp 65001 >nul

REM Move to the folder this script lives in, so it works from anywhere.
cd /d "%~dp0"

echo Starting Depth Camera to Robot (Developer Mode)...
echo (A browser tab will open at http://localhost:5005)
echo.

REM Hardcoded conda environment (recreate anywhere with:
REM     conda env create -f environment.yml
REM then update CONDA_PY below to that machine's env path).
REM NOTE: the Intel RealSense USB driver is an OS-level install, NOT part of
REM the conda env - install it separately on a new machine.
set "CONDA_PY=C:\Users\linfo\miniconda3\envs\sandskript\python.exe"
if exist "%CONDA_PY%" (
    "%CONDA_PY%" main.py
) else (
    echo ERROR: conda env python not found at %CONDA_PY%
    echo Create it with:  conda env create -f environment.yml
    echo then update the CONDA_PY path at the top of this file.
)

REM Keep the window open after the program exits so any error stays visible.
echo.
echo Program stopped. Press any key to close this window.
pause >nul
