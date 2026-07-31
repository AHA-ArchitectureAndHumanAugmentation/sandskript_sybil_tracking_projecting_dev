@echo off
REM -- Saved-toolpath replay launcher ----------------------------------------
REM Connect the robot, pick a saved bundle under paths\ and re-run it.
REM CONTAINED tool: close the main app first if it is connected to the robot
REM (one RTDE controller per robot). No camera needed.

set PYTHONUTF8=1
chcp 65001 >nul
cd /d "%~dp0"

echo Starting Toolpath Replay...
echo (A browser tab will open at http://localhost:5007)
echo.

REM Hardcoded conda environment (see run.bat / environment.yml).
set "CONDA_PY=C:\Users\linfo\miniconda3\envs\sandskript\python.exe"
if exist "%CONDA_PY%" (
    "%CONDA_PY%" replay_main.py
) else (
    echo ERROR: conda env python not found at %CONDA_PY%
    echo Create it with:  conda env create -f environment.yml
    echo then update the CONDA_PY path at the top of this file.
)

echo.
echo Program stopped. Press any key to close this window.
pause >nul
