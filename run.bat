@echo off
REM -- Depth Camera to Robot launcher ---------------------------------------
REM Double-click this file to start the program.

REM Use UTF-8 so Unicode log output does not crash the program.
set PYTHONUTF8=1
chcp 65001 >nul

REM Move to the folder this script lives in, so it works from anywhere.
cd /d "%~dp0"

echo Starting Depth Camera to Robot (Developer Mode)...
echo (A browser tab will open at http://localhost:5105)
echo.

REM _env.bat finds this machine's "sandskript" env in the usual
REM Miniconda/Anaconda locations. Set CONDA_PY yourself to override.
REM NOTE: the Intel RealSense USB driver is an OS-level install, NOT part
REM of the conda env - install it separately on a new machine.
call "%~dp0_env.bat"
if defined CONDA_PY (
    "%CONDA_PY%" main.py
) else (
    echo ERROR: could not find the "sandskript" conda env on this machine.
    echo Create it with:  conda env create -f environment.yml
    echo Or point at it directly:
    echo   set "CONDA_PY=X:\path\to\envs\sandskript\python.exe"
)

REM Keep the window open after the program exits so any error stays visible.
echo.
echo Program stopped. Press any key to close this window.
pause >nul
