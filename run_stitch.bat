@echo off
REM -- Multi-Cam Vision prototype launcher -----------------------------------
REM Merges the feeds of every connected RealSense D435i into one heightmap.
REM CONTAINED prototype: close the main app first (one process per RealSense).
REM With no camera attached it runs on a synthetic scene.

set PYTHONUTF8=1
chcp 65001 >nul
cd /d "%~dp0"

echo Starting Multi-Cam Vision prototype...
echo (A browser tab will open at http://localhost:5106)
echo.

REM _env.bat finds this machine's "sandskript" env in the usual
REM Miniconda/Anaconda locations. Set CONDA_PY yourself to override.
REM RealSense USB driver = OS-level install, not part of the env.
call "%~dp0_env.bat"
if defined CONDA_PY (
    "%CONDA_PY%" stitch_main.py
) else (
    echo ERROR: could not find the "sandskript" conda env on this machine.
    echo Create it with:  conda env create -f environment.yml
    echo Or point at it directly:
    echo   set "CONDA_PY=X:\path\to\envs\sandskript\python.exe"
)

echo.
echo Program stopped. Press any key to close this window.
pause >nul
