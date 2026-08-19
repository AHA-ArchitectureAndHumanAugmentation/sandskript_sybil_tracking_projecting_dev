@echo off
REM -- Scheduler launcher -----------------------------------------------------
REM A read-only ledger of the toolpaths saved under paths\: which path, when.
REM CONTAINED tool, and it touches no hardware and writes nothing, so unlike the
REM replay and Multi-Cam tools it is safe to leave running beside the main app.

set PYTHONUTF8=1
chcp 65001 >nul
cd /d "%~dp0"

echo Starting Scheduler...
echo (A browser tab will open at http://localhost:5108)
echo.

REM _env.bat finds this machine's "sandskript" env in the usual
REM Miniconda/Anaconda locations. Set CONDA_PY yourself to override.
call "%~dp0_env.bat"
if defined CONDA_PY (
    "%CONDA_PY%" scheduler_main.py
) else (
    echo ERROR: could not find the "sandskript" conda env on this machine.
    echo Create it with:  conda env create -f environment.yml
    echo Or point at it directly:
    echo   set "CONDA_PY=X:\path\to\envs\sandskript\python.exe"
)

echo.
echo Program stopped. Press any key to close this window.
pause >nul
