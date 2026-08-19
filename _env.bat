@echo off
REM -- Conda environment resolver --------------------------------------------
REM Called by run.bat / run_replay.bat / run_scheduler.bat / run_stitch.bat.
REM Sets CONDA_PY to this machine's "sandskript" env python, or leaves it
REM empty if it cannot find one.
REM
REM WHY THIS FILE EXISTS: every launcher used to hardcode one absolute path
REM (C:\Users\linfo\miniconda3\...). That works on exactly one machine, and
REM the repo's own "copy check list.txt" lists fixing it as step 1 after any
REM copy. The search order below covers the usual Miniconda/Anaconda install
REM locations, so a fresh clone normally needs no edit at all.
REM
REM To force a specific interpreter, set CONDA_PY before calling a launcher:
REM     set "CONDA_PY=D:\envs\sandskript\python.exe"
REM     run.bat

set "ENV_NAME=sandskript"

REM 1. Already set by the caller (and real) -- respect it.
if defined CONDA_PY if exist "%CONDA_PY%" goto :found

REM 2. The usual per-user install locations.
call :try "%USERPROFILE%\miniconda3\envs\%ENV_NAME%\python.exe"
if defined CONDA_PY goto :found
call :try "%USERPROFILE%\anaconda3\envs\%ENV_NAME%\python.exe"
if defined CONDA_PY goto :found
call :try "%LOCALAPPDATA%\miniconda3\envs\%ENV_NAME%\python.exe"
if defined CONDA_PY goto :found
call :try "%LOCALAPPDATA%\Continuum\miniconda3\envs\%ENV_NAME%\python.exe"
if defined CONDA_PY goto :found

REM 3. Machine-wide installs.
call :try "C:\ProgramData\miniconda3\envs\%ENV_NAME%\python.exe"
if defined CONDA_PY goto :found
call :try "C:\ProgramData\Anaconda3\envs\%ENV_NAME%\python.exe"
if defined CONDA_PY goto :found

REM 4. Ask conda itself, if it happens to be on PATH.
for /f "delims=" %%P in ('conda run -n %ENV_NAME% python -c "import sys; print(sys.executable)" 2^>nul') do (
    call :try "%%P"
)
if defined CONDA_PY goto :found

REM Nothing found -- leave CONDA_PY empty; the launcher prints the error.
set "CONDA_PY="
exit /b 1

:try
if not defined CONDA_PY if exist %1 set "CONDA_PY=%~1"
exit /b 0

:found
exit /b 0
