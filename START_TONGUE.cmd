@echo off
setlocal
cd /d "%~dp0"
if not defined SOFA_ROOT (
    echo Set SOFA_ROOT to your SOFA installation directory first.
    echo Example: set "SOFA_ROOT=D:\SOFA\SOFA_v25.12.00_Win64"
    pause
    exit /b 1
)
python tools\launch_sofa.py
if errorlevel 1 pause
