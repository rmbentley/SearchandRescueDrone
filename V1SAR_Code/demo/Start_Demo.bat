@echo off
cd /d "%~dp0"
title SAR Drone AI System - Full Hardware-Free Demo

where py >nul 2>&1
if %ERRORLEVEL% equ 0 (
    py run_demo.py
    goto end
)

where python >nul 2>&1
if %ERRORLEVEL% equ 0 (
    python run_demo.py
    goto end
)

echo [ERROR] Python was not found in PATH. Please install Python 3.8+ to run the demo.
pause

:end
