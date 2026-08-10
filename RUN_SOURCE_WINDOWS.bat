@echo off
setlocal EnableExtensions
cd /d "%~dp0"

if not exist ".venv_windows\Scripts\python.exe" (
    echo The one-time Windows installation has not been completed.
    echo Run INSTALL_AMBATUDRONE_WINDOWS.bat first.
    echo.
    pause
    exit /b 1
)

call ".venv_windows\Scripts\activate.bat"
if errorlevel 1 exit /b 1

python "droneControl4Motor.py"
