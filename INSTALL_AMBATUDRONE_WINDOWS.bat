@echo off
setlocal EnableExtensions
cd /d "%~dp0"

title AmbatuDrone Windows Installer
color 0B

echo.
echo  AMBATUDRONE WINDOWS INSTALLER
echo  =============================
echo.
echo  This installer will build and install AmbatuDrone.exe for this PC.
echo  Keep all propellers removed during the first hardware test.
echo.

if not exist "droneControl4Motor.py" (
    echo ERROR: droneControl4Motor.py is missing.
    echo Extract the complete ZIP before running this installer.
    goto :failed
)

if not exist "requirements-windows.txt" (
    echo ERROR: requirements-windows.txt is missing.
    echo Extract the complete ZIP before running this installer.
    goto :failed
)

set "PYTHON_CMD="

py -3.13 -c "import sys; raise SystemExit(sys.version_info[:2] != (3,13))" >nul 2>&1
if not errorlevel 1 set "PYTHON_CMD=py -3.13"

if not defined PYTHON_CMD (
    py -3 -c "import sys; raise SystemExit(sys.version_info[:2] not in ((3,11),(3,12),(3,13)))" >nul 2>&1
    if not errorlevel 1 set "PYTHON_CMD=py -3"
)

if not defined PYTHON_CMD (
    python -c "import sys; raise SystemExit(sys.version_info[:2] not in ((3,11),(3,12),(3,13)))" >nul 2>&1
    if not errorlevel 1 set "PYTHON_CMD=python"
)

if not defined PYTHON_CMD (
    echo ERROR: A supported Python installation was not found.
    echo.
    echo Install 64-bit Python 3.13 from:
    echo https://www.python.org/downloads/
    echo.
    echo During installation, enable "Add python.exe to PATH".
    echo Then run this installer again.
    goto :failed
)

echo Detected Python:
%PYTHON_CMD% --version
echo.

if not exist ".venv_windows\Scripts\python.exe" (
    echo [1/5] Creating a private Python environment...
    %PYTHON_CMD% -m venv ".venv_windows"
    if errorlevel 1 goto :failed
) else (
    echo [1/5] Reusing the existing private Python environment.
)

call ".venv_windows\Scripts\activate.bat"
if errorlevel 1 goto :failed

echo.
echo [2/5] Installing the Windows dependencies...
python -m pip install --upgrade pip setuptools wheel
if errorlevel 1 goto :failed
python -m pip install -r "requirements-windows.txt"
if errorlevel 1 goto :failed

echo.
echo [3/5] Testing the controller command and telemetry formats...
python "droneControl4Motor.py" --self-test
if errorlevel 1 goto :failed

echo.
echo [4/5] Building AmbatuDrone.exe...
python -m PyInstaller --noconfirm --clean --onefile --windowed --name AmbatuDrone --collect-all pygame --hidden-import serial.tools.list_ports "droneControl4Motor.py"
if errorlevel 1 goto :failed

if not exist "dist\AmbatuDrone.exe" (
    echo ERROR: The build finished without creating dist\AmbatuDrone.exe.
    goto :failed
)

set "INSTALL_DIR=%LOCALAPPDATA%\AmbatuDrone"
if not exist "%INSTALL_DIR%" mkdir "%INSTALL_DIR%"
if errorlevel 1 goto :failed

copy /Y "dist\AmbatuDrone.exe" "%INSTALL_DIR%\AmbatuDrone.exe" >nul
if errorlevel 1 (
    echo ERROR: Windows could not replace the installed application.
    echo Close AmbatuDrone.exe if it is running, then try again.
    goto :failed
)

echo.
echo [5/5] Creating Desktop and Start Menu shortcuts...
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0CREATE_WINDOWS_SHORTCUTS.ps1"
if errorlevel 1 (
    echo WARNING: The application installed, but Windows could not create shortcuts.
    echo You can launch it directly from:
    echo %INSTALL_DIR%\AmbatuDrone.exe
)

echo.
echo  INSTALLATION COMPLETE
echo  =====================
echo  Installed application:
echo  %INSTALL_DIR%\AmbatuDrone.exe
echo.
echo  The app will open in Demo Mode so it cannot command the motors yet.
echo.
start "" "%INSTALL_DIR%\AmbatuDrone.exe"
pause
exit /b 0

:failed
echo.
echo  INSTALLATION STOPPED
echo  ====================
echo  Read START_HERE_WINDOWS.txt or copy the complete error shown above.
echo.
pause
exit /b 1
