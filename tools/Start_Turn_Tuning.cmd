@echo off
setlocal
set "type8Python=%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
if not exist "%type8Python%" set "type8Python=%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
if not exist "%type8Python%" (
    echo Python 3.11 or 3.12 was not found.
    pause
    exit /b 1
)
pushd "%~dp0.."
"%type8Python%" "%~dp0turn_tuning.py"
if errorlevel 1 pause
popd
endlocal
