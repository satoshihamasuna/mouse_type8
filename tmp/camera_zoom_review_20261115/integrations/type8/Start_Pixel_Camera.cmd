@echo off
setlocal
set "type8Python=%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
if not exist "%type8Python%" (
    echo Python 3.11 was not found. Run tools\pixel_camera_capture.py with your Python.
    pause
    exit /b 1
)
pushd "%~dp0.."
"%type8Python%" "%~dp0pixel_camera_capture.py" gui
if errorlevel 1 pause
popd
endlocal
