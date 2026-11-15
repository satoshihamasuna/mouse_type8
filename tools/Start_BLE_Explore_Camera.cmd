@echo off
setlocal
cd /d "%~dp0.."
set "MOUSE_CAMERA_PYTHON=%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
if exist "%MOUSE_CAMERA_PYTHON%" (
  "%MOUSE_CAMERA_PYTHON%" tools\ble\mouse_ble_explore_camera_gui.py
) else (
  py -3.11 tools\ble\mouse_ble_explore_camera_gui.py
)
if errorlevel 1 pause
