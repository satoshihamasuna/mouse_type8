@echo off
setlocal
set "PIXEL_CAMERA_ROOT=%~dp0"
if not exist "%PIXEL_CAMERA_ROOT%.venv\Scripts\pythonw.exe" (
    echo Run the setup commands in docs\USB_REMOTE.md first.
    pause
    exit /b 1
)
start "" /D "%PIXEL_CAMERA_ROOT%" "%PIXEL_CAMERA_ROOT%.venv\Scripts\pythonw.exe" -m pixel_camera.gui
