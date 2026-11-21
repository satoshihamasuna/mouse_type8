@echo off
setlocal
if not exist "%~dp0.venv\Scripts\pythonw.exe" (
    echo Install requirements.txt into .venv first.
    pause
    exit /b 1
)
pushd "%~dp0"
start "" "%~dp0.venv\Scripts\pythonw.exe" -m pixel_camera.review
popd
endlocal
