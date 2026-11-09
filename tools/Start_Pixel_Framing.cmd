@echo off
setlocal
if not defined PIXEL_FRAMING_PYTHON set "PIXEL_FRAMING_PYTHON=D:\venv\venv3-11\Scripts\pythonw.exe"
if not exist "%PIXEL_FRAMING_PYTHON%" (
    echo Set PIXEL_FRAMING_PYTHON to Python with opencv-contrib-python, Pillow and Tk.
    pause
    exit /b 1
)
pushd "%~dp0.."
start "Pixel Framing" "%PIXEL_FRAMING_PYTHON%" "%~dp0pixel_camera_framing.py"
popd
endlocal
