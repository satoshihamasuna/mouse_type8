@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "SCRIPT_DIR=%~dp0"
set "IOC_COUNT=0"
set "IOC_FILE="

for %%F in ("%SCRIPT_DIR%*.ioc") do (
    if exist "%%~fF" (
        set /a IOC_COUNT+=1
        set "IOC_FILE=%%~fF"
    )
)

if "%IOC_COUNT%"=="0" (
    echo No .ioc file found in "%SCRIPT_DIR%".
    exit /b 1
)

if not "%IOC_COUNT%"=="1" (
    echo Multiple .ioc files found in "%SCRIPT_DIR%".
    echo Please open one explicitly:
    for %%F in ("%SCRIPT_DIR%*.ioc") do if exist "%%~fF" echo   %%~nxF
    exit /b 1
)

start "" "%IOC_FILE%"
