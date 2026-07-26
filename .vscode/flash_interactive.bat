@echo off
setlocal enabledelayedexpansion

set "BOARD=%~1"
set "PROBE=%~2"
set "BUILD_TYPE=Debug"
set "SCRIPT_DIR=%~dp0"
set "WORKSPACE_ROOT=%SCRIPT_DIR%.."

:: J-Link installation path
set "JLINK_DIR=E:\stm32\IDE\STM32CubeIDE_1.19.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.jlink.win32_2.5.0.202506031126\tools\bin"

:: Validate parameters
if "%BOARD%"=="" exit /b 1
if "%PROBE%"=="" exit /b 1

:: Board config
if /I "%BOARD%"=="damiao_h7" (
    set "TARGET=stm32h7x"
    set "JLINK_DEV=STM32H743BI"
    goto :board_ok
)
if /I "%BOARD%"=="dji_c" (
    set "TARGET=stm32f4x"
    set "JLINK_DEV=STM32F407IG"
    goto :board_ok
)
exit /b 1

:board_ok

:: ELF path
set "ELF_PATH=%WORKSPACE_ROOT%\build\%BOARD%\%BUILD_TYPE%\base.elf"

if not exist "%ELF_PATH%" (
    echo [ERROR] ELF file not found: %ELF_PATH%
    pause
    exit /b 1
)

echo Flashing: [%PROBE%] %BOARD%

:: Select flash method based on probe
if /I "%PROBE%"=="jlink" goto :flash_jlink

:: OpenOCD (stlink / daplink)
if /I "%PROBE%"=="stlink"  set "IFACE=interface/stlink.cfg"
if /I "%PROBE%"=="daplink" set "IFACE=interface/cmsis-dap.cfg"

openocd -f %IFACE% -f target/%TARGET%.cfg -c "program build/%BOARD%/%BUILD_TYPE%/base.elf verify reset exit"

if errorlevel 1 (
    echo [ERROR] OpenOCD flash failed
    pause
    exit /b 1
)

goto :done

:: SEGGER J-Link
:flash_jlink
set "JLINK_EXE=%JLINK_DIR%\JLink.exe"
set "JLINK_SCRIPT=%TEMP%\vscode_flash_%BOARD%.jlink"

if not exist "%JLINK_EXE%" (
    echo [ERROR] JLink.exe not found
    pause
    exit /b 1
)

(
    echo loadfile "%ELF_PATH%"
    echo r
    echo g
    echo exit
) > "%JLINK_SCRIPT%"

"%JLINK_EXE%" -device %JLINK_DEV% -if SWD -speed 4000 -autoconnect 1 -CommanderScript "%JLINK_SCRIPT%"

set "JLINK_ERR=%errorlevel%"
if exist "%JLINK_SCRIPT%" del "%JLINK_SCRIPT%" >nul 2>&1

if %JLINK_ERR% neq 0 (
    echo [ERROR] J-Link flash failed
    pause
    exit /b 1
)

:: Done
:done
echo Flash complete!
exit /b 0
