@echo off
setlocal EnableExtensions

rem Build the BldcClamp Keil target from the repository root.
set "PROJECT=%~dp0MDK-ARM\BldcClamp.uvprojx"
set "TARGET=BldcClamp"
rem 输出目录由 uvprojx 的 <OutputDirectory> 决定 = ..\_archive\builds\，
rem <OutputName> 固定为 BldcClamp ⇒ 每次只产出一套 BldcClamp.*（不再一版一个文件名）
set "OUTPUT_DIR=%~dp0_archive\builds"
set "BUILD_LOG=%OUTPUT_DIR%\BldcClamp.build_log.htm"
set "AXF_FILE=%OUTPUT_DIR%\BldcClamp.axf"
set "BUILD_MODE=%~1"

if defined BUILD_MODE if /I not "%BUILD_MODE%"=="full" (
    echo Usage: build_keil.bat [full]
    echo   no argument  Incremental build
    echo   full         Clean and rebuild all files
    pause
    exit /b 1
)

if not defined UV4_EXE set "UV4_EXE=%UV4_PATH%"
if not defined UV4_EXE if exist "%ProgramFiles%\Keil\UV4\UV4.exe" set "UV4_EXE=%ProgramFiles%\Keil\UV4\UV4.exe"
if not defined UV4_EXE if exist "%ProgramFiles(x86)%\Keil\UV4\UV4.exe" set "UV4_EXE=%ProgramFiles(x86)%\Keil\UV4\UV4.exe"
if not defined UV4_EXE if exist "C:\Keil_v5\UV4\UV4.exe" set "UV4_EXE=C:\Keil_v5\UV4\UV4.exe"
if not defined UV4_EXE for /f "delims=" %%I in ('where UV4.exe 2^>nul') do if not defined UV4_EXE set "UV4_EXE=%%I"

if not defined UV4_EXE (
    echo ERROR: Keil UV4.exe was not found.
    echo Set UV4_EXE to the full path of UV4.exe and run this script again.
    pause
    exit /b 1
)

if not exist "%PROJECT%" (
    echo ERROR: Keil project not found:
    echo        %PROJECT%
    pause
    exit /b 1
)

if /I "%BUILD_MODE%"=="full" (
    echo Cleaning %TARGET%...
    "%UV4_EXE%" -c "%PROJECT%" -t "%TARGET%"
    set "RESULT=%ERRORLEVEL%"
    if not "%RESULT%"=="0" (
        echo Clean failed with exit code %RESULT%.
        pause
        exit /b %RESULT%
    )
)

echo Building %TARGET%...
"%UV4_EXE%" -b "%PROJECT%" -t "%TARGET%" -j0
set "RESULT=%ERRORLEVEL%"

if not "%RESULT%"=="0" (
    echo Build failed with exit code %RESULT%.
    pause
    exit /b %RESULT%
)

rem UV4 can return before the GUI build log is fully written. Check the result.
if not exist "%BUILD_LOG%" (
    echo ERROR: Keil did not produce a build log.
    pause
    exit /b 2
)

findstr /I /C:"Target not created." /C:") - Error(s)" "%BUILD_LOG%" >nul
if "%ERRORLEVEL%"=="0" (
    echo Build failed. See:
    echo        %BUILD_LOG%
    pause
    exit /b 1
)

if not exist "%AXF_FILE%" (
    echo ERROR: Build log did not report failure, but the AXF output is missing.
    echo        %AXF_FILE%
    pause
    exit /b 2
)

echo Build completed successfully.
echo Output directory: %OUTPUT_DIR%\
pause
exit /b 0
