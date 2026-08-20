@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "SCRIPT_DIR=%~dp0"
set "DESTINATION=%SCRIPT_DIR%"
if "%DESTINATION:~-1%"=="\" set "DESTINATION=%DESTINATION:~0,-1%"
set "CACHE_ROOT=%DESTINATION%\archive"
set "RELEASE_TAG=v3.0.0"
set "MODE=all"
set "CLEAN_MODE=safe"

if not "%~1"=="" set "MODE=%~1"
if not "%~2"=="" set "CLEAN_MODE=%~2"

if /I not "%MODE%"=="all" if /I not "%MODE%"=="autonomous" if /I not "%MODE%"=="piloted" (
    echo Usage: %~nx0 [all^|autonomous^|piloted] [safe^|force-clean]
    exit /b 1
)

if /I not "%CLEAN_MODE%"=="safe" if /I not "%CLEAN_MODE%"=="force-clean" (
    echo Usage: %~nx0 [all^|autonomous^|piloted] [safe^|force-clean]
    exit /b 1
)

set "TEMP_ROOT=%TEMP%\ratm-trajectories-%RANDOM%%RANDOM%"
set "REPO_BASE=https://github.com/Drone-Racing/drone-racing-dataset/releases/download/%RELEASE_TAG%"
set "CREATED_AUTONOMOUS=0"
set "CREATED_PILOTED=0"

mkdir "%CACHE_ROOT%" >nul 2>nul

if /I "%MODE%"=="all" (
    call :prepare_target autonomous
    if errorlevel 1 exit /b 1
    call :prepare_target piloted
    if errorlevel 1 exit /b 1
)

if /I "%MODE%"=="autonomous" (
    call :prepare_target autonomous
    if errorlevel 1 exit /b 1
)

if /I "%MODE%"=="piloted" (
    call :prepare_target piloted
    if errorlevel 1 exit /b 1
)

mkdir "%TEMP_ROOT%" || exit /b 1

if /I "%MODE%"=="all" (
    call :download_and_extract autonomous autonomous_zipchunk01 autonomous_zipchunk02 autonomous_zipchunk03
    if errorlevel 1 goto :fail
    call :download_and_extract piloted piloted_zipchunk01 piloted_zipchunk02 piloted_zipchunk03 piloted_zipchunk04 piloted_zipchunk05 piloted_zipchunk06 piloted_zipchunk07
    if errorlevel 1 goto :fail
)

if /I "%MODE%"=="autonomous" (
    call :download_and_extract autonomous autonomous_zipchunk01 autonomous_zipchunk02 autonomous_zipchunk03
    if errorlevel 1 goto :fail
)

if /I "%MODE%"=="piloted" (
    call :download_and_extract piloted piloted_zipchunk01 piloted_zipchunk02 piloted_zipchunk03 piloted_zipchunk04 piloted_zipchunk05 piloted_zipchunk06 piloted_zipchunk07
    if errorlevel 1 goto :fail
)

rd /s /q "%TEMP_ROOT%"
echo Trajectory CSV download complete.
exit /b 0

:prepare_target
set "GROUP=%~1"
set "TARGET_DIR=%DESTINATION%\%GROUP%"

if not exist "%TARGET_DIR%" (
    mkdir "%TARGET_DIR%" || exit /b 1
    if /I "%GROUP%"=="autonomous" set "CREATED_AUTONOMOUS=1"
    if /I "%GROUP%"=="piloted" set "CREATED_PILOTED=1"
    exit /b 0
)

dir /b "%TARGET_DIR%\*" >nul 2>nul
if errorlevel 1 exit /b 0

if /I "%CLEAN_MODE%"=="force-clean" (
    rd /s /q "%TARGET_DIR%" || exit /b 1
    mkdir "%TARGET_DIR%" || exit /b 1
    if /I "%GROUP%"=="autonomous" set "CREATED_AUTONOMOUS=1"
    if /I "%GROUP%"=="piloted" set "CREATED_PILOTED=1"
    exit /b 0
)

echo Destination already contains data in "%TARGET_DIR%". Re-run with force-clean to replace it.
exit /b 1

:download_and_extract
set "GROUP=%~1"
shift

set "GROUP_CACHE=%CACHE_ROOT%\%GROUP%"
set "GROUP_TEMP=%TEMP_ROOT%\%GROUP%"
set "ZIP_PATH=%CACHE_ROOT%\%GROUP%.zip"

mkdir "%GROUP_CACHE%" >nul 2>nul
mkdir "%GROUP_TEMP%" || exit /b 1

:download_loop
if "%~1"=="" goto :rebuild_zip
set "CHUNK=%~1"
if exist "%GROUP_CACHE%\%CHUNK%" (
    curl -C - -L "%REPO_BASE%/%CHUNK%" -o "%GROUP_CACHE%\%CHUNK%" || exit /b 1
) else (
    curl -L "%REPO_BASE%/%CHUNK%" -o "%GROUP_CACHE%\%CHUNK%" || exit /b 1
)
shift
goto :download_loop

:rebuild_zip
copy /b "%GROUP_CACHE%\*_zipchunk*" "%ZIP_PATH%" >nul || exit /b 1

set "EXTRACT_ROOT=%GROUP_TEMP%\extracted"
mkdir "%EXTRACT_ROOT%" || exit /b 1

tar -xf "%ZIP_PATH%" -C "%EXTRACT_ROOT%" || exit /b 1

set "ROBOCOPY_EXIT=0"
robocopy "%EXTRACT_ROOT%\%GROUP%" "%DESTINATION%\%GROUP%" "*_cam_ts_sync.csv" /s /njh /njs /ndl /nc /ns /np >nul
if errorlevel 8 exit /b 1
if errorlevel 1 set "ROBOCOPY_EXIT=1"

robocopy "%EXTRACT_ROOT%\%GROUP%" "%DESTINATION%\%GROUP%" "*_500hz_freq_sync.csv" /s /njh /njs /ndl /nc /ns /np >nul
if errorlevel 8 exit /b 1
if errorlevel 1 set "ROBOCOPY_EXIT=1"
exit /b 0

:fail
if "%CREATED_AUTONOMOUS%"=="1" if exist "%DESTINATION%\autonomous" rd /s /q "%DESTINATION%\autonomous"
if "%CREATED_PILOTED%"=="1" if exist "%DESTINATION%\piloted" rd /s /q "%DESTINATION%\piloted"
if exist "%TEMP_ROOT%" rd /s /q "%TEMP_ROOT%"
exit /b 1
