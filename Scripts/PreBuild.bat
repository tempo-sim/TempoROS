REM Copyright Tempo Simulation, LLC. All Rights Reserved
@echo off

if defined TEMPO_SKIP_PREBUILD (
    if not "%TEMPO_SKIP_PREBUILD%"=="0" (
        if not "%TEMPO_SKIP_PREBUILD%"=="" (
            echo Skipping TempoROS prebuild steps because TEMPO_SKIP_PREBUILD is %TEMPO_SKIP_PREBUILD%
            exit /b 0
        )
    )
)

REM Simply call the individual scripts from the same directory
bash %~dp0GenROSIDL.sh %*
if %errorlevel% neq 0 exit /b %errorlevel%
bash %~dp0GenROSBP.sh %3
if %errorlevel% neq 0 exit /b %errorlevel%

exit /b 0
