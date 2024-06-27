@echo off

REM Simply call the individual scripts from the same directory
bash %~dp0GenROSIDL.sh %*
if %errorlevel% neq 0 exit /b %errorlevel%
bash %~dp0GenROSBP.sh %3
if %errorlevel% neq 0 exit /b %errorlevel%

exit /b 0
