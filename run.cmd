@echo off
rem Build (only what changed) and run the game.
rem   run            debug build + tests + run
rem   run fast       debug build + run, skip tests
rem   run release    optimized build + tests + run
rem   run watch      rebuild + swap in the game on every save (like nodemon)
setlocal
if /I "%~1"=="watch" (
  powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\watch.ps1" %2 %3
  exit /b %errorlevel%
)
set CONFIG=debug
set EXTRA=
if /I "%~1"=="release" set CONFIG=release
if /I "%~1"=="fast" set EXTRA=-NoTests
if /I "%~2"=="fast" set EXTRA=-NoTests
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\build.ps1" -Config %CONFIG% %EXTRA%