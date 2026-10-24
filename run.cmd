@echo off
rem Build (only what changed) and run the game. Closes a running copy first.
rem   run            debug build + tests + run
rem   run release    optimized build + tests + run
rem   run fast       debug build + run, skip tests
setlocal
set CONFIG=debug
set EXTRA=
if /I "%~1"=="release" set CONFIG=release
if /I "%~1"=="fast" set EXTRA=-NoTests
if /I "%~2"=="fast" set EXTRA=-NoTests
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\build.ps1" -Config %CONFIG% %EXTRA%
