@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0start.ps1" -Real
if errorlevel 1 pause
