@echo off
TITLE ExpeAR Pro - Auto Server + Tunnel
SETLOCAL

cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\start-server-session.ps1"
if errorlevel 1 (
  echo.
  echo Errore durante l'avvio della sessione server/tunnel.
)
echo.
pause
