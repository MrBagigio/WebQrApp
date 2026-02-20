@echo off
SETLOCAL EnableDelayedExpansion

set "PORT=%~1"
set "SUBDOMAIN=%~2"

if not defined PORT set "PORT=8080"
if not defined SUBDOMAIN (
  echo [ERRORE] Subdomain mancante. Uso: localtunnel-watchdog.bat ^<port^> ^<subdomain^>
  exit /b 1
)

title LocalTunnel Watchdog - %SUBDOMAIN%
echo ======================================================
echo  LocalTunnel Watchdog (auto-reconnect)
echo  URL: https://%SUBDOMAIN%.loca.lt
echo  Porta locale: %PORT%
echo ======================================================
echo.

:loop
echo [%date% %time%] Avvio tunnel...
npx localtunnel --port %PORT% --subdomain %SUBDOMAIN%
set "ERR=!ERRORLEVEL!"
echo [%date% %time%] Tunnel disconnesso (code !ERR!). Reconnect tra 3 secondi...
timeout /t 3 >nul
goto loop
