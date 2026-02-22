@echo off
TITLE Avvio Rapido AR Restoration
echo ==========================================
echo    AVVIO MOTORE AR - RESTORATION
echo ==========================================
echo.

:: 1. Controlla se Node.js è installato
node -v >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERRORE] Node.js non trovato. Per favore installalo da nodejs.org
    pause
    exit /b
)

:: 2. Avvia il server e apri la pagina
echo [1/2] Avvio server sulla porta 8080...
start "" http://localhost:8080/restoration-ar.html

echo [2/2] Esecuzione server...
echo Premere CTRL+C per fermare il server.
echo.
npx http-server -c-1 -p 8080
pause
