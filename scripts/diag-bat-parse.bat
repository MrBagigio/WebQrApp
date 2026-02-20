@echo on
setlocal EnableDelayedExpansion
cd /d "%~dp0.."

echo STEP A
set "SUBDOMAIN=webqrapp-live-%USERNAME%"
set "SUBDOMAIN=%SUBDOMAIN: =%"
set "SUBDOMAIN=%SUBDOMAIN:_=-%"
set "FULLURL=https://%SUBDOMAIN%.loca.lt/restoration-ar.html"
echo SUB=!SUBDOMAIN!
echo URL=!FULLURL!

echo STEP B
tasklist /v /fi "imagename eq cmd.exe" | findstr /i /c:"LocalTunnel Watchdog - %SUBDOMAIN%" >nul
echo ERR=%ERRORLEVEL%

echo STEP C
start "LocalTunnel Watchdog" /min "%~dp0localtunnel-watchdog.bat" 8080 %SUBDOMAIN%
echo DONE
