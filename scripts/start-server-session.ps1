$ErrorActionPreference = 'Stop'

Set-Location -Path (Split-Path -Parent $PSScriptRoot)

Write-Host "======================================================"
Write-Host "   AR MOUNTAIN EXPEDITION - PRO AUTO-LAUNCHER"
Write-Host "======================================================"
Write-Host ""

function Get-LocalIPv4 {
  try {
    $ip = Get-NetIPAddress -AddressFamily IPv4 |
      Where-Object { $_.IPAddress -notlike '169.*' -and $_.IPAddress -notlike '127.*' -and $_.InterfaceAlias -notmatch 'Loopback' } |
      Select-Object -First 1 -ExpandProperty IPAddress
    if ($ip) { return $ip }
  } catch {}
  return '127.0.0.1'
}

Write-Host "[1/3] Server locale..."
$listen = Get-NetTCPConnection -State Listen -LocalPort 8080 -ErrorAction SilentlyContinue | Select-Object -First 1
if ($listen) {
  Write-Host "WebAR Server gia' attivo (PID $($listen.OwningProcess))."
} else {
  Write-Host "Avvio WebAR Server su porta 8080..."
  Start-Process -FilePath 'cmd.exe' -ArgumentList '/c', 'npx http-server -p 8080 -c-1' -WindowStyle Minimized
  Start-Sleep -Seconds 2
}

Write-Host "[2/3] Tunnel watchdog..."
$u = ($env:USERNAME -replace '[^a-zA-Z0-9-]', '').ToLower()
if ([string]::IsNullOrWhiteSpace($u)) { $u = 'user' }
$subdomain = "webqrapp-live-$u"
$fullUrl = "https://$subdomain.loca.lt/mindar-app/index.html"

$watchdog = Get-CimInstance Win32_Process |
  Where-Object {
    $_.Name -eq 'cmd.exe' -and
    $_.CommandLine -match 'localtunnel-watchdog\.bat' -and
    $_.CommandLine -match [Regex]::Escape($subdomain)
  } |
  Select-Object -First 1

if ($watchdog) {
  Write-Host "Watchdog tunnel gia' attivo (PID $($watchdog.ProcessId))."
} else {
  $watchdogBat = Join-Path (Split-Path -Parent $PSScriptRoot) 'scripts\localtunnel-watchdog.bat'
  Start-Process -FilePath 'cmd.exe' -ArgumentList '/c', "`"$watchdogBat`" 8080 $subdomain" -WindowStyle Minimized
  Start-Sleep -Seconds 2
  Write-Host "Watchdog tunnel avviato (auto-reconnect ON)."
}

Write-Host "[3/3] Link sessione..."
$linkFile = Join-Path (Split-Path -Parent $PSScriptRoot) 'latest_link.txt'
Set-Content -Path $linkFile -Value $fullUrl -Encoding ascii

$localIp = Get-LocalIPv4
Write-Host ""
Write-Host "URL: $fullUrl"
Write-Host "PSW (se richiesta da loca.lt): $localIp"
Write-Host "Local Endpoint: http://$localIp`:8080/mindar-app/index.html"
Write-Host ""
Write-Host "Lascia questa finestra aperta durante la sessione."
