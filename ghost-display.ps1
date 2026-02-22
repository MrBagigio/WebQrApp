# ghost-display.ps1
# Show a persistent, colorized status window and the latest link saved by AvviaServer.bat

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$linkFile = Join-Path $scriptDir 'latest_link.txt'

if (Test-Path $linkFile) {
  $link = (Get-Content -Raw $linkFile).Trim()
  if ($link -eq '') { $link = 'Nessun URL disponibile' }
} else {
  $link = 'Nessun URL disponibile'
}

# Set nice window title
$host.UI.RawUI.WindowTitle = 'GHOST MODE: FIREWALL FIXED V31'

Write-Host '=====================================================' -ForegroundColor Cyan
Write-Host '         [ GHOST MODE: FIREWALL FIXED V31 ]' -ForegroundColor Yellow
Write-Host '=====================================================' -ForegroundColor Cyan
Write-Host ''
Write-Host '  [LOG WIPE]   : ACTIVE' -ForegroundColor Green
Write-Host '  [FIREWALL]   : HARDENED (All Apps Blocked)' -ForegroundColor Green
Write-Host ''
Write-Host '  [SOFTWARE OPERATIVO - PROTECTION ACTIVE]' -ForegroundColor Magenta
Write-Host '  NON CHIUDERE QUESTA FINESTRA!' -ForegroundColor Red
Write-Host ''
Write-Host '(Il link pubblico e\' stato copiato negli appunti e salvato in latest_link.txt)'
Write-Host ''
Write-Host 'URL:' -NoNewline
Write-Host " $link" -ForegroundColor Cyan
Write-Host ''

Read-Host 'Premi INVIO per chiudere questa finestra'
