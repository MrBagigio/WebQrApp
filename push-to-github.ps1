# PowerShell script to initialize and push WebQrApp to GitHub
# Edit the variables below with your GitHub username and desired repo name.

$githubUser = "<TUO_UTENTE>"    # replace with your GitHub username
$repoName   = "WebQrApp"         # or change if you prefer another name

# Ensure git is available
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    Write-Error "Git non è installato o non è nel PATH. Installa Git e riprova."
    exit 1
}

# Go to project directory (adjust if script placed elsewhere)
Set-Location -Path "$PSScriptRoot"

Write-Host "Inizializzo repository locale..."
git init

Write-Host "Aggiungo tutti i file e faccio il commit iniziale..."
git add .
git commit -m "Initial commit - WebQrApp"

Write-Host "Aggiungo remoto origin a https://github.com/$githubUser/$repoName.git"
git remote add origin "https://github.com/$githubUser/$repoName.git"

git branch -M main

Write-Host "Push verso GitHub (scala in cima, ti potrebbe chiedere credenziali)..."
git push -u origin main

Write-Host "Fatto. Ora vai su GitHub, crea il repository se non esiste, quindi abilita GitHub Pages nelle impostazioni."
