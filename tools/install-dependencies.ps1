param([Parameter(Mandatory=$true)][string]$SourceRoot)
$ErrorActionPreference = 'Stop'
$repo = Split-Path $PSScriptRoot -Parent
$manifest = Get-Content -Raw (Join-Path $repo 'docs/dependencies.json') | ConvertFrom-Json
$source = (Resolve-Path -LiteralPath $SourceRoot).Path
$files = Get-ChildItem -LiteralPath $source -File -Recurse
$missing = @()
foreach ($entry in $manifest) {
    $target = Join-Path $repo $entry.path
    if ((Test-Path -LiteralPath $target) -and ((Get-FileHash -LiteralPath $target -Algorithm SHA256).Hash -eq $entry.sha256)) { continue }
    $match = $null
    foreach ($candidate in ($files | Where-Object Name -eq $entry.name)) {
        if ((Get-FileHash -LiteralPath $candidate.FullName -Algorithm SHA256).Hash -eq $entry.sha256) { $match = $candidate; break }
    }
    if ($null -eq $match) { $missing += $entry.path; continue }
    New-Item -ItemType Directory -Force (Split-Path $target -Parent) | Out-Null
    Copy-Item -LiteralPath $match.FullName -Destination $target
}
if ($missing.Count -gt 0) { $missing | ForEach-Object { Write-Host "Missing exact dependency: $_" }; throw 'Dependencies incomplete; use the matching original package. No substitute files were guessed.' }
Write-Host 'All dependency files match the recorded SHA256 hashes.'
