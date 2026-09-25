# Repairs CoMaps symlinks on Windows and restores missing files.
Set-Location $PSScriptRoot
$dev = (Get-ItemProperty 'HKLM:\SOFTWARE\Microsoft\Windows\CurrentVersion\AppModelUnlock' -ErrorAction SilentlyContinue).AllowDevelopmentWithoutDevLicense
if ($dev -ne 1) {
  Write-Host "Developer Mode is OFF. Turn it on in the window that opens, then run this again." -ForegroundColor Yellow
  Start-Process "ms-settings:developers"
  exit 1
}
git config core.symlinks true
$links = @(git ls-files -s | Where-Object { $_ -match '^120000' } | ForEach-Object { $_.Split("`t")[1] })
Write-Host "Recreating $($links.Count) symlinks..."
foreach ($l in $links) { Remove-Item -LiteralPath $l -Force -ErrorAction SilentlyContinue }
git checkout -- $links
git checkout -- 3party/icu/CMakeLists.txt
$ok = @($links | Where-Object { (Get-Item -LiteralPath $_ -Force -ErrorAction SilentlyContinue).LinkType -eq 'SymbolicLink' }).Count
Write-Host "Symlinks OK: $ok of $($links.Count)"
Write-Host "3party\icu\CMakeLists.txt present: $(Test-Path 3party\icu\CMakeLists.txt)"
if ($ok -eq $links.Count) { Write-Host "Done." -ForegroundColor Green } else { Write-Host "Some symlinks failed - is Developer Mode on?" -ForegroundColor Red }
