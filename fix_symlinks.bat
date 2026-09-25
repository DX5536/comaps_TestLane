@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0fix_symlinks.ps1" > "%~dp0fix_symlinks_log.txt" 2>&1
type "%~dp0fix_symlinks_log.txt"
pause
