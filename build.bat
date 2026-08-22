@echo off
setlocal

set "ROOT_DIR=%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 "%ROOT_DIR%scripts\build.py" %*
) else (
    python "%ROOT_DIR%scripts\build.py" %*
)
