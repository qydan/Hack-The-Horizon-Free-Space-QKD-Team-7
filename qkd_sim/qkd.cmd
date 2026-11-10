@echo off
rem Windows launcher for qkd.py: uses the project's .venv if there is one. Usage:  .\qkd doctor
setlocal
set "HERE=%~dp0"
set "PY=python"
if exist "%HERE%..\.venv\Scripts\python.exe" set "PY=%HERE%..\.venv\Scripts\python.exe"
if exist "%HERE%.venv\Scripts\python.exe" set "PY=%HERE%.venv\Scripts\python.exe"
"%PY%" "%HERE%qkd.py" %*
exit /b %errorlevel%
