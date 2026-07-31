@echo off
rem Windows equivalent of start.sh: run the server from the project venv
rem (created with `uv sync`) so no manual activation is needed.
cd /d "%~dp0"
if exist .venv\Scripts\skybrushd.exe (
    .venv\Scripts\skybrushd.exe -p 5001 -c etc/conf/skybrush-outdoor.jsonc
) else (
    skybrushd -p 5001 -c etc/conf/skybrush-outdoor.jsonc
)
