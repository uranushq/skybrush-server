#!/bin/sh
# macOS/Linux equivalent of start.bat.
# Runs the server through uv, which creates/updates .venv from uv.lock on
# first run, so no manual environment setup is needed.
cd "$(dirname "$0")"
exec uv run skybrushd -p 5001 -c etc/conf/skybrush-outdoor.jsonc
