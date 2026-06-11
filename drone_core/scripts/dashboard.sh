#!/usr/bin/env bash
# Open the live drone_core dashboard (reads /tmp/drone_core_stats.json)
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "$ROOT_DIR/tools/dashboard.py" "$@"
