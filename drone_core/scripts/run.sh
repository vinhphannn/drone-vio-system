#!/usr/bin/env bash
# =============================================================================
# run.sh — Launch the drone_core pipeline
# Usage: ./scripts/run.sh [path/to/config.yaml]
# =============================================================================
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BINARY="$ROOT_DIR/build/drone_core"
CONFIG="${1:-$ROOT_DIR/config/drone_core.yaml}"

if [[ ! -f "$BINARY" ]]; then
    echo "[run.sh] Binary not found. Building first..."
    bash "$ROOT_DIR/scripts/build.sh"
fi

if [[ ! -f "$CONFIG" ]]; then
    echo "[run.sh] ERROR: Config not found: $CONFIG"
    exit 1
fi

echo "[run.sh] Starting drone_core..."
exec "$BINARY" "$CONFIG"
