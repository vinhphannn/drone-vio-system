#!/usr/bin/env bash
# =============================================================================
# build.sh — Configure and compile drone_core
# Usage: ./scripts/build.sh [Debug|Release]
# =============================================================================
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BUILD_TYPE="${1:-Release}"

echo "=== drone_core build ==="
echo "  Root      : $ROOT_DIR"
echo "  Build dir : $BUILD_DIR"
echo "  Build type: $BUILD_TYPE"
echo ""

cmake -S "$ROOT_DIR" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build "$BUILD_DIR" --parallel "$(nproc)"

# Symlink compile_commands.json to project root for clangd/IDEs
ln -sf "$BUILD_DIR/compile_commands.json" "$ROOT_DIR/compile_commands.json"

echo ""
echo "=== Build SUCCESS ==="
echo ""
echo "Run:  $BUILD_DIR/drone_core $ROOT_DIR/config/drone_core.yaml"
echo "  or: ./scripts/run.sh"
echo ""
echo "Tips:"
echo "  IMU device : ls /dev/ttyACM* | ls /dev/ttyUSB*"
echo "  Camera     : ls /dev/video*  | v4l2-ctl --list-devices"
echo "  Permissions: sudo usermod -aG dialout,video \$USER && relogin"
