#!/bin/sh
set -e

SRC_DIR="${1:-.}"
BUILD_DIR="${2:-$SRC_DIR/build-win}"

export WINEPREFIX="${MSVC_WINEPREFIX:-/root/.wine-msvc}"
export PATH="/opt/msvc/bin/x64:$PATH"

if [ ! -f "$WINEPREFIX/system.reg" ]; then
    echo "[haptx] creating the MSVC Wine prefix..."
    WINEDEBUG=-all wineboot -u >/dev/null 2>&1 || true
    wineserver -w 2>/dev/null || true
fi

CC=cl CXX=cl cmake -S "$SRC_DIR" -B "$BUILD_DIR" -G Ninja \
    -DCMAKE_SYSTEM_NAME=Windows \
    -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD_DIR" -j

echo "built in $BUILD_DIR -- run with: wine $BUILD_DIR/test.exe -c config/test.yaml"
