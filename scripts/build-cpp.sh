#!/bin/sh

set -e

SRC="${1:?usage: haptx-build.sh <source.cpp> [extra cl arguments...]}"
shift

if [ ! -f "$SRC" ]; then
    echo "no such file: $SRC" >&2
    exit 1
fi

DIR="$(cd "$(dirname "$SRC")" && pwd)"
BASE="$(basename "$SRC" .cpp)"

if [ ! -x /opt/msvc/bin/x64/cl ]; then
    echo "MSVC toolchain missing at /opt/msvc -- was the image built without it?" >&2
    exit 1
fi

export WINEPREFIX="${MSVC_WINEPREFIX:-/root/.wine-msvc}"
export PATH="/opt/msvc/bin/x64:$PATH"

if [ ! -f "$WINEPREFIX/system.reg" ]; then
    echo "[haptx] creating the MSVC Wine prefix (first build in this container)..."
    WINEDEBUG=-all wineboot -u >/dev/null 2>&1 || true
    wineserver -w 2>/dev/null || true
fi

SDK_UNIX="${HAPTX_SDK_UNIX:-/root/.wine/drive_c/Program Files/HaptX/SDK}"
SDK='Z:\root\.wine\drive_c\Program Files\HaptX\SDK'

cd "$DIR"
cl /nologo /std:c++17 /EHsc /O2 /MD /wd4251 \
   '/DHAPTXAPI_DLLEXPORT=__declspec(dllimport)' \
   "/I${SDK}\\Include" \
   "$(basename "$SRC")" \
   "/Fe:${BASE}.exe" \
   "$@" \
   /link "${SDK}\\Dll\\Release\\HaptxApi.lib" user32.lib gdi32.lib

for dll in "$SDK_UNIX/Dll/Release/HaptxApi.dll" "$SDK_UNIX/ThirdParty/OpenVR/openvr_api.dll"; do
    [ -f "$dll" ] && [ ! -f "$DIR/$(basename "$dll")" ] && cp "$dll" "$DIR/"
done

echo "built $DIR/${BASE}.exe -- run it with: wine ${BASE}.exe"
