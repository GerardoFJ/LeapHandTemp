#!/bin/bash
# Smoke-test the running container: is the patched Wine actually installed, and
# does the container see the hardware? Run from the host.
#
# Usage:  scripts/check.sh [CONTAINER]     (default: haptx)
set -u

NAME="${1:-haptx}"

if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    echo "container '$NAME' is not running. Start it with:" >&2
    echo "  docker compose up -d" >&2
    exit 1
fi

echo "--- wine version (must match the tag the patch was built against) ---"
docker exec "$NAME" wine --version

echo
echo "--- patched modules present? ---"
for f in wineusb.sys winusb.dll setupapi.dll cfgmgr32.dll; do
    docker exec "$NAME" test -f "/opt/wine-staging/lib/wine/x86_64-windows/$f" \
        && echo "  $f" || echo "  $f  *** MISSING ***"
done
docker exec "$NAME" test -f /opt/wine-staging/lib/wine/x86_64-unix/wineusb.so \
    && echo "  wineusb.so (Unix half)" || echo "  wineusb.so  *** MISSING ***"

echo
echo "--- winusb.dll implemented, not the upstream stub? ---"
docker exec "$NAME" bash -c \
    'grep -ao "WinUsb_[A-Za-z]*" /opt/wine-staging/lib/wine/x86_64-windows/winusb.dll \
       | sort -u | wc -l' \
    | awk '{ if ($1 > 5) print "  " $1 " distinct WinUsb_* symbols -- patched"; \
             else print "  only " $1 " -- STILL THE UPSTREAM STUB" }'

echo
echo "--- hot-plug prerequisite: /run/udev mounted? ---"
docker exec "$NAME" test -e /run/udev/control \
    && echo "  yes" \
    || echo "  NO -- replugging a device will not be noticed. Add
       - /run/udev:/run/udev:ro"

echo
echo "--- HaptX devices visible to the container ---"
found=$(docker exec "$NAME" bash -c '
for d in /sys/bus/usb/devices/*/; do
    v=$(cat "$d/idVendor" 2>/dev/null) || continue
    [ "$v" = "3181" ] || continue
    printf "  %-10s %s:%s  %s\n" "$(basename "$d")" "$v" \
        "$(cat "$d/idProduct" 2>/dev/null)" "$(cat "$d/product" 2>/dev/null)"
done')
if [ -n "$found" ]; then
    echo "$found"
else
    echo "  none found (VID 3181) -- gloves/Airpack not powered or not plugged in."
    echo "  Check on the host with:  lsusb -d 3181:"
fi

echo
echo "--- SDK installed in the prefix? ---"
docker exec "$NAME" test -f \
    "/root/.wine/drive_c/Program Files/HaptX/SDK/Tools/HaptxDashboard/HaptxDashboard.exe" \
    && echo "  yes" \
    || echo "  NO -- the image should ship it; was it built without sdk/haptx-prefix.tar.gz?"
