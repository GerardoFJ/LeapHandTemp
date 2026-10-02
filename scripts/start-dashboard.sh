#!/bin/bash
# Start the HaptX Dashboard, correctly.
#
# Two things make this more than "run the exe":
#
#  - The Dashboard needs a display, and must not have LIBGL_ALWAYS_SOFTWARE or
#    GALLIUM_DRIVER set -- with an NVIDIA driver present those stop the window
#    from ever appearing.
#
#  - A Wine session is held open across the run. The brief settle before
#    starting is margin only; see SETTLE below.
#
# If the Dashboard reports "Could not find a device with VID: 0x3181", the
# cause is almost certainly unbound devices rather than timing -- run
# haptx-bind.sh, or restart the container so the entrypoint binds them.
#
# Usage:  haptx-dashboard.sh [--settle SECONDS] [--foreground]
set -u

# 3s is margin, not a measured requirement: a settle of 0 connected reliably in
# testing once the devices were bound. The original 18s dated from before the
# SPDRP_ADDRESS bug was fixed and had no justification afterwards. Some margin
# is kept only because the failure is silent and unrecoverable -- libusb's
# device list is refreshed by hot-plug alone, and no arrival event fires for a
# device that was already plugged in, so an early scan poisons the session.
SETTLE="${HAPTX_USB_SETTLE:-3}"
FOREGROUND=0
while [ $# -gt 0 ]; do
    case "$1" in
        --settle)     SETTLE="$2"; shift 2 ;;
        --foreground) FOREGROUND=1; shift ;;
        -h|--help)    sed -n '2,17p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

export WINEPREFIX="${WINEPREFIX:-/root/.wine}"
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER

DASH="$WINEPREFIX/drive_c/Program Files/HaptX/SDK/Tools/HaptxDashboard"
if [ ! -f "$DASH/HaptxDashboard.exe" ]; then
    echo "HaptxDashboard.exe not found under $DASH" >&2
    echo "The image ships the SDK -- this prefix does not have it. Was the" >&2
    echo "image built without sdk/haptx-prefix.tar.gz, or is an old prefix" >&2
    echo "volume shadowing /root/.wine?" >&2
    exit 1
fi

/usr/local/bin/haptx-killall.sh > /dev/null 2>&1

echo "[haptx] holding a Wine session open, settling ${SETTLE}s..."
( wine cmd /c "ping -n 900 127.0.0.1" > /dev/null 2>&1 & )
sleep "$SETTLE"

cd "$DASH" || exit 1
if [ "$FOREGROUND" = "1" ]; then
    exec wine ./HaptxDashboard.exe
fi

nohup wine ./HaptxDashboard.exe > /tmp/haptx-dashboard.log 2>&1 < /dev/null &
sleep 25

if pgrep -f HaptxDashboard.exe > /dev/null; then
    echo "[haptx] Dashboard running. Log: /tmp/haptx-dashboard.log"
    echo "[haptx] SDK session log:"
    ls -t "$WINEPREFIX"/drive_c/ProgramData/HaptX/SDK/Logs/System/*.log 2>/dev/null | head -1
else
    echo "[haptx] Dashboard exited. Last lines of /tmp/haptx-dashboard.log:" >&2
    tail -20 /tmp/haptx-dashboard.log >&2
    exit 1
fi
