#!/bin/bash
# Two things make this more than "run the exe":
#
#  - The Dashboard needs a display, and must not have LIBGL_ALWAYS_SOFTWARE or
#    GALLIUM_DRIVER set -- with an NVIDIA driver present those stop the window
#    from ever appearing.
#
#  - A Wine session is held open across the run. The brief settle before
#    starting is margin only; see SETTLE below.
#
# Usage:  haptx-dashboard.sh [--settle SECONDS] [--foreground] [--kill]
set -u
SETTLE="${HAPTX_USB_SETTLE:-3}"
FOREGROUND=0
KILLSESSION=0
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib.sh"

while [ $# -gt 0 ]; do
    case "$1" in
        --settle)     SETTLE="$2"; shift 2 ;;
        --foreground) FOREGROUND=1; shift ;;
        --kill)       KILLSESSION=1; shift ;;
        -h|--help)    sed -n '2,11p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

# Kill dashbaord and wine session
if [ "$KILLSESSION" = "1" ]; then

    kill_session > /dev/null 2>&1
    echo "HaptxDashboard and wine session killed"
    exit 0

fi


export WINEPREFIX="${WINEPREFIX:-/root/.wine}"
unset LIBGL_ALWAYS_SOFTWARE GALLIUM_DRIVER

DASH="$WINEPREFIX/drive_c/Program Files/HaptX/SDK/Tools/HaptxDashboard"
if [ ! -f "$DASH/HaptxDashboard.exe" ]; then
    echo "HaptxDashboard.exe not found under $DASH" >&2
    exit 1
fi

#CLEAN START
kill_session > /dev/null 2>&1

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
