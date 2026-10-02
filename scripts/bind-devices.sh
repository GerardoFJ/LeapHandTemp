#!/bin/bash
set -u

export WINEPREFIX="${WINEPREFIX:-/root/.wine}"
SETTLE="${HAPTX_USB_SETTLE:-18}"

log() { echo "[haptx] $*"; }

if [ ! -d "$WINEPREFIX" ]; then
    log "no Wine prefix at $WINEPREFIX"
    exit 1
fi

if ! grep -qls '^3181$' /sys/bus/usb/devices/*/idVendor 2>/dev/null; then
    log "no HaptX devices on the USB bus -- nothing to bind"
    exit 0
fi

# wineusb.sys enumerates asynchronously; the devnodes do not exist immediately.
log "letting Wine enumerate USB (${SETTLE}s)..."
( wine cmd /c "ping -n 300 127.0.0.1" >/dev/null 2>&1 & )
sleep "$SETTLE"

/usr/local/bin/haptx-killall.sh >/dev/null 2>&1

log "binding HaptX devices:"
python3 /usr/local/share/haptx/bind-haptx-devices.py "$WINEPREFIX/system.reg"
