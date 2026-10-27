
### Common actions on HaptX deplayment

kill_session() {
    ## Function to kill HaptX and wine session to fresh start
    pkill -TERM -f HaptxDashboard.exe 2>/dev/null
    sleep 1

    wineserver -k 2>/dev/null
    sleep 1

    pkill -9 -f HaptxDashboard.exe 2>/dev/null
    for p in wineserver winedevice.exe services.exe explorer.exe plugplay.exe \
            rpcss.exe svchost.exe winedbg.exe start.exe cmd.exe ping.exe; do
        pkill -9 -f "$p" 2>/dev/null
    done
    sleep 2

    if ps -eo pid,comm | grep -iE "haptx|wine|CppStarter" | grep -v grep; then
        echo "^^ still alive"
    else
        echo "(none)"
    fi


}

bindevices() {
    ## Function to bind USB devices
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

    kill_session >/dev/null 2>&1

    log "binding HaptX devices:"
    python3 /usr/local/bin/scripts/bind-haptx-devices.py "$WINEPREFIX/system.reg"

}


