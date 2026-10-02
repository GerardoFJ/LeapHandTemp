#!/bin/bash
# Container entrypoint: bind whatever hardware is attached, then hand off.
set -u

if [ "${HAPTX_SKIP_BIND:-0}" = "1" ]; then
    echo "[haptx] HAPTX_SKIP_BIND=1, leaving device bindings alone"
else
    /usr/local/bin/haptx-bind.sh || true
    echo "[haptx] ready. Start the Dashboard with: haptx-dashboard.sh"
fi

exec "$@"
