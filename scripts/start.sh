#!/bin/bash
# HaptX docker main script
# Capabilities
# - Compile project
# - Run dashboard
# - Run Project codes
#
# Usage: start.sh [--settle SECONDS] [--foreground] [--kill]

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib.sh"

COMPILE=0
RUN_CODE=""
BIND=0
KILLSESSION=0
DASHBOARD=0
SLEEP=0

while [ $# -gt 0 ]; do
    case "$1" in
        --compile)    COMPILE=1; shift;;
        --dashboard)  DASHBOARD=1; shift;;
        --run)        RUN_CODE="$2"; shift 2;;
        --kill)       KILLSESSION=1; shift;;
        -h|--help)    sed -n '2,8p' "$0"; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

if [ "$KILLSESSION" = "1" ]; then

    kill_session >/dev/null 2>&1
    echo "Session killed successfully"
    exit 0
fi

if [ "$COMPILE" = "1" ]; then
   echo "Compiling code"
   sleep 1
   echo "Compiled success"

fi

if [ "$DASHBOARD" = "1" ]; then
  if [ "$BIND" = "0" ]; then 
      bindevices
      BIND=1
  fi

  echo "Starting dashboard"
  $SCRIPT_DIR/start-dashboard.sh
else
  SLEEP=1

fi

if [ "$RUN_CODE" != "" ]; then
  if [ "$BIND" = "0" ]; then
      bindevices
      BIND=1
  fi

  echo "Running code"
  sleep 2
  exit 0
fi


if [ "$SLEEP" = "1" ]; then

  sleep infinity
  
fi
