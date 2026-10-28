#!/bin/bash
# HaptX docker main script
# Capabilities
# - Compile project
# - Run dashboard
# - Run Project codes
#
# Usage: start.sh [--compile] [--dashboard] [--no-bind] [--run "Program to run"] [--run-config '--params'] [--kill] [--help]
# 
# If you want to run this with docker compose send the START_ARGS variable with the args.
#
# Example:
# START_ARGS="--compile --dashboard --no-bind --run test --run-config '--help' " docker compose up 
#

set -u
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/lib.sh"

COMPILE=0
RUN_CODE=""
BIND=0
KILLSESSION=0
DASHBOARD=0
SLEEP=0
CODE_CONFIG=""

while [ $# -gt 0 ]; do
    case "$1" in
        --compile)    COMPILE=1; shift;;
        --dashboard)  DASHBOARD=1; shift;;
        --no-bind)    BIND=1; shift;;
        --run)        RUN_CODE="$2"; shift 2;;
        --run-config) CODE_CONFIG="$2"; shift 2;;
        --kill)       KILLSESSION=1; shift;;
        -h|--help)    sed -n '2,14p' "$0"; exit 0 ;;
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
   if /workspace/test_compile.sh; then
     echo "Compiled success"
   else
     echo "Compile FAILED"
     exit 1
   fi

fi

if [ "$DASHBOARD" = "1" ]; then
  if [ "$BIND" = "0" ]; then 
      bindevices
      BIND=1
  fi

  echo "Starting dashboard"
  $SCRIPT_DIR/start-dashboard.sh

  if [ "$RUN_CODE" = "" ]; then
      SLEEP=1
  fi

elif [ "$COMPILE" != "1" ]; then
  SLEEP=1

fi

if [ "$RUN_CODE" != "" ]; then
  if [ "$BIND" = "0" ]; then
      bindevices
      BIND=1
  fi
  if [ ! -d "/workspace/build-win/" ]; then
    echo "No compilation folder please run --compile first"
    exit 1
  fi
  if [ ! -f "/workspace/build-win/$RUN_CODE.exe" ]; then
    echo "No file found to run, please check program name"
    exit 1
  fi 
  echo "Running code $RUN_CODE"
  wine /workspace/build-win/$RUN_CODE.exe $CODE_CONFIG
  echo "Code finished"
  exit 0
fi


if [ "$SLEEP" = "1" ]; then
  if [ "$DASHBOARD" != "1" ]; then
    echo "No arguments passed keeping docker session, if you want to see options please send -h"
  fi
  echo "Press Ctrl + C to stop the container"
  sleep infinity
  
fi
