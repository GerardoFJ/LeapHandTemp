#!/bin/bash

pkill -TERM -f HaptxDashboard.exe 2>/dev/null
pkill -TERM -f CppStarterProject.exe 2>/dev/null
sleep 2

wineserver -k 2>/dev/null
for i in $(seq 1 15); do
    pgrep -x wineserver >/dev/null || break
    sleep 1
done

pkill -9 -f HaptxDashboard.exe 2>/dev/null
pkill -9 -f CppStarterProject.exe 2>/dev/null
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
