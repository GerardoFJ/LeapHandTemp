#!/usr/bin/env python3

import re
import shutil
import sys
import time

WINUSB_IFACE    = "{DEE824EF-729B-4A0E-9C14-B7117D33A817}"   # GUID_DEVINTERFACE_WINUSB
USBDEVICE_CLASS = "{88BAE032-5A81-49F0-BC3D-A4FF138216D6}"
ENUM_PREFIX     = "[System\\\\ControlSet001\\\\Enum\\\\USB\\\\VID_3181&PID_"

path = sys.argv[1] if len(sys.argv) > 1 else "/root/.wine/system.reg"


def service_for(device_id):
    """device_id looks like VID_3181&PID_0202&MI_00 -- decide its driver."""
    if "&MI_" in device_id:
        return "WinUSB", True          # composite interface
    if "PID_0202" in device_id:
        return "USBCCGP", False        # composite parent
    return "WinUSB", False             # glove, or anything else HaptX


def main():
    try:
        text = open(path, encoding="utf-8", errors="surrogateescape").read()
    except FileNotFoundError:
        print("  no registry at %s -- has wine run yet?" % path)
        return 0

    lines = text.split("\n")
    out, i, changed, already = [], 0, [], 0

    while i < len(lines):
        line = lines[i]
        if not line.startswith(ENUM_PREFIX) or "]" not in line:
            out.append(line)
            i += 1
            continue

        key = line[1:line.index("]")]
        # skip the Properties/Device Parameters subkeys; only the device key
        if "\\\\Properties\\\\" in key or "\\\\Device Parameters" in key:
            out.append(line)
            i += 1
            continue

        block, i = [line], i + 1
        while i < len(lines) and not lines[i].startswith("["):
            block.append(lines[i])
            i += 1

        parts = key.split("\\\\")
        device_id = parts[-2]                    # VID_3181&PID_xxxx[&MI_yy]
        service, is_interface = service_for(device_id)

        body = "\n".join(block)
        if '"Service"="%s"' % service in body and USBDEVICE_CLASS in body:
            already += 1
            out.extend(block)
            continue

        new, saw_service = [], False
        for b in block:
            if b.startswith('"Service"='):
                b, saw_service = '"Service"="%s"' % service, True
            elif b.startswith('"ClassGUID"='):
                b = '"Class"="USBDevice"\n"ClassGUID"="%s"' % USBDEVICE_CLASS
            elif b.startswith('"Class"='):
                continue                          # rewritten alongside ClassGUID
            new.append(b)
        while new and new[-1] == "":
            new.pop()
        if not saw_service:
            new.append('"Service"="%s"' % service)

        # Only composite interfaces need the interface class spelled out; a
        # non-composite device is found through GUID_DEVINTERFACE_USB_DEVICE,
        # so leave whatever the installer wrote for the gloves alone.
        if is_interface:
            new.append("")
            new.append("[%s\\\\Device Parameters] 1789355563" % key)
            new.append('"DeviceInterfaceGUIDs"=str(7):"%s\\0"' % WINUSB_IFACE)
        new.append("")

        out.extend(new)
        changed.append((device_id, service))

    if not changed:
        if already:
            print("  %d HaptX device(s) already bound, nothing to do" % already)
        else:
            print("  no HaptX devices found in the registry "
                  "(not plugged in, or wine has not enumerated yet)")
        return 0

    shutil.copy(path, "%s.bak-bind-%s" % (path, time.strftime("%Y%m%d-%H%M%S")))
    open(path, "w", encoding="utf-8", errors="surrogateescape").write("\n".join(out))

    for dev, svc in changed:
        print("  %-34s -> Service=%s" % (dev, svc))
    if already:
        print("  (%d already bound)" % already)
    return 0


if __name__ == "__main__":
    sys.exit(main())
