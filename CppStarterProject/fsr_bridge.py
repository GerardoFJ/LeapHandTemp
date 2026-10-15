#!/usr/bin/env python3
"""
Forward the Teensy FSR matrix to fsr_fingertip.exe over UDP.

The Teensy (FSR_Development/Teensy_Project, src/main.cpp) prints
    DIM,<rows>,<cols>,<range_v>     once at boot
    F,<micros>,<c0>,<c1>,...        one line per frame, raw ADC LSB, row-major

Every frame is re-sent as one self-describing UDP datagram
    M,<rows>,<cols>,<micros>,<c0>,<c1>,...
so the receiver never depends on having seen the DIM line, which the Teensy
only prints once at boot. Values are passed through raw; baseline, inversion
and scaling happen in fsr_fingertip.cpp.

The haptx container runs with network_mode: host, so 127.0.0.1 reaches it.

    ./fsr_bridge.py                       # auto-detect the Teensy
    ./fsr_bridge.py --serial /dev/ttyACM0
    ./fsr_bridge.py --demo                # no Teensy: a synthetic moving press
"""

import argparse
import math
import socket
import sys
import time

TEENSY_VID = 0x16C0


def find_teensy():
    try:
        from serial.tools import list_ports
    except ImportError:
        sys.exit("pyserial missing:  pip install pyserial")
    for port in list_ports.comports():
        if port.vid == TEENSY_VID:
            return port.device
    for port in list_ports.comports():
        if "ACM" in port.device:
            return port.device
    return None


class Dims:
    def __init__(self, rows, cols):
        self.rows, self.cols = rows, cols

    def resolve(self, n_cells):
        """Grid shape for a frame of n_cells values, or None if unknown."""
        if self.rows and self.cols and self.rows * self.cols == n_cells:
            return self.rows, self.cols
        side = int(round(math.sqrt(n_cells)))
        if side * side == n_cells:
            return side, side
        return None


def run_serial(args, sock, dest):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial missing:  pip install pyserial")

    dims = Dims(args.rows, args.cols)
    warned_shape = False

    while True:
        port = args.serial or find_teensy()
        if port is None:
            print("waiting for the Teensy to appear...", end="\r", flush=True)
            time.sleep(1.0)
            continue
        try:
            ser = serial.Serial(port, 115200, timeout=1.0)
        except serial.SerialException as e:
            print(f"cannot open {port}: {e}; retrying")
            time.sleep(1.0)
            continue

        print(f"reading {port} -> udp {dest[0]}:{dest[1]}")
        sent, t_report = 0, time.monotonic()
        try:
            while True:
                line = ser.readline().decode("ascii", "replace").strip()
                if not line:
                    continue
                fields = line.split(",")
                if fields[0] == "DIM" and len(fields) >= 3:
                    dims.rows, dims.cols = int(fields[1]), int(fields[2])
                    print(f"DIM {dims.rows}x{dims.cols}")
                    continue
                if fields[0] != "F" or len(fields) < 3:
                    continue

                cells = fields[2:]
                shape = dims.resolve(len(cells))
                if shape is None:
                    if not warned_shape:
                        print(f"frame has {len(cells)} cells and no DIM line was seen; "
                              f"pass --rows/--cols")
                        warned_shape = True
                    continue

                msg = f"M,{shape[0]},{shape[1]},{fields[1]}," + ",".join(cells)
                sock.sendto(msg.encode("ascii"), dest)
                sent += 1

                now = time.monotonic()
                if now - t_report >= 1.0:
                    rate = sent / (now - t_report)
                    print(f"{shape[0]}x{shape[1]}  {rate:5.1f} frames/s   ", end="\r", flush=True)
                    if args.print:
                        print("\n" + line)
                    sent, t_report = 0, now
        except serial.SerialException as e:
            print(f"\nserial error: {e}; reconnecting")
            ser.close()
            time.sleep(1.0)


def run_demo(args, sock, dest):
    """A press that circles the pad and breathes, in the Teensy's own sign
    convention: raw ADC goes negative under force, about -30000 LSB at 25 N."""
    rows, cols = args.rows or 4, args.cols or 4
    print(f"demo {rows}x{cols} -> udp {dest[0]}:{dest[1]}  (Ctrl-C to stop)")
    t0 = time.monotonic()
    period = 1.0 / 50.0
    next_t = t0
    while True:
        t = time.monotonic() - t0
        cr = (rows - 1) / 2 + (rows - 1) / 2 * math.cos(2 * math.pi * 0.25 * t)
        cc = (cols - 1) / 2 + (cols - 1) / 2 * math.sin(2 * math.pi * 0.25 * t)
        amp = args.demo_peak * (0.5 + 0.5 * math.sin(2 * math.pi * 0.1 * t))
        cells = []
        for r in range(rows):
            for c in range(cols):
                d2 = (r - cr) ** 2 + (c - cc) ** 2
                cells.append(str(int(-amp * math.exp(-d2 / 1.0))))
        msg = f"M,{rows},{cols},{int(t * 1e6)}," + ",".join(cells)
        sock.sendto(msg.encode("ascii"), dest)
        next_t += period
        time.sleep(max(0.0, next_t - time.monotonic()))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", help="Teensy serial port (default: auto-detect)")
    ap.add_argument("--host", default="127.0.0.1", help="where fsr_fingertip.exe runs")
    ap.add_argument("--udp-port", type=int, default=9870)
    ap.add_argument("--rows", type=int, help="grid rows if the DIM line was missed")
    ap.add_argument("--cols", type=int, help="grid cols if the DIM line was missed")
    ap.add_argument("--demo", action="store_true", help="send a synthetic press, no Teensy")
    ap.add_argument("--demo-peak", type=float, default=15000.0,
                    help="peak LSB of the synthetic press (default 15000)")
    ap.add_argument("--print", action="store_true", help="echo one raw line per second")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (args.host, args.udp_port)
    try:
        if args.demo:
            run_demo(args, sock, dest)
        else:
            run_serial(args, sock, dest)
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
