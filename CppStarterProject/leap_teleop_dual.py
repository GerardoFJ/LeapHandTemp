#!/usr/bin/env python3
"""
Teleoperate two LEAP Hands at once, one per HaptX glove.

    ik_stream.exe --haptx.hand both --UDP 9871: "H2,L,..." + "H2,R,..."--> leap_teleop_dual.py
        L frames -> leap_mapping.json       -> left LEAP Hand
        R frames -> leap_mapping_right.json -> right LEAP Hand

Both gloves arrive on one port; the hand letter of each frame picks its mapping
and its LEAP Hand. Each LEAP Hand is either plugged into this PC (--left-serial,
--right-serial) or driven by leap_hand_node.py on another PC (--left-send,
--right-send: one node per hand, each listening on its own port). Per hand, the
mapping, smoothing and speed limit are leap_teleop.py's, so a hand moves exactly
as it does with leap_teleop.py --real --no-viewer. There is no MuJoCo view.

Usage (hands plugged in here need dynamixel_sdk: ~/LEAP_Development/venv):
    ./leap_teleop_dual.py --list            # the LEAP Hands (FTDI adapters) plugged in here
    ./leap_teleop_dual.py --left-serial /dev/serial/by-id/usb-FTDI_..._A \\
                          --right-serial /dev/serial/by-id/usb-FTDI_..._B
    ./leap_teleop_dual.py --left-send 10.0.0.5:9872 --right-send 10.0.0.5:9874
    ./leap_teleop_dual.py --hands right --right-serial /dev/ttyUSB0

To tell the adapters apart, plug in one hand at a time and run --list, or run
leap_hand_node.py --check --serial <port> and watch which hand moves.
"""

import argparse
import glob
import os
import socket
import sys
import time

import numpy as np

from leap_teleop import HANDS, N_JOINTS, GloveStream, Mapping, TargetSender, default_config


class HandChannel:
    """One glove -> one LEAP Hand: its mapping, smoothing and outputs, as in
    leap_teleop.py's main loop."""

    def __init__(self, hand, config, leap=None, sender=None):
        self.hand = hand
        self.mapping = Mapping(config)
        self.leap = leap
        self.sender = sender
        self.human = None
        self.last_rx = 0.0
        self.count = 0
        self.target = np.zeros(N_JOINTS)
        self.smooth = np.zeros(N_JOINTS)
        self.started = False

    def step(self, now, dt, timeout):
        """Map the newest frame and move the hand one tick. True while the glove is live."""
        live = self.human is not None and now - self.last_rx <= timeout
        if live:
            self.target = self.mapping.target(self.human, self.target)
            if not self.started:
                self.smooth[:] = self.target   # start at the hand's pose, not from flat
                self.started = True
        alpha = 1.0 - self.mapping.smoothing ** (dt * 60.0)
        self.smooth += alpha * (self.target - self.smooth)
        # The hand only moves once its glove has been seen; afterwards a stale
        # glove just repeats the held pose.
        if self.started:
            if self.leap is not None:
                self.leap.send(self.smooth, self.mapping.max_speed * dt)
            if self.sender is not None:
                self.sender.send(self.smooth)
        return live

    def status(self, live):
        state = "live" if live else ("STALE -- holding" if self.human is not None else "no data yet")
        return f"{self.hand} {state:<16} frames {self.count:<7}"


class DualGloveStream:
    """Non-blocking UDP receiver for both gloves: each frame goes to the channel
    of its hand letter, which keeps only the newest."""

    def __init__(self, port, channels):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.channels = channels      # {"L": HandChannel, "R": HandChannel}
        self.bad = 0
        self.unused = 0

    def poll(self):
        now = time.monotonic()
        while True:
            try:
                data = self.sock.recv(4096)
            except BlockingIOError:
                return
            frame = GloveStream.parse(data.decode("ascii", "replace"))
            if frame is None:
                self.bad += 1
                continue
            channel = self.channels.get(frame[0])
            if channel is None:
                self.unused += 1      # a glove whose hand is not teleoperated (--hands)
                continue
            channel.human, channel.last_rx = frame[1], now
            channel.count += 1


def ftdi_ports():
    return sorted(glob.glob("/dev/serial/by-id/*FTDI*"))


def parse_flip(text, hand):
    try:
        flip = [int(x) for x in text.split(",") if x.strip()]
    except ValueError:
        sys.exit(f"--{hand}-flip wants motor IDs, e.g. --{hand}-flip 0,4,8")
    if any(not 0 <= i < N_JOINTS for i in flip):
        sys.exit(f"--{hand}-flip IDs are 0-15")
    return flip


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hands", choices=["both", *HANDS], default="both",
                    help="which hands to teleoperate (default both)")
    ap.add_argument("--port", type=int, default=9871, help="UDP port of the glove stream")
    ap.add_argument("--rate", type=float, default=60.0, help="control loop rate [Hz]")
    ap.add_argument("--timeout", type=float, default=0.5,
                    help="glove silence [s] after which its hand holds its pose")
    ap.add_argument("--list", action="store_true",
                    help="list the LEAP Hands (FTDI adapters) plugged in here, then exit")
    for hand in HANDS:
        g = ap.add_argument_group(f"{hand} hand")
        g.add_argument(f"--{hand}-config",
                       help=f"mapping JSON (default {os.path.basename(default_config(hand))})")
        g.add_argument(f"--{hand}-serial", metavar="PORT",
                       help="the LEAP Hand is plugged in here, on this port")
        g.add_argument(f"--{hand}-send", metavar="HOST[:PORT]",
                       help=f"send the targets to leap_hand_node.py --hand {hand} on HOST "
                            f"(port 9872)")
        g.add_argument(f"--{hand}-flip", default="",
                       help="motor IDs that move the wrong way on this hand "
                            "(see leap_hand_node.py --check)")
    hw = ap.add_argument_group("hands plugged in here")
    hw.add_argument("--baud", type=int, default=1000000)
    hw.add_argument("--leap-api", help="LEAP_Hand_API/python folder")
    hw.add_argument("--current-limit", type=float, default=550)
    hw.add_argument("--kp", type=float, default=600)
    hw.add_argument("--kd", type=float, default=200)
    args = ap.parse_args()

    if args.list:
        ports = ftdi_ports()
        print("\n".join(ports) if ports else "no FTDI adapter in /dev/serial/by-id")
        return

    plan = {}
    for hand in HANDS if args.hands == "both" else [args.hands]:
        serial = getattr(args, f"{hand}_serial")
        send = getattr(args, f"{hand}_send")
        if not serial and not send:
            ports = ftdi_ports()
            sys.exit(f"the {hand} hand has nowhere to go: --{hand}-serial PORT (plugged in here) "
                     f"or --{hand}-send HOST[:PORT] (leap_hand_node.py)\n"
                     + ("LEAP Hands plugged in here:\n  " + "\n  ".join(ports) if ports
                        else "no LEAP Hand is plugged in here"))
        config = getattr(args, f"{hand}_config") or default_config(hand)
        if not os.path.exists(config):
            sys.exit(f"no mapping at {config} -- run ./leap_calibrate.py --hand {hand} first")
        plan[hand] = (config, serial, send, parse_flip(getattr(args, f"{hand}_flip"), hand))

    serials = [os.path.realpath(p[1]) for p in plan.values() if p[1]]
    if len(set(serials)) != len(serials):
        sys.exit("--left-serial and --right-serial are the same port")
    if serials:
        try:
            import dynamixel_sdk  # noqa: F401 -- LEAP_Hand_API only imports it when a port opens
        except ImportError:
            sys.exit("dynamixel_sdk is missing: run with ~/LEAP_Development/venv/bin/python, "
                     "or pip install dynamixel_sdk")

    # Listen before any torque comes on, so a busy port fails with the hands untouched.
    channels = {}
    try:
        stream = DualGloveStream(args.port, channels)
    except OSError as e:
        sys.exit(f"cannot listen on UDP {args.port}: {e} -- is leap_teleop.py still running?")

    from leap_hand_node import RealLeap, open_and_release, stop_on_sigterm
    stop_on_sigterm()
    try:
        for hand, (config, serial, send, flip) in plan.items():
            leap = None
            if serial:
                print(f"{hand} hand:")
                leap = RealLeap(args.leap_api, serial, args.baud, args.current_limit,
                                args.kp, args.kd, flip)
            sender = TargetSender(send, hand) if send else None
            channels[HANDS[hand]] = HandChannel(hand, config, leap, sender)
            outs = ([serial] if serial else []) + \
                ([f"leap_hand_node.py on {sender.dest[0]}:{sender.dest[1]}"] if sender else [])
            print(f"{hand}: mapping {os.path.basename(config)} -> {' + '.join(outs)}"
                  + (f", flipped {flip}" if flip else ""))

        period = 1.0 / args.rate
        last = time.monotonic()
        last_print = 0.0
        print(f"waiting for the glove stream on UDP {args.port}... Ctrl-C to stop")
        while True:
            now = time.monotonic()
            dt = now - last
            last = now

            stream.poll()
            states = [ch.status(ch.step(now, dt, args.timeout)) for ch in channels.values()]

            if now - last_print >= 1.0:
                last_print = now
                extra = f"   other glove ignored {stream.unused}" if stream.unused else ""
                extra += f"   bad frames {stream.bad}" if stream.bad else ""
                print("\r  " + " | ".join(states) + extra, end="", flush=True)

            time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        print()
        for ch in channels.values():
            if ch.leap is not None:
                print(f"{ch.hand} hand: ", end="")
                open_and_release(ch.leap)


if __name__ == "__main__":
    main()
