#!/usr/bin/env python3
"""
Drive a real LEAP Hand from joint targets sent over the network by leap_teleop.py.

    glove PC                                          LEAP PC (this script)
    leap_teleop.py --send <LEAP PC ip>  --UDP 9872-->  leap_hand_node.py --USB--> LEAP Hand

Each datagram is one frame of targets, plain text:
    LEAP,<L|R>,<seq>,<16 joint angles>
in radians, sim convention (0 = flat hand, LEAP_Hand_API's "LEAPsim"), in motor
ID order: index 0-3, middle 4-7, ring 8-11, thumb 12-15, each finger
[MCP side, MCP forward, PIP, DIP]. Frames for the other hand (--hand) are
dropped, so a left-hand mapping can never drive a right hand.

On this side:
  * targets are clipped to LEAP_Hand_API's joint limits, then converted to the
    motors' convention (+pi), with --flip joints mirrored first;
  * each motor moves at most --max-speed, so a jump in the stream (or the first
    frame) never snaps the hand;
  * if the stream stops for --timeout the hand holds where it is;
  * on Ctrl-C it opens the hand slowly, then turns the torque off.

Setup on the LEAP PC (only this file is needed from the glove side):
    pip install numpy dynamixel_sdk
    git clone https://github.com/leap-hand/LEAP_Hand_API ~/LEAP_Hand_API
    sudo usermod -aG dialout $USER                     # serial access, then log in again

Usage:
    ./leap_hand_node.py --hand right --check     # first time: check each joint's direction
    ./leap_hand_node.py --hand right             # listen for leap_teleop.py on UDP 9872
    ./leap_hand_node.py --hand right --allow 192.168.1.20 --flip 0,4,8
"""

import argparse
import glob
import os
import signal
import socket
import sys
import time

import numpy as np

N_JOINTS = 16
NAMES = ["if_rot", "if_mcp", "if_pip", "if_dip", "mf_rot", "mf_mcp", "mf_pip", "mf_dip",
         "rf_rot", "rf_mcp", "rf_pip", "rf_dip", "th_cmc", "th_axl", "th_mcp", "th_ipl"]
SIDE = [0, 4, 8]

HANDS = {"left": "L", "right": "R"}
FINGER = {"if": "index", "mf": "middle", "rf": "ring"}


def hints(hand):
    """What a positive angle does in the MuJoCo hand leap_teleop.py shows. The
    real hand should match; --check walks through these. Menagerie's right hand
    mirrors the left, so its side-to-side joints go the other way."""
    side = "THUMB" if hand == "left" else "LITTLE-FINGER"
    out = {}
    for p, f in FINGER.items():
        out[f"{p}_rot"] = f"{f} swings sideways toward the {side} side"
        out[f"{p}_mcp"] = f"{f} bends forward at the knuckle (closes)"
        out[f"{p}_pip"] = f"{f} middle joint bends (closes)"
        out[f"{p}_dip"] = f"{f} tip joint bends (closes)"
    out["th_cmc"] = "thumb lifts out of the palm plane, toward the palm side"
    out["th_axl"] = "thumb twists about its own length (compare with the sim)"
    out["th_mcp"] = "thumb bends toward the fingers"
    out["th_ipl"] = "thumb tip bends"
    return out

API_SEARCH = ["~/LEAP_Hand_API/python", "~/LEAP_Development/LEAP_Hand_API/python"]


def import_leap_api(path=None):
    """LEAP_Hand_API's DynamixelClient and leap_hand_utils, from --leap-api,
    next to this script, or the usual clone locations."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [path] if path else [here] + [os.path.expanduser(p) for p in API_SEARCH]
    for candidate in candidates:
        if candidate and os.path.isdir(os.path.join(candidate, "leap_hand_utils")):
            sys.path.insert(0, candidate)
            break
    try:
        from leap_hand_utils.dynamixel_client import DynamixelClient
        import leap_hand_utils.leap_hand_utils as lhu
    except ImportError as e:
        sys.exit(f"cannot import LEAP_Hand_API ({e}).\n"
                 f"  pip install dynamixel_sdk numpy, and pass --leap-api <LEAP_Hand_API/python>")
    return DynamixelClient, lhu


# ---------------------------------------------------------------------- hand
class RealLeap:
    """The physical hand, set up like LEAP_Hand_API's LeapNode (position-current
    control, its PID gains, a current cap), with an explicit or searched port.

    Angles in and out are sim convention; `flip` lists motor IDs whose
    direction is mirrored on this hand.
    """

    def __init__(self, api_path=None, port=None, baud=1000000, current_limit=550,
                 kp=600, kd=200, flip=()):
        DynamixelClient, self.lhu = import_leap_api(api_path)
        self.motors = list(range(N_JOINTS))
        self.sign = np.ones(N_JOINTS)
        self.sign[list(flip)] = -1.0
        self.sim_min, self.sim_max = self.lhu.LEAPsim_limits()

        self.client = None
        for candidate in [port] if port else self.find_ports():
            try:
                client = DynamixelClient(self.motors, candidate, baud)
                client.connect()
            except Exception as e:  # the API raises plain OSError/RuntimeError
                print(f"  {candidate}: {e}")
                continue
            self.client = client
            print(f"LEAP Hand on {candidate} at {baud} baud")
            break
        if self.client is None:
            sys.exit("could not open the LEAP Hand -- check the cable, --serial and --baud "
                     "(LEAP_Hand_API uses 1000000), and that you are in the dialout group")

        ones = np.ones(N_JOINTS)
        c = self.client
        c.sync_write(self.motors, ones * 5, 11, 1)                 # current-based position
        # Hold where the hand is: set the goal to the present position before
        # the torque comes on, so enabling it cannot make the hand jump.
        self.command = np.array(c.read_pos(), dtype=float)
        c.write_desired_pos(self.motors, self.command)
        c.set_torque_enabled(self.motors, True)
        c.sync_write(self.motors, ones * kp, 84, 2)                # P gain
        c.sync_write(SIDE, np.ones(3) * kp * 0.75, 84, 2)          # softer side-to-side
        c.sync_write(self.motors, ones * 0, 82, 2)                 # I gain
        c.sync_write(self.motors, ones * kd, 80, 2)                # D gain
        c.sync_write(SIDE, np.ones(3) * kd * 0.75, 80, 2)
        c.sync_write(self.motors, ones * current_limit, 102, 2)    # current cap [mA]

    @staticmethod
    def find_ports():
        by_id = sorted(glob.glob("/dev/serial/by-id/*FTDI*"))
        if len(by_id) == 1:
            return by_id
        return by_id + [p for p in ("/dev/ttyUSB0", "/dev/ttyUSB1") if os.path.exists(p)]

    def to_real(self, sim):
        sim = np.clip(np.asarray(sim, float), self.sim_min, self.sim_max)
        return self.lhu.LEAPsim_to_LEAPhand(self.sign * sim)

    def to_sim(self, real):
        return self.sign * self.lhu.LEAPhand_to_LEAPsim(np.asarray(real, float))

    def send(self, sim_target, max_step):
        """Step toward sim_target, at most max_step [rad] per motor."""
        goal = self.to_real(sim_target)
        self.command += np.clip(goal - self.command, -max_step, max_step)
        self.client.write_desired_pos(self.motors, self.command)
        return float(np.abs(goal - self.command).max())

    def move_to(self, sim_target, speed, rate=60.0):
        """Blocking move at `speed` [rad/s]."""
        while self.send(sim_target, speed / rate) > 1e-3:
            time.sleep(1.0 / rate)

    def read_current(self):
        return np.asarray(self.client.read_cur(), dtype=float)

    def close(self):
        # DynamixelClient.disconnect() turns the torque off: the fingers go limp.
        self.client.disconnect()


# ------------------------------------------------------------------- network
class TargetReceiver:
    def __init__(self, port, hand, allow=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.hand = HANDS[hand]
        self.wrong_hand = 0
        self.allow = allow
        self.target = None
        self.seq = -1
        self.last_rx = 0.0
        self.count = 0
        self.ignored = 0
        self.sender = None

    def poll(self):
        while True:
            try:
                data, (host, _) = self.sock.recvfrom(2048)
            except BlockingIOError:
                return
            if self.allow and host != self.allow:
                self.ignored += 1
                continue
            f = data.decode("ascii", "replace").strip().split(",")
            if len(f) != 3 + N_JOINTS or f[0] != "LEAP":
                self.ignored += 1
                continue
            if f[1] != self.hand:
                if self.wrong_hand == 0:
                    print(f"\n{host} is sending targets for the "
                          f"{'left' if f[1] == 'L' else 'right'} hand -- ignored "
                          f"(leap_teleop.py --hand, or --hand here)")
                self.wrong_hand += 1
                self.ignored += 1
                continue
            try:
                seq = int(f[2])
                target = np.array([float(x) for x in f[3:]])
            except ValueError:
                self.ignored += 1
                continue
            now = time.monotonic()
            # Drop late, reordered packets; a restarted sender begins at 0 again.
            if seq <= self.seq and now - self.last_rx < 1.0:
                continue
            if not np.all(np.isfinite(target)):
                self.ignored += 1
                continue
            self.seq, self.target, self.last_rx, self.sender = seq, target, now, host
            self.count += 1


# ---------------------------------------------------------------------- main
def stop_on_sigterm():
    """Treat SIGTERM (a closed terminal, kill) like Ctrl-C, so the exit path runs."""
    def handler(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, handler)


def open_and_release(hand):
    """Open the hand slowly, then turn the torque off."""
    print("opening the hand...")
    try:
        hand.move_to(np.zeros(N_JOINTS), speed=1.5)
        time.sleep(0.3)
    except Exception as e:
        print(f"could not open the hand: {e}")
    hand.close()
    print("torque off")


def run_check(hand, speed, which):
    """Each joint +0.4 rad and back, so its direction can be compared with the sim."""
    expected = hints(which)
    print("\nDirection check. Each joint moves +0.4 rad from the flat hand and back.\n"
          "Note the IDs that move the OTHER way and pass them as --flip.\n")
    home = np.zeros(N_JOINTS)
    hand.move_to(home, speed)
    for i, name in enumerate(NAMES):
        input(f"  ID {i:2d} {name:<7}  expected: {expected[name]}.  Enter to move > ")
        pose = home.copy()
        pose[i] = 0.4
        hand.move_to(pose, speed)
        time.sleep(0.8)
        hand.move_to(home, speed)
    print("\ndone.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hand", choices=HANDS, default="left",
                    help="which LEAP hand this is (default left); frames for the other "
                         "hand are ignored")
    ap.add_argument("--port", type=int, default=9872, help="UDP port to listen on")
    ap.add_argument("--allow", metavar="IP", help="only accept targets from this address")
    ap.add_argument("--rate", type=float, default=60.0, help="motor command rate [Hz]")
    ap.add_argument("--max-speed", type=float, default=6.0,
                    help="fastest any joint may move [rad/s] (default 6)")
    ap.add_argument("--timeout", type=float, default=0.5,
                    help="stream silence [s] after which the hand holds still")
    ap.add_argument("--on-exit", choices=["open", "limp"], default="open",
                    help="open the hand slowly before turning the torque off (default), "
                         "or just turn it off")
    ap.add_argument("--check", action="store_true",
                    help="move one joint at a time to check directions, then exit")
    hw = ap.add_argument_group("hardware")
    hw.add_argument("--serial", help="port (default: the FTDI in /dev/serial/by-id, "
                                     "then /dev/ttyUSB0, /dev/ttyUSB1)")
    hw.add_argument("--baud", type=int, default=1000000)
    hw.add_argument("--leap-api", help="LEAP_Hand_API/python folder "
                                       "(default: next to this file, ~/LEAP_Hand_API/python)")
    hw.add_argument("--current-limit", type=float, default=550,
                    help="mA; LEAP recommends 350 for the Lite, 550 for the full motors")
    hw.add_argument("--kp", type=float, default=600)
    hw.add_argument("--kd", type=float, default=200)
    hw.add_argument("--flip", default="",
                    help="comma-separated motor IDs that move the wrong way (see --check)")
    args = ap.parse_args()

    try:
        flip = [int(x) for x in args.flip.split(",") if x.strip()]
    except ValueError:
        sys.exit("--flip wants motor IDs, e.g. --flip 0,4,8")
    if any(not 0 <= i < N_JOINTS for i in flip):
        sys.exit("--flip IDs are 0-15")

    stop_on_sigterm()
    hand = RealLeap(args.leap_api, args.serial, args.baud, args.current_limit,
                    args.kp, args.kd, flip)
    if flip:
        print("flipped: " + ", ".join(f"{i} {NAMES[i]}" for i in flip))
    try:
        if args.check:
            run_check(hand, speed=1.0, which=args.hand)
            return
        rx = TargetReceiver(args.port, args.hand, args.allow)
        ips = os.popen("hostname -I 2>/dev/null").read().split()
        print(f"listening on UDP {args.port}  (this PC: {', '.join(ips) or 'see ip addr'})\n"
              f"on the glove PC:  ./leap_teleop.py --hand {args.hand} "
              f"--send {ips[0] if ips else '<this ip>'}")
        period = 1.0 / args.rate
        last_status = 0.0
        frames_at_status = 0
        was_live = False
        while True:
            now = time.monotonic()
            rx.poll()
            live = rx.target is not None and now - rx.last_rx <= args.timeout
            if live:
                hand.send(rx.target, args.max_speed * period)
            elif was_live:
                print(f"\nstream silent for {args.timeout}s -- holding")
            was_live = live

            if now - last_status >= 1.0:
                rate = (rx.count - frames_at_status) / max(now - last_status, 1e-6)
                frames_at_status, last_status = rx.count, now
                current = np.abs(hand.read_current())
                state = f"live from {rx.sender}" if live else \
                    ("holding" if rx.target is not None else "waiting for leap_teleop.py")
                print(f"\r  {state:<32} {rate:5.1f} frames/s   "
                      f"max current {current.max():5.0f} mA ({NAMES[int(current.argmax())]})"
                      f"   ignored {rx.ignored}   ", end="", flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if args.on_exit == "open":
            open_and_release(hand)
        else:
            hand.close()
            print("torque off")


if __name__ == "__main__":
    main()
