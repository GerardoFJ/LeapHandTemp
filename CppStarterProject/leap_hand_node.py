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

Effort feedback (for the glove's finger brakes, haptx_hand_stream.exe --feedback):
the hand has no torque sensors, but in current-based position control a
finger that is held short of its target by an object draws more current, up
to --current-limit. Per finger, the effort is the largest of its closing
joints' (current - --free-current) / (--current-limit - --free-current),
counted only while the joint is more than --min-lag behind its command and
nearly still (so the current of plain motion does not count). It goes back to
whoever sends the targets:
    LFB,<L|R>,<seq>,<thumb>,<index>,<middle>,<ring>      0..1 each, UDP 9873
--log writes everything behind it to a CSV, and --probe closes the hand on an
object by itself, to measure what grasping looks like without the glove.

Setup on the LEAP PC (only this file is needed from the glove side):
    pip install numpy dynamixel_sdk
    git clone https://github.com/leap-hand/LEAP_Hand_API ~/LEAP_Hand_API
    sudo usermod -aG dialout $USER                     # serial access, then log in again

Usage:
    ./leap_hand_node.py --hand right --check     # first time: check each joint's direction
    ./leap_hand_node.py --hand right             # listen for leap_teleop.py on UDP 9872
    ./leap_hand_node.py --hand right --allow 192.168.1.20 --flip 0,4,8
    ./leap_hand_node.py --hand right --probe --log grasp.csv   # measure grasps, no glove
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

# The joints that close each LEAP finger on an object, in motor IDs: MCP
# forward, PIP, DIP; for the thumb th_cmc (into the palm), th_mcp, th_ipl.
# Side-to-side joints and the thumb's twist don't count.
LEAP_FINGERS = ["thumb", "index", "middle", "ring"]
CLOSING = {"thumb": [12, 14, 15], "index": [1, 2, 3], "middle": [5, 6, 7], "ring": [9, 10, 11]}

# --probe closes on this (sim convention): a full fist, thumb across.
GRASP = np.zeros(16)
GRASP[[1, 5, 9]], GRASP[[2, 6, 10]], GRASP[[3, 7, 11]] = 1.5, 1.6, 1.1
GRASP[12], GRASP[14], GRASP[15] = 1.4, 0.9, 0.6


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

    def read_state(self):
        """One bus read: position [rad], velocity [rad/s] and the current
        pushing each joint toward a larger sim angle (= closing) [mA]."""
        pos, vel, cur = self.client.read_pos_vel_cur()
        return (self.to_sim(pos), self.sign * np.asarray(vel, float),
                self.sign * np.asarray(cur, float))

    def command_sim(self):
        return self.to_sim(self.command)

    def close(self):
        # DynamixelClient.disconnect() turns the torque off: the fingers go limp.
        self.client.disconnect()


# -------------------------------------------------------------------- effort
class EffortEstimator:
    """How hard each LEAP finger pushes against something, 0..1."""

    def __init__(self, current_limit, free_ma=120.0, min_lag=0.08, still=1.0):
        self.limit, self.free, self.min_lag, self.still = current_limit, free_ma, min_lag, still
        self.history = []              # last 3 raw readings, for a median against spikes
        self.joint = np.zeros(N_JOINTS)
        self.effort = np.zeros(len(LEAP_FINGERS))

    def update(self, command, pos, vel, cur):
        lag = command - pos            # > 0: held back from closing
        active = (lag > self.min_lag) & (np.abs(vel) < self.still)
        raw = np.clip((cur - self.free) / max(self.limit - self.free, 1.0), 0.0, 1.0)
        self.joint = np.where(active, raw, 0.0)
        reading = np.array([self.joint[CLOSING[f]].max() for f in LEAP_FINGERS])
        self.history = (self.history + [reading])[-3:]
        self.effort = np.median(np.array(self.history), axis=0)
        return self.effort


class FeedbackSender:
    def __init__(self, port, hand):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port, self.hand, self.seq = port, HANDS[hand], 0

    def send(self, host, effort):
        msg = f"LFB,{self.hand},{self.seq}," + ",".join(f"{e:.3f}" for e in effort)
        try:
            self.sock.sendto(msg.encode("ascii"), (host, self.port))
        except OSError:
            pass
        self.seq += 1


class StateLog:
    """CSV of everything behind the effort, one row per reading."""

    def __init__(self, path):
        self.fh = open(path, "w")
        cols = ["t", "phase"]
        for kind in ("target", "command", "pos", "vel", "cur"):
            cols += [f"{kind}_{n}" for n in NAMES]
        cols += [f"effort_{f}" for f in LEAP_FINGERS]
        self.fh.write(",".join(cols) + "\n")
        self.t0 = time.monotonic()
        print(f"logging to {path}")

    def write(self, phase, target, command, pos, vel, cur, effort):
        target = np.full(N_JOINTS, np.nan) if target is None else target
        values = np.concatenate([target, command, pos, vel, cur, effort])
        self.fh.write(f"{time.monotonic() - self.t0:.4f},{phase},"
                      + ",".join(f"{v:.4f}" for v in values) + "\n")

    def close(self):
        self.fh.close()


def effort_text(effort):
    return " ".join(f"{f[0].upper()} {e:.2f}" for f, e in zip(LEAP_FINGERS, effort))


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


def run_probe(hand, estimator, log, rate, speed=0.8):
    """Close the hand on whatever is in it, hold, open; report per finger."""
    print("\nProbe: put an object where the fingers close (or nothing, for the free\n"
          "baseline). The hand closes slowly into a fist, holds 2 s and opens.\n")
    period = 1.0 / rate
    n = 0
    while True:
        answer = input("  Enter = close   q = quit > ").strip().lower()
        if answer == "q":
            return
        n += 1
        peak_effort = np.zeros(len(LEAP_FINGERS))
        peak_cur = np.zeros(len(LEAP_FINGERS))
        for phase, target, secs in ((f"close{n}", GRASP, 3.5), (f"hold{n}", GRASP, 2.0),
                                    (f"open{n}", np.zeros(N_JOINTS), 3.0)):
            end = time.monotonic() + secs
            while time.monotonic() < end:
                t = time.monotonic()
                hand.send(target, speed * period)
                pos, vel, cur = hand.read_state()
                effort = estimator.update(hand.command_sim(), pos, vel, cur)
                if log:
                    log.write(phase, target, hand.command_sim(), pos, vel, cur, effort)
                if not phase.startswith("open"):
                    peak_effort = np.maximum(peak_effort, effort)
                    peak_cur = np.maximum(peak_cur, [cur[CLOSING[f]].max() for f in LEAP_FINGERS])
                print(f"\r    {phase:<7} effort {effort_text(effort)}   ", end="", flush=True)
                time.sleep(max(0.0, period - (time.monotonic() - t)))
        print("\n    peak effort  " + effort_text(peak_effort))
        print("    peak current " + " ".join(f"{f[0].upper()} {c:4.0f}"
                                            for f, c in zip(LEAP_FINGERS, peak_cur)) + " mA")


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
    fb = ap.add_argument_group("effort feedback")
    fb.add_argument("--feedback-port", type=int, default=9873,
                    help="UDP port on the target sender's PC (haptx_hand_stream.exe --feedback)")
    fb.add_argument("--no-feedback", action="store_true", help="don't send the effort back")
    fb.add_argument("--feedback-rate", type=float, default=30.0,
                    help="motor reads per second for the effort [Hz] (default 30; LEAP "
                         "suggests staying under 90 bus queries/s)")
    fb.add_argument("--free-current", type=float, default=120.0,
                    help="mA a joint may draw without counting as pushing (default 120)")
    fb.add_argument("--min-lag", type=float, default=0.08,
                    help="rad a joint must be behind its command to count (default 0.08)")
    fb.add_argument("--still", type=float, default=1.0,
                    help="rad/s above which a joint counts as moving, not pushing (default 1)")
    fb.add_argument("--log", metavar="CSV", help="log targets, positions, currents and effort")
    fb.add_argument("--probe", action="store_true",
                    help="close the hand on an object by itself and report the effort, exit")
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
    estimator = EffortEstimator(args.current_limit, args.free_current, args.min_lag, args.still)
    log = StateLog(args.log) if args.log else None
    try:
        if args.check:
            run_check(hand, speed=1.0, which=args.hand)
            return
        if args.probe:
            run_probe(hand, estimator, log, args.feedback_rate)
            return
        rx = TargetReceiver(args.port, args.hand, args.allow)
        feedback = None if args.no_feedback else FeedbackSender(args.feedback_port, args.hand)
        ips = os.popen("hostname -I 2>/dev/null").read().split()
        print(f"listening on UDP {args.port}  (this PC: {', '.join(ips) or 'see ip addr'})\n"
              f"on the glove PC:  ./leap_teleop.py --hand {args.hand} "
              f"--send {ips[0] if ips else '<this ip>'}")
        period = 1.0 / args.rate
        read_period = 1.0 / args.feedback_rate
        next_read = 0.0
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

            if now >= next_read:
                next_read = now + read_period
                pos, vel, cur = hand.read_state()
                effort = estimator.update(hand.command_sim(), pos, vel, cur)
                if feedback is not None and rx.sender is not None:
                    feedback.send(rx.sender, effort)
                if log:
                    log.write("live" if live else "hold", rx.target, hand.command_sim(),
                              pos, vel, cur, effort)

            if now - last_status >= 1.0:
                rate = (rx.count - frames_at_status) / max(now - last_status, 1e-6)
                frames_at_status, last_status = rx.count, now
                state = f"live from {rx.sender}" if live else \
                    ("holding" if rx.target is not None else "waiting for leap_teleop.py")
                print(f"\r  {state:<30} {rate:5.1f} frames/s   effort {effort_text(estimator.effort)}"
                      f"   ignored {rx.ignored}   ", end="", flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if log:
            log.close()
        if args.on_exit == "open":
            open_and_release(hand)
        else:
            hand.close()
            print("torque off")


if __name__ == "__main__":
    main()
