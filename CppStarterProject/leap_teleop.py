#!/usr/bin/env python3
"""
Teleoperate a LEAP Hand from the HaptX glove: MuJoCo (unless --no-viewer), plus
the real hand on another PC (--send) or plugged into this one (--real).

    glove -> haptx_hand_stream.exe (Wine) --UDP 9871--> leap_teleop.py -> MuJoCo viewer
                                     --send: --UDP 9872--> leap_hand_node.py -> LEAP Hand
                                     --real: ---------------------------------> LEAP Hand

haptx_hand_stream.exe sends one datagram per frame (see its header):
    H2,<L|R>,<seq>,<t_ms>, then per finger (thumb, index, middle, ring, pinky):
    <ang_ok>,<mcp>,<pip>,<dip>,<add>,<ori_ok>,<3 joint quaternions w,x,y,z>

leap_mapping.json turns that into the 16 LEAP joints, one gain and offset per
joint (the thumb joints mix several inputs). Inputs are either the solver's
angles ("finger"/"channel") or, after leap_calibrate.py, "features": the
signed rotation of one glove joint about an axis found during calibration,
measured from your own open hand. LEAP has three fingers plus the thumb: its
index and middle follow yours, its ring finger follows your pinky.

Joint conventions (checked against LEAP_Hand_Sim's robot.urdf):
  * LEAP motor IDs: index 0-3, middle 4-7, ring 8-11, thumb 12-15; per finger
    [MCP side, MCP forward, PIP, DIP]. Menagerie lists each finger's actuators
    as [mcp, rot, pip, dip] -- the first two are swapped, the JSON maps by name.
  * Sim angle 0 is the flat hand; the real hand is sim + pi.

Setup (host):
    pip install mujoco dynamixel_sdk numpy
    git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie ~/LEAP_Development/mujoco_menagerie

Usage:
    ./leap_teleop.py                    # sim only, waits for the glove stream
    ./leap_teleop.py --hand right       # right glove + right LEAP hand (default: left)
    ./leap_teleop.py --fake             # sim only, synthetic open/close, no glove
    ./leap_teleop.py --monitor          # also print human angles -> LEAP targets
    ./leap_calibrate.py                 # fit leap_mapping.json to your hand (do this first)
    ./leap_teleop.py --send 192.168.1.50   # + the hand on the PC running leap_hand_node.py
    ./leap_teleop.py --real             # + a hand plugged in here (leap_hand_node.py's code)
    ./leap_teleop.py --real --no-viewer # the real hand only, no MuJoCo (mujoco not needed)
"""

import argparse
import json
import math
import os
import socket
import sys
import time

import numpy as np

HOME = os.path.expanduser("~")
HERE = os.path.dirname(os.path.abspath(__file__))
MENAGERIE = os.path.join(HOME, "LEAP_Development/mujoco_menagerie/leap_hand")
HANDS = {"left": "L", "right": "R"}

# LEAP motor IDs 0-15, as Menagerie names the joints.
LEAP_JOINTS = ["if_rot", "if_mcp", "if_pip", "if_dip", "mf_rot", "mf_mcp", "mf_pip", "mf_dip",
               "rf_rot", "rf_mcp", "rf_pip", "rf_dip", "th_cmc", "th_axl", "th_mcp", "th_ipl"]


def default_model(hand):
    return os.path.join(MENAGERIE, f"scene_{hand}.xml")


def default_config(hand):
    """The left hand keeps the original file names; the right hand gets its own."""
    return os.path.join(HERE, "leap_mapping.json" if hand == "left" else "leap_mapping_right.json")


def default_raw(hand):
    return os.path.join(HERE, "leap_calib_raw.json" if hand == "left"
                        else "leap_calib_raw_right.json")


FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
CHANNELS = ["mcp", "pip", "dip", "add"]
N_JOINTS = 16


# ---------------------------------------------------------------- rotations
def quat_mul(a, b):
    """Hamilton product of [..., 4] arrays in w, x, y, z order."""
    aw, ax, ay, az = np.moveaxis(a, -1, 0)
    bw, bx, by, bz = np.moveaxis(b, -1, 0)
    return np.stack([aw * bw - ax * bx - ay * by - az * bz,
                     aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw], -1)


def quat_conj(q):
    return q * np.array([1.0, -1.0, -1.0, -1.0])


def rotvec(q):
    """Rotation vector (axis * angle [rad]) of [..., 4] unit quaternions."""
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    q = np.where(q[..., :1] < 0, -q, q)          # q and -q are the same rotation
    v = q[..., 1:]
    s = np.linalg.norm(v, axis=-1, keepdims=True)
    angle = 2.0 * np.arctan2(s, q[..., :1])
    return np.where(s > 1e-9, v / np.maximum(s, 1e-12) * angle, 2.0 * v)


def quat_axis_angle(axis, angle):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    return np.r_[math.cos(angle / 2.0), math.sin(angle / 2.0) * axis]


def relative_rotvecs(q, rest):
    """Per joint, the rotation from the rest (open hand) orientation, in the
    joint's own frame: rotvec(rest^-1 * q). q, rest: [3, 4]."""
    return rotvec(quat_mul(quat_conj(rest), q))


# --------------------------------------------------------------------- input
class GloveStream:
    """Non-blocking UDP receiver that keeps the newest frame.

    human = {finger: {"angles": {mcp, pip, dip, add} or None,
                      "q": [3, 4] joint quaternions or None}}
    """

    def __init__(self, port, hand=None):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.want = HANDS.get(hand)    # "L"/"R": frames from the other glove are dropped
        self.human = None
        self.hand = None
        self.last_rx = 0.0
        self.count = 0
        self.wrong_hand = 0
        self.warned_old = False

    def poll(self):
        got = False
        while True:
            try:
                data = self.sock.recv(4096)
            except BlockingIOError:
                break
            text = data.decode("ascii", "replace")
            if text.startswith("H,") and not self.warned_old:
                print("\nthe glove stream is the old 'H' format -- rebuild haptx_hand_stream.cpp")
                self.warned_old = True
            frame = self.parse(text)
            if frame is not None and self.want and frame[0] != self.want:
                if self.wrong_hand == 0:
                    print(f"\nthe glove stream is the {'left' if frame[0] == 'L' else 'right'} "
                          f"glove but this is set up for the other hand -- ignoring it "
                          f"(haptx_hand_stream.exe --hand, or --hand here)")
                self.wrong_hand += 1
                continue
            if frame is not None:
                self.hand, self.human = frame
                self.last_rx = time.monotonic()
                self.count += 1
                got = True
        return got

    PER_FINGER = 6 + 12

    @classmethod
    def parse(cls, text):
        f = text.strip().split(",")
        if len(f) != 4 + cls.PER_FINGER * len(FINGERS) or f[0] != "H2":
            return None
        human = {}
        try:
            for i, finger in enumerate(FINGERS):
                b = 4 + cls.PER_FINGER * i
                angles = {ch: float(f[b + 1 + k]) for k, ch in enumerate(CHANNELS)} \
                    if int(f[b]) else None
                q = np.array([float(x) for x in f[b + 6:b + 18]]).reshape(3, 4) \
                    if int(f[b + 5]) else None
                if angles is not None or q is not None:
                    human[finger] = {"angles": angles, "q": q}
        except ValueError:
            return None
        return f[1], human


class FakeGlove:
    """A hand slowly opening and closing, with a little spread, for testing.
    Flexion is negative, as AlternateHandIk reports it. Joint quaternions are
    hinges about z (flexion) and y (spread at the MCP)."""

    def __init__(self):
        self.t0 = time.monotonic()
        self.hand, self.human, self.last_rx, self.count = "L", None, 0.0, 0

    def poll(self):
        t = time.monotonic() - self.t0
        close = 0.5 * (1.0 - math.cos(2.0 * math.pi * 0.25 * t))   # 0..1 every 4 s
        spread = 0.25 * math.sin(2.0 * math.pi * 0.1 * t)
        self.human = {}
        for i, finger in enumerate(FINGERS):
            if finger == "thumb":
                a = {"mcp": -0.9 * close, "pip": -0.7 * close, "dip": -0.6 * close,
                     "add": 0.8 * close}
            else:
                a = {"mcp": -1.3 * close, "pip": -1.4 * close, "dip": -1.0 * close,
                     "add": spread * (i - 2) / 2.0}
            q = np.array([quat_mul(quat_axis_angle([0, 0, 1], a["mcp"]),
                                   quat_axis_angle([0, 1, 0], a["add"])),
                          quat_axis_angle([0, 0, 1], a["pip"]),
                          quat_axis_angle([0, 0, 1], a["dip"])])
            self.human[finger] = {"angles": a, "q": q}
        self.last_rx = time.monotonic()
        self.count += 1
        return True


# ------------------------------------------------------------------- mapping
class Mapping:
    """target = offset + sum(gain * input) per LEAP joint, inputs being either

      {"finger", "channel", "gain"}  the IK's angle (needs AlternateHandIk), or
      {"feature", "gain"}            a calibrated joint-rotation feature.

    A joint is one input written inline ("finger"/"channel"/"gain" on the joint
    itself) or a list of them in "inputs".
    """

    def __init__(self, path):
        with open(path) as fh:
            cfg = json.load(fh)
        joints = sorted(cfg["joints"], key=lambda j: j["id"])
        if [j["id"] for j in joints] != list(range(N_JOINTS)):
            sys.exit(f"{path}: need exactly one entry per LEAP motor id 0-15")

        self.rest = {f: np.array(q, float) for f, q in cfg.get("rest", {}).items()}
        self.features = {}
        for name, ft in cfg.get("features", {}).items():
            if ft["finger"] not in self.rest:
                sys.exit(f"{path}: feature {name} needs a rest pose for {ft['finger']}")
            self.features[name] = (ft["finger"], int(ft["joint"]), np.array(ft["axis"], float))

        self.inputs = []
        for j in joints:
            inputs = j["inputs"] if "inputs" in j else [j]   # [] = held at offset
            parsed = []
            for inp in inputs:
                gain = float(inp.get("gain", 1.0))
                if "feature" in inp:
                    if inp["feature"] not in self.features:
                        sys.exit(f"{path}: joint {j['id']} uses unknown feature {inp['feature']}")
                    parsed.append(("feature", inp["feature"], gain))
                elif inp.get("finger") in FINGERS and inp.get("channel") in CHANNELS:
                    parsed.append(("channel", (inp["finger"], inp["channel"]), gain))
                else:
                    sys.exit(f"{path}: joint {j['id']} has an input with no feature or "
                             f"finger/channel")
            self.inputs.append(parsed)
        self.joints = joints
        self.names = [j["mjcf"] for j in joints]
        self.offset = np.array([j.get("offset", 0.0) for j in joints])
        self.smoothing = float(cfg.get("smoothing", 0.0))
        self.max_speed = float(cfg.get("max_speed_rad_s", 8.0))

    def feature_values(self, human):
        """{feature name: rad} for every feature whose finger sent orientations."""
        rv = {}
        for finger, rest in self.rest.items():
            q = (human or {}).get(finger, {}).get("q")
            if q is not None:
                rv[finger] = relative_rotvecs(q, rest)
        return {name: float(rv[finger][joint] @ axis)
                for name, (finger, joint, axis) in self.features.items() if finger in rv}

    def values(self, human):
        """Every input value available this frame, keyed as the inputs are."""
        out = {("feature", k): v for k, v in self.feature_values(human).items()}
        for finger, data in (human or {}).items():
            for ch, v in (data.get("angles") or {}).items():
                out[("channel", (finger, ch))] = v
        return out

    def target(self, human, previous):
        """Sim-convention LEAP targets. A joint whose inputs are missing this
        frame keeps its previous target instead of snapping open."""
        values = self.values(human)
        out = previous.copy()
        for i, inputs in enumerate(self.inputs):
            keys = [(kind, key) for kind, key, _ in inputs]
            if all(k in values for k in keys):
                out[i] = self.offset[i] + sum(gain * values[(kind, key)]
                                              for kind, key, gain in inputs)
        return out


# ---------------------------------------------------------------------- real
class TargetSender:
    """Sends sim-convention targets to leap_hand_node.py:
    "LEAP,<L|R>,<seq>,<16 rad>". The node drops frames meant for the other hand."""

    def __init__(self, dest, hand):
        host, _, port = dest.partition(":")
        self.dest = (socket.gethostbyname(host), int(port or 9872))
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.hand = HANDS[hand]
        self.seq = 0

    def send(self, sim_target):
        msg = f"LEAP,{self.hand},{self.seq}," + ",".join(f"{v:.4f}" for v in sim_target)
        try:
            self.sock.sendto(msg.encode("ascii"), self.dest)
        except OSError:
            pass   # network hiccup: the node holds the pose until packets return
        self.seq += 1


# ----------------------------------------------------------------------- sim
class SimLeap:
    def __init__(self, model_path):
        try:
            import mujoco
            import mujoco.viewer
        except ImportError:
            sys.exit("mujoco missing:  pip install mujoco")
        if not os.path.exists(model_path):
            sys.exit(f"no model at {model_path}\n  git clone --depth 1 "
                     f"https://github.com/google-deepmind/mujoco_menagerie ~/LEAP_Development/mujoco_menagerie")
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = None

    def bind(self, names):
        ids = []
        for name in names:
            act = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_act")
            if act < 0:
                sys.exit(f"the model has no actuator '{name}_act' -- is it the Menagerie LEAP hand?")
            ids.append(act)
        self.act = np.array(ids)
        self.low = self.model.actuator_ctrlrange[self.act, 0]
        self.high = self.model.actuator_ctrlrange[self.act, 1]

    def open_viewer(self):
        import mujoco.viewer
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

    def running(self):
        return self.viewer is None or self.viewer.is_running()

    def step(self, sim_target, wall_dt):
        self.data.ctrl[self.act] = np.clip(sim_target, self.low, self.high)
        n = max(1, int(round(wall_dt / self.model.opt.timestep)))
        for _ in range(min(n, 50)):
            self.mujoco.mj_step(self.model, self.data)
        if self.viewer is not None:
            self.viewer.sync()


class NoSim:
    """Stands in for SimLeap with --no-viewer, so mujoco is never imported. The
    real hand does not need the sim: RealLeap clips to LEAP_Hand_API's limits."""

    viewer = None

    def running(self):
        return True

    def step(self, sim_target, wall_dt):
        pass


# ---------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hand", choices=HANDS, default="left",
                    help="which glove and LEAP hand (default left)")
    ap.add_argument("--port", type=int, default=9871, help="UDP port of the glove stream")
    ap.add_argument("--config", help="joint mapping JSON (default: leap_mapping.json for the "
                                     "left hand, leap_mapping_right.json for the right)")
    ap.add_argument("--model", help="LEAP hand MJCF scene (default: Menagerie's, for --hand)")
    ap.add_argument("--fake", action="store_true", help="synthetic glove, no stream needed")
    ap.add_argument("--monitor", action="store_true", help="print human angles and LEAP targets")
    ap.add_argument("--no-viewer", action="store_true",
                    help="no MuJoCo at all (no window, mujoco not needed)")
    ap.add_argument("--rate", type=float, default=60.0, help="control loop rate [Hz]")
    ap.add_argument("--timeout", type=float, default=0.5,
                    help="glove silence [s] after which the hand holds its pose")
    real = ap.add_argument_group("real hand")
    real.add_argument("--send", metavar="HOST[:PORT]",
                      help="send the targets to leap_hand_node.py on HOST (port 9872)")
    real.add_argument("--real", action="store_true",
                      help="drive a LEAP Hand plugged into this PC (same options as "
                           "leap_hand_node.py: --serial --baud --leap-api --current-limit "
                           "--kp --kd --flip)")
    real.add_argument("--serial")
    real.add_argument("--baud", type=int, default=1000000)
    real.add_argument("--leap-api")
    real.add_argument("--current-limit", type=float, default=550)
    real.add_argument("--kp", type=float, default=600)
    real.add_argument("--kd", type=float, default=200)
    real.add_argument("--flip", default="")
    args = ap.parse_args()

    config = args.config or default_config(args.hand)
    if not os.path.exists(config):
        sys.exit(f"no mapping at {config} -- run ./leap_calibrate.py --hand {args.hand} first")
    mapping = Mapping(config)
    if args.no_viewer:
        sim = NoSim()
    else:
        sim = SimLeap(args.model or default_model(args.hand))
        sim.bind(mapping.names)
    glove = FakeGlove() if args.fake else GloveStream(args.port, args.hand)
    leap = None
    if args.real:
        from leap_hand_node import RealLeap, open_and_release, stop_on_sigterm
        stop_on_sigterm()
        flip = [int(x) for x in args.flip.split(",") if x.strip()]
        leap = RealLeap(args.leap_api, args.serial, args.baud, args.current_limit,
                        args.kp, args.kd, flip)
    sender = TargetSender(args.send, args.hand) if args.send else None

    if not args.no_viewer:
        sim.open_viewer()

    target = np.zeros(N_JOINTS)
    smooth = np.zeros(N_JOINTS)
    started = False
    period = 1.0 / args.rate
    last = time.monotonic()
    last_print = 0.0
    print(f"{args.hand} hand, mapping {os.path.basename(config)}")
    print("waiting for the glove stream on UDP %d..." % args.port if not args.fake else "fake glove")

    try:
        while sim.running():
            now = time.monotonic()
            dt = now - last
            last = now

            glove.poll()
            live = glove.human is not None and now - glove.last_rx <= args.timeout
            if live:
                target = mapping.target(glove.human, target)
                if not started:
                    smooth[:] = target   # start at the hand's pose, not from flat
                    started = True
            # Exponential smoothing; the alpha is per 60 Hz tick so it does not
            # depend on --rate.
            alpha = 1.0 - mapping.smoothing ** (dt * 60.0)
            smooth += alpha * (target - smooth)

            sim.step(smooth, dt)
            # The real hand only moves once the glove has been seen; until
            # then it stays where it is. Afterwards a stale glove just repeats
            # the held pose.
            if started:
                if leap is not None:
                    leap.send(smooth, mapping.max_speed * dt)
                if sender is not None:
                    sender.send(smooth)

            if args.monitor and now - last_print >= 0.2:
                last_print = now
                print_monitor(glove, live, mapping, smooth)
            elif not args.monitor and now - last_print >= 1.0:
                last_print = now
                state = "live" if live else ("STALE -- holding" if glove.human else "no data yet")
                sent = f"  sent {sender.seq} to {sender.dest[0]}:{sender.dest[1]}" if sender else ""
                print(f"\r  glove {state:<18} frames {glove.count:<8}{sent}", end="", flush=True)

            time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        print()
        if leap is not None:
            open_and_release(leap)
        if sim.viewer is not None:
            sim.viewer.close()


def print_monitor(glove, live, mapping, smooth):
    lines = ["\033[2J\033[H" + f"glove {'live' if live else 'STALE'}  frames {glove.count}",
             "IK angles [deg]  mcp    pip    dip    add   orientations"]
    for finger in FINGERS:
        data = (glove.human or {}).get(finger) or {}
        a = data.get("angles")
        angles = " ".join(f"{math.degrees(a[c]):6.0f}" for c in CHANNELS) if a else \
            "    --     --     --     --"
        lines.append(f"  {finger:<8}   {angles}   {'yes' if data.get('q') is not None else '--'}")
    features = mapping.feature_values(glove.human)
    if features:
        lines.append("calibrated features [deg]")
        names = sorted(features)
        for i in range(0, len(names), 4):
            lines.append("  " + "  ".join(f"{n:>15} {math.degrees(features[n]):5.0f}"
                                          for n in names[i:i + 4]))
    lines.append("LEAP target [deg]")
    for i in range(0, N_JOINTS, 4):
        lines.append("  " + "  ".join(f"{mapping.names[k]:>6} {math.degrees(smooth[k]):5.0f}"
                                      for k in range(i, i + 4)))
    print("\n".join(lines), flush=True)


if __name__ == "__main__":
    main()
