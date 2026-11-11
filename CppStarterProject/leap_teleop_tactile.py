#!/usr/bin/env python3
"""
Teleoperate one or two LEAP Hands from the HaptX gloves, and send back how hard
each LEAP finger presses on something, as a fingertip force in newtons, so the
glove can render it on its fingertip tactors and brakes.

    ik_stream_feedback.exe --H2,L + H2,R (UDP 9871)--> leap_teleop_tactile.py --USB--> LEAP Hands
    ik_stream_feedback.exe <--LFT,L + LFT,R (UDP 9873)--

The hands move exactly as with leap_teleop_dual.py (same mapping, smoothing and
speed limit). They must be plugged into this PC: the force comes from their
motor currents. One feedback datagram per reading and hand:
    LFT,<L|R>,<seq>,<thumb>,<index>,<middle>,<ring>      fingertip force [N], >= 0
It goes to the address the glove frames come from (or --feedback-host).

The force uses the servo calibration of leap-tactile-regrasp (hardware/
fit_servo.py -> results/servo_fit_right.json, LEAP palm up) and its MuJoCo hand
model. For each joint that closes a finger:
    tau = k_t (I - offset) - gravity(q) - damping v - friction sign(v)    [N.m]
counted only while the joint is held short of its command (--min-lag) and
nearly still (--still). The finger's force is the largest tau / lever over its
joints, the lever being the joint's distance to the fingertip (tip Jacobian).
In free motion it stays near 0 and it rises once you close a finger past an
object. A load that pushes a finger open shows far less (the gears hold most
of it): squeezing is felt, an object's weight is not.

Usage (~/LEAP_Development/venv has dynamixel_sdk and mujoco):
    ./leap_teleop_tactile.py --list
    ./leap_teleop_tactile.py --left-serial /dev/serial/by-id/usb-FTDI_..._A \\
                             --right-serial /dev/serial/by-id/usb-FTDI_..._B
    ./leap_teleop_tactile.py --hands right --right-serial /dev/ttyUSB0
"""

import argparse
import json
import os
import socket
import sys
import time

import numpy as np

from leap_hand_node import CLOSING, LEAP_FINGERS, NAMES
from leap_teleop import HANDS, N_JOINTS, GloveStream, default_config
from leap_teleop_dual import HandChannel, ftdi_ports, parse_flip

RL_REPO = os.path.expanduser("~/RL_Training/leap-tactile-regrasp")
DEFAULT_FIT = os.path.join(RL_REPO, "results", "servo_fit_right.json")
DEFAULT_MODEL = os.path.join(RL_REPO, "ltr", "assets", "leap_hand.xml")
WRIST_JOINTS = ("wrist_yaw", "wrist_tilt", "wrist_unyaw")    # removed: the hand is mounted
TIP_SITES = {"thumb": "th_tip", "index": "if_tip", "middle": "mf_tip", "ring": "rf_tip"}
GRAVITY_PALM_UP = (0.0, 0.0, -9.81)    # the calibration's mount (fit_servo.MOUNT["up"])
MIN_LEVER = 0.02                       # m: never divide by a vanishing lever
# Without the fit JSON: the medians of servo_fit_right.json.
FALLBACK = {"kt": 4.26e-4, "i_cap_ma": 737.0, "offset_ma": -2.0, "damping": 0.016,
            "friction": 0.0053}


def load_fit(path):
    """k_t [N.m/mA], and per joint the current offset [mA], damping [N.m.s/rad]
    and friction [N.m], from fit_servo.py's JSON."""
    if not path or not os.path.exists(path):
        print(f"no calibration at {path} -- using its median values")
        return {"kt": FALLBACK["kt"], "cap_ma": FALLBACK["i_cap_ma"],
                "offset": np.full(N_JOINTS, FALLBACK["offset_ma"]),
                "damping": np.full(N_JOINTS, FALLBACK["damping"]),
                "friction": np.full(N_JOINTS, FALLBACK["friction"])}
    with open(path) as fh:
        fit = json.load(fh)
    dyn = {d["joint"]: d for d in fit.get("dynamics", []) if "kp" in d}
    return {"kt": float(fit["kt"]), "cap_ma": float(fit["i_cap_ma"]),
            "offset": np.array([st["offset_ma"] if st else 0.0 for st in fit["stiffness"]]),
            "damping": np.array([dyn[n]["damping"] if n in dyn else FALLBACK["damping"]
                                 for n in NAMES]),
            "friction": np.array([dyn[n]["friction"] if n in dyn else FALLBACK["friction"]
                                  for n in NAMES])}


class ContactModel:
    """How hard each LEAP finger presses on something: fingertip force [N] per
    finger, thumb/index/middle/ring. Inputs in sim convention (0 = flat hand,
    + = closing), as RealLeap.read_state() gives them."""

    def __init__(self, fit, model_xml, min_lag=0.01, still=0.5):
        import mujoco
        self.mj = mujoco
        spec = mujoco.MjSpec.from_file(model_xml)
        for name in WRIST_JOINTS:
            joint = spec.joint(name)
            if joint is not None:
                spec.delete(joint)
        spec.option.gravity = list(GRAVITY_PALM_UP)
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        jnt = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in NAMES]
        sites = [mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, TIP_SITES[f])
                 for f in LEAP_FINGERS]
        if min(jnt) < 0 or min(sites) < 0:
            raise ValueError(f"{model_xml} lacks the LEAP joints or fingertip sites")
        self.qadr, self.dof = self.m.jnt_qposadr[jnt], self.m.jnt_dofadr[jnt]
        self.sites = sites
        self.jac = np.zeros((3, self.m.nv))
        self.fit, self.min_lag, self.still = fit, min_lag, still
        self.history = []
        self.torque = np.zeros(N_JOINTS)              # contact torque per joint [N.m]
        self.force = np.zeros(len(LEAP_FINGERS))      # fingertip force per finger [N]

    def gravity_and_levers(self, q):
        """Torque each joint needs to hold q against gravity [N.m], and each
        closing joint's lever to its fingertip [m]."""
        m, d = self.m, self.d
        d.qpos[self.qadr] = q
        d.qvel[:] = 0.0
        self.mj.mj_forward(m, d)
        gravity = d.qfrc_bias[self.dof].copy()
        levers = np.zeros(N_JOINTS)
        for finger, site in zip(LEAP_FINGERS, self.sites):
            self.mj.mj_jacSite(m, d, self.jac, None, site)
            ids = CLOSING[finger]
            levers[ids] = np.linalg.norm(self.jac[:, self.dof[ids]], axis=0)
        return gravity, levers

    def update(self, command, pos, vel, cur):
        fit = self.fit
        gravity, levers = self.gravity_and_levers(pos)
        moving = np.abs(vel) > 0.05
        tau = (fit["kt"] * (cur - fit["offset"]) - gravity - fit["damping"] * vel
               - fit["friction"] * np.sign(vel) * moving)
        pushing = (command - pos > self.min_lag) & (np.abs(vel) < self.still)
        self.torque = np.where(pushing, np.maximum(tau, 0.0), 0.0)
        reading = np.array([np.max(self.torque[CLOSING[f]] /
                                   np.maximum(levers[CLOSING[f]], MIN_LEVER))
                            for f in LEAP_FINGERS])
        self.history = (self.history + [reading])[-3:]      # median of 3 against spikes
        self.force = np.median(np.array(self.history), axis=0)
        return self.force


class ForceSender:
    def __init__(self, port, hand):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.port, self.letter, self.seq = port, HANDS[hand], 0

    def send(self, host, force):
        msg = f"LFT,{self.letter},{self.seq}," + ",".join(f"{f:.3f}" for f in force)
        try:
            self.sock.sendto(msg.encode("ascii"), (host, self.port))
        except OSError:
            pass
        self.seq += 1


class GloveReceiver:
    """Both gloves on one port: each frame goes to its hand's channel. Also
    remembers who sends them, which is where the feedback goes."""

    def __init__(self, port, channels):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)
        self.channels = channels      # {"L": HandChannel, "R": HandChannel}
        self.sender = None
        self.bad = 0
        self.unused = 0

    def poll(self):
        now = time.monotonic()
        while True:
            try:
                data, (host, _) = self.sock.recvfrom(4096)
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
            self.sender = host


class TactileHand:
    def __init__(self, channel, model, sender):
        self.channel, self.model, self.sender = channel, model, sender
        self.read_errors = 0

    def status(self, live):
        forces = " ".join(f"{f[0].upper()}{v:4.1f}" for f, v in zip(LEAP_FINGERS, self.model.force))
        return self.channel.status(live) + f"{forces} N"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hands", choices=["both", *HANDS], default="both",
                    help="which hands to teleoperate (default both)")
    ap.add_argument("--port", type=int, default=9871, help="UDP port of the glove stream")
    ap.add_argument("--rate", type=float, default=60.0, help="motor command rate [Hz]")
    ap.add_argument("--timeout", type=float, default=0.5,
                    help="glove silence [s] after which its hand holds its pose")
    ap.add_argument("--list", action="store_true",
                    help="list the LEAP Hands (FTDI adapters) plugged in here, then exit")
    for hand in HANDS:
        g = ap.add_argument_group(f"{hand} hand")
        g.add_argument(f"--{hand}-config",
                       help=f"mapping JSON (default {os.path.basename(default_config(hand))})")
        g.add_argument(f"--{hand}-serial", metavar="PORT", help="the LEAP Hand's port")
        g.add_argument(f"--{hand}-flip", default="",
                       help="motor IDs that move the wrong way on this hand "
                            "(see leap_hand_node.py --check)")
    fb = ap.add_argument_group("force feedback")
    fb.add_argument("--feedback-port", type=int, default=9873,
                    help="UDP port of ik_stream_feedback.exe (default 9873)")
    fb.add_argument("--feedback-host",
                    help="where ik_stream_feedback.exe runs (default: wherever the glove "
                         "frames come from)")
    fb.add_argument("--feedback-rate", type=float, default=30.0,
                    help="motor reads per second and hand [Hz] (default 30; with the 60 Hz "
                         "commands that is LEAP's suggested 90 bus queries/s)")
    fb.add_argument("--fit", default=DEFAULT_FIT, help="fit_servo.py JSON (servo calibration)")
    fb.add_argument("--model", default=DEFAULT_MODEL, help="MuJoCo LEAP hand of the calibration")
    fb.add_argument("--min-lag", type=float, default=0.01,
                    help="rad a joint must be behind its command to count as pushing "
                         "(default 0.01)")
    fb.add_argument("--still", type=float, default=0.5,
                    help="rad/s above which a joint counts as moving, not pushing (default 0.5)")
    hw = ap.add_argument_group("hardware")
    hw.add_argument("--baud", type=int, default=1000000)
    hw.add_argument("--leap-api", help="LEAP_Hand_API/python folder")
    hw.add_argument("--current-limit", type=float, default=550,
                    help="driver units (x1.34 mA); the calibration used 550")
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
        if not serial:
            ports = ftdi_ports()
            sys.exit(f"the {hand} hand needs --{hand}-serial PORT: the force comes from its "
                     f"motor currents, so it must be plugged in here\n"
                     + ("LEAP Hands plugged in here:\n  " + "\n  ".join(ports) if ports
                        else "no LEAP Hand is plugged in here"))
        config = getattr(args, f"{hand}_config") or default_config(hand)
        if not os.path.exists(config):
            sys.exit(f"no mapping at {config} -- run ./leap_calibrate.py --hand {hand} first")
        plan[hand] = (config, serial, parse_flip(getattr(args, f"{hand}_flip"), hand))

    serials = [os.path.realpath(p[1]) for p in plan.values()]
    if len(set(serials)) != len(serials):
        sys.exit("--left-serial and --right-serial are the same port")
    try:
        import dynamixel_sdk  # noqa: F401 -- LEAP_Hand_API only imports it when a port opens
        import mujoco  # noqa: F401
    except ImportError as e:
        sys.exit(f"{e.name} is missing: run with ~/LEAP_Development/venv/bin/python")
    if not os.path.exists(args.model):
        sys.exit(f"no hand model at {args.model} (pass --model: leap-tactile-regrasp's "
                 f"ltr/assets/leap_hand.xml)")
    fit = load_fit(args.fit)
    print(f"calibration: k_t {fit['kt'] * 1000:.3f} N.m/A, cap {fit['cap_ma']:.0f} mA, "
          f"gravity palm up")
    if "left" in plan:
        print("  (measured on the right hand; the left uses the same values)")

    # Everything that can fail without hardware comes first, so a mistake never
    # leaves a hand with its torque on.
    channels = {}
    try:
        rx = GloveReceiver(args.port, channels)
    except OSError as e:
        sys.exit(f"cannot listen on UDP {args.port}: {e} -- is another teleop script running?")
    models = {hand: ContactModel(fit, args.model, args.min_lag, args.still) for hand in plan}

    from leap_hand_node import RealLeap, open_and_release, stop_on_sigterm
    stop_on_sigterm()
    hands = []
    try:
        for hand, (config, serial, flip) in plan.items():
            print(f"{hand} hand:")
            leap = RealLeap(args.leap_api, serial, args.baud, args.current_limit,
                            args.kp, args.kd, flip)
            channel = HandChannel(hand, config, leap)
            channels[HANDS[hand]] = channel
            hands.append(TactileHand(channel, models[hand], ForceSender(args.feedback_port, hand)))
            print(f"{hand}: mapping {os.path.basename(config)} -> {serial}"
                  + (f", flipped {flip}" if flip else ""))

        period = 1.0 / args.rate
        read_period = 1.0 / args.feedback_rate
        last = time.monotonic()
        next_read = last
        last_print = 0.0
        print(f"waiting for the glove stream on UDP {args.port}; forces go back to "
              f"{args.feedback_host or 'its sender'}:{args.feedback_port}... Ctrl-C to stop")
        while True:
            now = time.monotonic()
            dt = now - last
            last = now

            rx.poll()
            live = [h.channel.step(now, dt, args.timeout) for h in hands]

            if now >= next_read:
                next_read = max(next_read + read_period, now)
                dest = args.feedback_host or rx.sender
                for h in hands:
                    try:
                        pos, vel, cur = h.channel.leap.read_state()
                    except Exception:  # a dropped bus read: skip this one
                        h.read_errors += 1
                        continue
                    force = h.model.update(h.channel.leap.command_sim(), pos, vel, cur)
                    if dest is not None:
                        h.sender.send(dest, force)

            if now - last_print >= 1.0:
                last_print = now
                extra = f"   other glove ignored {rx.unused}" if rx.unused else ""
                extra += f"   bad frames {rx.bad}" if rx.bad else ""
                errors = sum(h.read_errors for h in hands)
                extra += f"   read errors {errors}" if errors else ""
                print("\r  " + " | ".join(h.status(lv) for h, lv in zip(hands, live)) + extra,
                      end="", flush=True)

            time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        print()
        dest = args.feedback_host or rx.sender
        for h in hands:
            if dest is not None:
                h.sender.send(dest, np.zeros(len(LEAP_FINGERS)))   # release the glove now
            print(f"{h.channel.hand} hand: ", end="")
            open_and_release(h.channel.leap)


if __name__ == "__main__":
    main()
