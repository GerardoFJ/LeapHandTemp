#!/usr/bin/env python3
"""
Fit leap_mapping.json to your hand: hold a few poses, get gains and offsets.

The MuJoCo viewer shows each pose on the LEAP hand; copy it with the glove,
press Enter (with the other hand) and hold still for a couple of seconds.

The LEAP Hand has three fingers and a thumb. Its index and middle follow
yours, and its RING finger follows your PINKY (your ring finger is unused).

What gets fitted, from the joint orientations haptx_hand_stream.exe sends:
  * The "open" pose is the reference. Every other pose is measured as each
    glove joint's rotation away from it (a rotation vector, so it is signed
    and does not care which rest offset the HaptX skeleton bakes in).
  * Fingers: each joint's bend axis is the direction it rotated in the fist,
    and the MCP's spread axis is what is left of the spread pose after taking
    the bend out. LEAP joint = gain * (rotation about that axis) + offset, a
    straight line fit over the poses.
  * Thumb: the human and LEAP thumbs do not share a joint layout, so each LEAP
    thumb joint is a small ridge regression over the glove thumb's base swing
    (its main directions of motion across the poses) and its two bends.

    ./leap_calibrate.py                  # run haptx_hand_stream.exe first
    ./leap_calibrate.py --hand right     # right glove + right LEAP: leap_mapping_right.json
    ./leap_calibrate.py --out my.json    # don't overwrite leap_mapping.json
    ./leap_calibrate.py --from-raw leap_calib_raw.json   # re-fit, no glove

The previous mapping is kept as leap_mapping.json.bak, and every recorded
frame goes to leap_calib_raw.json, which --from-raw can re-fit later.
"""

import argparse
import datetime
import json
import math
import os
import shutil
import sys
import time

import numpy as np

from leap_teleop import (CHANNELS, HANDS, LEAP_JOINTS, GloveStream, SimLeap, default_config,
                         default_model, default_raw, relative_rotvecs)

DEG = math.pi / 180.0

# LEAP finger prefix -> the human finger that drives it.
LEAP_FINGERS = {"if": "index", "mf": "middle", "rf": "pinky"}
USED = ["thumb", "index", "middle", "pinky"]
THUMB_JOINTS = ["th_cmc", "th_axl", "th_mcp", "th_ipl"]

# LEAP sim angles [rad]; anything not listed is 0 (the flat hand). The thumb
# targets were solved on the Menagerie left hand: th_cmc swings the thumb out
# of the palm plane, th_mcp/th_ipl bend it toward the fingers, th_axl twists it
# about its own axis (no clean human equivalent; only the pinky pinch, which
# the LEAP thumb barely reaches, uses it).
FIST = {f"{p}_{j}": v for p in LEAP_FINGERS
        for j, v in (("mcp", 1.5), ("pip", 1.6), ("dip", 1.1))}
THUMB_SIDE = {"th_mcp": 1.3, "th_ipl": 0.2}

# groups: which joints a pose calibrates. "flex" = finger MCP/PIP/DIP,
# "spread" = finger side-to-side, "thumb" = the four thumb joints.
POSES = [
    ("open", "Hand FLAT, all fingers straight and TOGETHER.\n"
             "  Thumb straight, sticking out SIDEWAYS in the plane of the palm (an 'L').\n"
             "  This is the reference pose: be as flat and relaxed as you can.",
     {}, ("flex", "spread", "thumb")),
    ("fist", "Curl all four fingers into a FIST.\n"
             "  Thumb straight, lying along the side of the index (like a thumbs-up).",
     {**FIST, **THUMB_SIDE}, ("flex", "spread", "thumb")),
    ("spread", "Hand flat, fingers straight and SPREAD as wide as you can.\n"
               "  Thumb out sideways again (the 'L').",
     {"if_rot": 0.35, "rf_rot": -0.35}, ("flex", "spread", "thumb")),
    ("thumb_side", "Hand flat, fingers together.\n"
                   "  Thumb straight, pressed against the side of the index.",
     THUMB_SIDE, ("flex", "spread", "thumb")),
    ("thumb_forward", "Hand flat, fingers together.\n"
                      "  Thumb straight, pointing straight OUT of the palm (90 deg to it).",
     {"th_cmc": 1.6}, ("flex", "thumb")),
    ("thumb_curl", "Hand flat, fingers together.\n"
                   "  Thumb out of the palm as before, then BEND it so the tip curls\n"
                   "  over the palm toward the base of the index.",
     {"th_cmc": 1.6, "th_mcp": 0.9, "th_ipl": 0.8}, ("flex", "thumb")),
    ("pinch_index", "Touch the THUMB tip to the INDEX tip (an 'OK' sign).\n"
                    "  The other fingers straight.",
     {"th_cmc": 1.45, "th_mcp": 0.85, "th_ipl": 0.1,
      "if_mcp": 0.9, "if_pip": 0.8, "if_dip": 0.5}, ("thumb",)),
    ("pinch_middle", "Touch the THUMB tip to the MIDDLE tip.\n"
                     "  The other fingers straight.",
     {"th_cmc": 1.9, "th_mcp": 0.75, "th_ipl": 0.1,
      "mf_mcp": 1.0, "mf_pip": 0.9, "mf_dip": 0.55}, ("thumb",)),
    ("pinch_pinky", "Touch the THUMB tip to the PINKY tip (the LEAP's ring finger).\n"
                    "  The other fingers straight.",
     {"th_cmc": 2.05, "th_axl": -0.3, "th_mcp": 0.4, "th_ipl": 0.15,
      "rf_mcp": 1.3, "rf_pip": 0.85, "rf_dip": 0.5}, ("thumb",)),
]
POSE_BY_NAME = {p[0]: p for p in POSES}

# The targets above are for the left hand. Menagerie's right hand is its mirror
# image: the same targets, except that the side-to-side joints and the thumb's
# twist turn the other way (checked by rendering both).
LEFT_POSES = POSES
MIRRORED = ("if_rot", "mf_rot", "rf_rot", "th_axl")


def use_hand(hand):
    global POSES, POSE_BY_NAME
    POSES = [(name, text, {k: -v if hand == "right" and k in MIRRORED else v
                           for k, v in targets.items()}, groups)
             for name, text, targets, groups in LEFT_POSES]
    POSE_BY_NAME = {p[0]: p for p in POSES}

SETTLE_S = 1.0
RECORD_S = 2.0
MIN_SPAN = {"flex": 8 * DEG, "spread": 3 * DEG}  # less than this across poses: dead joint
JITTER_WARN = 15 * DEG                           # p5..p95 while "holding still"
MAX_GAIN = 5.0                                    # a gloved fist bends the MCP little
THUMB_RIDGE = 0.02                               # on standardized inputs, per pose
THUMB_MIN_RMS = 4 * DEG                          # weaker thumb-base directions are noise


# ------------------------------------------------------------------ recording
def record(glove, seconds):
    """Glove frames received during `seconds`: [{finger: {"angles", "q"}}]."""
    glove.poll()                         # drop whatever queued while waiting
    frames = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if glove.poll() and glove.human:
            frames.append(glove.human)
        time.sleep(0.003)
    return frames


def quats(frames, finger):
    """[N, 3, 4] joint quaternions of `finger` over the frames that have them."""
    qs = [fr[finger]["q"] for fr in frames if fr.get(finger, {}).get("q") is not None]
    return np.array(qs).reshape(-1, 3, 4)


def check(frames):
    """Problems worth redoing a pose for, as text lines."""
    if len(frames) < 20:
        return [f"only {len(frames)} glove frames -- is haptx_hand_stream.exe running?"]
    problems = []
    for finger in USED:
        q = quats(frames, finger)
        if len(q) < 0.5 * len(frames):
            problems.append(f"{finger}: the glove sent no joint orientations in most frames")
            continue
        # Movement while holding, measured from this pose's own first frame.
        r = relative_rotvecs(q, q[0])                         # [N, 3, 3]
        spread = np.percentile(r, 95, 0) - np.percentile(r, 5, 0)
        for k, label in enumerate(("base", "middle", "tip")):
            if spread[k].max() > JITTER_WARN:
                problems.append(f"{finger} {label} joint moved {spread[k].max() / DEG:.0f} deg "
                                f"while holding")
    return problems


# --------------------------------------------------------------- statistics
def mean_quat(q):
    """Average of [N, 4] nearby unit quaternions."""
    q = np.where((q @ q[0])[:, None] < 0, -q, q)
    m = q.mean(0)
    return m / np.linalg.norm(m)


def pose_rotvecs(recordings, rest):
    """{pose: {finger: [3, 3] median rotation vector of each joint from rest}}."""
    out = {}
    for pose, frames in recordings.items():
        out[pose] = {}
        for finger, r0 in rest.items():
            q = quats(frames, finger)
            if len(q):
                out[pose][finger] = np.median(relative_rotvecs(q, r0), 0)
    return out


def unit(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else None


def principal_axes(vectors):
    """Directions and RMS sizes of [M, 3] vectors around zero (the open pose)."""
    _, s, vt = np.linalg.svd(np.asarray(vectors), full_matrices=False)
    return vt, s / math.sqrt(len(vectors))


# ------------------------------------------------------------------ fitting
def full_target(targets, names):
    return np.array([targets.get(n, 0.0) for n in names])


def fit_line(xs, ys, group, label, warnings):
    xs, ys = np.asarray(xs), np.asarray(ys)
    if len(xs) == 0:
        warnings.append(f"{label}: no poses recorded, held at 0")
        return 0.0, 0.0
    if np.ptp(ys) < 1e-6:
        # Every pose wants the same LEAP angle (e.g. the middle finger never
        # spreads): hold it there.
        return 0.0, float(ys.mean())
    if np.ptp(xs) < MIN_SPAN[group]:
        warnings.append(f"{label}: the glove barely moved ({np.ptp(xs) / DEG:.1f} deg) "
                        f"across poses -- held at its average")
        return 0.0, float(ys.mean())
    gain, offset = np.polyfit(xs, ys, 1)
    if abs(gain) > MAX_GAIN:
        warnings.append(f"{label}: gain {gain:.1f} clipped to {MAX_GAIN} -- the glove moves "
                        f"much less than the LEAP joint should")
        gain = math.copysign(MAX_GAIN, gain)
        offset = float(np.mean(ys - gain * xs))
    return float(gain), float(offset)


def pose_angles(recordings):
    """{pose: {finger: {channel: median rad}}} for the fingers the IK gave
    angles for in most frames of that pose."""
    out = {}
    for pose, frames in recordings.items():
        out[pose] = {}
        for finger in USED:
            rows = [fr[finger]["angles"] for fr in frames if (fr.get(finger) or {}).get("angles")]
            if rows and len(rows) >= 0.5 * len(frames):
                out[pose][finger] = {c: float(np.median([r[c] for r in rows])) for c in CHANNELS}
    return out


def finger_features(R, warnings, fingers):
    """Bend axes per joint and an MCP spread axis, for human fingers that have
    no IK angles. Returns {name: {"finger", "joint", "axis"}}."""
    features = {}
    for finger in fingers:
        for k in range(3):
            fist = R.get("fist", {}).get(finger)
            axis = unit(fist[k]) if fist is not None else None
            if axis is None or np.linalg.norm(fist[k]) < MIN_SPAN["flex"]:
                # No usable fist: the joint's main direction over the flex poses.
                rows = [R[p][finger][k] for p in R
                        if "flex" in POSE_BY_NAME[p][3] and finger in R[p]]
                if not rows:
                    continue
                vt, _ = principal_axes(rows)
                axis = vt[0]
                if fist is not None and fist[k] @ axis < 0:
                    axis = -axis
            features[f"{finger}.flex{k + 1}"] = {"finger": finger, "joint": k,
                                                 "axis": axis.tolist()}
        spread = R.get("spread", {}).get(finger)
        if spread is not None:
            bend = np.array(features[f"{finger}.flex1"]["axis"]) \
                if f"{finger}.flex1" in features else np.zeros(3)
            side = spread[0] - (spread[0] @ bend) * bend
            if np.linalg.norm(side) >= MIN_SPAN["spread"]:
                features[f"{finger}.spread"] = {"finger": finger, "joint": 0,
                                                "axis": unit(side).tolist()}
            elif finger != "middle":
                warnings.append(f"{finger}: barely moved sideways in the spread pose "
                                f"({np.linalg.norm(side) / DEG:.1f} deg) -- no spread")
    return features


def thumb_features(R, warnings):
    """The thumb base's main swing directions plus one bend axis for each of
    the two outer thumb joints, over the thumb poses."""
    poses = [p for p in R if "thumb" in POSE_BY_NAME[p][3] and "thumb" in R[p]]
    features = {}
    if len(poses) < 3:
        warnings.append("thumb: fewer than 3 thumb poses -- thumb held still")
        return features
    vt, rms = principal_axes([R[p]["thumb"][0] for p in poses])
    for d in range(3):
        if rms[d] >= THUMB_MIN_RMS:
            features[f"thumb.base{d + 1}"] = {"finger": "thumb", "joint": 0,
                                              "axis": vt[d].tolist()}
    for k in (1, 2):
        vt, rms = principal_axes([R[p]["thumb"][k] for p in poses])
        if rms[0] >= THUMB_MIN_RMS:
            axis = vt[0]
            curl = R.get("thumb_curl", {}).get("thumb")
            if curl is not None and curl[k] @ axis < 0:
                axis = -axis
            features[f"thumb.bend{k + 1}"] = {"finger": "thumb", "joint": k,
                                              "axis": axis.tolist()}
        else:
            warnings.append(f"thumb joint {k + 1}: barely bent across the poses -- not used")
    if not any(n.startswith("thumb.base") for n in features):
        warnings.append("thumb base: barely moved across the poses -- check the glove")
    return features


def feature_value(R_pose, ft):
    r = R_pose.get(ft["finger"])
    return None if r is None else float(r[ft["joint"]] @ np.array(ft["axis"]))


def input_key(inp):
    return ("feature", inp["feature"]) if "feature" in inp else \
        ("channel", inp["finger"], inp["channel"])


def pose_values(R, A, features):
    """{pose: {input key: value}}: every feature and IK angle per pose."""
    out = {}
    for pose in R:
        values = {}
        for name, ft in features.items():
            x = feature_value(R[pose], ft)
            if x is not None:
                values[("feature", name)] = x
        for finger, channels in A.get(pose, {}).items():
            for channel, x in channels.items():
                values[("channel", finger, channel)] = x
        out[pose] = values
    return out


def fit(recordings, names):
    """recordings: {pose: frames}. Returns (config parts, warnings, values).

    Fingers fit on AlternateHandIk's angles when it gave them in every pose:
    they are zero at the flat hand and split a fist's bend between PIP and
    DIP the way the hand looks, which the joint orientations do not (in a fist
    the solver's legacy blend puts nearly all of it on one joint). The thumb
    has no IK angles, so it always uses joint rotations."""
    warnings = []
    if "open" not in recordings:
        sys.exit("the 'open' pose is the reference -- it cannot be skipped")
    rest = {}
    for finger in USED:
        q = quats(recordings["open"], finger)
        if len(q) == 0:
            sys.exit(f"no {finger} orientations in the open pose -- is the stream H2?")
        rest[finger] = np.array([mean_quat(q[:, k]) for k in range(3)])
    R = pose_rotvecs(recordings, rest)

    A = pose_angles(recordings)
    with_angles = {f for f in LEAP_FINGERS.values() if all(f in A[p] for p in A)}
    without = [f for f in LEAP_FINGERS.values() if f not in with_angles]
    if without:
        warnings.append(f"no IK angles for {', '.join(without)} -- using joint rotations "
                        f"(is haptx_hand_stream.exe running with --ik alternate?)")
    features = {**finger_features(R, warnings, without), **thumb_features(R, warnings)}
    V = pose_values(R, A, features)
    joints = []
    index = {n: i for i, n in enumerate(names)}

    for i, name in enumerate(names):
        if name.startswith("th_"):
            continue
        prefix, joint = name.split("_")
        finger = LEAP_FINGERS[prefix]
        if finger in with_angles:
            inp = {"finger": finger, "channel": "add" if joint == "rot" else joint}
        else:
            inp = {"feature": f"{finger}.spread" if joint == "rot" else
                   f"{finger}.flex{['mcp', 'pip', 'dip'].index(joint) + 1}"}
        group = "spread" if joint == "rot" else "flex"
        wanted = {POSE_BY_NAME[p][2].get(name, 0.0) for p in R if group in POSE_BY_NAME[p][3]}
        if len(wanted) == 1:
            # Every pose wants the same angle (the LEAP middle finger never
            # spreads): hold it there, whatever the glove does.
            joints.append({"id": i, "mjcf": name, "offset": wanted.pop(), "inputs": []})
            continue
        xs, ys = [], []
        for pose, values in V.items():
            x = values.get(input_key(inp))
            if group in POSE_BY_NAME[pose][3] and x is not None:
                xs.append(x)
                ys.append(POSE_BY_NAME[pose][2].get(name, 0.0))
        label = f"{name} <- " + (inp.get("feature") or f"{finger} {inp['channel']}")
        gain, offset = fit_line(xs, ys, group, label, warnings)
        entry = {"id": i, "mjcf": name, "offset": round(offset, 4)}
        entry["inputs"] = [{**inp, "gain": round(gain, 4)}] if gain else []
        joints.append(entry)

    thumb_names = sorted(n for n in features if n.startswith("thumb."))
    poses = [p for p in R if "thumb" in POSE_BY_NAME[p][3] and "thumb" in R[p]]
    gains = np.zeros((len(THUMB_JOINTS), len(thumb_names)))
    Y = np.array([[POSE_BY_NAME[p][2].get(j, 0.0) for j in THUMB_JOINTS] for p in poses]) \
        if poses else np.zeros((1, len(THUMB_JOINTS)))
    offsets = Y.mean(0)
    if thumb_names and len(poses) >= 3:
        X = np.array([[V[p][("feature", n)] for n in thumb_names] for p in poses])
        mu, sd = X.mean(0), X.std(0)
        Xs = (X - mu) / sd
        W = np.linalg.solve(Xs.T @ Xs + THUMB_RIDGE * len(poses) * np.eye(len(thumb_names)),
                            Xs.T @ (Y - Y.mean(0)))
        gains = (W / sd[:, None]).T
        offsets = Y.mean(0) - gains @ mu
    for k, name in enumerate(THUMB_JOINTS):
        joints.append({"id": index[name], "mjcf": name, "offset": round(float(offsets[k]), 4),
                       "inputs": [{"feature": n, "gain": round(float(g), 4)}
                                  for n, g in zip(thumb_names, gains[k]) if abs(g) > 1e-4]})
    joints.sort(key=lambda j: j["id"])

    used = {inp["feature"] for j in joints for inp in j["inputs"] if "feature" in inp}
    features = {n: {**ft, "axis": [round(a, 5) for a in ft["axis"]]}
                for n, ft in features.items() if n in used}
    rest = {f: np.round(q, 6).tolist() for f, q in rest.items()
            if any(ft["finger"] == f for ft in features.values())}
    return {"rest": rest, "features": features, "joints": joints}, warnings, V


def predict(cfg, values_pose):
    out = []
    for j in cfg["joints"]:
        values = [values_pose.get(input_key(i)) for i in j["inputs"]]
        if any(v is None for v in values):
            out.append(float("nan"))
        else:
            out.append(j["offset"] + sum(i["gain"] * v for i, v in zip(j["inputs"], values)))
    return np.array(out)


def report(cfg, V, names):
    """Worst joint per pose: how far the fitted mapping lands from the target."""
    print("\nfit check: how close each pose lands on the LEAP target")
    for pose, values_pose in V.items():
        _, _, targets, groups = POSE_BY_NAME[pose]
        err = np.abs(predict(cfg, values_pose) - full_target(targets, names))
        relevant = [i for i, n in enumerate(names)
                    if ("thumb" in groups and n.startswith("th_"))
                    or ("flex" in groups and not n.startswith("th_") and not n.endswith("rot"))
                    or ("spread" in groups and n.endswith("rot"))]
        err = err[relevant]
        if np.all(np.isnan(err)):
            continue
        k = relevant[int(np.nanargmax(err))]
        worst = float(np.nanmax(err)) / DEG
        flag = "  <-- redo this pose or check the glove" if worst > 20 else ""
        print(f"  {pose:<14} worst {names[k]:<7} {worst:5.1f} deg   "
              f"mean {float(np.nanmean(err)) / DEG:4.1f} deg{flag}")


# ----------------------------------------------------------------------- run
def run(glove, show, ask, settle_s=SETTLE_S, record_s=RECORD_S):
    """Walk through POSES. Returns {pose: frames}, or None if the user quit."""
    recordings = {}
    i = 0
    while i < len(POSES):
        name, text, targets, _ = POSES[i]
        show(targets)
        print(f"\n[{i + 1}/{len(POSES)}] {name}\n  {text}")
        answer = ask("  Enter = record   s = skip   b = back   q = quit > ").strip().lower()
        if answer == "q":
            return None
        if answer == "b":
            i = max(i - 1, 0)          # re-recording a pose replaces it
            continue
        if answer == "s":
            if name == "open":
                print("  the open pose is the reference -- it cannot be skipped")
                continue
            i += 1
            continue

        print("  hold it...", end="", flush=True)
        time.sleep(settle_s)
        print(" recording", end="", flush=True)
        frames = record(glove, record_s)
        print(f" -- {len(frames)} frames")
        problems = check(frames)
        for p in problems:
            print(f"  ! {p}")
        if problems:
            again = ask("  r = redo   Enter = keep anyway > ").strip().lower()
            if again == "r":
                continue
        recordings[name] = frames
        i += 1
    return recordings


def to_raw(recordings, hand):
    def frame(fr):
        return {f: {"angles": d["angles"],
                    "q": None if d["q"] is None else np.round(d["q"], 5).tolist()}
                for f, d in fr.items()}
    return {"format": "leap_calib_raw/2", "hand": hand,
            "poses": {p: [frame(fr) for fr in frames] for p, frames in recordings.items()}}


def from_raw(path, hand="left"):
    with open(path) as fh:
        raw = json.load(fh)
    if raw.get("format") != "leap_calib_raw/2":
        sys.exit(f"{path} is from the old calibration (angles only) -- record it again")
    if raw.get("hand", "left") != hand:     # recordings before --hand were all left
        sys.exit(f"{path} was recorded for the {raw.get('hand', 'left')} hand, not the {hand}")
    return {p: [{f: {"angles": d["angles"],
                     "q": None if d["q"] is None else np.array(d["q"])}
                 for f, d in fr.items()} for fr in frames]
            for p, frames in raw["poses"].items() if p in POSE_BY_NAME}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hand", choices=HANDS, default="left",
                    help="which glove and LEAP hand (default left)")
    ap.add_argument("--port", type=int, default=9871, help="UDP port of the glove stream")
    ap.add_argument("--config", help="mapping to write, and to take smoothing etc. from if it "
                                     "exists (default: leap_mapping.json for the left hand, "
                                     "leap_mapping_right.json for the right)")
    ap.add_argument("--out", help="where to write the fitted mapping (default: --config)")
    ap.add_argument("--model", help="LEAP hand MJCF scene (default: Menagerie's, for --hand)")
    ap.add_argument("--no-viewer", action="store_true", help="text instructions only")
    ap.add_argument("--from-raw", metavar="JSON",
                    help="re-fit a saved raw recording instead of recording")
    args = ap.parse_args()
    use_hand(args.hand)
    config = args.config or default_config(args.hand)
    out_path = args.out or config

    if os.path.exists(config):
        with open(config) as fh:
            base = json.load(fh)
    else:
        base = {"joints": [{"id": i, "mjcf": n} for i, n in enumerate(LEAP_JOINTS)]}
    names = [j["mjcf"] for j in sorted(base["joints"], key=lambda j: j["id"])]
    sim = None
    print(f"{args.hand} hand -> {out_path}")

    if args.from_raw:
        recordings = from_raw(args.from_raw, args.hand)
    else:
        sim = SimLeap(args.model or default_model(args.hand))
        qadr = [sim.model.jnt_qposadr[sim.mujoco.mj_name2id(sim.model,
                                                            sim.mujoco.mjtObj.mjOBJ_JOINT, n)]
                for n in names]
        if not args.no_viewer:
            sim.open_viewer()

        def show(targets):
            # Kinematic display: put the hand straight into the pose, no physics.
            sim.data.qpos[qadr] = full_target(targets, names)
            sim.mujoco.mj_forward(sim.model, sim.data)
            if sim.viewer is not None:
                sim.viewer.sync()

        glove = GloveStream(args.port, args.hand)
        print("waiting for the glove stream...", end="", flush=True)
        t0 = time.monotonic()
        while not glove.poll():
            if time.monotonic() - t0 > 10:
                sys.exit(f"\nnothing on UDP {args.port} -- start haptx_hand_stream.exe first")
            time.sleep(0.05)
        print(" ok")
        print("Copy each pose shown on the LEAP hand in the viewer. The LEAP's ring finger\n"
              "is your PINKY; just keep your ring finger next to the others. Use the other\n"
              "hand for the keyboard, and keep the glove hand still while it records.")
        try:
            recordings = run(glove, show, input)
        except (KeyboardInterrupt, EOFError):
            recordings = None
        if not recordings:
            print("\nnothing written")
            return

    parts, warnings, V = fit(recordings, names)
    report(parts, V, names)
    if warnings:
        print("\nwarnings:")
        for w in warnings:
            print(f"  ! {w}")

    cfg = {
        "_doc": [
            "Written by leap_calibrate.py -- rerun it rather than editing by hand",
            f"(./leap_calibrate.py --hand {args.hand} --from-raw "
            f"{os.path.basename(default_raw(args.hand))} re-fits without the glove).",
            "LEAP target [rad, sim convention: 0 = flat] = offset + sum(gain * feature).",
            "A feature is one glove joint's rotation [rad] about 'axis', measured from 'rest',",
            "the joint orientations of the calibration's open pose.",
            "The LEAP ring finger follows the human pinky.",
        ],
        "smoothing": base.get("smoothing", 0.35),
        "max_speed_rad_s": base.get("max_speed_rad_s", 8.0),
        "calibration": {"date": datetime.datetime.now().isoformat(timespec="seconds"),
                        "hand": args.hand, "poses": list(recordings)},
        **parts,
    }
    if os.path.exists(out_path):
        shutil.copyfile(out_path, out_path + ".bak")
    with open(out_path, "w") as fh:
        json.dump(cfg, fh, indent=2)
    print(f"\nwrote {out_path}")
    if not args.from_raw:
        raw_path = os.path.join(os.path.dirname(os.path.abspath(out_path)),
                                os.path.basename(default_raw(args.hand)))
        with open(raw_path, "w") as fh:
            json.dump(to_raw(recordings, args.hand), fh)
        print(f"raw recordings: {raw_path}")
    print(f"try it:  ./leap_teleop.py --hand {args.hand} --monitor")
    if sim is not None and sim.viewer is not None:
        sim.viewer.close()


if __name__ == "__main__":
    main()
