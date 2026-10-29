#!/usr/bin/env python3
"""Debug the tactile sensor's serial stream without ROS 2.

Frame format that AcquisitionNode.cpp expects:

    DLE STX | frame number (4 bytes, little endian) | size_x | size_y |
    size_x * size_y values, x changing fastest | DLE ETX

DLE = 0x10, STX = 0x02, ETX = 0x03. A 0x10 inside the frame is sent twice.

  raw     hex dump of every byte received
  frames  decode frames, print the matrices, and run a copy of the C++
          state machine on the same bytes to show where it disagrees

With --config, the device, baud rate, sensor size and patches come from the
ROS YAML file, and each frame is printed as its patches.

Examples:
  python3 serial_debug.py raw
  python3 serial_debug.py --config tactile.yaml
  python3 serial_debug.py --config tactile.yaml --patch BACK_UP --patch FRONT_UP
  python3 serial_debug.py --config tactile.yaml --save dump.bin
  python3 serial_debug.py --config tactile.yaml --replay dump.bin
"""

import argparse
import sys
import time

DLE, STX, ETX = 0x10, 0x02, 0x03

DEFAULT_PORT = "/dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_56AAAD9B81683EF9-if00"
DEFAULT_BAUD = 921600


def find_params(data):
    """Find the dict holding device/patches, also when it is nested under ros__parameters."""
    if isinstance(data, dict):
        if "device" in data or "patches" in data:
            return data
        for value in data.values():
            found = find_params(value)
            if found is not None:
                return found
    return None


def load_config(path):
    """Read device, baudrate, rows, cols and patches from the ROS YAML file."""
    try:
        import yaml
    except ImportError:
        sys.exit("PyYAML is missing: sudo apt install python3-yaml  (or: pip install pyyaml)")
    with open(path) as f:
        params = find_params(yaml.safe_load(f))
    if params is None:
        sys.exit(f"{path}: no 'device' or 'patches' key found")

    rows, cols = params.get("rows"), params.get("cols")
    print(f"Config {path}: device {params.get('device')}, baudrate {params.get('baudrate')}, "
          f"sensor {rows} x {cols}")
    patches, owner = [], {}
    for p in params.get("patches") or []:
        name = p.get("name", "?")
        if "mappings" not in p:
            # Like "default": the whole matrix, unchanged
            patches.append({"name": name, "rows": rows, "cols": cols, "cells": None})
            print(f"  patch {name:<12} {rows} x {cols}, whole matrix")
            continue
        prows, pcols, cells = p["rows"], p["cols"], {}
        for m in p["mappings"]:
            dst = (m["dst"]["x"], m["dst"]["y"])
            src = (m["src"]["x"], m["src"]["y"])
            if not (0 <= dst[0] < prows and 0 <= dst[1] < pcols):
                print(f"  WARNING: {name}: dst {dst} is outside the patch ({prows} x {pcols})")
            if rows is not None and cols is not None and not (0 <= src[0] < rows and 0 <= src[1] < cols):
                print(f"  WARNING: {name}: src {src} is outside the sensor ({rows} x {cols})")
            if dst in cells:
                print(f"  WARNING: {name}: dst {dst} is mapped twice")
            if src in owner and owner[src] != name:
                print(f"  WARNING: {name}: src {src} is also used by {owner[src]}")
            cells[dst] = src
            owner[src] = name
        line = f"  patch {name:<12} {prows} x {pcols}, {len(cells)} mappings"
        if cells:
            xs = [s[0] for s in cells.values()]
            ys = [s[1] for s in cells.values()]
            line += f", src x {min(xs)}-{max(xs)}, y {min(ys)}-{max(ys)}"
        if len(cells) < prows * pcols:
            line += f", {prows * pcols - len(cells)} cells unmapped"
        print(line)
        patches.append({"name": name, "rows": prows, "cols": pcols, "cells": cells})
    return {"device": params.get("device"), "baudrate": params.get("baudrate"),
            "rows": rows, "cols": cols, "patches": patches}


def read_chunks(args):
    """Yield received bytes from the port or a replay file; b"" means nothing arrived yet."""
    if args.replay:
        with open(args.replay, "rb") as f:
            while chunk := f.read(4096):
                yield chunk
        return

    try:
        import serial
    except ImportError:
        sys.exit("pyserial is missing: sudo apt install python3-serial  (or: pip install pyserial)")

    try:
        port = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {args.port}: {e}\n"
                 "Check that the device shows up in /dev/serial/by-id/, that your user is in the\n"
                 "'dialout' group, and that nothing else (e.g. Acquisition_Node) has the port open.")
    print(f"Opened {args.port} at {args.baud} baud, Ctrl+C to stop")
    with port:
        while True:
            yield port.read(port.in_waiting or 1)


def hexdump_line(offset, data):
    hexpart = " ".join(f"{b:02x}" for b in data)
    text = "".join(chr(b) if 32 <= b < 127 else "." for b in data)
    return f"{offset:08x}  {hexpart:<47}  |{text}|"


class RawDump:
    def __init__(self):
        self.total = 0

    def feed(self, chunk):
        for i in range(0, len(chunk), 16):
            print(hexdump_line(self.total + i, chunk[i:i + 16]))
        self.total += len(chunk)

    def summary(self, elapsed):
        print(f"\n{self.total} bytes in {elapsed:.1f} s ({self.total / max(elapsed, 1e-9):.0f} B/s)")


class FrameParser:
    """Reference decoder: DLE STX <payload> DLE ETX, with DLE DLE meaning a 0x10 byte."""

    def __init__(self):
        self.in_frame = False
        self.escape = False
        self.payload = bytearray()
        self.junk = 0  # bytes received outside any frame

    def feed(self, b):
        """Feed one byte. Returns ("frame", payload, note), ("error", text) or None."""
        if not self.escape:
            if b == DLE:
                self.escape = True
            elif self.in_frame:
                self.payload.append(b)
            else:
                self.junk += 1
            return None

        self.escape = False
        if b == DLE:
            if self.in_frame:
                self.payload.append(DLE)
            else:
                self.junk += 2
            return None
        if b == STX:
            unfinished = self.in_frame and len(self.payload) > 0
            payload, self.payload, self.in_frame = bytes(self.payload), bytearray(), True
            if unfinished:
                return ("frame", payload, "no DLE ETX before the next DLE STX")
            return None
        if self.in_frame:
            self.in_frame = False
            if b == ETX:
                return ("frame", bytes(self.payload), None)
            return ("error", f"DLE {b:02x} is not a valid escape, dropped a frame "
                             f"after {len(self.payload)} payload bytes")
        self.junk += 2
        return None


def decode(payload):
    """Split a payload into header fields and matrix[x][y]. Returns (frame, problems)."""
    if len(payload) < 6:
        return None, [f"payload is only {len(payload)} bytes, shorter than the 6-byte header"]
    sx, sy = payload[4], payload[5]
    values = payload[6:]
    problems = []
    if len(values) != sx * sy:
        problems.append(f"{len(values)} values, but the header says {sx} x {sy} = {sx * sy}")
    matrix = [[None] * sy for _ in range(sx)]
    for i, v in enumerate(values[:sx * sy]):
        matrix[i % sx][i // sx] = v
    frame = {"number": int.from_bytes(payload[:4], "little"), "sx": sx, "sy": sy,
             "values": values, "matrix": matrix}
    return frame, problems


def patch_value(patch, frame, offset, x, y):
    """Value at patch cell (x, y): an int, None if the frame lacks it, "" if the cell is unmapped."""
    if patch["cells"] is None:
        src = (x, y)
    else:
        src = patch["cells"].get((x, y))
        if src is None:
            return ""
    fx, fy = src[0] + offset, src[1] + offset
    if 0 <= fx < frame["sx"] and 0 <= fy < frame["sy"]:
        return frame["matrix"][fx][fy]
    return None


def print_grid(grid):
    """Print grid[x][y]: ints, None as "?" (no data), "" as "." (unmapped)."""
    ncols = len(grid[0]) if grid else 0
    print("      y" + "".join(f"{y:4d}" for y in range(ncols)))
    for x, row in enumerate(grid):
        print(f"  x{x:3d} " + "".join("   ." if v == "" else "   ?" if v is None else f"{v:4d}"
                                      for v in row))


class CppParser:
    """Copy of the state machine in do_stuff() in AcquisitionNode.cpp.

    acquisition_matrix is kept as one flat array, like the C++ memory, so an
    out-of-range [x][y] lands where it would in the C++. It starts as zeros here;
    the C++ never actually clears it (`0 >> acquisition_matrix[i][j]` does nothing).
    """

    def __init__(self, rows, cols):
        self.rows, self.cols = rows, cols
        self.memory = [0] * (rows * cols)
        self.state = 0
        self.prev_byte = 0
        self.dle_count = 0
        self.frame_count = 0
        self.size_x = self.size_y = self.size_count = 0
        self.x_count = self.y_count = 0
        self.aborts = self.early_ends = self.out_of_bounds = 0
        self.oob_reported = False

    def leftovers(self):
        names = ("x_count", "y_count", "frame_count", "size_count")
        return ", ".join(f"{n}={getattr(self, n)}" for n in names if getattr(self, n))

    def feed(self, got):
        """Feed one byte. Returns a list of ("warn", text) and ("complete", snapshot)."""
        events = []
        prev = self.prev_byte
        if self.state == 0:
            if prev == DLE and got == STX:
                self.state = 1
                self.oob_reported = False
        elif prev != DLE and got == DLE:
            self.dle_count += 1
        elif prev == DLE and got != DLE and self.dle_count % 2 == 1:
            self.aborts += 1
            msg = f"DLE {got:02x} in state {self.state} sent it back to state 0"
            left = self.leftovers()
            if left:
                msg += f"; {left} not reset, so the next frame starts misaligned"
            events.append(("warn", msg))
            self.dle_count = 0
            self.state = 0
        elif prev == DLE and got == DLE and self.dle_count % 2 == 0:
            self.dle_count += 1
        else:
            self.dle_count = self.dle_count + 1 if got == DLE else 0
            if self.state == 1:
                self.frame_count += 1
                if self.frame_count >= 4:
                    self.frame_count = 0
                    self.state = 2
            elif self.state == 2:
                if self.size_count == 0:
                    self.size_x = got
                elif self.size_count == 1:
                    self.size_y = got
                self.size_count += 1
                if self.size_count >= 2:
                    self.size_count = 0
                    self.state = 3
            elif prev == DLE and got == ETX:
                # Only reachable when prev was a stuffed 0x10 value, so this 0x03 is a value too
                self.early_ends += 1
                events.append(("warn", f"value 0x10 followed by value 0x03 at x={self.x_count}, "
                                       f"y={self.y_count} was taken as the end of the frame"))
                self.x_count = self.y_count = 0
                self.state = 0
            else:
                self.store(got, events)
        self.prev_byte = got
        return events

    def store(self, got, events):
        x, y = self.x_count, self.y_count
        index = x * self.cols + y
        if x >= self.rows or y >= self.cols:
            self.out_of_bounds += 1
            if not self.oob_reported:
                self.oob_reported = True
                where = ("another cell" if index < len(self.memory)
                         else "memory past the end of the array (stack corruption)")
                events.append(("warn", f"writes acquisition_matrix[{x}][{y}] but the array is "
                                       f"[{self.rows}][{self.cols}], so it hits {where}"))
        if index < len(self.memory):
            self.memory[index] = got
        self.x_count += 1
        if self.x_count == self.size_x:
            self.x_count = 0
            self.y_count += 1
            if self.y_count == self.size_y:
                self.y_count = 0
                self.state = 0
                events.append(("complete", {"sx": self.size_x, "sy": self.size_y,
                                            "memory": self.memory[:]}))


class FrameDebugger:
    def __init__(self, args, config, patches):
        self.args = args
        self.config = config
        self.patches = patches
        self.offset = 0  # added to the config's src x/y to index the frame
        self.ref = FrameParser()
        self.cpp = CppParser(args.cpp_rows, args.cpp_cols)
        self.cpp_done = None  # C++ matrix snapshot from the last frame it completed
        self.size = None
        self.last_number = None
        self.bytes = 0
        self.ok = self.malformed = self.dropped = 0
        self.cpp_mismatches = self.cpp_missed = 0
        self.last_stats = time.monotonic()
        self.last_counts = (0, 0)

    def feed(self, chunk):
        for b in chunk:
            for kind, info in self.cpp.feed(b):
                if kind == "complete":
                    self.cpp_done = info
                else:
                    print(f"C++ parser: {info}")
            event = self.ref.feed(b)
            if event:
                self.handle(event)
        self.bytes += len(chunk)
        if not self.args.replay:
            self.print_stats()

    def handle(self, event):
        if event[0] == "error":
            self.dropped += 1
            self.cpp_done = None
            print(f"ERROR: {event[1]}")
            return
        _, payload, note = event
        frame, problems = decode(payload)
        if frame is None:
            self.dropped += 1
            self.cpp_done = None
            print(f"ERROR: {problems[0]}")
            return
        if problems:
            self.malformed += 1
        else:
            self.ok += 1
        self.check_size(frame)
        self.check_number(frame)
        cpp_msg, cpp_bad = self.compare_cpp(frame, comparable=not problems)

        count = self.ok + self.malformed
        show = (count - 1) % self.args.every == 0
        if show or problems or note or cpp_bad:
            values = frame["values"]
            line = f"frame {frame['number']}: size_x={frame['sx']} size_y={frame['sy']}, {len(values)} values"
            if values:
                line += f", min {min(values)} max {max(values)} nonzero {sum(v > 0 for v in values)}"
            if note:
                line += f"  [{note}]"
            print(line)
        if show:
            if self.patches:
                for patch in self.patches:
                    self.print_patch(patch, frame)
            else:
                print_grid(frame["matrix"])
        for p in problems:
            print(f"  PROBLEM: {p}")
        if cpp_msg and (show or cpp_bad):
            print(f"  {cpp_msg}")

    def print_patch(self, patch, frame):
        nrows = patch["rows"] if patch["rows"] is not None else frame["sx"]
        ncols = patch["cols"] if patch["cols"] is not None else frame["sy"]
        grid = [[patch_value(patch, frame, self.offset, x, y) for y in range(ncols)]
                for x in range(nrows)]
        numbers = [v for row in grid for v in row if isinstance(v, int)]
        title = f"  {patch['name']} ({nrows} x {ncols})"
        if numbers:
            title += f" max {max(numbers)}"
        print(title)
        print_grid(grid)

    def pick_offset(self, sx, sy):
        if self.args.offset != "auto":
            return int(self.args.offset)
        rows, cols = self.config["rows"], self.config["cols"]
        if rows is not None and cols is not None and (sx, sy) == (rows + 1, cols + 1):
            return 1
        return 0

    def check_size(self, frame):
        size = (frame["sx"], frame["sy"])
        if size == self.size:
            return
        self.size = size
        sx, sy = size
        print(f"Frame size is size_x={sx}, size_y={sy}.")
        if self.config:
            rows, cols = self.config["rows"], self.config["cols"]
            self.offset = o = self.pick_offset(sx, sy)
            if o:
                print(f"  config is {rows} x {cols}, so config (x, y) is frame [x+{o}][y+{o}] "
                      f"(row 0 and col 0 skipped, like the C++ does)")
            else:
                print(f"  config is {rows} x {cols}, so config (x, y) is frame [x][y]")
            if rows is not None and cols is not None and size != (rows + o, cols + o):
                print("  -> the frame size does not match the config; patch cells outside "
                      "the frame show as ?")
        rows, cols = self.args.cpp_rows, self.args.cpp_cols
        print(f"  AcquisitionNode.cpp buffer is [{rows}][{cols}] and it publishes rows 1-{rows - 1}, "
              f"cols 1-{cols - 1} (row 0 and col 0 are dropped).")
        if sx > rows or sy > cols:
            print("  -> the frame does not fit the C++ buffer, so the C++ writes out of bounds")
        elif size != (rows, cols):
            print("  -> the frame is smaller than the C++ buffer, so the published image includes "
                  "cells the sensor never writes")

    def check_number(self, frame):
        number, last = frame["number"], self.last_number
        self.last_number = number
        if last is not None and number not in (last, (last + 1) & 0xFFFFFFFF):
            print(f"  frame number jumped {last} -> {number}")

    def compare_cpp(self, frame, comparable):
        """Returns (message, is_problem) about what the C++ made of this frame."""
        done, self.cpp_done = self.cpp_done, None
        if done is None:
            self.cpp_missed += 1
            return ("C++: did not complete this frame, so the image it publishes "
                    "mixes old and new values"), True
        if not comparable:
            return None, False
        if (done["sx"], done["sy"]) != (frame["sx"], frame["sy"]):
            self.cpp_mismatches += 1
            return (f"C++: read the size as {done['sx']} x {done['sy']} "
                    f"instead of {frame['sx']} x {frame['sy']}"), True
        rows, cols = self.args.cpp_rows, self.args.cpp_cols
        diffs = [(x, y, done["memory"][x * cols + y], frame["matrix"][x][y])
                 for x in range(min(frame["sx"], rows))
                 for y in range(min(frame["sy"], cols))
                 if done["memory"][x * cols + y] != frame["matrix"][x][y]]
        if diffs:
            self.cpp_mismatches += 1
            examples = ", ".join(f"[{x}][{y}] C++={c} real={r}" for x, y, c, r in diffs[:5])
            return f"C++: {len(diffs)} cells differ, e.g. {examples}", True
        if frame["sx"] > rows or frame["sy"] > cols:
            return "C++: the cells that fit in its buffer match, the rest went out of bounds", True
        return "C++: same values", False

    def counts(self):
        return (f"ok {self.ok}, malformed {self.malformed}, dropped {self.dropped}, "
                f"junk bytes {self.ref.junk} | C++: mismatches {self.cpp_mismatches}, "
                f"missed {self.cpp_missed}, resets {self.cpp.aborts}, "
                f"early ends {self.cpp.early_ends}, out-of-bounds writes {self.cpp.out_of_bounds}")

    def print_stats(self):
        now = time.monotonic()
        dt = now - self.last_stats
        if dt < 1:
            return
        frames = self.ok + self.malformed
        db, df = self.bytes - self.last_counts[0], frames - self.last_counts[1]
        print(f"[{db / dt:.0f} B/s, {df / dt:.1f} frames/s] {self.counts()}")
        self.last_stats, self.last_counts = now, (self.bytes, frames)

    def summary(self, elapsed):
        print(f"\n{self.bytes} bytes in {elapsed:.1f} s. {self.counts()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mode", nargs="?", choices=("frames", "raw"), default="frames")
    parser.add_argument("--config", metavar="YAML",
                        help="ROS config file with device, baudrate, rows, cols and patches")
    parser.add_argument("--port", help=f"overrides the config's device (default {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int,
                        help=f"overrides the config's baudrate (default {DEFAULT_BAUD}); "
                             "USB CDC devices like the XIAO ignore it")
    parser.add_argument("--patch", action="append", metavar="NAME",
                        help="only print this patch from the config (repeatable; default: all)")
    parser.add_argument("--offset", choices=("auto", "0", "1"), default="auto",
                        help="added to the config's src x and y to index the frame; auto uses 1 "
                             "when the frame is one row and one col bigger than the config")
    parser.add_argument("--save", metavar="FILE", help="also write every received byte to FILE")
    parser.add_argument("--replay", metavar="FILE",
                        help="read bytes from FILE (made with --save) instead of the port")
    parser.add_argument("--every", type=int, default=1, metavar="N",
                        help="print the matrix of every Nth frame (default 1)")
    parser.add_argument("--cpp-rows", type=int, default=7,
                        help="first dimension of acquisition_matrix in AcquisitionNode.cpp (rows_ + 1)")
    parser.add_argument("--cpp-cols", type=int, default=23,
                        help="second dimension of acquisition_matrix in AcquisitionNode.cpp (cols_ + 1)")
    args = parser.parse_args()

    config = load_config(args.config) if args.config else None
    if config:
        args.port = args.port or config["device"]
        args.baud = args.baud or config["baudrate"]
    args.port = args.port or DEFAULT_PORT
    args.baud = args.baud or DEFAULT_BAUD

    patches = config["patches"] if config else []
    if args.patch:
        names = [p["name"] for p in patches]
        unknown = [n for n in args.patch if n not in names]
        if unknown:
            sys.exit(f"Unknown patch {', '.join(unknown)}; the config has: "
                     f"{', '.join(names) or 'no patches'}")
        patches = [p for p in patches if p["name"] in args.patch]

    debugger = RawDump() if args.mode == "raw" else FrameDebugger(args, config, patches)
    save = open(args.save, "wb") if args.save else None
    start = last_data = time.monotonic()
    warned = False
    try:
        for chunk in read_chunks(args):
            now = time.monotonic()
            if not chunk:
                if not warned and now - last_data > 2:
                    print("(no data for 2 s)")
                    warned = True
                continue
            last_data, warned = now, False
            if save:
                save.write(chunk)
            debugger.feed(chunk)
    except KeyboardInterrupt:
        pass
    except OSError as e:
        print(f"Serial error: {e}")
    finally:
        if save:
            save.close()
    debugger.summary(time.monotonic() - start)


if __name__ == "__main__":
    main()
