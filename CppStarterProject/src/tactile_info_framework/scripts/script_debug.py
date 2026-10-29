#!/usr/bin/env python3
"""Debug the tactile sensor's serial stream without ROS 2.

Frame format that AcquisitionNode.cpp expects:

    DLE STX | frame number (4 bytes, little endian) | size_x | size_y |
    size_x * size_y values, x changing fastest | DLE ETX

DLE = 0x10, STX = 0x02, ETX = 0x03. A 0x10 inside the frame is sent twice.

  raw     hex dump of every byte received
  frames  decode frames, print the matrices, and run a copy of the C++
          state machine on the same bytes to show where it disagrees

Examples:
  python3 serial_debug.py raw
  python3 serial_debug.py frames --save dump.bin
  python3 serial_debug.py frames --replay dump.bin
"""

import argparse
import sys
import time

DLE, STX, ETX = 0x10, 0x02, 0x03

DEFAULT_PORT = "/dev/serial/by-id/usb-Seeed_XIAO_nRF52840_Sense_56AAAD9B81683EF9-if00"


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
        return None, [f"payload is only {len(payload)} bytes, shorter than the 6-byte hader"]
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


def print_matrix(frame):
    print("      y" + "".join(f"{y:4d}" for y in range(frame["sy"])))
    for x, row in enumerate(frame["matrix"]):
        print(f"  x{x:3d} " + "".join("   ?" if v is None else f"{v:4d}" for v in row))


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
    def __init__(self, args):
        self.args = args
        self.ref = FrameParser()
        self.cpp = CppParser(args.rows, args.cols)
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
            print_matrix(frame)
        for p in problems:
            print(f"  PROBLEM: {p}")
        if cpp_msg and (show or cpp_bad):
            print(f"  {cpp_msg}")

    def check_size(self, frame):
        size = (frame["sx"], frame["sy"])
        if size == self.size:
            return
        self.size = size
        sx, sy = size
        rows, cols = self.args.rows, self.args.cols
        print(f"Frame size is size_x={sx}, size_y={sy}. The C++ buffer is [{rows}][{cols}] and it "
              f"publishes rows 1-{rows - 1}, cols 1-{cols - 1} (row 0 and col 0 are dropped).")
        if sx > rows or sy > cols:
            print("  -> the frame does not fit the C++ buffer, so the C++ writes out of bounds")
        elif size != (rows, cols):
            print("  -> the frame is smaller than the buffer, so the published image includes "
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
        rows, cols = self.args.rows, self.args.cols
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
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=921600,
                        help="same as the C++; USB CDC devices like the XIAO ignore it")
    parser.add_argument("--save", metavar="FILE", help="also write every received byte to FILE")
    parser.add_argument("--replay", metavar="FILE",
                        help="read bytes from FILE (made with --save) instead of the port")
    parser.add_argument("--every", type=int, default=1, metavar="N",
                        help="print the matrix of every Nth frame (default 1)")
    parser.add_argument("--rows", type=int, default=7,
                        help="first dimension of acquisition_matrix in the C++ (rows_ + 1)")
    parser.add_argument("--cols", type=int, default=23,
                        help="second dimension of acquisition_matrix in the C++ (cols_ + 1)")
    args = parser.parse_args()

    debugger = RawDump() if args.mode == "raw" else FrameDebugger(args)
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
