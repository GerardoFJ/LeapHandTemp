import numpy as np
import time
import csv
from datetime import datetime

from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu


# ============================================================
# LEAP HAND CYCLIC MOVEMENT AND CURRENT MONITORING TEST
# ============================================================

class LeapNode:

    def __init__(self):

        # Controller parameters
        self.kP = 600
        self.kI = 0
        self.kD = 200

        # Current limit in the units expected by the API.
        # Original example uses 550 for the full-motor hand.
        self.curr_lim = 550

        self.prev_pos = self.pos = self.curr_pos = (
            lhu.allegro_to_LEAPhand(np.zeros(16))
        )

        self.motors = list(range(16))

        # Connect to the hand
        ports = [
            ('/dev/ttyUSB0', 1000000),
            ('/dev/ttyUSB1', 1000000),
        ]

        connected = False

        for port, baudrate in ports:
            try:
                print("Trying to connect to", port)

                self.dxl_client = DynamixelClient(
                    self.motors,
                    port,
                    baudrate
                )

                self.dxl_client.connect()
                connected = True

                print("Connected to", port)
                break

            except Exception as e:
                print("Connection failed:", e)

        if not connected:
            raise RuntimeError(
                "Could not connect to the LEAP Hand. "
                "Check the USB port and permissions."
            )

        # Position-current control mode
        self.dxl_client.sync_write(
            self.motors,
            np.ones(16) * 5,
            11,
            1
        )

        self.dxl_client.set_torque_enabled(
            self.motors,
            True
        )

        # Position controller gains
        self.dxl_client.sync_write(
            self.motors,
            np.ones(16) * self.kP,
            84,
            2
        )

        self.dxl_client.sync_write(
            [0, 4, 8],
            np.ones(3) * (self.kP * 0.75),
            84,
            2
        )

        self.dxl_client.sync_write(
            self.motors,
            np.ones(16) * self.kI,
            82,
            2
        )

        self.dxl_client.sync_write(
            self.motors,
            np.ones(16) * self.kD,
            80,
            2
        )

        self.dxl_client.sync_write(
            [0, 4, 8],
            np.ones(3) * (self.kD * 0.75),
            80,
            2
        )

        # Motor current limit
        self.dxl_client.sync_write(
            self.motors,
            np.ones(16) * self.curr_lim,
            102,
            2
        )

        self.dxl_client.write_desired_pos(
            self.motors,
            self.curr_pos
        )

    # --------------------------------------------------------
    # Command the hand
    # --------------------------------------------------------

    def set_allegro(self, pose):

        pose = lhu.allegro_to_LEAPhand(
            np.array(pose),
            zeros=False
        )

        self.prev_pos = self.curr_pos
        self.curr_pos = np.array(pose)

        self.dxl_client.write_desired_pos(
            self.motors,
            self.curr_pos
        )

    # --------------------------------------------------------
    # Read motor data
    # --------------------------------------------------------

    def read_pos(self):
        return self.dxl_client.read_pos()

    def read_cur(self):
        return self.dxl_client.read_cur()

    def read_vel(self):
        return self.dxl_client.read_vel()

    def pos_vel_eff_srv(self):
        return self.dxl_client.read_pos_vel_cur()


# ============================================================
# TEST CONFIGURATION
# ============================================================

TEST_DURATION = 120       # Total test duration, seconds
SAMPLE_RATE = 10          # Logging rate, Hz

# Time spent at each commanded position
OPEN_HOLD = 0.5
PARTIAL_HOLD = 0.5
CURL_HOLD = 0.5

# Joint positions in Allegro-compatible radians.
# Start with modest movement and increase only after validation.
OPEN_POSE = np.zeros(16)

PARTIAL_POSE = np.ones(16) * 0.30

CURL_POSE = np.ones(16) * 0.60


# ============================================================
# DATA LOGGING
# ============================================================

def log_measurement(writer, hand, start_time, cycle, phase):

    elapsed = time.monotonic() - start_time

    # Read joint position and motor current
    position = np.asarray(hand.read_pos()).flatten()
    current = np.asarray(hand.read_cur()).flatten()

    if len(position) != 16 or len(current) != 16:
        raise RuntimeError(
            "Unexpected number of position/current values."
        )

    writer.writerow(
        [elapsed, cycle, phase]
        + position.tolist()
        + current.tolist()
    )

    print(
        "t={:7.2f}s | cycle={:4d} | {:7s} | "
        "mean_abs_current={:.2f} | max_abs_current={:.2f}".format(
            elapsed,
            cycle,
            phase,
            np.mean(np.abs(current)),
            np.max(np.abs(current))
        )
    )

    return current


# ============================================================
# MAIN TEST
# ============================================================

def main():

    hand = LeapNode()

    filename = "leaphand_test_{}.csv".format(
        datetime.now().strftime("%Y%m%d_%H%M%S")
    )

    sample_period = 1.0 / SAMPLE_RATE

    print("\n" + "=" * 60)
    print("LEAP HAND CYCLIC CURRENT TEST")
    print("=" * 60)
    print("Duration:", TEST_DURATION, "seconds")
    print("Sampling rate:", SAMPLE_RATE, "Hz")
    print("CSV output:", filename)
    print("\nKeep the hand unobstructed.")
    print("Press Ctrl+C to stop the test.\n")

    time.sleep(2)

    start_time = time.monotonic()
    next_sample = start_time

    cycle = 0

    try:

        with open(filename, "w", newline="") as csvfile:

            writer = csv.writer(csvfile)

            header = (
                ["elapsed_s", "cycle", "phase"]
                + [
                    "position_{}".format(i)
                    for i in range(16)
                ]
                + [
                    "current_{}".format(i)
                    for i in range(16)
                ]
            )

            writer.writerow(header)

            # Each phase is held for the configured duration.
            phases = [
                ("open", OPEN_POSE, OPEN_HOLD),
                ("partial", PARTIAL_POSE, PARTIAL_HOLD),
                ("curl", CURL_POSE, CURL_HOLD),
                ("open", OPEN_POSE, OPEN_HOLD),
            ]

            while time.monotonic() - start_time < TEST_DURATION:

                cycle += 1

                for phase, pose, hold_time in phases:

                    if time.monotonic() - start_time >= TEST_DURATION:
                        break

                    # Send the position command once per phase
                    hand.set_allegro(pose)

                    print(
                        "\nCycle {}: {}".format(
                            cycle,
                            phase.upper()
                        )
                    )

                    phase_start = time.monotonic()

                    while time.monotonic() - phase_start < hold_time:

                        elapsed = time.monotonic() - start_time

                        if elapsed >= TEST_DURATION:
                            break

                        now = time.monotonic()

                        if now >= next_sample:

                            log_measurement(
                                writer,
                                hand,
                                start_time,
                                cycle,
                                phase
                            )

                            csvfile.flush()

                            next_sample = now + sample_period

                        time.sleep(0.005)

    except KeyboardInterrupt:

        print("\nTest interrupted by user.")

    finally:

        # Attempt to leave the hand in its open position.
        try:
            print("\nReturning hand to open pose...")
            hand.set_allegro(OPEN_POSE)
            time.sleep(1)

        except Exception as e:
            print("Could not command the open pose:", e)

        print("Test finished.")
        print("Measurements saved to:", filename)


if __name__ == "__main__":
    main()
