#!/usr/bin/env python3

import argparse
import csv
import math
import time
from pathlib import Path

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_VELOCITY_CONTROL,
    INPUT_MODE_VEL_RAMP,
)
from odrive.utils import dump_errors


def get_axis(odrv, axis_index):
    if axis_index == 0:
        return odrv.axis0
    if axis_index == 1:
        return odrv.axis1
    raise ValueError("axis must be 0 or 1")


def read_error_snapshot(axis):
    """Read ODrive error fields as integer values."""
    try:
        return {
            "axis_error": int(axis.error),
            "motor_error": int(axis.motor.error),
            "encoder_error": int(axis.encoder.error),
            "controller_error": int(axis.controller.error),
        }
    except Exception:
        return {
            "axis_error": -1,
            "motor_error": -1,
            "encoder_error": -1,
            "controller_error": -1,
        }


def make_profile(max_vel, hold_time, zero_time):
    """
    Velocity profile in turns/s.

    Sequence:
      0 -> +max_vel -> 0 -> -max_vel -> 0
    """
    return [
        (zero_time, 0.0),
        (hold_time, +max_vel),
        (zero_time, 0.0),
        (hold_time, -max_vel),
        (zero_time, 0.0),
    ]


def configure_axis_for_velocity_control(axis, vel_ramp_rate=None):
    """Set basic velocity control mode. Keeps existing gains unless explicitly changed elsewhere."""
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP

    if vel_ramp_rate is not None:
        axis.controller.config.vel_ramp_rate = float(vel_ramp_rate)


def wait_short(duration_sec):
    end = time.monotonic() + duration_sec
    while time.monotonic() < end:
        time.sleep(0.01)


def run_test(
    odrv,
    axis,
    axis_index,
    bandwidths,
    profile,
    sample_hz,
    output_csv,
    vel_ramp_rate=None,
    settle_time=0.5,
):
    dt = 1.0 / sample_hz
    rows = []

    print(f"Using axis{axis_index}")
    print(f"vbus_voltage = {odrv.vbus_voltage:.2f} V")

    odrv.clear_errors()
    wait_short(0.2)

    configure_axis_for_velocity_control(axis, vel_ramp_rate=vel_ramp_rate)

    # Enter closed loop once before the test.
    axis.controller.input_vel = 0.0
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    wait_short(0.5)

    for bw in bandwidths:
        print("\n" + "=" * 60)
        print(f"Testing encoder bandwidth = {bw}")
        print("=" * 60)

        odrv.clear_errors()
        wait_short(0.2)

        axis.controller.input_vel = 0.0
        wait_short(settle_time)

        # Set encoder bandwidth.
        axis.encoder.config.bandwidth = float(bw)
        wait_short(settle_time)

        # Re-enter closed loop after changing estimator setting.
        axis.controller.input_vel = 0.0
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        wait_short(settle_time)

        actual_bw = float(axis.encoder.config.bandwidth)
        print(f"Actual bandwidth = {actual_bw}")

        test_start = time.monotonic()
        segment_index = 0

        for duration, cmd_vel in profile:
            segment_index += 1
            print(f"segment {segment_index}: cmd_vel={cmd_vel:.3f} turns/s, duration={duration:.2f}s")

            segment_start = time.monotonic()
            axis.controller.input_vel = float(cmd_vel)

            next_sample_time = time.monotonic()

            while True:
                now = time.monotonic()
                if now - segment_start >= duration:
                    break

                if now < next_sample_time:
                    time.sleep(min(0.001, next_sample_time - now))
                    continue

                t = now - test_start
                err = read_error_snapshot(axis)

                try:
                    row = {
                        "bandwidth": actual_bw,
                        "axis_index": axis_index,
                        "segment_index": segment_index,
                        "t": t,
                        "cmd_vel": float(cmd_vel),
                        "input_vel": float(axis.controller.input_vel),
                        "vel_estimate": float(axis.encoder.vel_estimate),
                        "pos_estimate": float(axis.encoder.pos_estimate),
                        "iq_measured": float(axis.motor.current_control.Iq_measured),
                        "iq_setpoint": float(axis.motor.current_control.Iq_setpoint),
                        "vbus_voltage": float(odrv.vbus_voltage),
                        **err,
                    }
                except Exception as e:
                    print(f"Read failed at t={t:.3f}: {e}")
                    row = {
                        "bandwidth": actual_bw,
                        "axis_index": axis_index,
                        "segment_index": segment_index,
                        "t": t,
                        "cmd_vel": float(cmd_vel),
                        "input_vel": math.nan,
                        "vel_estimate": math.nan,
                        "pos_estimate": math.nan,
                        "iq_measured": math.nan,
                        "iq_setpoint": math.nan,
                        "vbus_voltage": math.nan,
                        "axis_error": -2,
                        "motor_error": -2,
                        "encoder_error": -2,
                        "controller_error": -2,
                    }

                rows.append(row)
                next_sample_time += dt

        axis.controller.input_vel = 0.0
        wait_short(1.0)

        print("Errors after this bandwidth test:")
        dump_errors(odrv)

    axis.controller.input_vel = 0.0
    wait_short(0.2)
    axis.requested_state = AXIS_STATE_IDLE

    fieldnames = [
        "bandwidth",
        "axis_index",
        "segment_index",
        "t",
        "cmd_vel",
        "input_vel",
        "vel_estimate",
        "pos_estimate",
        "iq_measured",
        "iq_setpoint",
        "vbus_voltage",
        "axis_error",
        "motor_error",
        "encoder_error",
        "controller_error",
    ]

    output_csv = Path(output_csv)
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nSaved CSV: {output_csv}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="ODrive encoder bandwidth response test"
    )

    parser.add_argument(
        "--axis",
        type=int,
        default=1,
        choices=[0, 1],
        help="ODrive axis index to test. Default: 1",
    )

    parser.add_argument(
        "--bandwidths",
        type=float,
        nargs="+",
        default=[100.0, 300.0, 500.0, 750.0, 1000.0, 1500.0],
        help="Encoder bandwidth values to test.",
    )

    parser.add_argument(
        "--max-vel",
        type=float,
        default=2.0,
        help="Command velocity magnitude in turns/s. Default: 2.0",
    )

    parser.add_argument(
        "--hold-time",
        type=float,
        default=5.0,
        help="Duration for each nonzero velocity segment [s]. Default: 2.0",
    )

    parser.add_argument(
        "--zero-time",
        type=float,
        default=5.0,
        help="Duration for each zero velocity segment [s]. Default: 1.0",
    )

    parser.add_argument(
        "--sample-hz",
        type=float,
        default=100.0,
        help="Sampling frequency [Hz]. Default: 100",
    )

    parser.add_argument(
        "--vel-ramp-rate",
        type=float,
        default=25,
        help="Optional vel_ramp_rate override. If omitted, current ODrive value is used.",
    )

    parser.add_argument(
        "--output",
        type=str,
        default="odrive_encoder_bandwidth_test.csv",
        help="Output CSV path.",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    print("WARNING: Lift the wheel off the ground before running this test.")
    print("Press Ctrl+C within 3 seconds to cancel.")
    time.sleep(3.0)

    profile = make_profile(
        max_vel=args.max_vel,
        hold_time=args.hold_time,
        zero_time=args.zero_time,
    )

    print("Connecting to ODrive...")
    odrv = odrive.find_any(timeout=10)

    if odrv is None:
        raise RuntimeError("ODrive not found")

    axis = get_axis(odrv, args.axis)

    try:
        run_test(
            odrv=odrv,
            axis=axis,
            axis_index=args.axis,
            bandwidths=args.bandwidths,
            profile=profile,
            sample_hz=args.sample_hz,
            output_csv=args.output,
            vel_ramp_rate=args.vel_ramp_rate,
        )

    except KeyboardInterrupt:
        print("\nInterrupted. Stopping motor...")
        try:
            axis.controller.input_vel = 0.0
            time.sleep(0.2)
            axis.requested_state = AXIS_STATE_IDLE
        except Exception as e:
            print(f"Failed to stop cleanly: {e}")
        raise

    except Exception as e:
        print(f"\nTest failed: {e}")
        try:
            axis.controller.input_vel = 0.0
            time.sleep(0.2)
            axis.requested_state = AXIS_STATE_IDLE
        except Exception as stop_error:
            print(f"Failed to stop cleanly: {stop_error}")
        raise


if __name__ == "__main__":
    main()