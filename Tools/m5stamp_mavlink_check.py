#!/usr/bin/env python3

import argparse
import time
from collections import Counter

from pymavlink import mavutil


WATCHED_TYPES = {
    "HEARTBEAT",
    "DISTANCE_SENSOR",
    "STATUSTEXT",
    "RC_CHANNELS",
    "EKF_STATUS_REPORT",
    "OPTICAL_FLOW",
    "OPTICAL_FLOW_RAD",
}


def main() -> int:
    parser = argparse.ArgumentParser(description="Check M5Stamp Fly MAVLink output")
    parser.add_argument("--connect", default="udpin:0.0.0.0:14550")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument(
        "--prearm-check",
        action="store_true",
        help="Run pre-arm checks without arming the vehicle",
    )
    args = parser.parse_args()

    link = mavutil.mavlink_connection(args.connect, autoreconnect=True)
    print(f"Listening on {args.connect}", flush=True)

    if args.connect.startswith(("udpout:", "udp:")):
        link.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
        print("Sent GCS heartbeat to establish the UDP return path", flush=True)

    heartbeat = link.wait_heartbeat(timeout=15)
    if heartbeat is None:
        print("ERROR: HEARTBEAT timeout", flush=True)
        return 1

    print(
        f"HEARTBEAT sys={link.target_system} comp={link.target_component} "
        f"type={heartbeat.type} autopilot={heartbeat.autopilot}",
        flush=True,
    )
    link.mav.request_data_stream_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        10,
        1,
    )

    if args.prearm_check:
        link.mav.command_long_send(
            link.target_system,
            link.target_component,
            mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        print("Requested pre-arm checks (vehicle remains disarmed)", flush=True)

    counts = Counter({"HEARTBEAT": 1})
    distances = []
    rangefinder_texts = []
    end_time = time.monotonic() + args.duration

    while time.monotonic() < end_time:
        message = link.recv_match(blocking=True, timeout=1)
        if message is None:
            continue

        message_type = message.get_type()
        if message_type not in WATCHED_TYPES:
            continue
        counts[message_type] += 1

        if message_type == "DISTANCE_SENSOR":
            current = int(message.current_distance)
            distances.append(current)
            if len(distances) <= 20:
                print(
                    f"DISTANCE_SENSOR current={current}cm "
                    f"min={message.min_distance}cm max={message.max_distance}cm "
                    f"orientation={message.orientation}",
                    flush=True,
                )
        elif message_type == "STATUSTEXT":
            text = str(message.text)
            text_lower = text.lower()
            if (
                "rangefinder" in text_lower
                or "vl53" in text_lower
                or "prearm" in text_lower
                or "arm:" in text_lower
            ):
                rangefinder_texts.append(text)
                print(f"STATUSTEXT {text}", flush=True)

    print("\nMessage counts:")
    for message_type in sorted(WATCHED_TYPES):
        print(f"  {message_type}: {counts[message_type]}")

    if distances:
        print(
            f"DISTANCE_SENSOR samples={len(distances)} "
            f"min={min(distances)}cm max={max(distances)}cm "
            f"changed={len(set(distances)) > 1}"
        )
    else:
        print("DISTANCE_SENSOR samples=0")

    no_data = [text for text in rangefinder_texts if "no data" in text.lower()]
    print(f"Rangefinder No Data STATUSTEXT count={len(no_data)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
