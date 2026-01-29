#!/usr/bin/env python3
"""
trajectory_collectory.py
WebSocket listener for FTC custom dashboard packets (wss://192.168.43.1:8001/).
Prints debug info. Auto-saves normalized records to TinyDB when required fields found.

Record schema (TinyDB):
{
  "distance": float,            # meters
  "tangential_speed": float,    # m/s
  "angle_deg": float,           # degrees
  "hit": bool,
  "timestamp": "ISO8601",
  "raw": <truncated raw packet>
}
"""
import argparse
import asyncio
import datetime
import json
import ssl

import websockets
from tinydb import TinyDB

# CONFIG
DEFAULT_WS = "wss://192.168.43.1:8001/"
DEFAULT_DB = "trajectory_db.json"
TRUNCATE_RAW = 800  # chars to keep of raw packet for storage

# Candidate key sets (lowercase)
DIST_KEYS = {"distance", "dist", "range", "r", "distance_m", "dist_m", "distanceMeters"}
ANGLE_KEYS = {
    "angle",
    "angle_deg",
    "theta",
    "theta_deg",
    "pitch",
    "elevation",
    "servo_angle",
}
SPEED_KEYS = {
    "speed",
    "v",
    "velocity",
    "tangential_speed",
    "exit_speed",
    "exitVelocity",
    "exit_speed_m_s",
}
RPM_KEYS = {"rpm", "motor_rpm", "measured_rpm", "wheel_rpm", "shooter_rpm"}
HIT_KEYS = {"hit", "success", "goal", "in", "scored", "made", "is_goal"}


def find_key_recursive(obj, keyset):
    """Search dict/list recursively for the first matching key in keyset. Return value or None."""
    if isinstance(obj, dict):
        for k, v in obj.items():
            lk = str(k).lower()
            if lk in keyset:
                return v
            # recurse
            res = find_key_recursive(v, keyset)
            if res is not None:
                return res
    elif isinstance(obj, list):
        for item in obj:
            res = find_key_recursive(item, keyset)
            if res is not None:
                return res
    return None


def parse_bool(val):
    if isinstance(val, bool):
        return val
    if val is None:
        return None
    s = str(val).strip().lower()
    if s in {"1", "true", "t", "yes", "y", "made", "scored", "hit"}:
        return True
    if s in {"0", "false", "f", "no", "n", "miss", "false"}:
        return False
    return None


def parse_float(val):
    if val is None:
        return None
    try:
        return float(val)
    except Exception:
        return None


def rpm_to_speed(rpm, wheel_diameter_m):
    # v = pi * d * rpm / 60
    if rpm is None or wheel_diameter_m is None:
        return None
    try:
        return (3.141592653589793 * float(wheel_diameter_m) * float(rpm)) / 60.0
    except Exception:
        return None


def save_trajectory(
    distance, tangential_speed, angle_deg, hit, db_path=DEFAULT_DB, raw_packet=None
):
    """Validate inputs, normalize, and save to TinyDB. Returns inserted doc id or None."""
    d = parse_float(distance)
    s = parse_float(tangential_speed)
    a = parse_float(angle_deg)
    h = parse_bool(hit)
    if d is None or s is None or a is None or h is None:
        print(
            "[SAVE SKIP] missing/invalid field(s):",
            f"distance={d}, speed={s}, angle={a}, hit={h}",
        )
        return None
    rec = {
        "distance": d,
        "tangential_speed": s,
        "angle_deg": a,
        "hit": bool(h),
        "timestamp": datetime.datetime.utcnow().isoformat() + "Z",
    }
    if raw_packet is not None:
        rec["raw"] = raw_packet[:TRUNCATE_RAW]
    db = TinyDB(db_path)
    doc_id = db.insert(rec)
    print(
        f"[SAVED] doc_id={doc_id} distance={d:.3f}m speed={s:.3f}m/s angle={a:.2f}° hit={h}"
    )
    return doc_id


async def packet_listener(ws_url, db_path, wheel_diameter_m, autosave=True):
    ssl_context = ssl._create_unverified_context()
    try:
        async with websockets.connect(
            ws_url, ssl=ssl_context, ping_interval=None
        ) as ws:
            print(f"[CONNECT] {ws_url}")
            async for msg in ws:
                ts = datetime.datetime.utcnow().isoformat() + "Z"
                print("\n" + "=" * 80)
                print(f"[{ts}] RAW PACKET (len={len(msg)})")
                print(msg[:1000])  # show first 1000 chars
                parsed = None
                try:
                    parsed = json.loads(msg)
                    print(
                        "[PARSED JSON] keys top-level:",
                        (
                            list(parsed.keys())
                            if isinstance(parsed, dict)
                            else type(parsed)
                        ),
                    )
                except Exception as e:
                    print("[PARSE FAIL]", e)

                # Extract candidates
                distance = None
                angle = None
                speed = None
                rpm = None
                hit = None

                if parsed is not None:
                    distance = find_key_recursive(parsed, DIST_KEYS)
                    angle = find_key_recursive(parsed, ANGLE_KEYS)
                    speed = find_key_recursive(parsed, SPEED_KEYS)
                    rpm = find_key_recursive(parsed, RPM_KEYS)
                    hit = find_key_recursive(parsed, HIT_KEYS)

                    # debug print of what we found
                    print(
                        "[EXTRACT] distance:",
                        distance,
                        " angle:",
                        angle,
                        " speed:",
                        speed,
                        " rpm:",
                        rpm,
                        " hit:",
                        hit,
                    )

                # compute tangential_speed if only rpm present and wheel_diameter provided
                tangential_speed = parse_float(speed)
                if (
                    tangential_speed is None
                    and rpm is not None
                    and wheel_diameter_m is not None
                ):
                    tangential_speed = rpm_to_speed(parse_float(rpm), wheel_diameter_m)
                    if tangential_speed is not None:
                        print(
                            f"[DERIVED] tangential_speed from rpm={rpm} -> {tangential_speed:.3f} m/s (wheel d={wheel_diameter_m} m)"
                        )

                # attempt auto-save if we have all required fields
                if autosave:
                    if (
                        distance is not None
                        and tangential_speed is not None
                        and angle is not None
                        and hit is not None
                    ):
                        save_trajectory(
                            distance,
                            tangential_speed,
                            angle,
                            hit,
                            db_path=db_path,
                            raw_packet=msg,
                        )
                    else:
                        print(
                            "[AUTO-SAVE SKIPPED] missing fields; inspect packet and map keys manually."
                        )
                else:
                    print("[AUTO-SAVE DISABLED]")

    except Exception as e:
        print("[ERROR] websocket listener failed:", repr(e))


def main():
    p = argparse.ArgumentParser(prog="trajectory_collectory.py")
    p.add_argument("--ws", default=DEFAULT_WS, help="WebSocket URL (wss://...)")
    p.add_argument("--db", default=DEFAULT_DB, help="TinyDB JSON path")
    p.add_argument(
        "--wheel-diameter",
        type=float,
        default=None,
        help="Wheel diameter (meters) to derive speed from RPM",
    )
    p.add_argument(
        "--no-autosave",
        action="store_true",
        help="Do not automatically save detected records",
    )
    args = p.parse_args()

    try:
        asyncio.run(
            packet_listener(
                args.ws, args.db, args.wheel_diameter, autosave=not args.no_autosave
            )
        )
    except KeyboardInterrupt:
        print("\n[STOP] interrupted by user")


if __name__ == "__main__":
    main()
