#!/usr/bin/env python3
"""
flourish.py
Converts a TinyDB file into a Flourish-compatible CSV.
"""

import os
import pandas as pd
import sys
from tinydb import TinyDB


def tinydb_to_flourish(db_path: str, output_path: str = "flourish_data.csv") -> None:
    if not os.path.exists(db_path):
        print(f"[ERROR] Database file not found: {db_path}")
        sys.exit(1)

    db = TinyDB(db_path)
    records = db.all()
    if not records:
        print("[WARN] No records found in database.")
        sys.exit(0)

    df = pd.DataFrame(records)

    # Flatten nested structure if necessary
    if "_default" in df.columns:
        df = pd.json_normalize(df["_default"])

    # Optional: clean timestamps for Flourish readability
    if "timestamp" in df.columns:
        df["timestamp"] = df["timestamp"].str.replace(" ", "T", n=1)
        df["timestamp"] = pd.to_datetime(df["timestamp"], errors="coerce")

    # Reorder columns for clarity
    cols = [
        "prototype_name",
        "iteration",
        "distance",
        "tangential_speed",
        "angle_deg",
        "hit",
        "timestamp",
    ]
    df = df[[c for c in cols if c in df.columns]]

    # Export to CSV
    df.to_csv(output_path, index=False)
    print(f"[OK] Exported {len(df)} records to {output_path}")

if __name__ == "__main__":
    db_file = input("Enter TinyDB file path (default: trajectory_db.json): ").strip() or "trajectory_db.json"
    tinydb_to_flourish(db_file)
