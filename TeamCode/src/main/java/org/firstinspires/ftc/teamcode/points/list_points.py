#!/usr/bin/env python3
"""
List points in simple table.
"""
from getpass import getpass

from utils import read_db


def main():
    pw = getpass("Enter master password: ").encode()
    try:
        db, _ = read_db(pw)
    except Exception as e:
        print("Failed to open DB:", e)
        return
    # Print sorted
    for name in sorted(db.keys()):
        classes = db[name]
        parts = [f"{cls}:{count}" for cls, count in sorted(classes.items())]
        print(f"{name}\t" + " | ".join(parts))

if __name__ == "__main__":
    main()
