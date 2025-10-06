#!/usr/bin/env python3
"""
Interactive point changer.
Prompts:
  Name
  Point change (int)
  Point class (blank -> regular)
  Give To (blank -> none)
"""
from getpass import getpass
from typing import Dict

from utils import read_db, write_db


def ensure_person(db: Dict, name: str):
    if name not in db:
        db[name] = {}

def main():
    pw = getpass("Enter master password: ").encode()
    try:
        db, salt = read_db(pw)
    except Exception as e:
        print("Failed to open DB:", e)
        return

    name = input("Name: ").strip()
    if not name:
        print("Name required.")
        return
    try:
        change = int(input("Point change: ").strip())
    except ValueError:
        print("Point change must be integer.")
        return
    pclass = input("Point class (or blank for regular): ").strip() or "regular"
    recipient = input(f"{'Take them from' if change>0 else 'Give these to'} (or blank): ").strip() or None

    ensure_person(db, name)
    db[name].setdefault(pclass, 0)
    db[name][pclass] += change

    action = f"{'Added' if change>0 else 'Took'} {abs(change)} {pclass} point(s) {'from '+name if change<0 else 'for '+name}"
    if recipient:
        ensure_person(db, recipient)
        db[recipient].setdefault(pclass, 0)
        db[recipient][pclass] -= change  # transfer
        action = f"Took {abs(change)} {pclass} point(s) from {name} and gave to {recipient}" if change<0 else f"Transferred {change} {pclass} point(s) from {name} to {recipient}"

    write_db(db, pw, salt)
    print(action)

if __name__ == "__main__":
    main()
