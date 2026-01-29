#!/usr/bin/env python3
"""
trajectory_manual_input.py

A rapid, keybind-driven command-line tool to log projectile trajectory data.

This script captures single key presses for immediate action, allowing for fast
data logging. It displays a live history of the last 5 shots and allows
any of them to be undone.
"""

import datetime
import math
import os
import sys
import time
from collections import deque
from tinydb import TinyDB, Query

# --- Third-Party Libraries ---
try:
    import readchar
except ImportError:
    print(
        "[ERROR] The 'readchar' library is required. Please install it using: pip install readchar"
    )
    sys.exit(1)

# --- CONFIG ---
DEFAULT_DB = "trajectory_db.json"


# --- Database & Core Logic ---


def save_trajectory(
    db, prototype_name, iteration, distance, tangential_speed, angle_deg, hit
):
    """Saves a single trajectory record and returns its ID and key data."""
    rec = {
        "prototype_name": str(prototype_name),
        "iteration": int(iteration),
        "distance": float(distance),
        "tangential_speed": float(tangential_speed),
        "angle_deg": float(angle_deg),
        "hit": hit,
        "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    }
    doc_id = db.insert(rec)
    print_feedback(
        f"[SAVED] {rec['prototype_name']} {rec['iteration']} (doc_id={doc_id})"
    )
    return doc_id


# --- Calculation Helpers ---


def calculate_tangential_speed(rpm, wheel_diameter_m):
    return (math.pi * wheel_diameter_m * rpm) / 60.0


def calculate_geometry(launcher_depth_m, height_difference_m):
    distance_m = math.sqrt(launcher_depth_m**2 + height_difference_m**2)
    angle_rad = math.atan2(height_difference_m, launcher_depth_m)
    return distance_m, math.degrees(angle_rad)


# --- User Input Helpers ---


def get_float_input(prompt, min_val=None, max_val=None):
    """Temporarily reverts to standard input for multi-character entry."""
    while True:
        try:
            value = float(input(prompt))
            if (min_val is not None and value < min_val) or (
                max_val is not None and value > max_val
            ):
                print(f"Error: Value must be between {min_val} and {max_val}.")
                continue
            return value
        except ValueError:
            print("Invalid input. Please enter a number.")


def get_int_input(prompt, min_val=None):
    while True:
        try:
            value = int(input(prompt))
            if min_val is not None and value < min_val:
                print(f"Error: Value must be at least {min_val}.")
                continue
            return value
        except ValueError:
            print("Invalid input. Please enter a whole number.")


# --- UI & Display ---


def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def print_feedback(message, duration=1):
    """Prints a message and pauses briefly."""
    print(message)
    time.sleep(duration)


def display_history(history):
    """Displays the last 5 shots."""
    print("--- History ---")
    if history:
        # Display with newest shot at the top (index 1)
        for i, record in enumerate(reversed(history), 1):
            status = "HIT " if record["hit"] else "MISS"
            print(
                f" {i}. [{status}] Angle: {record['angle_deg']:.2f}°, Dist: {record['dist']:.2f} m"
            )
    print("-" * 45)


def display_main_ui(session_params, calculated_vals, history):
    """Prints the main user interface screen."""
    clear_screen()
    print("=" * 45)
    print(f" PROTOTYPE: {session_params['name']} | ITERATION: {session_params['iter']}")
    print("-" * 45)
    print(
        f"  [S] Motor Speed Ratio: {session_params['speed_ratio']:.2f} ({calculated_vals['rpm']:.0f} RPM)"
    )
    print(f"  [D] Launcher Depth:    {session_params['depth']:.2f} meters")
    print("-" * 45)
    print("           CALCULATED VALUES")
    print(f"           Tangential Speed: {calculated_vals['speed']:.2f} m/s")
    print(f"           Distance to Goal: {calculated_vals['dist']:.2f} m")
    print(f"           Launch Angle:     {calculated_vals['angle']:.2f}°")
    print("=" * 45)

    display_history(history)

    print("ACTIONS: [I]n [O]ut | [S]peed [D]epth | [U]ndo [E]dit [H]elp [Q]uit")
    print("Enter action: ", end="", flush=True)


# --- Action Handlers ---


def undo_action(db, history):
    """Handles the multi-level undo functionality."""
    if not history:
        print_feedback("[ERROR] No records in history to undo.")
        return

    while True:
        clear_screen()
        print("--- Undo a Recent Shot ---")
        display_history(history)
        print("Select record to delete [1-5], or [C]ancel: ", end="", flush=True)

        choice = readchar.readkey()

        if choice.lower() == "c":
            print_feedback("Undo cancelled.", duration=0.5)
            return

        if choice.isdigit() and 1 <= int(choice) <= len(history):
            # Convert deque to list for easier indexing
            history_list = list(history)
            # Reverse list to match display order (1 is the newest)
            history_list.reverse()

            record_to_remove = history_list.pop(int(choice) - 1)
            db.remove(doc_ids=[record_to_remove["doc_id"]])

            # Rebuild the deque from the modified list (restoring original order)
            history.clear()
            history.extend(reversed(history_list))

            print_feedback(f"Record {record_to_remove['doc_id']} deleted.")
            return
        else:
            print_feedback(
                f"Invalid choice '{choice}'. Please try again.", duration=1.5
            )


def edit_parameters(params):
    """Allows the user to edit the initial setup parameters mid-session."""
    while True:
        clear_screen()
        print("--- Edit Session Parameters ---")
        print(f"[1] Wheel Diameter:    {params['wheel_diameter_m']:.4f} m")
        print(f"[2] Motor Max RPM:     {params['motor_max_rpm']:.0f} RPM")
        print(f"[3] Launcher Height:   {params['launcher_height_m']:.2f} m")
        print(f"[4] Goal Height:       {params['goal_height_m']:.2f} m")
        print("\n[B]ack to main screen")

        choice = input("Select a parameter to change: ").lower().strip()

        if choice == "1":
            params["wheel_diameter_m"] = get_float_input(
                "Enter new wheel diameter (m): ", min_val=0.001
            )
        elif choice == "2":
            params["motor_max_rpm"] = get_float_input(
                "Enter new motor max RPM: ", min_val=1
            )
        elif choice == "3":
            params["launcher_height_m"] = get_float_input(
                "Enter new launcher height (m): ", min_val=0
            )
        elif choice == "4":
            params["goal_height_m"] = get_float_input(
                "Enter new goal height (m): ", min_val=0
            )
        elif choice == "b":
            params["height_difference_m"] = (
                params["goal_height_m"] - params["launcher_height_m"]
            )
            print("Parameters updated.")
            break
        else:
            print("Invalid selection.")
            input("Press Enter to continue...")
    return params


def show_help():
    """Displays the help menu."""
    clear_screen()
    print(
        """
--- Command Help ---

[I]n / [O]ut
  Log a shot as a HIT ('In') or a MISS ('Out').

[S]peed
  Set a new motor commanded speed ratio (e.g., 0.75 for 75%).

[D]epth
  Set a new launcher depth (horizontal distance from goal base).

[E]dit
  Change the core physical parameters set at the start of the session.

[U]ndo
  Delete the last shot you recorded. Can only be done once per shot.

[T]stats
  Show the hit/miss/accuracy statistics for the current session.

[H]elp
  Display this help screen.

[Q]uit
  Exit the program.
    """
    )
    input("\nPress Enter to return...")


def main():
    """Main program loop for data entry."""
    clear_screen()
    print("--- Manual Trajectory Data Collector ---")
    prototype_name = input("Enter Prototype Name: ").strip()
    iteration = get_int_input("Enter Iteration: ", min_val=1)

    physical_params = {
        "wheel_diameter_m": get_float_input(
            "Enter shooter wheel diameter (m): ", min_val=0.001
        ),
        "motor_max_rpm": get_float_input("Enter motor's max RPM: ", min_val=1),
        "launcher_height_m": get_float_input("Enter launcher height (m): ", min_val=0),
        "goal_height_m": get_float_input("Enter goal height (m): ", min_val=0),
    }
    physical_params["height_difference_m"] = (
        physical_params["goal_height_m"] - physical_params["launcher_height_m"]
    )

    commanded_speed_ratio = 0.8
    launcher_depth_m = 2.0
    history = deque(maxlen=5)  # <<< KEY CHANGE: History tracking
    db = TinyDB(DEFAULT_DB)

    while True:
        current_rpm = physical_params["motor_max_rpm"] * commanded_speed_ratio
        tangential_speed_ms = calculate_tangential_speed(
            current_rpm, physical_params["wheel_diameter_m"]
        )
        distance_m, angle_deg = calculate_geometry(
            launcher_depth_m, physical_params["height_difference_m"]
        )

        session_params = {
            "name": prototype_name,
            "iter": iteration,
            "speed_ratio": commanded_speed_ratio,
            "depth": launcher_depth_m,
        }
        calculated_vals = {
            "rpm": current_rpm,
            "speed": tangential_speed_ms,
            "dist": distance_m,
            "angle": angle_deg,
        }

        display_main_ui(session_params, calculated_vals, history)

        action = readchar.readkey().lower()

        if action in ("i", "o"):
            was_hit = action == "i"
            doc_id = save_trajectory(
                db,
                prototype_name,
                iteration,
                distance_m,
                tangential_speed_ms,
                angle_deg,
                was_hit,
            )
            history.append(
                {
                    "doc_id": doc_id,
                    "hit": was_hit,
                    "angle_deg": angle_deg,
                    "dist": distance_m,
                }
            )

        elif action == "s":
            clear_screen()
            commanded_speed_ratio = get_float_input(
                "Enter new speed ratio [0.0-1.0]: ", min_val=0.0, max_val=1.0
            )

        elif action == "d":
            clear_screen()
            launcher_depth_m = get_float_input(
                "Enter new depth (meters): ", min_val=0.0
            )

        elif action == "u":
            undo_action(db, history)

        elif action == "e":
            physical_params = edit_parameters(physical_params)

        elif action in ("h", "help"):
            show_help()

        elif action == "q":
            print("\nExiting program.")
            break

        # Add a small delay to prevent accidental double-presses for non-logging actions
        else:
            time.sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user. Exiting.")
        sys.exit(0)
