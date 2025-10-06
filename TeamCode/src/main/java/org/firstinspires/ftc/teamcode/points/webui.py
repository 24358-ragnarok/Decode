#!/usr/bin/env python3
"""
Web UI for the encrypted points database.
Run with: python3 webui.py
Then open: http://localhost:24358
"""

import secrets
from flask import (
    Flask,
    render_template,
    request,
    session,
    redirect,
    url_for,
    jsonify,
)

from utils import read_db, write_db

app = Flask(__name__)
app.secret_key = secrets.token_hex(32)


def ensure_person(db, name):
    """Initializes a person in the database if they don't exist."""
    if name not in db:
        db[name] = {}


@app.route("/")
def index():
    """Redirects to the scoreboard if logged in, otherwise to the login page."""
    if "password" not in session:
        return redirect(url_for("login"))
    return redirect(url_for("scoreboard"))


@app.route("/login", methods=["GET", "POST"])
def login():
    """Handles user login."""
    if request.method == "POST":
        password = request.form.get("password", "").encode()
        try:
            read_db(password)  # Test password validity
            session["password"] = password
            return redirect(url_for("scoreboard"))
        except Exception:
            return render_template(
                "login.html", error="Invalid password or corrupted database"
            )
    return render_template("login.html")


@app.route("/logout", methods=["POST"])
def logout():
    """Clears the session to log the user out."""
    session.clear()
    return jsonify({"success": True})


@app.route("/scoreboard")
def scoreboard():
    """Displays the ranked scoreboard."""
    if "password" not in session:
        return redirect(url_for("login"))

    try:
        db, _ = read_db(session["password"])

        # Get all unique point classes for the filter dropdown
        all_pclasses = set()
        for person_data in db.values():
            all_pclasses.update(person_data.keys())

        # Determine the filter
        selected_pclass = request.args.get("pclass", "combined")

        # Process data for the scoreboard
        scoreboard_data = []
        for name, data in db.items():
            combined_score = sum(data.values())
            entry = {
                "name": name,
                "points_by_class": data,
                "combined": combined_score,
            }
            scoreboard_data.append(entry)

        # Sort the data based on the selected filter
        sort_key = lambda x: x["points_by_class"].get(selected_pclass, 0) if selected_pclass != "combined" else x["combined"]
        scoreboard_data.sort(key=sort_key, reverse=True)

        return render_template(
            "scoreboard.html",
            scoreboard_data=scoreboard_data,
            pclasses=sorted(list(all_pclasses)),
            selected_pclass=selected_pclass,
        )
    except Exception as e:
        session.clear()
        return redirect(url_for("login", error=f"Error reading DB: {e}"))


@app.route("/management")
def management():
    """Page for updating points."""
    if "password" not in session:
        return redirect(url_for("login"))
    db, _ = read_db(session["password"])
    return render_template("management.html", people=sorted(db.keys()))


@app.route("/people")
def people():
    """Page for adding/removing people."""
    if "password" not in session:
        return redirect(url_for("login"))
    db, _ = read_db(session["password"])
    return render_template("people.html", people=sorted(db.keys()))


# --- API Endpoints ---

@app.route("/api/update", methods=["POST"])
def update_points():
    """API endpoint to handle point updates."""
    if "password" not in session:
        return jsonify({"success": False, "message": "Not authenticated"}), 401

    try:
        data = request.json
        name = data.get("name")
        change = data.get("change")
        pclass = data.get("pclass", "regular")
        recipient = data.get("recipient")

        if not name or change is None:
            return jsonify({"success": False, "message": "Missing required fields"}), 400

        password = session["password"]
        db, salt = read_db(password)

        ensure_person(db, name)
        db[name].setdefault(pclass, 0)
        db[name][pclass] += change

        action_msg = f"{'Added' if change > 0 else 'Removed'} {abs(change)} {pclass} points {'to' if change > 0 else 'from'} {name}."

        if recipient:
            ensure_person(db, recipient)
            db[recipient].setdefault(pclass, 0)
            db[recipient][pclass] -= change
            action_msg = f"Transferred {abs(change)} {pclass} points from {name if change < 0 else recipient} to {recipient if change < 0 else name}."

        write_db(db, password, salt)
        return jsonify({"success": True, "message": action_msg})

    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500

@app.route("/api/person", methods=["POST", "DELETE"])
def manage_person():
    """API endpoint to add or remove a person."""
    if "password" not in session:
        return jsonify({"success": False, "message": "Not authenticated"}), 401

    try:
        data = request.json
        name = data.get("name")
        if not name:
            return jsonify({"success": False, "message": "Name is required."}), 400

        password = session["password"]
        db, salt = read_db(password)

        if request.method == "POST":
            if name in db:
                return jsonify({"success": False, "message": f"'{name}' already exists."}), 409
            db[name] = {}
            message = f"Added '{name}' to the database."

        elif request.method == "DELETE":
            if name not in db:
                return jsonify({"success": False, "message": f"'{name}' not found."}), 404
            del db[name]
            message = f"Removed '{name}' from the database."

        write_db(db, password, salt)
        return jsonify({"success": True, "message": message})

    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500


if __name__ == "__main__":
    print("🚀 Starting Points Dashboard Web UI")
    print("📍 Navigate to: http://localhost:24358")
    print("🔐 Enter your master password to unlock")
    print("\n⚠️  Press CTRL+C to stop the server\n")
    app.run(host="127.0.0.1", port=24358, debug=True)