#!/usr/bin/env python3
"""
trajectory_interactive.py

Fully interactive prototype-based shot analyzer.
- Reads TinyDB with 'prototype_name' and 'iteration' fields
- Prompts user for weighting scheme
- Computes weighted hit/miss stats per prototype
- Generates interactive visualizations with Plotly

Requirements:
pip install tinydb pandas plotly
"""

import os
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go
import sys
from tinydb import TinyDB


def load_db(db_path: str) -> pd.DataFrame:
    """Loads and cleans data from the TinyDB file."""
    if not os.path.exists(db_path):
        print(f"[ERROR] DB file not found: {db_path}")
        sys.exit(1)
    db = TinyDB(db_path)
    recs = db.all()
    if not recs:
        print("[WARN] DB is empty.")
    df = pd.DataFrame(recs)

    required_cols = ["hit", "timestamp", "prototype_name", "iteration"]
    if not all(col in df.columns for col in required_cols):
        print(f"[ERROR] DB missing required fields. Expected: {required_cols}")
        print("Please run the migration script first to update your database.")
        sys.exit(1)

    df = df.dropna(subset=["hit", "timestamp"])
    df["hit"] = df["hit"].astype(bool)

    # NEW LINE: Standardize timestamp by replacing the space separator with a 'T'
    df["timestamp"] = df["timestamp"].str.replace(" ", "T", n=1)

    df["timestamp"] = pd.to_datetime(df["timestamp"], errors="coerce")
    df = df.dropna(subset=["timestamp"])
    df = df.sort_values("timestamp").reset_index(drop=True)
    return df


def group_by_prototype(df: pd.DataFrame) -> pd.DataFrame:
    """Creates a unique group ID from prototype_name and iteration."""
    df["prototype_id"] = df["prototype_name"] + " v" + df["iteration"].astype(str)
    return df


def limit_shots(df: pd.DataFrame, shot_limit: int = None) -> pd.DataFrame:
    """Limits each prototype group to the first N shots."""
    if shot_limit is None or shot_limit <= 0:
        return df

    limited = df.groupby("prototype_id").head(shot_limit).reset_index(drop=True)
    return limited


def apply_weights(df: pd.DataFrame, max_weight: float) -> pd.DataFrame:
    """Assigns linear weights to shots within each prototype group."""

    def weight_series(subdf: pd.DataFrame) -> pd.Series:
        n = len(subdf)
        if n == 1:
            return pd.Series([1.0], index=subdf.index)
        return pd.Series(
            [1.0 + i * (max_weight - 1.0) / (n - 1) for i in range(n)],
            index=subdf.index,
        )

    df["weight"] = df.groupby("prototype_id", group_keys=False).apply(
        weight_series, include_groups=False
    )
    return df


def summarize(df: pd.DataFrame) -> pd.DataFrame:
    """Calculates weighted statistics for each prototype group."""
    weighted = df.groupby("prototype_id").apply(
        lambda g: pd.Series(
            {
                "total_weight": g["weight"].sum(),
                "hits_weighted": (g["hit"] * g["weight"]).sum(),
                "shots": len(g),
            }
        )
    )
    weighted["misses_weighted"] = weighted["total_weight"] - weighted["hits_weighted"]
    weighted["hit_pct"] = 100 * weighted["hits_weighted"] / weighted["total_weight"]
    weighted = weighted.reset_index()
    return weighted


def interactive_plot(df: pd.DataFrame, summary: pd.DataFrame):
    """Generates and displays the interactive Plotly figure."""
    # Set renderer to browser to avoid printing raw HTML/JS to console
    import plotly.io as pio

    pio.renderers.default = "browser"

    fig = go.Figure()

    summary = summary.sort_values("prototype_id").reset_index(drop=True)
    prototype_order = summary["prototype_id"].tolist()

    prototype_acc = df.groupby("prototype_id")["hit"].mean().reset_index()
    prototype_acc["rank"] = prototype_acc["hit"].rank(method="dense", ascending=False)
    prototype_acc["rank_norm"] = (
        (prototype_acc["rank"] - 1) / (prototype_acc["rank"].max() - 1)
        if prototype_acc["rank"].max() > 1
        else 0
    )

    colorscale = px.colors.diverging.PiYG[::-1]

    def rank_to_color(r):
        idx = int(r * (len(colorscale) - 1))
        return colorscale[idx]

    rank_color_map = {
        row["prototype_id"]: rank_to_color(row["rank_norm"])
        for _, row in prototype_acc.iterrows()
    }

    for _, row in summary.iterrows():
        prototype_id = row["prototype_id"]
        hit_pct = row["hit_pct"]
        error_pct = row.get("hit_pct_error", 0)

        hover_text = f"<b>{prototype_id}</b><br>Avg Hit Rate: {hit_pct:.1f}%"
        if error_pct > 0:
            hover_text += f"<br>Error: +/- {error_pct:.1f}%"

        fig.add_trace(
            go.Bar(
                x=[prototype_id],
                y=[hit_pct],
                width=0.4,
                name=f"{prototype_id} Hit %",
                opacity=0.7,
                marker_color=rank_color_map.get(prototype_id, "gray"),
                error_y=dict(
                    type="data",
                    array=[error_pct],
                    visible=True if error_pct > 0 else False,
                    thickness=2,
                    color="black",
                ),
                hovertext=hover_text,
                hoverinfo="text",
            )
        )

    df["color"] = df["hit"].map({True: "green", False: "red"})
    df["shot_idx"] = df.groupby("prototype_id").cumcount() + 1

    fig.add_trace(
        go.Scatter(
            x=df["prototype_id"],
            y=df["shot_idx"],
            mode="markers",
            marker=dict(
                color=df["color"],
                size=df["weight"] * 5,
                line=dict(width=1, color="black"),
            ),
            text=[
                f"Hit: {h}, Weight: {w:.2f}" for h, w in zip(df["hit"], df["weight"])
            ],
            hoverinfo="text",
            name="Shots",
        )
    )

    fig.update_layout(
        xaxis_title="Prototype/Iteration",
        yaxis_title="Accuracy and Shots",
        title="Per-Prototype Weighted Accuracy with Hit/Miss Scatter",
        showlegend=True,
        legend_title_text="Trace Type",
        xaxis={"categoryorder": "array", "categoryarray": prototype_order},
    )

    fig.update_layout(
        template=None,
        font=dict(family="Inter, Helvetica, sans-serif", size=14),
        plot_bgcolor="rgba(255,255,255,0)",
        paper_bgcolor="rgba(255,255,255,0)",
        title=dict(x=0.02, font=dict(size=22, color="#222")),
        yaxis=dict(gridcolor="rgba(200,200,200,0.3)", zeroline=False),
        xaxis=dict(showline=True, linewidth=1, linecolor="#444", mirror=True),
    )


    fig.show()


def main():
    """Main function to drive the interactive analysis."""
    db_path = (
        input("Enter TinyDB path (default: trajectory_db.json): ").strip()
        or "trajectory_db.json"
    )
    weight_input = (
        input("Enter max weight for last shot (e.g., 1.0 or 1.0/5.0): ").strip()
        or "1.0/5.0"
    )
    shot_limit_input = (
            input("Enter shot limit per prototype (default: none): ").strip() or ""
    )

    shot_limit = None
    if shot_limit_input:
        try:
            shot_limit = int(shot_limit_input)
            if shot_limit <= 0:
                print("[WARN] Shot limit must be positive. Ignoring.")
                shot_limit = None
        except ValueError:
            print("[WARN] Invalid shot limit. Using all shots.")
            shot_limit = None

    midpoint_weight, low_weight, high_weight, has_error = 0.0, 0.0, 0.0, False

    if "/" in weight_input:
        try:
            w1, w2 = sorted([float(x) for x in weight_input.split("/")])
            low_weight, high_weight = w1, w2
            midpoint_weight = (low_weight + high_weight) / 2
            has_error = True
        except (ValueError, IndexError):
            print("[ERROR] Invalid weight range. Use 'low/high'. Exiting.")
            return
    else:
        try:
            midpoint_weight = float(weight_input)
        except ValueError:
            print("[ERROR] Invalid weight. Use a number. Exiting.")
            return

    df = load_db(db_path)
    if df.empty:
        print("[EXIT] No usable records.")
        return

    df_grouped = group_by_prototype(df)
    df_limited = limit_shots(df_grouped, shot_limit)

    if shot_limit is not None:
        print(f"[INFO] Limiting analysis to first {shot_limit} shots per prototype.")

    df_weighted = apply_weights(df_limited.copy(), midpoint_weight)
    summary = summarize(df_weighted)
    summary["hit_pct_error"] = 0.0

    if has_error:
        summary_low = summarize(apply_weights(df_limited.copy(), low_weight))
        summary_high = summarize(apply_weights(df_limited.copy(), high_weight))
        summary = summary.merge(
            summary_low[["prototype_id", "hit_pct"]],
            on="prototype_id",
            suffixes=("", "_low"),
        )
        summary = summary.merge(
            summary_high[["prototype_id", "hit_pct"]],
            on="prototype_id",
            suffixes=("", "_high"),
        )
        summary["hit_pct_error"] = (
            summary["hit_pct_high"] - summary["hit_pct_low"]
        ) / 2.0

    print("\n--- Prototype Summary ---")
    for _, row in summary.iterrows():
        prototype_id, hit_pct, shots, error = (
            row["prototype_id"],
            row["hit_pct"],
            int(row["shots"]),
            row["hit_pct_error"],
        )
        if error > 0.01:
            print(
                f"Prototype '{prototype_id}': {hit_pct:.1f} [+/- {error:.1f}]% hit rate over {shots} launches"
            )
        else:
            print(
                f"Prototype '{prototype_id}': {hit_pct:.1f}% hit rate over {shots} launches"
            )
    print("-------------------------\n")

    interactive_plot(df_weighted, summary)


if __name__ == "__main__":
    main()
