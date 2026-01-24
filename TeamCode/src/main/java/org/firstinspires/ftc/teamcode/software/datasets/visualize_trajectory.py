#!/usr/bin/env python3
"""
FTC Trajectory Engine Visualization Tool

Creates stunning heatmaps and visualizations of the advanced trajectory prediction system.
Perfect for portfolios, presentations, and debugging.

Dependencies:
    pip install numpy matplotlib scipy pandas seaborn
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import LinearSegmentedColormap
import pandas as pd
from pathlib import Path
import seaborn as sns

# Configure modern, professional styling
plt.style.use("seaborn-v0_8-darkgrid")
sns.set_palette("husl")

# FTC Field Constants (inches)
FIELD_WIDTH = 144
FIELD_HEIGHT = 144

# Launch Zone Boundaries (from Settings.java)
CLOSE_ZONE_CORNERS = np.array(
    [
        [72, 72],  # Front corner
        [15, 128],  # Left corner
        [129, 128],  # Right corner
    ]
)

FAR_ZONE_CORNERS = np.array(
    [
        [72, 24],  # Front corner
        [50, 0],  # Left corner
        [95, 0],  # Right corner
    ]
)

# Interpolation Settings (from Settings.java)
IDW_POWER = 2.5
MIN_INTERPOLATION_DISTANCE = 0.1


def load_calibration_data(zone_name):
    """Load calibration data from CSV file."""
    csv_path = Path(__file__).parent / f"{zone_name}_zone.csv"

    if not csv_path.exists():
        print(f"Warning: {csv_path} not found, using defaults")
        if zone_name == "close":
            return pd.DataFrame(
                {
                    "x": [58.0, 50.0, 65.0, 75.0],
                    "y": [99.0, 85.0, 110.0, 95.0],
                    "rpm": [2665.0, 2750.0, 2850.0, 2700.0],
                    "pitch": [45.2, 43.5, 46.0, 44.0],
                }
            )
        else:
            return pd.DataFrame(
                {
                    "x": [60.0, 55.0, 65.0, 72.0],
                    "y": [18.0, 10.0, 20.0, 15.0],
                    "rpm": [3545.0, 3600.0, 3500.0, 3550.0],
                    "pitch": [32.5, 31.5, 33.0, 32.0],
                }
            )

    return pd.read_csv(csv_path)


def point_in_triangle(point, triangle):
    """Check if a point is inside a triangle using barycentric coordinates."""
    x, y = point
    x1, y1 = triangle[0]
    x2, y2 = triangle[1]
    x3, y3 = triangle[2]

    det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3)
    if abs(det) < 1e-10:
        return False

    lambda1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / det
    lambda2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / det
    lambda3 = 1.0 - lambda1 - lambda2

    return lambda1 >= 0 and lambda2 >= 0 and lambda3 >= 0


def idw_interpolation(x, y, calibration_data, power=IDW_POWER):
    """
    Inverse Distance Weighting interpolation - matches Java implementation.

    Returns (rpm, pitch) tuple.
    """
    weighted_rpm = 0
    weighted_pitch = 0
    total_weight = 0

    for _, point in calibration_data.iterrows():
        dx = x - point["x"]
        dy = y - point["y"]
        distance = np.sqrt(dx * dx + dy * dy)

        # If we're exactly on a point, use it directly
        if distance < MIN_INTERPOLATION_DISTANCE:
            return point["rpm"], point["pitch"]

        weight = 1.0 / (distance**power)
        weighted_rpm += weight * point["rpm"]
        weighted_pitch += weight * point["pitch"]
        total_weight += weight

    if total_weight == 0:
        return calibration_data.iloc[0]["rpm"], calibration_data.iloc[0]["pitch"]

    return weighted_rpm / total_weight, weighted_pitch / total_weight


def predict_trajectory(x, y, close_data, far_data):
    """
    Predict RPM and pitch for a given position.
    Matches the Java TrajectoryEngine logic exactly.
    """
    point = np.array([x, y])

    # Determine which zone we're in
    in_close_zone = point_in_triangle(point, CLOSE_ZONE_CORNERS)
    in_far_zone = point_in_triangle(point, FAR_ZONE_CORNERS)

    if in_close_zone:
        return idw_interpolation(x, y, close_data)
    elif in_far_zone:
        return idw_interpolation(x, y, far_data)
    else:
        # Outside both zones - use closest zone's nearest preset
        # For visualization, we'll return NaN to show undefined regions
        return np.nan, np.nan


def create_heatmap(resolution=200):
    """
    Create comprehensive heatmap visualization of the trajectory engine.
    """
    # Load calibration data
    close_data = load_calibration_data("close")
    far_data = load_calibration_data("far")

    # Create grid
    x = np.linspace(0, FIELD_WIDTH, resolution)
    y = np.linspace(0, FIELD_HEIGHT, resolution)
    X, Y = np.meshgrid(x, y)

    # Calculate predictions for entire field
    print("Computing trajectory predictions across field...")
    RPM = np.zeros_like(X)
    PITCH = np.zeros_like(X)

    for i in range(resolution):
        if i % 20 == 0:
            print(f"  Progress: {i}/{resolution}")
        for j in range(resolution):
            rpm, pitch = predict_trajectory(X[i, j], Y[i, j], close_data, far_data)
            RPM[i, j] = rpm
            PITCH[i, j] = pitch

    print("Generating visualizations...")

    # Create figure with subplots
    fig = plt.figure(figsize=(20, 10))
    gs = fig.add_gridspec(2, 3, hspace=0.3, wspace=0.3)

    # ===== RPM Heatmap =====
    ax1 = fig.add_subplot(gs[0, :2])

    # Create custom colormap (blue to red, professional)
    colors = ["#2E86AB", "#06A77D", "#F5CB5C", "#E76F51", "#D62828"]
    n_bins = 100
    cmap = LinearSegmentedColormap.from_list("trajectory", colors, N=n_bins)

    # Plot RPM heatmap
    im1 = ax1.contourf(X, Y, RPM, levels=50, cmap=cmap, alpha=0.9)

    # Add zone boundaries
    close_poly = patches.Polygon(
        CLOSE_ZONE_CORNERS,
        fill=False,
        edgecolor="white",
        linewidth=3,
        linestyle="--",
        label="Close Launch Zone",
    )
    far_poly = patches.Polygon(
        FAR_ZONE_CORNERS,
        fill=False,
        edgecolor="cyan",
        linewidth=3,
        linestyle="--",
        label="Far Launch Zone",
    )
    ax1.add_patch(close_poly)
    ax1.add_patch(far_poly)

    # Plot calibration points
    ax1.scatter(
        close_data["x"],
        close_data["y"],
        c="white",
        s=200,
        edgecolors="black",
        linewidths=2,
        marker="o",
        label="Close Zone Calibration",
        zorder=5,
    )
    ax1.scatter(
        far_data["x"],
        far_data["y"],
        c="cyan",
        s=200,
        edgecolors="black",
        linewidths=2,
        marker="s",
        label="Far Zone Calibration",
        zorder=5,
    )

    # Styling
    ax1.set_xlabel("Field X Position (inches)", fontsize=14, fontweight="bold")
    ax1.set_ylabel("Field Y Position (inches)", fontsize=14, fontweight="bold")
    ax1.set_title(
        "FTC Trajectory Engine - Predicted RPM Across Field",
        fontsize=16,
        fontweight="bold",
        pad=20,
    )
    ax1.set_xlim(0, FIELD_WIDTH)
    ax1.set_ylim(0, FIELD_HEIGHT)
    ax1.set_aspect("equal")
    ax1.grid(True, alpha=0.3)

    # Add colorbar
    cbar1 = plt.colorbar(im1, ax=ax1, orientation="vertical", pad=0.02)
    cbar1.set_label("Launcher RPM", fontsize=12, fontweight="bold")

    # ===== Pitch Heatmap =====
    ax2 = fig.add_subplot(gs[1, :2])

    # Pitch colormap (purple to orange for angles)
    pitch_colors = ["#6A0572", "#AB83A1", "#F4E285", "#FF8C42", "#FF3C38"]
    pitch_cmap = LinearSegmentedColormap.from_list("pitch", pitch_colors, N=n_bins)

    im2 = ax2.contourf(X, Y, PITCH, levels=50, cmap=pitch_cmap, alpha=0.9)

    # Add zone boundaries (same as above)
    close_poly2 = patches.Polygon(
        CLOSE_ZONE_CORNERS, fill=False, edgecolor="white", linewidth=3, linestyle="--"
    )
    far_poly2 = patches.Polygon(
        FAR_ZONE_CORNERS, fill=False, edgecolor="cyan", linewidth=3, linestyle="--"
    )
    ax2.add_patch(close_poly2)
    ax2.add_patch(far_poly2)

    # Plot calibration points
    ax2.scatter(
        close_data["x"],
        close_data["y"],
        c="white",
        s=200,
        edgecolors="black",
        linewidths=2,
        marker="o",
        zorder=5,
    )
    ax2.scatter(
        far_data["x"],
        far_data["y"],
        c="cyan",
        s=200,
        edgecolors="black",
        linewidths=2,
        marker="s",
        zorder=5,
    )

    ax2.set_xlabel("Field X Position (inches)", fontsize=14, fontweight="bold")
    ax2.set_ylabel("Field Y Position (inches)", fontsize=14, fontweight="bold")
    ax2.set_title(
        "FTC Trajectory Engine - Predicted Pitch Angle Across Field",
        fontsize=16,
        fontweight="bold",
        pad=20,
    )
    ax2.set_xlim(0, FIELD_WIDTH)
    ax2.set_ylim(0, FIELD_HEIGHT)
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.3)

    cbar2 = plt.colorbar(im2, ax=ax2, orientation="vertical", pad=0.02)
    cbar2.set_label("Launch Pitch (degrees)", fontsize=12, fontweight="bold")

    # Add title and metadata
    fig.suptitle(
        "Advanced Zone-Based Trajectory Prediction System",
        fontsize=20,
        fontweight="bold",
        y=0.98,
    )

    # Add timestamp
    from datetime import datetime

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    fig.text(
        0.99, 0.01, f"Generated: {timestamp}", ha="right", fontsize=8, style="italic"
    )

    plt.tight_layout()

    # Save high-resolution image
    output_path = Path(__file__).parent / "trajectory_visualization.png"
    plt.savefig(
        output_path, dpi=300, bbox_inches="tight", facecolor="white", edgecolor="none"
    )
    print(f"\n✓ Visualization saved to: {output_path}")

    return fig


def create_3d_surface():
    """Create 3D surface plot of RPM predictions."""

    close_data = load_calibration_data("close")
    far_data = load_calibration_data("far")

    resolution = 100
    x = np.linspace(0, FIELD_WIDTH, resolution)
    y = np.linspace(0, FIELD_HEIGHT, resolution)
    X, Y = np.meshgrid(x, y)

    print("Computing 3D surface...")
    RPM = np.zeros_like(X)
    for i in range(resolution):
        for j in range(resolution):
            rpm, _ = predict_trajectory(X[i, j], Y[i, j], close_data, far_data)
            RPM[i, j] = rpm

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection="3d")

    # Create surface
    surf = ax.plot_surface(
        X, Y, RPM, cmap="viridis", alpha=0.8, linewidth=0, antialiased=True
    )

    # Plot calibration points as scatter
    ax.scatter(
        close_data["x"],
        close_data["y"],
        close_data["rpm"],
        c="red",
        s=100,
        marker="o",
        label="Close Zone Data",
    )
    ax.scatter(
        far_data["x"],
        far_data["y"],
        far_data["rpm"],
        c="blue",
        s=100,
        marker="^",
        label="Far Zone Data",
    )

    ax.set_xlabel("Field X (inches)", fontsize=12, fontweight="bold")
    ax.set_ylabel("Field Y (inches)", fontsize=12, fontweight="bold")
    ax.set_zlabel("Launcher RPM", fontsize=12, fontweight="bold")
    ax.set_title(
        "3D Trajectory Surface - RPM Predictions",
        fontsize=16,
        fontweight="bold",
        pad=20,
    )
    ax.legend()

    # Add colorbar
    fig.colorbar(surf, ax=ax, shrink=0.5, aspect=5)

    output_path = Path(__file__).parent / "trajectory_3d.png"
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"✓ 3D visualization saved to: {output_path}")

    return fig


def main():
    """Generate all visualizations."""
    print("=" * 60)
    print("FTC TRAJECTORY ENGINE VISUALIZATION TOOL")
    print("=" * 60)
    print()

    # Create main heatmap
    print("1. Creating comprehensive heatmap...")
    create_heatmap(resolution=200)

    # Create 3D surface
    print("\n2. Creating 3D surface plot...")
    create_3d_surface()

    print("\n" + "=" * 60)
    print("✓ All visualizations generated successfully!")
    print("=" * 60)
    print("\nFiles created:")
    print("  - trajectory_visualization.png (main heatmap)")
    print("  - trajectory_3d.png (3D surface)")
    print("\nPerfect for portfolios and presentations!")

    # Show plots
    plt.show()


if __name__ == "__main__":
    main()
