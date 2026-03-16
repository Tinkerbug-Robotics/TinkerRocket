"""
Visualization for drift cast results.

- Wind profile plots (altitude vs speed/direction)
- Guidance map (matplotlib) showing pad, target, guidance point, descent track
- Interactive HTML map (folium)
"""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

from .wind_api import WindProfile
from .descent_sim import TrackPoint


def plot_wind_profile(wind_profile: WindProfile, save_path: str | None = None) -> None:
    """Plot wind speed and direction vs altitude."""
    if not wind_profile.layers:
        print("No wind data to plot.")
        return

    alts = [l.alt_ft for l in wind_profile.layers]
    speeds = [l.speed_kts for l in wind_profile.layers]
    dirs = [l.direction_deg for l in wind_profile.layers]

    fig, ax1 = plt.subplots(figsize=(8, 6))

    color_speed = "#1f77b4"
    color_dir = "#ff7f0e"

    ax1.plot(speeds, alts, "o-", color=color_speed, label="Speed (kts)", ms=4)
    ax1.set_xlabel("Wind Speed (knots)", color=color_speed)
    ax1.set_ylabel("Altitude AGL (ft)")
    ax1.tick_params(axis="x", labelcolor=color_speed)
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twiny()
    ax2.plot(dirs, alts, "s-", color=color_dir, label="Direction (°)", ms=4)
    ax2.set_xlabel("Wind From Direction (°)", color=color_dir)
    ax2.tick_params(axis="x", labelcolor=color_dir)
    ax2.set_xlim(0, 360)

    fig.suptitle(f"Wind Profile — ({wind_profile.location[0]:.2f}, {wind_profile.location[1]:.2f})\n"
                 f"{wind_profile.fetch_time}", fontsize=11)
    fig.tight_layout()

    if save_path is None:
        save_path = "wind_profile.png"
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"  Wind profile saved: {save_path}")
    plt.close()


def plot_guidance(
    launch_pt: tuple[float, float],
    landing_pt: tuple[float, float],
    guidance_pt: tuple[float, float],
    descent_track: list[TrackPoint],
    save_path: str | None = None,
) -> None:
    """Plot guidance map showing pad, target, guidance point, and descent track."""
    fig, ax = plt.subplots(figsize=(10, 8))

    # Descent track
    if descent_track:
        lats = [p.lat for p in descent_track]
        lons = [p.lon for p in descent_track]
        alts = [p.alt_agl_ft for p in descent_track]

        # Color by altitude
        for i in range(len(descent_track) - 1):
            color_val = alts[i] / max(alts) if max(alts) > 0 else 0
            color = plt.cm.viridis(color_val)
            ax.plot([lons[i], lons[i+1]], [lats[i], lats[i+1]],
                    "-", color=color, lw=2, alpha=0.7)

    # Key points
    ax.plot(launch_pt[1], launch_pt[0], "^", color="red", ms=14,
            label="Launch Pad", zorder=5)
    ax.plot(landing_pt[1], landing_pt[0], "*", color="green", ms=16,
            label="Landing Target", zorder=5)
    ax.plot(guidance_pt[1], guidance_pt[0], "D", color="blue", ms=12,
            label="Guidance Point (apogee)", zorder=5)

    # Connecting lines
    ax.plot([launch_pt[1], guidance_pt[1]], [launch_pt[0], guidance_pt[0]],
            "--", color="blue", alpha=0.4, lw=1, label="Boost trajectory (horiz)")

    ax.set_xlabel("Longitude (°)")
    ax.set_ylabel("Latitude (°)")
    ax.set_title("Reverse Drift Cast — Guidance Map")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal")

    # Add colorbar for altitude
    if descent_track and max(alts) > 0:
        sm = plt.cm.ScalarMappable(
            cmap=plt.cm.viridis,
            norm=plt.Normalize(0, max(alts)),
        )
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax, shrink=0.6, pad=0.02)
        cbar.set_label("Altitude AGL (ft)")

    fig.tight_layout()

    if save_path is None:
        save_path = "guidance_map.png"
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"  Guidance map saved: {save_path}")
    plt.close()


def generate_html_map(
    launch_pt: tuple[float, float],
    landing_pt: tuple[float, float],
    guidance_pt: tuple[float, float],
    descent_track: list[TrackPoint],
    output_path: str = "driftcast_map.html",
) -> None:
    """Generate an interactive HTML map with folium."""
    try:
        import folium
    except ImportError:
        print("folium not installed. Install with: pip install folium")
        return

    # Center map on the midpoint
    center_lat = (launch_pt[0] + landing_pt[0] + guidance_pt[0]) / 3
    center_lon = (launch_pt[1] + landing_pt[1] + guidance_pt[1]) / 3

    m = folium.Map(location=[center_lat, center_lon], zoom_start=14,
                   tiles="OpenStreetMap")

    # Launch pad
    folium.Marker(
        [launch_pt[0], launch_pt[1]],
        popup="Launch Pad",
        icon=folium.Icon(color="red", icon="rocket", prefix="fa"),
    ).add_to(m)

    # Landing target
    folium.Marker(
        [landing_pt[0], landing_pt[1]],
        popup="Landing Target",
        icon=folium.Icon(color="green", icon="bullseye", prefix="fa"),
    ).add_to(m)

    # Guidance point
    folium.Marker(
        [guidance_pt[0], guidance_pt[1]],
        popup=f"Guidance Point (apogee)",
        icon=folium.Icon(color="blue", icon="crosshairs", prefix="fa"),
    ).add_to(m)

    # Descent track
    if descent_track:
        track_coords = [[p.lat, p.lon] for p in descent_track]
        folium.PolyLine(
            track_coords,
            color="purple",
            weight=3,
            opacity=0.7,
            popup="Descent track",
        ).add_to(m)

    # Boost line (pad → guidance)
    folium.PolyLine(
        [[launch_pt[0], launch_pt[1]], [guidance_pt[0], guidance_pt[1]]],
        color="blue",
        weight=2,
        opacity=0.4,
        dash_array="10",
        popup="Boost trajectory (horiz. component)",
    ).add_to(m)

    m.save(output_path)
    print(f"  HTML map saved: {output_path}")
