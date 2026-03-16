"""Flight data plotting utilities.

Provides standard plot suites for passive flights, closed-loop simulations,
and comparison of simulation vs. real flight data.
"""
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import Optional


def plot_passive_flight(result, title: str = "Passive Flight",
                        save_path: Optional[str | Path] = None):
    """Plot passive flight results (altitude, speed, acceleration).

    Args:
        result: PassiveFlightResult from rocketpy_passive.
        title: Plot title prefix.
        save_path: If provided, save figure to this path.
    """
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    fig.suptitle(f"{title}", fontsize=14)

    t = result.time

    # Altitude
    ax = axes[0]
    ax.plot(t, result.altitude, "b-", linewidth=1.5)
    ax.set_ylabel("Altitude (m)")
    ax.grid(True, alpha=0.3)
    ax.axhline(y=result.apogee_m, color="r", linestyle="--", alpha=0.5,
               label=f"Apogee: {result.apogee_m:.1f} m")
    ax.legend()

    # Speed
    ax = axes[1]
    ax.plot(t, result.speed, "g-", linewidth=1.5)
    ax.set_ylabel("Speed (m/s)")
    ax.grid(True, alpha=0.3)
    ax.axhline(y=result.max_speed_mps, color="r", linestyle="--", alpha=0.5,
               label=f"Max: {result.max_speed_mps:.1f} m/s")
    ax.legend()

    # Acceleration
    ax = axes[2]
    ax.plot(t, result.acceleration / 9.80665, "r-", linewidth=1.5)
    ax.set_ylabel("Acceleration (g)")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color="k", linestyle="-", alpha=0.3)

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"Saved plot to {save_path}")

    return fig


def plot_roll_response(time, roll_rate_truth, roll_rate_est=None,
                       deflection_cmd=None, deflection_actual=None,
                       title: str = "Roll Response",
                       save_path: Optional[str | Path] = None):
    """Plot roll dynamics: rate and control commands.

    Args:
        time: Time array in seconds.
        roll_rate_truth: True roll rate in deg/s.
        roll_rate_est: Estimated roll rate in deg/s (optional).
        deflection_cmd: Commanded fin tab deflection in deg (optional).
        deflection_actual: Actual deflection in deg (optional).
        title: Plot title.
        save_path: If provided, save figure.
    """
    n_plots = 1
    if deflection_cmd is not None or deflection_actual is not None:
        n_plots = 2

    fig, axes = plt.subplots(n_plots, 1, figsize=(10, 4 * n_plots), sharex=True)
    if n_plots == 1:
        axes = [axes]
    fig.suptitle(title, fontsize=14)

    # Roll rate
    ax = axes[0]
    ax.plot(time, roll_rate_truth, "b-", linewidth=1.5, label="Truth")
    if roll_rate_est is not None:
        ax.plot(time, roll_rate_est, "r--", linewidth=1.0, label="Estimated")
    ax.set_ylabel("Roll Rate (deg/s)")
    ax.grid(True, alpha=0.3)
    ax.legend()

    # Deflection commands
    if n_plots > 1:
        ax = axes[1]
        if deflection_cmd is not None:
            ax.plot(time, deflection_cmd, "b-", linewidth=1.5, label="Commanded")
        if deflection_actual is not None:
            ax.plot(time, deflection_actual, "r--", linewidth=1.0, label="Actual")
        ax.set_ylabel("Fin Tab Deflection (deg)")
        ax.set_xlabel("Time (s)")
        ax.grid(True, alpha=0.3)
        ax.legend()
    else:
        axes[0].set_xlabel("Time (s)")

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")

    return fig


def plot_trajectory_3d(x, y, z, title: str = "3D Trajectory",
                       save_path: Optional[str | Path] = None):
    """Plot 3D trajectory.

    Args:
        x, y, z: Position arrays in meters (ENU).
        title: Plot title.
        save_path: If provided, save figure.
    """
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(x, y, z, "b-", linewidth=1.5)
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Up (m)")
    ax.set_title(title)

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")

    return fig


def plot_attitude(time, roll_deg, pitch_deg, yaw_deg,
                  roll_est=None, pitch_est=None, yaw_est=None,
                  title: str = "Attitude",
                  save_path: Optional[str | Path] = None):
    """Plot attitude angles (roll, pitch, yaw).

    Args:
        time: Time array.
        roll_deg, pitch_deg, yaw_deg: True Euler angles in degrees.
        roll_est, pitch_est, yaw_est: Estimated angles (optional).
        title: Plot title.
        save_path: Save path.
    """
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(title, fontsize=14)

    labels = ["Roll", "Pitch", "Yaw"]
    truth = [roll_deg, pitch_deg, yaw_deg]
    est = [roll_est, pitch_est, yaw_est]

    for i, (ax, label, tr, es) in enumerate(zip(axes, labels, truth, est)):
        ax.plot(time, tr, "b-", linewidth=1.5, label="Truth")
        if es is not None:
            ax.plot(time, es, "r--", linewidth=1.0, label="Estimated")
        ax.set_ylabel(f"{label} (deg)")
        ax.grid(True, alpha=0.3)
        ax.legend()

    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")

    return fig


def plot_sensor_data(time_imu, accel_x, accel_y, accel_z,
                     gyro_x, gyro_y, gyro_z,
                     title: str = "IMU Sensor Data",
                     save_path: Optional[str | Path] = None):
    """Plot raw IMU sensor data."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.suptitle(title, fontsize=14)

    ax = axes[0]
    ax.plot(time_imu, accel_x, label="X", alpha=0.8)
    ax.plot(time_imu, accel_y, label="Y", alpha=0.8)
    ax.plot(time_imu, accel_z, label="Z", alpha=0.8)
    ax.set_ylabel("Acceleration (m/s^2)")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1]
    ax.plot(time_imu, np.degrees(gyro_x), label="X", alpha=0.8)
    ax.plot(time_imu, np.degrees(gyro_y), label="Y", alpha=0.8)
    ax.plot(time_imu, np.degrees(gyro_z), label="Z", alpha=0.8)
    ax.set_ylabel("Angular Rate (deg/s)")
    ax.set_xlabel("Time (s)")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches="tight")

    return fig
