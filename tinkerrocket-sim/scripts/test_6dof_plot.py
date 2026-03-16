"""Quick visual validation of 6-DOF passive flight."""
import sys
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, "/Users/christianpedersen/Documents/Hobbies/Model Rockets/Code/tinkerrocket-sim/src")

from tinkerrocket_sim.rocket.definition import RocketDefinition, MotorConfig
from tinkerrocket_sim.physics.sixdof import run_passive_6dof


def make_test_rocket():
    burn_time = 2.0
    n_points = 20
    times = np.linspace(0, burn_time, n_points)
    forces = np.ones(n_points) * 40.0
    forces[0] = 0.0
    forces[1] = 30.0
    forces[-2] = 25.0
    forces[-1] = 0.0

    motor = MotorConfig(
        thrust_times=times,
        thrust_forces=forces,
        total_mass=0.123,
        propellant_mass=0.06,
        designation="G40",
    )

    return RocketDefinition(
        name="TinkerRocket Mini Test",
        body_diameter=0.0574,
        body_length=0.40,
        nose_length=0.10,
        dry_mass=0.770,
        Cd=0.5,
        motor=motor,
        I_roll_launch=8.3e-4,
        I_roll_burnout=8.237e-4,
        I_transverse_launch=0.02,
        I_transverse_burnout=0.019,
    )


rd = make_test_rocket()
log = run_passive_6dof(rd, duration=30.0, dt=0.001, launch_angle_deg=85.0, log_interval=0.01)

t = log["time"]

fig, axes = plt.subplots(3, 2, figsize=(12, 10))
fig.suptitle(f"6-DOF Passive Flight — {rd.name}\n"
             f"Apogee: {log['apogee_m']:.1f}m | Max speed: {log['max_speed_mps']:.1f} m/s")

# Altitude
ax = axes[0, 0]
ax.plot(t, log["altitude"], 'b-')
ax.set_ylabel("Altitude (m)")
ax.set_title("Altitude vs Time")
ax.axhline(0, color='k', lw=0.5)
ax.grid(True, alpha=0.3)

# Speed
ax = axes[0, 1]
ax.plot(t, log["speed"], 'r-')
ax.set_ylabel("Speed (m/s)")
ax.set_title("Speed vs Time")
ax.grid(True, alpha=0.3)

# Thrust
ax = axes[1, 0]
ax.plot(t, log["thrust"], 'g-')
ax.set_ylabel("Thrust (N)")
ax.set_title("Thrust vs Time")
ax.grid(True, alpha=0.3)

# Attitude (pitch from vertical)
ax = axes[1, 1]
ax.plot(t, log["pitch_deg"], label="Pitch")
ax.plot(t, log["roll_deg"], label="Roll")
ax.plot(t, log["yaw_deg"], label="Yaw")
ax.set_ylabel("Angle (deg)")
ax.set_title("Euler Angles")
ax.legend()
ax.grid(True, alpha=0.3)

# Ground track (East vs North)
ax = axes[2, 0]
ax.plot(log["x"], log["y"], 'k-')
ax.set_xlabel("East (m)")
ax.set_ylabel("North (m)")
ax.set_title("Ground Track")
ax.set_aspect("equal")
ax.grid(True, alpha=0.3)

# Vertical acceleration
ax = axes[2, 1]
accel_mag = np.sqrt(np.array(log["accel_x"])**2 +
                    np.array(log["accel_y"])**2 +
                    np.array(log["accel_z"])**2)
ax.plot(t, log["accel_z"], 'b-', label="Vertical (z)")
ax.plot(t, accel_mag, 'r--', alpha=0.5, label="|a| total")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Acceleration (m/s²)")
ax.set_title("Acceleration")
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("/Users/christianpedersen/Documents/Hobbies/Model Rockets/Code/tinkerrocket-sim/6dof_test_flight.png", dpi=150)
plt.close()
print("Plot saved to 6dof_test_flight.png")
