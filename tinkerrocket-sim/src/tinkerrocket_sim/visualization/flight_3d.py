"""3D flight visualization — trajectory, rocket model, and animation.

Renders the rocket flying along its simulated trajectory using matplotlib 3D.
The rocket model is built from .ork geometry and rotated by the quaternion
attitude at each timestep.

Coordinate system:
    Simulation: ENU (East-North-Up) inertial frame
    Rocket body: X forward (nose), Y right, Z down (FRD in flight computer)
    Display: same as ENU — X=East, Y=North, Z=Up

Quaternion convention (from sim): NED/FRD body-to-NED rotation.
We convert to ENU for display.
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
from matplotlib import animation
from pathlib import Path
from typing import Optional

from .rocket_3d import RocketMesh


# ---------------------------------------------------------------------------
# Quaternion utilities
# ---------------------------------------------------------------------------

def _quat_to_rotation_matrix(q0, q1, q2, q3):
    """Convert scalar-first quaternion to 3x3 rotation matrix.

    NED/FRD convention: rotates body-frame vectors into NED frame.
    """
    # Normalize
    norm = np.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    if norm < 1e-10:
        return np.eye(3)
    q0, q1, q2, q3 = q0/norm, q1/norm, q2/norm, q3/norm

    return np.array([
        [1 - 2*(q2*q2 + q3*q3),   2*(q1*q2 - q0*q3),     2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3),       1 - 2*(q1*q1 + q3*q3), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2),       2*(q2*q3 + q0*q1),     1 - 2*(q1*q1 + q2*q2)],
    ])


def _ned_to_enu_rotation():
    """Rotation matrix from NED to ENU frame."""
    return np.array([
        [0, 1, 0],
        [1, 0, 0],
        [0, 0, -1],
    ], dtype=float)


def _transform_rocket_points(points, q0, q1, q2, q3, position_enu):
    """Transform rocket body-frame points to ENU world frame.

    Args:
        points: (N, 3) array in body frame (X=nose, Y=right, Z=down).
        q0..q3: NED/FRD quaternion (scalar-first).
        position_enu: (3,) ENU position of rocket CG.

    Returns:
        (N, 3) array in ENU frame.
    """
    # Body FRD to NED rotation
    R_body_to_ned = _quat_to_rotation_matrix(q0, q1, q2, q3)
    # NED to ENU
    R_ned_to_enu = _ned_to_enu_rotation()
    # Combined: body FRD to ENU
    R = R_ned_to_enu @ R_body_to_ned

    # Rocket mesh is in body frame with X=nose direction.
    # FRD body frame has X forward (nose), Y starboard, Z down.
    # The mesh already uses this convention.
    return (R @ points.T).T + position_enu


# ---------------------------------------------------------------------------
# Static 3D trajectory with rocket snapshots
# ---------------------------------------------------------------------------

def plot_flight_3d(df, rocket_mesh: RocketMesh,
                   n_snapshots: int = 8,
                   scale: float = 0.0,
                   title: str = "3D Flight Visualization",
                   save_path: Optional[str | Path] = None,
                   show: bool = True,
                   elev: float = 25.0,
                   azim: float = -60.0):
    """Plot 3D trajectory with rocket model snapshots at key times.

    Args:
        df: Simulation DataFrame with x, y, z, true_q0_ned..true_q3_ned columns.
        rocket_mesh: RocketMesh instance.
        n_snapshots: Number of rocket models to place along the trajectory.
        scale: Visual scale factor for rocket model. 0 = auto-scale.
        title: Plot title.
        save_path: If provided, save figure.
        show: Whether to display interactively.
        elev, azim: Initial viewing angles.

    Returns:
        matplotlib Figure.
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    x = df['x'].values  # East
    y = df['y'].values  # North
    z = df['z'].values  # Up

    # --- Trajectory line ---
    # Color by altitude
    from matplotlib.collections import LineCollection
    points = np.array([x, y, z]).T.reshape(-1, 1, 3)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Use simple color gradient along time
    t = df['time'].values
    colors = plt.cm.viridis(np.linspace(0, 1, len(t)))

    # Plot trajectory as a thin line
    ax.plot(x, y, z, color='#333333', linewidth=0.8, alpha=0.4, zorder=1)

    # Colored trajectory (thicker, semi-transparent)
    for i in range(0, len(x)-1, max(1, len(x)//500)):
        j = min(i + max(1, len(x)//500), len(x)-1)
        frac = i / max(1, len(x)-1)
        color = plt.cm.plasma(frac)
        ax.plot(x[i:j+1], y[i:j+1], z[i:j+1], color=color,
                linewidth=1.5, alpha=0.6, zorder=2)

    # --- Ground track (shadow) ---
    ax.plot(x, y, np.zeros_like(z), color='gray', linewidth=0.5,
            alpha=0.3, linestyle='--', zorder=0)

    # --- Launch pad marker ---
    ax.scatter([0], [0], [0], color='red', s=100, marker='*',
               zorder=5, label='Launch Pad')

    # --- Apogee marker ---
    apogee_idx = np.argmax(z)
    ax.scatter([x[apogee_idx]], [y[apogee_idx]], [z[apogee_idx]],
               color='gold', s=80, marker='^', zorder=5,
               label=f'Apogee ({z[apogee_idx]:.0f} m)')

    # --- Auto-scale rocket model ---
    if scale <= 0:
        # Scale so the rocket is clearly visible against the trajectory.
        # A real rocket is ~1 m long against a ~500 m trajectory, so we need
        # a large scale factor.  Target: rocket appears ~8% of plot extent.
        trajectory_extent = max(
            np.ptp(x) if np.ptp(x) > 0 else 1,
            np.ptp(y) if np.ptp(y) > 0 else 1,
            np.ptp(z) if np.ptp(z) > 0 else 1,
        )
        scale = trajectory_extent * 0.18 / max(rocket_mesh.total_length, 0.01)
        scale = max(scale, 1.0)  # Never shrink below actual size

    # --- Snapshot times ---
    # Choose interesting times: launch, max-q, burnout, apogee, descent
    has_thrust = 'thrust' in df.columns
    snapshot_times = _pick_snapshot_times(df, n_snapshots)

    # Check for quaternion columns
    has_quat = all(c in df.columns for c in
                   ['true_q0_ned', 'true_q1_ned', 'true_q2_ned', 'true_q3_ned'])

    # --- Draw rocket at each snapshot ---
    for snap_t in snapshot_times:
        idx = np.argmin(np.abs(t - snap_t))
        pos_enu = np.array([x[idx], y[idx], z[idx]])

        if has_quat:
            q0 = df['true_q0_ned'].iloc[idx]
            q1 = df['true_q1_ned'].iloc[idx]
            q2 = df['true_q2_ned'].iloc[idx]
            q3 = df['true_q3_ned'].iloc[idx]
        else:
            # Default: pointing up (NED convention: nose points North = down in NED)
            q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0

        _draw_rocket(ax, rocket_mesh, pos_enu, q0, q1, q2, q3,
                     scale=scale, alpha=0.7)

    # --- Axis labels and formatting ---
    ax.set_xlabel('East (m)', fontsize=10)
    ax.set_ylabel('North (m)', fontsize=10)
    ax.set_zlabel('Up (m)', fontsize=10)
    ax.set_title(title, fontsize=13, fontweight='bold')

    # Equal aspect ratio
    _set_equal_aspect(ax, x, y, z)

    ax.view_init(elev=elev, azim=azim)
    ax.legend(loc='upper left', fontsize=9)

    fig.tight_layout()

    if save_path:
        fig.savefig(str(save_path), dpi=150, bbox_inches='tight')
        print(f"3D plot saved to {save_path}")

    if show:
        plt.show()

    return fig


def _pick_snapshot_times(df, n: int):
    """Pick n interesting snapshot times from the simulation."""
    t = df['time'].values
    z = df['z'].values
    t_max = t[-1]

    if n <= 0:
        return []

    # Key events
    events = []

    # Launch (first movement)
    launch_idx = np.argmax(z > 1.0) if np.any(z > 1.0) else 0
    events.append(t[launch_idx])

    # Burnout (if thrust data available)
    if 'thrust' in df.columns:
        thrust = df['thrust'].values
        burn_mask = thrust > 0.1
        if np.any(burn_mask):
            burnout_idx = np.max(np.where(burn_mask))
            events.append(t[burnout_idx])

    # Apogee
    apogee_idx = np.argmax(z)
    events.append(t[apogee_idx])

    # Fill remaining slots with evenly-spaced times
    remaining = n - len(events)
    if remaining > 0:
        # Space between launch and slightly past apogee
        t_end = min(t[apogee_idx] + 2.0, t_max)
        t_start = events[0]
        extra = np.linspace(t_start, t_end, remaining + 2)[1:-1]
        events.extend(extra)

    # Sort and deduplicate (remove times too close together)
    events = sorted(set(events))
    if len(events) > n:
        # Subsample
        indices = np.linspace(0, len(events) - 1, n, dtype=int)
        events = [events[i] for i in indices]

    return events[:n]


def _draw_rocket(ax, mesh: RocketMesh, position_enu, q0, q1, q2, q3,
                 scale: float = 1.0, alpha: float = 0.8):
    """Draw a rocket model at a given position and orientation.

    Args:
        ax: matplotlib 3D axes.
        mesh: RocketMesh.
        position_enu: (3,) ENU position.
        q0..q3: NED/FRD quaternion.
        scale: Visual scale multiplier.
        alpha: Transparency.
    """
    # Build rotation matrix: body FRD → ENU
    R_body_to_ned = _quat_to_rotation_matrix(q0, q1, q2, q3)
    R_ned_to_enu = _ned_to_enu_rotation()
    R = R_ned_to_enu @ R_body_to_ned

    def transform(pts):
        """Transform (N,3) body-frame points to ENU world."""
        # Mesh convention: X=0 at nose tip, X increases toward tail.
        # FRD body frame: +X = forward (nose direction).
        # Convert mesh→FRD by negating X after centering on CG.
        body = pts.copy()
        body[:, 0] = -(body[:, 0] - mesh.cg_position)  # flip: nose at +X
        rotated = (R @ (body * scale).T).T
        return rotated + position_enu

    # --- Surfaces (nose, body tubes, transitions) ---
    for X, Y, Z, color in mesh.surfaces:
        # X, Y, Z are (n_x, n_theta) grids
        n_x, n_t = X.shape
        pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
        transformed = transform(pts)
        Xt = transformed[:, 0].reshape(n_x, n_t)
        Yt = transformed[:, 1].reshape(n_x, n_t)
        Zt = transformed[:, 2].reshape(n_x, n_t)

        ax.plot_surface(Xt, Yt, Zt, color=color, alpha=alpha,
                        edgecolor=color, linewidth=0.1, shade=True,
                        zorder=3)

    # --- Fins ---
    for verts, color in mesh.fins:
        transformed = transform(verts)
        # Draw as filled polygon
        poly = Poly3DCollection([transformed], alpha=alpha,
                                 facecolor=color, edgecolor='#222222',
                                 linewidth=0.8, zorder=4)
        ax.add_collection3d(poly)


def _set_equal_aspect(ax, x, y, z):
    """Set equal aspect ratio for 3D axes."""
    max_range = max(np.ptp(x), np.ptp(y), np.ptp(z)) / 2.0
    if max_range < 1:
        max_range = 1

    mid_x = (x.max() + x.min()) / 2.0
    mid_y = (y.max() + y.min()) / 2.0
    mid_z = (z.max() + z.min()) / 2.0

    # Ensure Z (altitude) axis starts near ground
    z_mid = max(mid_z, max_range)

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(max(0, z_mid - max_range), z_mid + max_range)


# ---------------------------------------------------------------------------
# Animated flight visualization
# ---------------------------------------------------------------------------

def animate_flight_3d(df, rocket_mesh: RocketMesh,
                      speed: float = 1.0,
                      fps: int = 30,
                      scale: float = 0.0,
                      title: str = "Flight Animation",
                      save_path: Optional[str | Path] = None,
                      show: bool = True):
    """Create an animated 3D flight visualization.

    Args:
        df: Simulation DataFrame.
        rocket_mesh: RocketMesh instance.
        speed: Playback speed multiplier.
        fps: Frames per second for saved animation.
        scale: Visual scale factor (0 = auto).
        title: Title.
        save_path: If provided, save as .mp4 or .gif.
        show: Display interactively.

    Returns:
        (fig, anim) tuple.
    """
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    x = df['x'].values
    y = df['y'].values
    z = df['z'].values
    t = df['time'].values

    has_quat = all(c in df.columns for c in
                   ['true_q0_ned', 'true_q1_ned', 'true_q2_ned', 'true_q3_ned'])

    # Auto-scale
    if scale <= 0:
        trajectory_extent = max(np.ptp(x), np.ptp(y), np.ptp(z), 1.0)
        scale = trajectory_extent * 0.08 / max(rocket_mesh.total_length, 0.01)
        scale = max(scale, 1.0)

    # Set up axes
    _set_equal_aspect(ax, x, y, z)
    ax.set_xlabel('East (m)')
    ax.set_ylabel('North (m)')
    ax.set_zlabel('Up (m)')

    # Launch pad
    ax.scatter([0], [0], [0], color='red', s=100, marker='*', zorder=5)

    # Downsample to animation frame rate
    dt_anim = 1.0 / fps / speed
    t_anim = np.arange(t[0], t[-1], dt_anim)

    # Interpolate trajectory for smooth animation
    x_anim = np.interp(t_anim, t, x)
    y_anim = np.interp(t_anim, t, y)
    z_anim = np.interp(t_anim, t, z)

    if has_quat:
        q0_anim = np.interp(t_anim, t, df['true_q0_ned'].values)
        q1_anim = np.interp(t_anim, t, df['true_q1_ned'].values)
        q2_anim = np.interp(t_anim, t, df['true_q2_ned'].values)
        q3_anim = np.interp(t_anim, t, df['true_q3_ned'].values)

    # Plot elements that persist
    trail_line, = ax.plot([], [], [], color='#333333', linewidth=1.0, alpha=0.5)
    time_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes, fontsize=11)
    alt_text = ax.text2D(0.02, 0.91, '', transform=ax.transAxes, fontsize=10)
    speed_text = ax.text2D(0.02, 0.87, '', transform=ax.transAxes, fontsize=10)

    # Rocket model artists (will be recreated each frame)
    rocket_artists = []

    def init():
        trail_line.set_data_3d([], [], [])
        time_text.set_text('')
        alt_text.set_text('')
        speed_text.set_text('')
        return trail_line, time_text, alt_text, speed_text

    def update(frame):
        nonlocal rocket_artists
        # Remove previous rocket artists
        for artist in rocket_artists:
            try:
                artist.remove()
            except Exception:
                pass
        rocket_artists = []

        # Trail
        trail_line.set_data_3d(x_anim[:frame+1], y_anim[:frame+1], z_anim[:frame+1])

        # Current position
        pos = np.array([x_anim[frame], y_anim[frame], z_anim[frame]])

        if has_quat:
            q = (q0_anim[frame], q1_anim[frame], q2_anim[frame], q3_anim[frame])
        else:
            q = (1.0, 0.0, 0.0, 0.0)

        # Draw rocket
        artists = _draw_rocket_animated(
            ax, rocket_mesh, pos, *q, scale=scale, alpha=0.85)
        rocket_artists = artists

        # Info text
        current_t = t_anim[frame]
        time_text.set_text(f'T = {current_t:.2f} s')
        alt_text.set_text(f'Alt = {z_anim[frame]:.1f} m')

        # Compute speed at this time
        spd_vals = df['speed'].values
        spd = np.interp(current_t, t, spd_vals)
        speed_text.set_text(f'Speed = {spd:.1f} m/s')

        return trail_line, time_text, alt_text, speed_text

    ax.set_title(title, fontsize=13, fontweight='bold')

    n_frames = len(t_anim)
    anim = animation.FuncAnimation(
        fig, update, init_func=init,
        frames=n_frames, interval=1000/fps, blit=False)

    if save_path:
        save_path = Path(save_path)
        if save_path.suffix == '.gif':
            writer = animation.PillowWriter(fps=fps)
        else:
            writer = animation.FFMpegWriter(fps=fps, bitrate=2000)
        anim.save(str(save_path), writer=writer)
        print(f"Animation saved to {save_path}")

    if show:
        plt.show()

    return fig, anim


def _draw_rocket_animated(ax, mesh, position_enu, q0, q1, q2, q3,
                          scale=1.0, alpha=0.8):
    """Draw rocket and return list of artists for removal."""
    R_body_to_ned = _quat_to_rotation_matrix(q0, q1, q2, q3)
    R_ned_to_enu = _ned_to_enu_rotation()
    R = R_ned_to_enu @ R_body_to_ned

    artists = []

    def transform(pts):
        body = pts.copy()
        body[:, 0] = -(body[:, 0] - mesh.cg_position)  # flip: nose at +X (FRD)
        rotated = (R @ (body * scale).T).T
        return rotated + position_enu

    # Surfaces — use wireframe for animation (faster than plot_surface)
    for X, Y, Z, color in mesh.surfaces:
        n_x, n_t = X.shape
        pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
        transformed = transform(pts)
        Xt = transformed[:, 0].reshape(n_x, n_t)
        Yt = transformed[:, 1].reshape(n_x, n_t)
        Zt = transformed[:, 2].reshape(n_x, n_t)

        # For animation, use reduced wireframe for speed
        stride = max(1, n_x // 8)
        t_stride = max(1, n_t // 8)
        surf = ax.plot_surface(Xt, Yt, Zt, color=color, alpha=alpha * 0.7,
                               edgecolor='none', shade=True,
                               rstride=stride, cstride=t_stride)
        artists.append(surf)

    # Fins
    for verts, color in mesh.fins:
        transformed = transform(verts)
        poly = Poly3DCollection([transformed], alpha=alpha * 0.9,
                                 facecolor=color, edgecolor='#333333',
                                 linewidth=0.5)
        ax.add_collection3d(poly)
        artists.append(poly)

    return artists
