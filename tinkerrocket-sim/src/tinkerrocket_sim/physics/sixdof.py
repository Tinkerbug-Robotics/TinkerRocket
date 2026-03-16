"""6-DOF rigid body rocket dynamics with RK4 integration.

State vector (13 elements):
    [x, y, z, vx, vy, vz, q0, q1, q2, q3, wx, wy, wz]

Coordinate frames:
    - Inertial: ENU (East-North-Up), origin at launch site
    - Body: x=forward (along rocket axis toward nose), y=right, z=down
      (FRD body frame, consistent with flight convention)

Body axis convention:
    - Roll (wx): rotation about body x-axis (rocket longitudinal axis)
    - Pitch (wy): rotation about body y-axis
    - Yaw (wz): rotation about body z-axis
"""
import numpy as np
from dataclasses import dataclass
from typing import Callable, Optional

from . import atmosphere as atm
from .drag import compute_Cd


@dataclass
class SimState:
    """Simulation state at a point in time."""
    time: float = 0.0
    # Position ENU (m)
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    # Velocity ENU (m/s)
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    # Quaternion (body-to-ENU, scalar first: q0 + q1*i + q2*j + q3*k)
    q0: float = 1.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0
    # Angular velocity body frame (rad/s)
    wx: float = 0.0  # roll rate
    wy: float = 0.0  # pitch rate
    wz: float = 0.0  # yaw rate

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z,
                         self.vx, self.vy, self.vz,
                         self.q0, self.q1, self.q2, self.q3,
                         self.wx, self.wy, self.wz])

    @staticmethod
    def from_array(arr: np.ndarray, time: float = 0.0) -> 'SimState':
        return SimState(
            time=time,
            x=arr[0], y=arr[1], z=arr[2],
            vx=arr[3], vy=arr[4], vz=arr[5],
            q0=arr[6], q1=arr[7], q2=arr[8], q3=arr[9],
            wx=arr[10], wy=arr[11], wz=arr[12],
        )

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @property
    def velocity(self) -> np.ndarray:
        return np.array([self.vx, self.vy, self.vz])

    @property
    def quaternion(self) -> np.ndarray:
        return np.array([self.q0, self.q1, self.q2, self.q3])

    @property
    def omega(self) -> np.ndarray:
        return np.array([self.wx, self.wy, self.wz])

    @property
    def speed(self) -> float:
        return np.linalg.norm(self.velocity)

    @property
    def altitude(self) -> float:
        return self.z  # ENU: z is up

    @property
    def euler_deg(self) -> tuple:
        """Roll, pitch, yaw in degrees."""
        r, p, y = quat_to_euler(self.quaternion)
        return np.degrees(r), np.degrees(p), np.degrees(y)


# --- Quaternion utilities ---

def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_to_dcm(q: np.ndarray) -> np.ndarray:
    """Quaternion to Direction Cosine Matrix (body-to-ENU rotation).

    q = [q0, q1, q2, q3] where q0 is scalar.
    Returns R such that v_enu = R @ v_body.
    """
    q0, q1, q2, q3 = q
    R = np.array([
        [1 - 2*(q2*q2 + q3*q3), 2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2)],
        [2*(q1*q2 + q0*q3), 1 - 2*(q1*q1 + q3*q3), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1 - 2*(q1*q1 + q2*q2)],
    ])
    return R


def quat_to_euler(q: np.ndarray) -> tuple:
    """Quaternion to Euler angles (roll, pitch, yaw) in radians.

    Uses 3-2-1 (ZYX) convention.
    """
    q0, q1, q2, q3 = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q0*q1 + q2*q3)
    cosr_cosp = 1 - 2*(q1*q1 + q2*q2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q0*q2 - q3*q1)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q0*q3 + q1*q2)
    cosy_cosp = 1 - 2*(q2*q2 + q3*q3)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quat_enu_flu_to_ned_frd(q: np.ndarray) -> np.ndarray:
    """Convert body-to-ENU (FLU) quaternion to body-to-NED (FRD).

    Chain: FRD → FLU → ENU → NED
    q_FRD2NED = q_ENU2NED ⊗ q_FLU2ENU ⊗ q_FRD2FLU
    where q_ENU2NED = [0, 1/√2, 1/√2, 0] and q_FRD2FLU = [0, 1, 0, 0].
    Combined closed form: [-(a+d)/√2, -(b+c)/√2, (c-b)/√2, (d-a)/√2].
    """
    inv2 = 1.0 / np.sqrt(2.0)
    a, b, c, d = q[0], q[1], q[2], q[3]
    ew = -(a + d) * inv2
    ex = -(b + c) * inv2
    ey = (c - b) * inv2
    ez = (d - a) * inv2
    if ew < 0:
        ew, ex, ey, ez = -ew, -ex, -ey, -ez
    return np.array([ew, ex, ey, ez])


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Euler angles to quaternion (3-2-1 ZYX convention)."""
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)

    q0 = cr*cp*cy + sr*sp*sy
    q1 = sr*cp*cy - cr*sp*sy
    q2 = cr*sp*cy + sr*cp*sy
    q3 = cr*cp*sy - sr*sp*cy

    return np.array([q0, q1, q2, q3])


def quat_derivative(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """Quaternion time derivative: q_dot = 0.5 * q * [0, omega].

    Args:
        q: [q0, q1, q2, q3] quaternion.
        omega: [wx, wy, wz] angular velocity in body frame (rad/s).

    Returns:
        q_dot: time derivative of quaternion.
    """
    q0, q1, q2, q3 = q
    wx, wy, wz = omega

    q_dot = 0.5 * np.array([
        -q1*wx - q2*wy - q3*wz,
         q0*wx + q2*wz - q3*wy,
         q0*wy - q1*wz + q3*wx,
         q0*wz + q1*wy - q2*wx,
    ])
    return q_dot


# --- Physics engine ---

class SixDOF:
    """6-DOF rocket flight dynamics."""

    G = 9.80665  # m/s^2

    def __init__(self, rocket_def):
        """
        Args:
            rocket_def: RocketDefinition with geometry, mass, motor, fin tabs.
        """
        self.rocket = rocket_def

    def initial_state(self, launch_angle_deg: float = 85.0,
                      heading_deg: float = 0.0) -> SimState:
        """Create initial state on the launch pad.

        Args:
            launch_angle_deg: Angle from horizontal (90=vertical).
            heading_deg: Heading direction (0=North).

        Returns:
            SimState at t=0.
        """
        # Rocket body x-axis points along the rocket (toward nose)
        # At launch: body x-axis points along the launch angle
        elevation_rad = np.radians(launch_angle_deg)
        heading_rad = np.radians(heading_deg)

        # In ENU: rocket axis direction
        # heading=0 means North, elevation=90 means straight up
        # Body x -> ENU direction:
        #   East component: cos(elev) * sin(heading)
        #   North component: cos(elev) * cos(heading)
        #   Up component: sin(elev)
        #
        # DCM body-to-ENU: R @ [1,0,0] = [cos(yaw)*cos(pitch), sin(yaw)*cos(pitch), -sin(pitch)]
        # So: pitch = -elevation_rad (negative pitch = nose up)
        #     yaw = pi/2 - heading_rad (heading=0 is North=ENU-y, yaw=0 is East=ENU-x)
        pitch = -elevation_rad
        yaw = np.pi/2 - heading_rad

        q = euler_to_quat(0.0, pitch, yaw)

        return SimState(
            time=0.0,
            q0=q[0], q1=q[1], q2=q[2], q3=q[3],
        )

    def derivatives(self, state: np.ndarray, t: float,
                    fin_tab_deflection_deg: float = 0.0,
                    wind_enu: np.ndarray = None) -> np.ndarray:
        """Compute state derivatives for integration.

        Args:
            state: 13-element state vector.
            t: Current time in seconds.
            fin_tab_deflection_deg: Fin tab deflection in degrees.
            wind_enu: Wind velocity in ENU frame (m/s), or None for calm.

        Returns:
            13-element state derivative vector.
        """
        pos = state[0:3]
        vel = state[3:6]
        q = state[6:10]
        omega = state[10:13]

        # Normalize quaternion
        q = quat_normalize(q)

        # Rotation matrix body-to-ENU
        R_b2e = quat_to_dcm(q)
        R_e2b = R_b2e.T

        alt = max(pos[2], 0.0)  # altitude (z=up in ENU)

        # --- Mass properties at current time ---
        mass = self.rocket.mass_at(t)
        I_roll = self.rocket.I_roll_at(t)
        I_trans = self.rocket.I_transverse_at(t)
        I_body = np.diag([I_roll, I_trans, I_trans])

        # --- Forces in body frame ---
        forces_body = np.zeros(3)
        moments_body = np.zeros(3)

        # 1. Thrust (along body x-axis)
        thrust = self.rocket.motor.thrust_at(t)
        forces_body[0] += thrust

        # 2. Gravity in body frame
        g_enu = np.array([0.0, 0.0, -self.G])
        g_body = R_e2b @ g_enu
        forces_body += mass * g_body

        # 3. Aerodynamic forces (drag, normal force, pitching moment)
        if wind_enu is not None:
            v_air_enu = vel - wind_enu
        else:
            v_air_enu = vel.copy()

        v_air_body = R_e2b @ v_air_enu
        v_air_mag = np.linalg.norm(v_air_body)

        if v_air_mag > 0.1:
            rho = atm.air_density(alt)
            q_dyn = 0.5 * rho * v_air_mag**2
            S_ref = self.rocket.reference_area

            # Cd from Barrowman component buildup (velocity and altitude dependent)
            Cd = compute_Cd(self.rocket, v_air_mag, alt)

            # --- Angle of attack ---
            # Body frame: x=forward (nose), y=right, z=down (FRD)
            # Lateral velocity is perpendicular to rocket axis
            v_axial = v_air_body[0]  # along rocket axis
            v_lat_y = v_air_body[1]
            v_lat_z = v_air_body[2]
            v_lat_mag = np.sqrt(v_lat_y**2 + v_lat_z**2)

            # Total angle of attack — only meaningful when the rocket has
            # significant forward speed.  At low axial speed the wind
            # dominates and the Barrowman model isn't valid, but dynamic
            # pressure is also tiny so forces are negligible.
            if v_axial > 1.0:
                alpha = np.arctan2(v_lat_mag, v_axial)
            else:
                alpha = 0.0

            # --- Axial drag (opposes velocity along body axis) ---
            # Include induced drag from angle of attack
            # At small α: ΔCd ≈ CNα × α² ≈ CNα × sin²(α)
            # The CN×sin(α) crossflow term adds to axial drag
            CNa = self.rocket.CNa_total
            if abs(alpha) > 1e-6 and CNa > 0:
                sin_a = np.sin(alpha)
                Cd_induced = CNa * sin_a * sin_a
                Cd_total = Cd + Cd_induced
            else:
                Cd_total = Cd

            drag_magnitude = q_dyn * S_ref * Cd_total
            # Drag opposes airspeed direction in body frame
            drag_direction = -v_air_body / v_air_mag
            forces_body += drag_magnitude * drag_direction

            # --- Normal force from angle of attack ---
            if abs(alpha) > 1e-4 and CNa > 0 and v_lat_mag > 1e-6:
                # Normal force coefficient:
                # - At small α: CN = CNα × α (linear Barrowman)
                # - At large α: CN = CNα × sin(α) × cos(α) (rolls off naturally)
                # This matches the linear model at small angles and handles
                # crossflow correctly at high AoA.
                sin_alpha = np.sin(alpha)
                cos_alpha = np.cos(alpha)
                CN = CNa * sin_alpha * cos_alpha

                F_normal = CN * q_dyn * S_ref

                # Direction: opposes lateral velocity (restoring force)
                # Acts in body Y-Z plane, perpendicular to rocket axis
                normal_dir_y = -v_lat_y / v_lat_mag
                normal_dir_z = -v_lat_z / v_lat_mag

                forces_body[1] += F_normal * normal_dir_y
                forces_body[2] += F_normal * normal_dir_z

                # --- Pitching moment from normal force at CP ---
                # The normal force acts at the CP location.
                # Moment about CG = r_CP × F_normal
                #
                # In body frame (x toward nose):
                #   r_CP = [CG - CP, 0, 0] from CG to CP along body axis
                #   (CG_from_nose < CP_from_nose for stable rocket
                #    → CG-CP < 0 → CP is behind CG in body x direction)
                cg_pos = self.rocket.cg_at(t)
                cp_pos = self.rocket.cp_from_nose
                dx = cg_pos - cp_pos  # negative for stable rocket (CP aft of CG)

                # Moment = r × F = [dx, 0, 0] × [0, Fy, Fz]
                # M_x = 0 (no roll moment from normal force)
                # M_y = -dx × Fz
                # M_z = dx × Fy
                moments_body[1] += -dx * (F_normal * normal_dir_z)
                moments_body[2] += dx * (F_normal * normal_dir_y)

            # --- Pitch/yaw damping ---
            # Opposes angular velocity in pitch and yaw.
            # M_damp = -C_damp_sum × q × S_ref / V × ω
            # where C_damp_sum = Σ(CNα_i × (x_CPi − x_CG)²)
            C_damp = self.rocket.C_pitch_damp
            if C_damp > 0 and v_air_mag > 1.0:
                damp_factor = C_damp * q_dyn * S_ref / v_air_mag
                moments_body[1] -= damp_factor * omega[1]  # pitch damping
                moments_body[2] -= damp_factor * omega[2]  # yaw damping

        # 4. Fin tab roll torque
        if abs(fin_tab_deflection_deg) > 0.001 and v_air_mag > 1.0:
            ft = self.rocket.fin_tabs
            V_ratio_sq = (v_air_mag / ft.V_ref) ** 2
            tau_roll = ft.n_tabs * ft.Kt_ref * V_ratio_sq * fin_tab_deflection_deg
            moments_body[0] += tau_roll

        # 5. Roll disturbance torque (constant, e.g. motor spin or asymmetry)
        if hasattr(self.rocket, 'roll_disturbance_torque'):
            moments_body[0] += self.rocket.roll_disturbance_torque

        # --- Equations of motion ---

        # Translational: F = m*a, compute acceleration in ENU
        a_body = forces_body / mass
        a_enu = R_b2e @ a_body

        # Rotational: Euler's equation I*omega_dot + omega x (I*omega) = M
        I_omega = I_body @ omega
        omega_dot = np.linalg.solve(I_body, moments_body - np.cross(omega, I_omega))

        # Quaternion kinematics
        q_dot = quat_derivative(q, omega)

        return np.concatenate([vel, a_enu, q_dot, omega_dot])

    def step_rk4(self, state: np.ndarray, t: float, dt: float,
                 fin_tab_deflection_deg: float = 0.0,
                 wind_enu: np.ndarray = None) -> np.ndarray:
        """Single RK4 integration step.

        Args:
            state: 13-element state vector.
            t: Current time.
            dt: Time step.
            fin_tab_deflection_deg: Fin tab deflection.
            wind_enu: Wind velocity ENU.

        Returns:
            New 13-element state vector at t + dt.
        """
        k1 = self.derivatives(state, t, fin_tab_deflection_deg, wind_enu)
        k2 = self.derivatives(state + 0.5*dt*k1, t + 0.5*dt, fin_tab_deflection_deg, wind_enu)
        k3 = self.derivatives(state + 0.5*dt*k2, t + 0.5*dt, fin_tab_deflection_deg, wind_enu)
        k4 = self.derivatives(state + dt*k3, t + dt, fin_tab_deflection_deg, wind_enu)

        new_state = state + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)

        # Re-normalize quaternion
        q = new_state[6:10]
        q = quat_normalize(q)
        new_state[6:10] = q

        return new_state


def run_passive_6dof(rocket_def,
                     duration: float = 30.0,
                     dt: float = 1e-4,
                     launch_angle_deg: float = 85.0,
                     log_interval: float = 0.001) -> dict:
    """Run a passive (no control) 6-DOF simulation.

    Args:
        rocket_def: RocketDefinition.
        duration: Maximum sim duration in seconds.
        dt: Physics timestep in seconds.
        launch_angle_deg: Launch angle from horizontal.
        log_interval: How often to log data (seconds).

    Returns:
        Dict with time-series arrays: time, altitude, speed, accel, roll_rate, etc.
    """
    sim = SixDOF(rocket_def)
    state_obj = sim.initial_state(launch_angle_deg)
    state = state_obj.to_array()
    t = 0.0

    # Logging
    log = {
        "time": [], "altitude": [], "speed": [],
        "accel_x": [], "accel_y": [], "accel_z": [],
        "roll_deg": [], "pitch_deg": [], "yaw_deg": [],
        "roll_rate_dps": [], "pitch_rate_dps": [], "yaw_rate_dps": [],
        "x": [], "y": [], "z": [],
        "vx": [], "vy": [], "vz": [],
        "thrust": [], "mass": [],
    }
    log_accum = 0.0
    max_alt = 0.0
    past_apogee = False

    n_steps = int(duration / dt)
    for step in range(n_steps):
        alt = state[2]  # z = up

        # Stop if rocket hit the ground after leaving it
        if alt < -1.0 and t > 1.0:
            break

        # Stop after landing (well past apogee and below ground)
        if past_apogee and alt < 0.0:
            break

        if alt > max_alt:
            max_alt = alt
        if max_alt > 10.0 and alt < max_alt - 5.0:
            past_apogee = True

        # Log at intervals
        log_accum += dt
        if log_accum >= log_interval:
            log_accum -= log_interval
            s = SimState.from_array(state, t)
            r, p, y = s.euler_deg

            log["time"].append(t)
            log["altitude"].append(alt)
            log["speed"].append(s.speed)
            log["x"].append(state[0])
            log["y"].append(state[1])
            log["z"].append(state[2])
            log["vx"].append(state[3])
            log["vy"].append(state[4])
            log["vz"].append(state[5])
            log["roll_deg"].append(r)
            log["pitch_deg"].append(p)
            log["yaw_deg"].append(y)
            log["roll_rate_dps"].append(np.degrees(state[10]))
            log["pitch_rate_dps"].append(np.degrees(state[11]))
            log["yaw_rate_dps"].append(np.degrees(state[12]))
            log["thrust"].append(rocket_def.motor.thrust_at(t))
            log["mass"].append(rocket_def.motor.mass_at(t))

            # Acceleration (approximate from derivatives)
            deriv = sim.derivatives(state, t)
            log["accel_x"].append(deriv[3])
            log["accel_y"].append(deriv[4])
            log["accel_z"].append(deriv[5])

        # Integrate
        state = sim.step_rk4(state, t, dt)
        t += dt

    # Convert to numpy arrays
    for key in log:
        log[key] = np.array(log[key])

    # Summary stats
    log["apogee_m"] = float(max_alt)
    if len(log["speed"]) > 0:
        log["max_speed_mps"] = float(np.max(log["speed"]))
    else:
        log["max_speed_mps"] = 0.0

    return log
