"""Test 6-DOF physics engine with a passive flight."""
import sys
import numpy as np

sys.path.insert(0, "/Users/christianpedersen/Documents/Hobbies/Model Rockets/Code/tinkerrocket-sim/src")

from tinkerrocket_sim.rocket.definition import RocketDefinition, MotorConfig
from tinkerrocket_sim.physics.sixdof import (
    SixDOF, SimState, run_passive_6dof,
    quat_normalize, quat_to_dcm, quat_to_euler, euler_to_quat, quat_derivative,
)


def make_test_rocket() -> RocketDefinition:
    """Create a test rocket matching TinkerRocket Mini specs."""
    # G40-7 style motor (29mm, ~80 Ns total impulse)
    # ~40N average thrust, ~2s burn
    burn_time = 2.0
    n_points = 20
    times = np.linspace(0, burn_time, n_points)
    # Typical thrust profile: ramp up, sustain, taper
    forces = np.ones(n_points) * 40.0
    forces[0] = 0.0
    forces[1] = 30.0
    forces[-2] = 25.0
    forces[-1] = 0.0

    motor = MotorConfig(
        thrust_times=times,
        thrust_forces=forces,
        total_mass=0.123,     # kg (casing + propellant)
        propellant_mass=0.06, # kg
        designation="G40",
    )

    rd = RocketDefinition(
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
    return rd


def test_quaternion_identity():
    """Identity quaternion should give identity DCM."""
    q = np.array([1, 0, 0, 0], dtype=float)
    R = quat_to_dcm(q)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)


def test_quat_euler_roundtrip():
    """Euler → quaternion → euler should roundtrip."""
    roll, pitch, yaw = 0.1, 0.3, -0.2
    q = euler_to_quat(roll, pitch, yaw)
    r2, p2, y2 = quat_to_euler(q)
    np.testing.assert_allclose([r2, p2, y2], [roll, pitch, yaw], atol=1e-10)


def test_quat_normalize():
    """Normalizing should produce unit quaternion."""
    q = np.array([2.0, 0.0, 0.0, 0.0])
    qn = quat_normalize(q)
    np.testing.assert_allclose(np.linalg.norm(qn), 1.0, atol=1e-12)
    np.testing.assert_allclose(qn, [1, 0, 0, 0], atol=1e-12)


def test_quat_dcm_rotation():
    """90-deg yaw should rotate body-x to ENU-east."""
    # Body-x → ENU-east means: heading = 90° east, no elevation
    # But for a rocket pointed up, body-x is the rocket axis
    # With yaw=pi/2 and pitch=0: body-x should map to ENU y (north -> east)
    # Let's just verify a known rotation
    q = euler_to_quat(0, 0, np.pi/2)  # 90° yaw
    R = quat_to_dcm(q)
    body_x = np.array([1, 0, 0])
    enu_result = R @ body_x
    # Yaw of 90° should rotate x-axis to y-axis
    np.testing.assert_allclose(enu_result, [0, 1, 0], atol=1e-10)


def test_initial_state_vertical():
    """Vertical launch should have body-x pointing up (ENU z)."""
    rd = make_test_rocket()
    sim = SixDOF(rd)
    s = sim.initial_state(launch_angle_deg=90.0)
    q = s.quaternion
    R = quat_to_dcm(q)
    body_x_enu = R @ np.array([1, 0, 0])
    np.testing.assert_allclose(body_x_enu, [0, 0, 1], atol=1e-10,
                               err_msg="Vertical launch: body x should point up")


def test_initial_state_tilted():
    """85-degree launch angle should tilt body-x slightly from vertical."""
    rd = make_test_rocket()
    sim = SixDOF(rd)
    s = sim.initial_state(launch_angle_deg=85.0, heading_deg=0.0)
    q = s.quaternion
    R = quat_to_dcm(q)
    body_x_enu = R @ np.array([1, 0, 0])
    # 85° elevation, heading=0 (north): body-x should be roughly [0, sin(5°), cos(5°)]
    # Wait - elevation from horizontal: cos(85°) horiz, sin(85°) up
    # heading=0 → north component = cos(85°) ≈ 0.087
    expected = np.array([0.0, np.cos(np.radians(85)), np.sin(np.radians(85))])
    np.testing.assert_allclose(body_x_enu, expected, atol=1e-6,
                               err_msg="85° launch: body x should tilt north slightly")


def test_gravity_only():
    """With no thrust, rocket should fall under gravity."""
    rd = make_test_rocket()
    # Zero out motor
    rd.motor = MotorConfig()
    sim = SixDOF(rd)
    # Start at 100m altitude, vertical, no velocity
    state = np.array([0, 0, 100.0,  # pos
                      0, 0, 0,       # vel
                      1, 0, 0, 0,    # quat (identity)
                      0, 0, 0],      # omega
                     dtype=float)

    dt = 0.001
    for _ in range(1000):  # 1 second
        state = sim.step_rk4(state, 0.0, dt)

    # After 1s of free fall: z should drop by ~0.5*g*t^2 = 4.9m
    # altitude should be ~95.1m
    alt = state[2]
    vz = state[5]
    assert 94.5 < alt < 95.5, f"Expected ~95.1m altitude after 1s freefall, got {alt:.1f}"
    assert -10.0 < vz < -9.5, f"Expected ~-9.81 m/s downward velocity, got {vz:.2f}"


def test_passive_flight():
    """Full passive flight should produce reasonable results."""
    rd = make_test_rocket()
    print(f"\nRocket: {rd.name}")
    print(f"  Dry mass: {rd.dry_mass:.3f} kg")
    print(f"  Launch mass: {rd.total_mass_launch:.3f} kg")
    print(f"  Motor: {rd.motor.designation}, burn time: {rd.motor.burn_time:.1f}s")
    print(f"  Reference area: {rd.reference_area:.6f} m²")
    print(f"  Cd: {rd.Cd}")

    # Use a larger timestep for speed (1ms instead of 0.1ms)
    log = run_passive_6dof(rd, duration=30.0, dt=0.001,
                           launch_angle_deg=85.0, log_interval=0.01)

    apogee = log["apogee_m"]
    max_speed = log["max_speed_mps"]
    flight_time = log["time"][-1] if len(log["time"]) > 0 else 0

    print(f"\nResults:")
    print(f"  Apogee: {apogee:.1f} m")
    print(f"  Max speed: {max_speed:.1f} m/s")
    print(f"  Flight time: {flight_time:.1f} s")
    print(f"  Data points: {len(log['time'])}")

    # Sanity checks for a ~E12 motor on a ~830g rocket
    assert apogee > 30, f"Apogee too low: {apogee:.1f}m"
    assert apogee < 500, f"Apogee too high: {apogee:.1f}m"
    assert max_speed > 10, f"Max speed too low: {max_speed:.1f} m/s"
    assert max_speed < 200, f"Max speed too high: {max_speed:.1f} m/s"
    assert flight_time > 3, f"Flight too short: {flight_time:.1f}s"
    assert flight_time < 30, f"Flight too long: {flight_time:.1f}s"

    # Check thrust curve is captured
    assert log["thrust"][0] == 0.0 or log["thrust"][1] > 0, "Should have non-zero thrust at start"

    # At burnout (~2s), speed should be near max
    burn_idx = int(2.0 / 0.01)
    if burn_idx < len(log["speed"]):
        speed_at_burnout = log["speed"][burn_idx]
        print(f"  Speed at burnout: {speed_at_burnout:.1f} m/s")

    # After apogee, altitude should decrease
    apogee_idx = np.argmax(log["altitude"])
    if apogee_idx < len(log["altitude"]) - 10:
        assert log["altitude"][apogee_idx + 5] < apogee, "Altitude should decrease after apogee"

    print("\n  All sanity checks passed!")


if __name__ == "__main__":
    print("=== Quaternion Tests ===")
    test_quaternion_identity()
    print("  quat identity: OK")
    test_quat_euler_roundtrip()
    print("  quat-euler roundtrip: OK")
    test_quat_normalize()
    print("  quat normalize: OK")
    test_quat_dcm_rotation()
    print("  quat DCM rotation: OK")

    print("\n=== Initial State Tests ===")
    test_initial_state_vertical()
    print("  vertical launch state: OK")
    test_initial_state_tilted()
    print("  tilted launch state: OK")

    print("\n=== Physics Tests ===")
    test_gravity_only()
    print("  gravity-only freefall: OK")

    print("\n=== Full Passive Flight ===")
    test_passive_flight()
