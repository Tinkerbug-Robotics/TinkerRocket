"""RocketPy passive flight simulation wrapper.

Uses RocketPy for passive (uncontrolled) flight simulation as a validation
baseline. Builds a RocketPy Rocket from our RocketDefinition.
"""
import numpy as np
import yaml
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

from rocketpy import Environment, SolidMotor, Rocket, Flight


@dataclass
class PassiveFlightResult:
    """Results from a passive RocketPy simulation."""
    apogee_m: float = 0.0
    max_speed_mps: float = 0.0
    max_mach: float = 0.0
    flight_time_s: float = 0.0
    max_acceleration_g: float = 0.0
    # Time series (sampled)
    time: np.ndarray = None
    altitude: np.ndarray = None
    speed: np.ndarray = None
    acceleration: np.ndarray = None
    # RocketPy Flight object for detailed access
    flight: Flight = None


def run_passive_flight(rocket_def,
                       launch_site_config: Optional[str | Path] = None,
                       sim_duration: float = 30.0,
                       ) -> PassiveFlightResult:
    """Run a passive RocketPy flight simulation.

    Args:
        rocket_def: RocketDefinition instance.
        launch_site_config: Path to launch_site.yaml.
        sim_duration: Maximum simulation duration in seconds.

    Returns:
        PassiveFlightResult with trajectory data.
    """
    # Load launch site config
    site = _load_site_config(launch_site_config)

    # Set up environment
    env = Environment(
        latitude=site["latitude_deg"],
        longitude=site["longitude_deg"],
        elevation=site["elevation_m"],
    )
    env.set_atmospheric_model(type="standard_atmosphere")

    # Create motor from thrust curve
    motor = _build_motor(rocket_def)

    # Create rocket
    rocket = _build_rocket(rocket_def, motor)

    # Run flight
    rail_length = 1.0  # m default
    inclination = 85.0  # deg from horizontal
    heading = 0.0  # deg

    flight = Flight(
        rocket=rocket,
        environment=env,
        rail_length=rail_length,
        inclination=inclination,
        heading=heading,
        max_time=sim_duration,
        time_overshoot=True,
    )

    # Extract results
    result = PassiveFlightResult()
    result.flight = flight
    result.apogee_m = flight.apogee - env.elevation
    result.max_speed_mps = flight.max_speed
    result.max_mach = flight.max_mach_number
    result.flight_time_s = flight.t_final

    # Sample time series
    t_sample = np.linspace(0, flight.t_final, num=1000)
    result.time = t_sample

    try:
        result.altitude = np.array([
            float(flight.altitude(t)) for t in t_sample
        ])
    except Exception:
        result.altitude = np.zeros_like(t_sample)

    try:
        result.speed = np.array([
            float(flight.speed(t)) for t in t_sample
        ])
    except Exception:
        result.speed = np.zeros_like(t_sample)

    try:
        result.acceleration = np.array([
            float(flight.acceleration(t)) for t in t_sample
        ])
    except Exception:
        result.acceleration = np.zeros_like(t_sample)

    result.max_acceleration_g = float(flight.max_acceleration) / 9.80665

    return result


def _load_site_config(config_path: Optional[str | Path]) -> dict:
    """Load launch site config or return defaults."""
    defaults = {
        "latitude_deg": 38.0,
        "longitude_deg": -122.0,
        "elevation_m": 0.0,
    }
    if config_path is None:
        return defaults

    path = Path(config_path)
    if not path.exists():
        return defaults

    with open(path) as f:
        cfg = yaml.safe_load(f)

    site = cfg.get("launch_site", {})
    for key in defaults:
        if key not in site:
            site[key] = defaults[key]
    return site


def _build_motor(rocket_def) -> SolidMotor:
    """Build a RocketPy SolidMotor from RocketDefinition."""
    mc = rocket_def.motor

    # RocketPy needs thrust curve as list of (time, thrust)
    if len(mc.thrust_times) < 2:
        # No thrust curve available, create a simple placeholder
        thrust_source = [(0.0, 0.0), (0.1, 50.0), (1.0, 50.0), (1.1, 0.0)]
        grain_outer_radius = 0.01
        grain_initial_height = 0.05
        burn_time = 1.0
    else:
        thrust_source = list(zip(
            mc.thrust_times.tolist(),
            mc.thrust_forces.tolist()
        ))
        burn_time = mc.burn_time
        # Estimate grain dimensions from motor diameter
        grain_outer_radius = rocket_def.body_diameter / 2 * 0.6  # approximate
        grain_initial_height = 0.05  # approximate

    motor = SolidMotor(
        thrust_source=thrust_source,
        dry_mass=mc.casing_mass,
        dry_inertia=(0.0001, 0.0001, 0.00005),  # approximate
        center_of_dry_mass_position=0.0,
        nozzle_radius=rocket_def.body_diameter / 2 * 0.3,
        grain_number=1,
        grain_density=1800,  # approximate, kg/m^3
        grain_outer_radius=grain_outer_radius,
        grain_initial_inner_radius=grain_outer_radius * 0.3,
        grain_initial_height=grain_initial_height,
        grain_separation=0.0,
        grains_center_of_mass_position=0.0,
        nozzle_position=0.0,
        burn_time=burn_time,
        throat_radius=rocket_def.body_diameter / 2 * 0.15,
        coordinate_system_orientation="nozzle_to_combustion_chamber",
    )
    return motor


def _build_rocket(rocket_def, motor: SolidMotor) -> Rocket:
    """Build a RocketPy Rocket from RocketDefinition."""
    rocket = Rocket(
        radius=rocket_def.body_diameter / 2,
        mass=rocket_def.dry_mass,
        inertia=(
            rocket_def.I_transverse_launch,
            rocket_def.I_transverse_launch,
            rocket_def.I_roll_launch,
        ),
        power_off_drag=rocket_def.Cd,
        power_on_drag=rocket_def.Cd,
        center_of_mass_without_motor=0,
        coordinate_system_orientation="tail_to_nose",
    )

    # Add motor at the tail
    rocket.add_motor(motor, position=0)

    # Add nose cone
    rocket.add_nose(
        length=rocket_def.nose_length,
        kind=rocket_def.nose_shape if rocket_def.nose_shape in
            ["vonkarman", "ogive", "conical", "elliptical", "parabolic"]
            else "vonkarman",
        position=rocket_def.body_length,  # at the top
    )

    # Add trapezoidal fins
    if rocket_def.fin_count > 0 and rocket_def.fin_span > 0:
        rocket.add_trapezoidal_fins(
            n=rocket_def.fin_count,
            root_chord=rocket_def.fin_root_chord,
            tip_chord=rocket_def.fin_tip_chord,
            span=rocket_def.fin_span,
            sweep_length=rocket_def.fin_sweep,
            position=rocket_def.fin_root_chord,  # near the tail
        )

    return rocket
