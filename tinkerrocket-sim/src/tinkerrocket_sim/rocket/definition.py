"""Rocket definition dataclass combining .ork data with YAML config overrides."""
import math
import yaml
import numpy as np
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from .ork_parser import OrkData, parse_ork
from .motor_db import find_motor
from ..physics.aero import compute_aero_properties


@dataclass
class FinTabConfig:
    """Fin tab aerodynamic parameters from CFD analysis.

    Defaults from 67mm fin can CFD (March 2026, 67mm-fin-tab-analysis/).
    Previous design (57.4mm): Kt_ref=4.20e-3, n_tabs=4.
    """
    Kt_ref: float = 5.34e-3  # N-m/deg at V_ref per tab (67mm fin can CFD)
    V_ref: float = 95.0  # m/s reference velocity
    n_tabs: int = 3  # number of fin tabs (67mm fin can has 3 fins)
    deflection_min: float = -20.0  # deg
    deflection_max: float = 20.0  # deg


@dataclass
class CruciformFinConfig:
    """4-fin cruciform control surface aerodynamic config.

    For a cruciform (4-fin at 0/90/180/270 deg) arrangement where each fin
    can be independently deflected.  Torques scale with (V/V_ref)^2.

    Kt_roll: roll torque coefficient per degree of common-mode deflection
             across all 4 fins (N-m/deg at V_ref).  Sum of 4 fins.
    Kt_pitch: pitch torque coefficient per degree of differential deflection
              between top and bottom fins (N-m/deg at V_ref).
    Kt_yaw: yaw torque coefficient per degree of differential deflection
            between right and left fins (N-m/deg at V_ref).

    The pitch/yaw coefficients encode Kf_normal * L_arm where Kf_normal is
    the fin normal force per degree and L_arm is the axial moment arm from
    CG to fin CP.  These should be tuned from CFD or estimated.
    """
    Kt_roll: float = 5.34e-3   # N-m/deg at V_ref (total, all 4 fins)
    Kt_pitch: float = 0.02     # N-m/deg differential at V_ref
    Kt_yaw: float = 0.02       # N-m/deg differential at V_ref
    V_ref: float = 95.0        # reference airspeed (m/s)
    max_deflection: float = 20.0  # per-fin max deflection (deg)


@dataclass
class MotorConfig:
    """Motor/thrust configuration."""
    thrust_times: np.ndarray = field(default_factory=lambda: np.array([0.0]))
    thrust_forces: np.ndarray = field(default_factory=lambda: np.array([0.0]))
    total_mass: float = 0.0  # kg (casing + propellant)
    propellant_mass: float = 0.0  # kg
    designation: str = ""

    @property
    def burn_time(self) -> float:
        """Motor burn time in seconds."""
        if len(self.thrust_times) > 1:
            return float(self.thrust_times[-1])
        return 0.0

    @property
    def casing_mass(self) -> float:
        return self.total_mass - self.propellant_mass

    def thrust_at(self, t: float) -> float:
        """Interpolate thrust at time t."""
        if t < 0 or t > self.burn_time or len(self.thrust_times) < 2:
            return 0.0
        return float(np.interp(t, self.thrust_times, self.thrust_forces))

    def mass_at(self, t: float) -> float:
        """Motor mass at time t (linear propellant consumption)."""
        if t <= 0:
            return self.total_mass
        if t >= self.burn_time:
            return self.casing_mass
        frac = t / self.burn_time
        return self.total_mass - frac * self.propellant_mass


@dataclass
class RocketDefinition:
    """Complete rocket definition for simulation."""
    name: str = "Unnamed Rocket"

    # Geometry
    body_diameter: float = 0.067  # m (outer, 67mm fin can)
    body_length: float = 0.40  # m (total)
    nose_length: float = 0.10  # m
    nose_shape: str = "ogive"
    reference_area: float = 0.0  # m^2 (computed from body_diameter)

    # Fins
    fin_count: int = 3
    fin_root_chord: float = 0.102  # m
    fin_tip_chord: float = 0.064  # m
    fin_span: float = 0.075  # m
    fin_sweep: float = 0.0  # m
    fin_thickness: float = 0.003  # m

    # Mass properties
    dry_mass: float = 0.770  # kg (everything except propellant)
    I_roll_launch: float = 8.3e-4  # kg-m^2
    I_roll_burnout: float = 8.237e-4  # kg-m^2
    I_transverse_launch: float = 0.02  # kg-m^2 (pitch/yaw)
    I_transverse_burnout: float = 0.019  # kg-m^2
    cg_position: float = 0.20  # m from nose tip

    # Aerodynamics
    Cd: float = 0.5  # drag coefficient (used if Barrowman model unavailable)

    # Normal force / stability (from Barrowman equations)
    CNa_total: float = 0.0  # total normal force coefficient slope (per radian)
    cp_from_nose: float = 0.0  # center of pressure from nose tip (m)
    cg_from_nose: float = 0.0  # center of gravity from nose tip (m)
    fin_position_from_nose: float = 0.0  # fin root LE from nose tip (m)
    C_pitch_damp: float = 0.0  # pitch/yaw damping coefficient

    # Transition data for aerodynamics (list of dicts)
    _transitions: list = field(default_factory=list)

    # Disturbances
    roll_disturbance_torque: float = 0.0  # N-m constant roll torque (e.g. motor spin)

    # Motor
    motor: MotorConfig = field(default_factory=MotorConfig)

    # Fin tabs (roll-only control)
    fin_tabs: FinTabConfig = field(default_factory=FinTabConfig)

    # Cruciform fins (3-axis guided control)
    cruciform_fins: CruciformFinConfig = field(default_factory=CruciformFinConfig)

    @property
    def total_mass_launch(self) -> float:
        return self.dry_mass + self.motor.propellant_mass

    @property
    def total_mass_burnout(self) -> float:
        return self.dry_mass

    def mass_at(self, t: float) -> float:
        """Total rocket mass at time t."""
        return self.dry_mass + self.motor.mass_at(t) - self.motor.casing_mass

    def I_roll_at(self, t: float) -> float:
        """Roll moment of inertia at time t (linear interp during burn)."""
        if t <= 0:
            return self.I_roll_launch
        if t >= self.motor.burn_time:
            return self.I_roll_burnout
        frac = t / self.motor.burn_time
        return self.I_roll_launch + frac * (self.I_roll_burnout - self.I_roll_launch)

    def I_transverse_at(self, t: float) -> float:
        """Transverse moment of inertia at time t."""
        if t <= 0:
            return self.I_transverse_launch
        if t >= self.motor.burn_time:
            return self.I_transverse_burnout
        frac = t / self.motor.burn_time
        return self.I_transverse_launch + frac * (self.I_transverse_burnout - self.I_transverse_launch)

    def cg_at(self, t: float) -> float:
        """CG position from nose tip at time t.

        During motor burn, CG shifts forward as propellant is consumed
        from the aft end.
        """
        if self.cg_from_nose <= 0:
            return self.body_length * 0.5

        # Simple model: propellant mass is at the aft end (motor location)
        # As propellant burns, CG shifts forward
        dry_mass = self.dry_mass
        prop_mass_remaining = self.motor.mass_at(t) - self.motor.casing_mass
        total_mass = dry_mass + prop_mass_remaining

        if total_mass <= 0:
            return self.cg_from_nose

        # Motor CG is approximately at 80% of body length from nose
        motor_cg = self.body_length * 0.85

        # Weighted average of dry CG and propellant CG
        dry_cg = self.cg_from_nose
        if prop_mass_remaining > 0:
            cg = (dry_mass * dry_cg + prop_mass_remaining * motor_cg) / total_mass
        else:
            cg = dry_cg

        return cg

    def _compute_aero(self):
        """Compute Barrowman aerodynamic properties from geometry."""
        aero = compute_aero_properties(self)
        self.CNa_total = aero['CNa_total']
        self.cp_from_nose = aero['cp_from_nose']
        self.C_pitch_damp = aero['C_damp']

    def __post_init__(self):
        if self.reference_area == 0.0 and self.body_diameter > 0:
            self.reference_area = math.pi * (self.body_diameter / 2) ** 2
        # Compute aero properties if geometry is available and CNa not yet set
        if self.CNa_total == 0.0 and self.body_diameter > 0 and self.fin_count > 0:
            self._compute_aero()


def from_ork(ork_path: str | Path,
             config_path: Optional[str | Path] = None) -> RocketDefinition:
    """Build RocketDefinition from .ork file with optional YAML overrides.

    Args:
        ork_path: Path to OpenRocket .ork file.
        config_path: Optional path to sim_config.yaml for overrides.

    Returns:
        RocketDefinition ready for simulation.
    """
    ork = parse_ork(ork_path)
    rd = _ork_to_definition(ork)

    if config_path:
        _apply_yaml_overrides(rd, Path(config_path))

    return rd


def from_yaml(config_path: str | Path) -> RocketDefinition:
    """Build RocketDefinition purely from YAML config (no .ork file)."""
    rd = RocketDefinition()
    _apply_yaml_overrides(rd, Path(config_path))
    return rd


def _ork_to_definition(ork: OrkData) -> RocketDefinition:
    """Convert parsed OrkData to RocketDefinition."""
    rd = RocketDefinition()
    rd.name = ork.name or "Unnamed"
    rd.body_diameter = ork.reference_diameter
    rd.body_length = ork.total_length

    if ork.nose_cone:
        rd.nose_length = ork.nose_cone.length
        rd.nose_shape = ork.nose_cone.shape

    # Use first fin set found
    if ork.fin_sets:
        fs = ork.fin_sets[0]
        rd.fin_count = fs.fin_count
        rd.fin_root_chord = fs.root_chord
        rd.fin_tip_chord = fs.tip_chord
        rd.fin_span = fs.span
        rd.fin_sweep = fs.sweep_length
        rd.fin_thickness = fs.thickness
        rd.fin_position_from_nose = fs.position_from_nose

    # Sum up dry mass from all components
    total_mass = 0.0
    if ork.nose_cone:
        total_mass += ork.nose_cone.mass
    for bt in ork.body_tubes:
        total_mass += bt.mass
    for fs in ork.fin_sets:
        total_mass += fs.mass
    for tr in ork.transitions:
        total_mass += tr.mass
    for mc in ork.mass_components:
        total_mass += mc.mass
    rd.dry_mass = total_mass

    # CG position from .ork parser
    if ork.cg_from_nose > 0:
        rd.cg_from_nose = ork.cg_from_nose

    # Transition data for aerodynamic calculation
    rd._transitions = []
    for tr in ork.transitions:
        rd._transitions.append({
            'length': tr.length,
            'fore_diameter': tr.fore_diameter,
            'aft_diameter': tr.aft_diameter,
            'position_from_nose': tr.position_from_nose,
        })

    # Motor: prefer the default config, fall back to first motor
    motor_data = None
    if ork.motors:
        if ork.default_motor_configid:
            for m in ork.motors:
                if m.configid == ork.default_motor_configid:
                    motor_data = m
                    break
        if motor_data is None:
            motor_data = ork.motors[0]

    if motor_data is not None:
        thrust_times = motor_data.thrust_curve_times
        thrust_forces = motor_data.thrust_curve_forces
        total_mass = motor_data.total_mass
        propellant_mass = motor_data.propellant_mass

        # If .ork didn't embed thrust data, look up from .eng files
        if not thrust_times and motor_data.designation:
            eng = find_motor(motor_data.designation)
            if eng:
                thrust_times = eng.thrust_times
                thrust_forces = eng.thrust_forces
                if total_mass == 0:
                    total_mass = eng.total_mass
                if propellant_mass == 0:
                    propellant_mass = eng.propellant_mass

        if thrust_times:
            rd.motor = MotorConfig(
                thrust_times=np.array(thrust_times, dtype=float),
                thrust_forces=np.array(thrust_forces, dtype=float),
                total_mass=total_mass,
                propellant_mass=propellant_mass,
                designation=motor_data.designation,
            )
            # Add motor casing to dry mass
            rd.dry_mass += rd.motor.casing_mass
        else:
            print(f"WARNING: No thrust data found for motor '{motor_data.designation}'. "
                  f"Place a .eng file in the motors/ directory.")

    rd.reference_area = math.pi * (rd.body_diameter / 2) ** 2

    # Compute Barrowman aerodynamic properties (CNα, CP, damping)
    rd._compute_aero()

    return rd


def _apply_yaml_overrides(rd: RocketDefinition, config_path: Path):
    """Apply YAML config overrides to RocketDefinition."""
    if not config_path.exists():
        return

    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    if not cfg:
        return

    # Fin tab config
    ft = cfg.get("fin_tabs", {})
    if ft:
        rd.fin_tabs = FinTabConfig(
            Kt_ref=ft.get("Kt_ref", rd.fin_tabs.Kt_ref),
            V_ref=ft.get("V_ref", rd.fin_tabs.V_ref),
            n_tabs=ft.get("n_tabs", rd.fin_tabs.n_tabs),
            deflection_min=ft.get("deflection_min", rd.fin_tabs.deflection_min),
            deflection_max=ft.get("deflection_max", rd.fin_tabs.deflection_max),
        )

    # Mass/inertia overrides
    mass = cfg.get("mass_overrides", {})
    if mass:
        if "dry_mass" in mass:
            rd.dry_mass = mass["dry_mass"]
        if "I_roll_launch" in mass:
            rd.I_roll_launch = mass["I_roll_launch"]
        if "I_roll_burnout" in mass:
            rd.I_roll_burnout = mass["I_roll_burnout"]
        if "I_transverse_launch" in mass:
            rd.I_transverse_launch = mass["I_transverse_launch"]
        if "I_transverse_burnout" in mass:
            rd.I_transverse_burnout = mass["I_transverse_burnout"]
        if "cg_position" in mass:
            rd.cg_position = mass["cg_position"]

    # Aero overrides
    aero = cfg.get("aerodynamics", {})
    if aero:
        if "Cd" in aero:
            rd.Cd = aero["Cd"]
