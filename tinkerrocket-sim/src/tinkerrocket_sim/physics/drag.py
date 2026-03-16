"""Barrowman component-buildup drag coefficient model.

Computes total axial Cd from rocket geometry, matching the approach used
by OpenRocket and standard references (Barrowman 1967, Mandell et al.).

Components:
    1. Body friction drag (turbulent flat-plate, form factor corrected)
    2. Nose pressure drag (shape-dependent)
    3. Fin friction drag (turbulent flat-plate, wetted area)
    4. Fin pressure drag (leading edge bluntness)
    5. Base drag (aft-end low-pressure wake)
    6. Fin-body interference drag

All Cd values are referenced to the body cross-section area (S_ref).
"""
import math
from . import atmosphere as atm


def compute_Cd(rocket_def, airspeed: float, altitude: float) -> float:
    """Compute total drag coefficient at given flight conditions.

    Args:
        rocket_def: RocketDefinition with geometry.
        airspeed: Airspeed in m/s.
        altitude: Altitude in meters.

    Returns:
        Total Cd referenced to body cross-section area.
    """
    if airspeed < 0.5:
        return 0.0

    rho = atm.air_density(altitude)
    T = atm.temperature(altitude)
    a = atm.speed_of_sound(altitude)
    Mach = airspeed / a if a > 0 else 0.0

    # Dynamic viscosity (Sutherland's law)
    mu = _dynamic_viscosity(T)
    if mu <= 0 or rho <= 0:
        return 0.5  # fallback

    d_body = rocket_def.body_diameter
    S_ref = rocket_def.reference_area
    L_body = rocket_def.body_length
    L_nose = rocket_def.nose_length
    nose_shape = getattr(rocket_def, 'nose_shape', 'ogive')

    # Fin geometry
    n_fins = rocket_def.fin_count
    root_chord = rocket_def.fin_root_chord
    tip_chord = rocket_def.fin_tip_chord
    span = rocket_def.fin_span
    fin_thickness = rocket_def.fin_thickness
    sweep = rocket_def.fin_sweep

    # --- 1. Body friction drag ---
    Cd_body_friction = _body_friction_drag(
        airspeed, rho, mu, d_body, L_body, L_nose, S_ref, Mach)

    # --- 2. Nose pressure drag ---
    Cd_nose = _nose_pressure_drag(nose_shape, L_nose, d_body, S_ref, Mach)

    # --- 3. Fin friction drag ---
    Cd_fin_friction = _fin_friction_drag(
        airspeed, rho, mu, n_fins, root_chord, tip_chord, span,
        fin_thickness, S_ref, Mach)

    # --- 4. Fin pressure drag (leading edge) ---
    Cd_fin_pressure = _fin_pressure_drag(
        n_fins, fin_thickness, root_chord, tip_chord, span, sweep, S_ref, Mach)

    # --- 5. Base drag ---
    Cd_base = _base_drag(Mach)

    # --- 6. Interference drag ---
    Cd_interference = _interference_drag(
        n_fins, fin_thickness, d_body, root_chord, S_ref)

    # --- 7. Protuberance drag (rail buttons, launch lugs, etc.) ---
    Cd_protuberance = _protuberance_drag(rocket_def, S_ref)

    Cd_total = (Cd_body_friction + Cd_nose + Cd_fin_friction +
                Cd_fin_pressure + Cd_base + Cd_interference +
                Cd_protuberance)

    return Cd_total


def _dynamic_viscosity(T: float) -> float:
    """Sutherland's law for dynamic viscosity of air.

    Args:
        T: Temperature in Kelvin.

    Returns:
        Dynamic viscosity in Pa-s.
    """
    if T <= 0:
        return 0.0
    T_ref = 291.15  # K
    mu_ref = 1.827e-5  # Pa-s
    C = 120.0  # K (Sutherland constant)
    return mu_ref * (T / T_ref) ** 1.5 * (T_ref + C) / (T + C)


def _skin_friction_Cf(Re: float, Mach: float,
                      length: float = 1.0,
                      roughness: float = 60e-6) -> float:
    """Turbulent flat-plate skin friction coefficient with surface roughness.

    Uses the Schlichting compressible formula for smooth surfaces,
    and the roughness-limited Cf when roughness dominates.
    OpenRocket uses 60 µm for "normal" finish (painted surface).

    Args:
        Re: Reynolds number.
        Mach: Mach number.
        length: Reference length for roughness ratio (m).
        roughness: Surface roughness height (m). Default 60 µm (painted).
    """
    if Re < 1e4:
        # Laminar (Blasius)
        return 1.328 / math.sqrt(max(Re, 1.0))

    # Smooth turbulent (Schlichting)
    log_Re = math.log10(Re)
    Cf_smooth = 0.455 / (log_Re ** 2.58)

    # Roughness-limited Cf (Schlichting-Prandtl, fully rough)
    # Cf = (1 / (1.89 + 1.62 * log10(L/ks)))^2.5
    if roughness > 0 and length > 0:
        ratio = length / roughness
        if ratio > 1.0:
            denom = 1.89 + 1.62 * math.log10(ratio)
            Cf_rough = (1.0 / denom) ** 2.5
        else:
            Cf_rough = Cf_smooth
        # Use the larger of smooth or rough-limited
        Cf = max(Cf_smooth, Cf_rough)
    else:
        Cf = Cf_smooth

    # Compressibility correction
    if Mach > 0.01:
        Cf /= (1.0 + 0.144 * Mach * Mach) ** 0.65

    return Cf


def _body_friction_drag(V, rho, mu, d_body, L_body, L_nose, S_ref, Mach):
    """Body tube + nose cone friction drag."""
    # Reynolds number based on total length
    L_total = L_body
    Re = rho * V * L_total / mu
    Cf = _skin_friction_Cf(Re, Mach, length=L_total)

    # Wetted area of body (cylinder)
    L_cylinder = L_body - L_nose
    S_wet_body = math.pi * d_body * max(L_cylinder, 0.0)

    # Wetted area of nose (approximate as cone)
    slant = math.sqrt(L_nose**2 + (d_body/2)**2)
    S_wet_nose = math.pi * (d_body/2) * slant

    S_wet_total = S_wet_body + S_wet_nose

    # Form factor for body of revolution (Hoerner)
    fineness = L_total / d_body if d_body > 0 else 10.0
    k_body = 1.0 + 1.5 / fineness**1.5 + 7.0 / fineness**3

    Cd = k_body * Cf * S_wet_total / S_ref
    return Cd


def _nose_pressure_drag(shape, L_nose, d_body, S_ref, Mach):
    """Nose cone pressure (wave) drag.

    At subsonic speeds, well-designed noses have very low pressure drag.
    """
    fineness = L_nose / d_body if d_body > 0 else 3.0

    if shape in ('ogive', 'vonkarman', 'haack', 'lvhaack', 'parabolic'):
        # Very low subsonic pressure drag for pointed shapes
        Cd = 0.0
    elif shape == 'conical':
        Cd = 0.0
    elif shape in ('ellipsoid', 'elliptical'):
        # Ellipsoid nose: pressure drag depends on fineness ratio.
        # OpenRocket model: Cd_nose = 0.5 * sin^2(theta) where theta is
        # the half-angle at the tip. For an ellipsoid, sin(theta) = d/(2*L)
        # at the blunt front, but also includes boundary layer separation
        # effects for low fineness ratios.
        sin_half_angle = (d_body / 2) / max(L_nose, 0.01)
        # Hoerner (1965) blunt body formula with fineness correction
        Cd = 0.5 * sin_half_angle ** 2
        # Scale to S_ref (nose base area = S_ref for matching body diameter)
        # Already referenced to base area, which equals S_ref
    else:
        Cd = 0.0

    # Transonic drag rise (simple model)
    if Mach > 0.8:
        Cd += _transonic_wave_drag(Mach, fineness)

    return Cd


def _transonic_wave_drag(Mach, fineness):
    """Simple transonic/supersonic wave drag model."""
    if Mach <= 0.8:
        return 0.0
    elif Mach <= 1.2:
        # Drag rise region (linear interpolation)
        frac = (Mach - 0.8) / 0.4
        Cd_peak = 0.2 / fineness  # higher fineness = lower wave drag
        return Cd_peak * frac
    else:
        # Supersonic (Prandtl-Glauert decline)
        Cd_peak = 0.2 / fineness
        beta = math.sqrt(max(Mach**2 - 1.0, 0.01))
        return Cd_peak / beta


def _fin_friction_drag(V, rho, mu, n_fins, root_chord, tip_chord, span,
                       thickness, S_ref, Mach):
    """Fin set friction drag."""
    if n_fins == 0 or span <= 0:
        return 0.0

    # Mean aerodynamic chord
    mac = (root_chord + tip_chord) / 2.0
    if mac <= 0:
        return 0.0

    # Reynolds number based on MAC
    Re = rho * V * mac / mu
    Cf = _skin_friction_Cf(Re, Mach, length=mac)

    # Wetted area (both sides of all fins)
    S_fin_one = 0.5 * (root_chord + tip_chord) * span  # planform area
    S_wet_fins = 2.0 * n_fins * S_fin_one

    # Form factor for fins (function of thickness/chord ratio)
    tc_ratio = thickness / mac if mac > 0 else 0.05
    k_fin = 1.0 + 2.0 * tc_ratio

    Cd = k_fin * Cf * S_wet_fins / S_ref
    return Cd


def _fin_pressure_drag(n_fins, thickness, root_chord, tip_chord, span,
                       sweep, S_ref, Mach):
    """Fin leading/trailing edge pressure drag (bluntness drag).

    Based on the fin cross-section thickness. Subsonic only.
    """
    if n_fins == 0 or thickness <= 0 or span <= 0:
        return 0.0

    mac = (root_chord + tip_chord) / 2.0
    if mac <= 0:
        return 0.0

    tc_ratio = thickness / mac

    # LE drag coefficient for square LE (Hoerner)
    # Cd_le ≈ 0.11 * (t/c) for square cross-section
    # Cd_le ≈ 0.04 * (t/c) for rounded cross-section
    Cd_tc = 0.11 * tc_ratio  # assume square cross-section (conservative)

    # Frontal area of all fin leading edges
    # Approximate: n_fins * span * thickness
    S_frontal = n_fins * span * thickness

    Cd = Cd_tc * S_frontal / S_ref
    return Cd


def _base_drag(Mach):
    """Base drag from blunt aft end.

    Empirical correlation from Hoerner (1965):
        Cd_base ≈ 0.12 + 0.13 * M^2 for subsonic
    Referenced to base area = S_ref for a cylindrical rocket.
    """
    if Mach < 1.0:
        return 0.12 + 0.13 * Mach * Mach
    else:
        # Supersonic base drag decreases
        return 0.25 / Mach


def _interference_drag(n_fins, thickness, d_body, root_chord, S_ref):
    """Fin-body interference drag.

    Empirical: proportional to fin root junction area.
    Hoerner (1965): Cd_int ≈ 0.28 * (t/c)^2 per junction,
    referenced to root_chord * thickness area.
    """
    if n_fins == 0 or thickness <= 0 or root_chord <= 0:
        return 0.0

    # Junction area per fin
    S_junction = root_chord * thickness
    # Interference Cd per junction (referenced to junction area)
    Cd_junction = 0.28

    Cd = Cd_junction * n_fins * S_junction / S_ref
    return Cd


def _protuberance_drag(rocket_def, S_ref):
    """Drag from rail buttons, launch lugs, and other protuberances.

    Uses the number of mass components tagged as rail buttons or launch lugs
    in the .ork parser to estimate frontal area. Each rail button is modeled
    as a bluff body with Cd ≈ 0.8 (Hoerner).

    For a typical 1010 rail button:
        ~11mm OD x ~7.5mm height → frontal area ≈ 82.5 mm^2
    """
    # Count rail buttons / launch lugs from parsed data
    # (stored in mass_components from ork_parser)
    n_buttons = getattr(rocket_def, '_n_rail_buttons', 2)  # default 2

    # Typical 1010 rail button dimensions
    button_width = 0.0111   # m (11.1mm)
    button_height = 0.0076  # m (7.6mm)
    S_frontal_each = button_width * button_height

    # Bluff body Cd referenced to frontal area
    Cd_button = 0.8

    Cd = Cd_button * n_buttons * S_frontal_each / S_ref
    return Cd
