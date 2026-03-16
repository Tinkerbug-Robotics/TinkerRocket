"""Barrowman aerodynamic normal force and center of pressure model.

Computes the total normal force coefficient slope (CNα) and center
of pressure (CP) position from rocket geometry using the Barrowman
(1967) equations, as implemented in OpenRocket and standard references
(Mandell, Caporaso, Bengen 1973).

Components:
    1. Nose cone: CNα_nose = 2.0 (for any pointed nose)
    2. Body tube: CNα_body ≈ 0 at small α (no normal force from cylinder)
    3. Fin set: Barrowman formula with fin-body interference factor
    4. Transition: conical transition lift (if diameter changes)

All CNα values are per radian, referenced to body cross-section area (S_ref).
"""
import math


# ============================================================================
# Nose Cone
# ============================================================================

def CNa_nose(nose_shape: str = "ogive") -> float:
    """Normal force coefficient slope for the nose cone.

    For any pointed nose shape, the subsonic CNα is 2.0 per radian,
    referenced to the base area (= S_ref when nose base matches body diameter).

    This is a fundamental result from slender body theory (von Kármán).
    """
    # All pointed nose shapes have CNα = 2.0 at subsonic speeds
    return 2.0


def cp_nose(nose_shape: str, nose_length: float) -> float:
    """Center of pressure position of the nose cone from the nose tip.

    Different nose shapes have different CP locations based on their
    pressure distributions (Barrowman 1967, Mandell et al. 1973).

    Args:
        nose_shape: Shape string (ogive, conical, ellipsoid, etc.)
        nose_length: Nose cone length in meters.

    Returns:
        CP position in meters from nose tip.
    """
    shape = nose_shape.lower()

    if shape in ('ogive', 'vonkarman', 'haack', 'lvhaack', 'parabolic'):
        return 0.466 * nose_length
    elif shape in ('conical', 'cone'):
        return 0.667 * nose_length
    elif shape in ('ellipsoid', 'elliptical'):
        # Ellipsoid distributes pressure more toward the base
        return 0.50 * nose_length
    elif shape in ('power', 'powerseries'):
        # Power series nose — depends on exponent, use ogive as default
        return 0.466 * nose_length
    else:
        # Default to ogive
        return 0.466 * nose_length


# ============================================================================
# Transition (boat tail or flare)
# ============================================================================

def CNa_transition(fore_diameter: float, aft_diameter: float,
                   body_diameter: float) -> float:
    """Normal force coefficient slope for a conical transition.

    Barrowman formula:
        CNα = 2 * ((d_aft/d_ref)² - (d_fore/d_ref)²)

    This accounts for the change in cross-sectional area creating lift.
    For a boat tail (aft < fore): negative CNα (destabilizing if aft of CG).
    For a flare (aft > fore): positive CNα.

    Args:
        fore_diameter: Forward diameter (m).
        aft_diameter: Aft diameter (m).
        body_diameter: Reference body diameter (m).

    Returns:
        CNα per radian, referenced to S_ref.
    """
    if body_diameter <= 0:
        return 0.0
    d_ref = body_diameter
    return 2.0 * ((aft_diameter / d_ref) ** 2 - (fore_diameter / d_ref) ** 2)


def cp_transition(transition_length: float, fore_diameter: float,
                  aft_diameter: float, transition_position: float) -> float:
    """Center of pressure of a conical transition from nose tip.

    Barrowman formula:
        x_CP = x_transition + L/3 * (1 + (1-d1/d2)/(1-(d1/d2)²))

    Simplified for the general case:
        x_CP ≈ x_transition + L/3 * (1 + (d_aft² + d_fore*d_aft) /
                                           (d_fore² + d_fore*d_aft + d_aft²))

    Wait, the correct Barrowman formula for transition CP (from transition front):
        x_CP_local = L/3 * [1 + (1 - d_fore/d_aft) / (1 - (d_fore/d_aft)²)]

    For a more robust formula using the volume centroid approach:
        x_CP_local = L * (d_fore² + 2*d_fore*d_aft + 3*d_aft²) /
                         (4 * (d_fore² + d_fore*d_aft + d_aft²))

    Args:
        transition_length: Length of transition (m).
        fore_diameter: Forward diameter (m).
        aft_diameter: Aft diameter (m).
        transition_position: Position of transition front from nose tip (m).

    Returns:
        CP position in meters from nose tip.
    """
    d1 = fore_diameter
    d2 = aft_diameter

    if transition_length <= 0:
        return transition_position

    # Volume-centroid based CP (robust for all diameter ratios)
    denom = d1 ** 2 + d1 * d2 + d2 ** 2
    if denom < 1e-12:
        x_local = transition_length / 3.0
    else:
        x_local = transition_length * (d1 ** 2 + 2 * d1 * d2 + 3 * d2 ** 2) / (4.0 * denom)

    return transition_position + x_local


# ============================================================================
# Fin Set
# ============================================================================

def CNa_fins(n_fins: int, root_chord: float, tip_chord: float,
             span: float, sweep: float, body_diameter: float,
             Mach: float = 0.0) -> float:
    """Normal force coefficient slope for a fin set (Barrowman 1967).

    Formula:
        CNα = K_fb × [4n(s/d)²] / [1 + √(1 + (2·lm/(Cr+Ct))²)]

    where:
        K_fb = 1 + d/(2s+d)  — fin-body interference factor
        n = number of fins
        s = semi-span (fin height from root to tip)
        d = body diameter at fin location
        lm = mid-chord line length = √(s² + (sweep + Ct/2 - Cr/2)²)
        Cr = root chord
        Ct = tip chord

    This formula comes from linear theory and is valid for subsonic flow
    with thin fins at small angles of attack.

    Args:
        n_fins: Number of fins.
        root_chord: Root chord length (m).
        tip_chord: Tip chord length (m).
        span: Fin semi-span / height (m).
        sweep: Leading edge sweep distance (m) — horizontal distance from
               root LE to tip LE.
        body_diameter: Body diameter at fin location (m).
        Mach: Mach number (for compressibility correction, currently unused
              for subsonic).

    Returns:
        CNα per radian, referenced to S_ref = π/4 × d².
    """
    if n_fins == 0 or span <= 0 or body_diameter <= 0:
        return 0.0

    s = span           # semi-span
    d = body_diameter
    Cr = root_chord
    Ct = tip_chord

    # Fin-body interference factor (Barrowman)
    K_fb = 1.0 + d / (2.0 * s + d)

    # Mid-chord line length
    mid_chord_x = sweep + Ct / 2.0 - Cr / 2.0
    lm = math.sqrt(s ** 2 + mid_chord_x ** 2)

    # Barrowman formula
    sum_chord = Cr + Ct
    if sum_chord <= 0:
        return 0.0

    arg = (2.0 * lm / sum_chord) ** 2
    denom = 1.0 + math.sqrt(1.0 + arg)

    CNa = K_fb * 4.0 * n_fins * (s / d) ** 2 / denom

    return CNa


def cp_fins(root_chord: float, tip_chord: float, span: float,
            sweep: float, fin_position_from_nose: float) -> float:
    """Center of pressure of fin set from nose tip (Barrowman 1967).

    The CP of the fin set, measured from the root chord leading edge:
        x_fin_cp = (Λ/3) × (Cr + 2Ct)/(Cr + Ct) +
                   (1/6) × (Cr + Ct - Cr·Ct/(Cr + Ct))

    where Λ is the leading edge sweep distance.

    Args:
        root_chord: Root chord (m).
        tip_chord: Tip chord (m).
        span: Semi-span (m).
        sweep: Leading edge sweep distance (m).
        fin_position_from_nose: Position of fin root leading edge from
                                 nose tip (m).

    Returns:
        CP position in meters from nose tip.
    """
    Cr = root_chord
    Ct = tip_chord

    sum_chord = Cr + Ct
    if sum_chord <= 0:
        return fin_position_from_nose

    # CP from fin root leading edge (Barrowman)
    x_local = (sweep / 3.0) * (Cr + 2.0 * Ct) / sum_chord
    x_local += (1.0 / 6.0) * (Cr + Ct - Cr * Ct / sum_chord)

    return fin_position_from_nose + x_local


# ============================================================================
# Pitch/Yaw Damping
# ============================================================================

def pitch_damping_sum(CNa_nose_val: float, cp_nose_pos: float,
                      CNa_fins_val: float, cp_fins_pos: float,
                      CNa_trans_val: float, cp_trans_pos: float,
                      cg_from_nose: float) -> float:
    """Compute the pitch damping sum: Σ(CNα_i × (x_CPi − x_CG)²).

    The pitch damping moment opposes angular velocity in pitch and yaw.
    When the rocket pitches at rate ω, each aerodynamic component sees a
    local change in angle of attack proportional to its distance from CG.
    This creates a restoring (damping) moment.

    The damping moment is:
        M_damp = -C_damp_sum × q × S_ref × ω / V

    where C_damp_sum = Σ(CNα_i × (x_i − x_CG)²) summed over all
    aerodynamic components (nose, fins, transitions).

    Returns:
        C_damp_sum in units of m² (per radian).
    """
    C = 0.0
    C += CNa_nose_val * (cp_nose_pos - cg_from_nose) ** 2
    if CNa_fins_val != 0:
        C += CNa_fins_val * (cp_fins_pos - cg_from_nose) ** 2
    if CNa_trans_val != 0 and cp_trans_pos > 0:
        C += CNa_trans_val * (cp_trans_pos - cg_from_nose) ** 2
    return C


# ============================================================================
# Total Aerodynamics
# ============================================================================

def compute_aero_properties(rocket_def) -> dict:
    """Compute all aerodynamic properties from rocket geometry.

    Uses Barrowman equations to compute CNα and CP from the geometry
    parsed from the .ork file.

    Args:
        rocket_def: RocketDefinition with geometry fields.

    Returns:
        Dict with:
            CNa_total: Total normal force coefficient slope (per radian)
            cp_from_nose: Center of pressure from nose tip (m)
            CNa_nose: Nose cone contribution
            CNa_fins: Fin set contribution
            CNa_transition: Transition contribution (if any)
            stability_margin_cal: Static stability margin in calibers
            C_damp: Pitch damping coefficient
    """
    d_body = rocket_def.body_diameter

    # --- Nose cone ---
    cna_nose = CNa_nose(rocket_def.nose_shape)
    cp_nose_pos = cp_nose(rocket_def.nose_shape, rocket_def.nose_length)

    # --- Transitions ---
    cna_trans_total = 0.0
    cp_trans_weighted = 0.0
    for tr in getattr(rocket_def, '_transitions', []):
        cna_t = CNa_transition(tr['fore_diameter'], tr['aft_diameter'], d_body)
        cp_t = cp_transition(tr['length'], tr['fore_diameter'],
                             tr['aft_diameter'], tr['position_from_nose'])
        cna_trans_total += cna_t
        cp_trans_weighted += cna_t * cp_t

    # --- Fin set ---
    fin_pos = getattr(rocket_def, 'fin_position_from_nose',
                      rocket_def.body_length - rocket_def.fin_root_chord)
    cna_fin = CNa_fins(
        rocket_def.fin_count,
        rocket_def.fin_root_chord,
        rocket_def.fin_tip_chord,
        rocket_def.fin_span,
        rocket_def.fin_sweep,
        d_body,
    )
    cp_fin_pos = cp_fins(
        rocket_def.fin_root_chord,
        rocket_def.fin_tip_chord,
        rocket_def.fin_span,
        rocket_def.fin_sweep,
        fin_pos,
    )

    # --- Total ---
    cna_total = cna_nose + cna_fin + cna_trans_total

    # Weighted CP position
    if cna_total != 0.0:
        cp_total = (cna_nose * cp_nose_pos +
                    cna_fin * cp_fin_pos +
                    cp_trans_weighted) / cna_total
    else:
        cp_total = rocket_def.body_length / 2.0

    # Stability margin in calibers (CP - CG, positive = stable)
    cg = getattr(rocket_def, 'cg_from_nose', rocket_def.body_length * 0.5)
    if d_body > 0:
        stability_cal = (cp_total - cg) / d_body
    else:
        stability_cal = 0.0

    # Pitch damping sum: Σ(CNα_i × (x_i − x_CG)²)
    # For transition: use weighted average CP if there are transitions
    cp_trans_avg = 0.0
    if cna_trans_total != 0:
        cp_trans_avg = cp_trans_weighted / cna_trans_total

    c_damp = pitch_damping_sum(cna_nose, cp_nose_pos,
                               cna_fin, cp_fin_pos,
                               cna_trans_total, cp_trans_avg,
                               cg)

    return {
        'CNa_total': cna_total,
        'cp_from_nose': cp_total,
        'CNa_nose': cna_nose,
        'CNa_fins': cna_fin,
        'CNa_transition': cna_trans_total,
        'stability_margin_cal': stability_cal,
        'C_damp': c_damp,
    }
