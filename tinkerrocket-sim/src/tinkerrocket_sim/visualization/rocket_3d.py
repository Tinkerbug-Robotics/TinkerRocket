"""Generate 3D rocket mesh from OrkData / RocketDefinition geometry.

Creates matplotlib-compatible surface data (vertices + faces) for nose cones,
body tubes, transitions, and fins.  All coordinates are in the rocket body
frame with X pointing forward (nose direction) and origin at the nose tip.
"""
import numpy as np
from typing import Optional


# ---------------------------------------------------------------------------
# Profile generators (2D cross-sections along the rocket axis)
# ---------------------------------------------------------------------------

def _ogive_profile(length: float, base_radius: float, n_points: int = 30):
    """Generate ogive nose cone profile (x, r) from tip to base."""
    if length <= 0 or base_radius <= 0:
        return np.array([0.0]), np.array([0.0])
    rho = (base_radius**2 + length**2) / (2 * base_radius)
    x = np.linspace(0, length, n_points)
    r = np.sqrt(rho**2 - (length - x)**2) + base_radius - rho
    r = np.clip(r, 0, base_radius)
    return x, r


def _conical_profile(length: float, base_radius: float, n_points: int = 20):
    """Generate conical nose cone profile."""
    x = np.linspace(0, length, n_points)
    r = base_radius * x / length
    return x, r


def _haack_profile(length: float, base_radius: float, C: float = 1.0 / 3.0,
                   n_points: int = 30):
    """Generate Von Karman / Haack series nose cone profile."""
    if length <= 0 or base_radius <= 0:
        return np.array([0.0]), np.array([0.0])
    x = np.linspace(0, length, n_points)
    theta = np.arccos(1 - 2 * x / length)
    r = base_radius / np.sqrt(np.pi) * np.sqrt(theta - np.sin(2 * theta) / 2 + C * np.sin(theta)**3)
    return x, r


def _nose_profile(shape: str, length: float, base_radius: float, n_points: int = 30):
    """Generate nose cone profile based on shape name."""
    shape = shape.lower()
    if shape in ("ogive", "tangent ogive"):
        return _ogive_profile(length, base_radius, n_points)
    elif shape in ("conical", "cone"):
        return _conical_profile(length, base_radius, n_points)
    elif shape in ("haack", "von karman", "vonkarman"):
        return _haack_profile(length, base_radius, n_points=n_points)
    elif shape == "parabolic":
        x = np.linspace(0, length, n_points)
        r = base_radius * (2 * x / length - (x / length)**2)
        return x, r
    elif shape == "ellipsoid":
        x = np.linspace(0, length, n_points)
        r = base_radius * np.sqrt(1 - (1 - x / length)**2)
        return x, r
    else:
        # Default to ogive
        return _ogive_profile(length, base_radius, n_points)


# ---------------------------------------------------------------------------
# Surface-of-revolution generator
# ---------------------------------------------------------------------------

def _revolve_profile(x_profile, r_profile, n_theta: int = 24):
    """Revolve an (x, r) profile around the X axis to create a 3D surface.

    Returns:
        X, Y, Z arrays suitable for matplotlib plot_surface, each (n_x, n_theta+1).
    """
    theta = np.linspace(0, 2 * np.pi, n_theta + 1)
    X = np.outer(x_profile, np.ones_like(theta))
    Y = np.outer(r_profile, np.cos(theta))
    Z = np.outer(r_profile, np.sin(theta))
    return X, Y, Z


# ---------------------------------------------------------------------------
# Fin geometry
# ---------------------------------------------------------------------------

def _fin_vertices(root_chord: float, tip_chord: float, span: float,
                  sweep: float, position_from_nose: float,
                  body_radius: float, angle_rad: float):
    """Generate vertices for a single trapezoidal fin.

    Returns an (N, 3) array of vertices forming a closed polygon for the fin
    in the rocket body frame (X forward, Y/Z lateral), rotated around the body
    axis by angle_rad.

    Fin profile (looking from the side, root at bottom):
        LE_root ---- LE_tip
           |             \\
        TE_root ---- TE_tip
    """
    # Fin corners in the fin plane (x = axial, y = radial from body surface)
    x_root_le = position_from_nose
    x_root_te = position_from_nose + root_chord
    x_tip_le = position_from_nose + sweep
    x_tip_te = x_tip_le + tip_chord

    # Radial distances
    r_root = body_radius
    r_tip = body_radius + span

    # Build polygon (closed loop) in axial-radial plane
    poly_x = np.array([x_root_le, x_tip_le, x_tip_te, x_root_te, x_root_le])
    poly_r = np.array([r_root, r_tip, r_tip, r_root, r_root])

    # Rotate into 3D: fin extends radially at angle_rad from Y axis
    cos_a = np.cos(angle_rad)
    sin_a = np.sin(angle_rad)

    verts = np.zeros((len(poly_x), 3))
    verts[:, 0] = poly_x
    verts[:, 1] = poly_r * cos_a
    verts[:, 2] = poly_r * sin_a

    return verts


# ---------------------------------------------------------------------------
# Main mesh builder
# ---------------------------------------------------------------------------

class RocketMesh:
    """Collection of 3D surface data representing a rocket.

    Attributes:
        surfaces: list of (X, Y, Z) arrays for surface-of-revolution components.
        fins: list of (N, 3) vertex arrays for fin polygons.
        total_length: total rocket length (m).
        body_radius: maximum body radius (m).
        cg_position: CG from nose tip (m), if known.
        cp_position: CP from nose tip (m), if known.
    """

    def __init__(self):
        self.surfaces = []      # [(X, Y, Z, color_str), ...]
        self.fins = []           # [(verts, color_str), ...]
        self.total_length = 0.0
        self.body_radius = 0.0
        self.cg_position = 0.0
        self.cp_position = 0.0

    def add_surface(self, X, Y, Z, color='steelblue'):
        self.surfaces.append((X, Y, Z, color))

    def add_fin(self, verts, color='firebrick'):
        self.fins.append((verts, color))


def build_rocket_mesh(rocket_def, n_theta: int = 24) -> RocketMesh:
    """Build a RocketMesh from a RocketDefinition.

    Args:
        rocket_def: RocketDefinition instance (has geometry from .ork parsing).
        n_theta: circumferential resolution for surfaces of revolution.

    Returns:
        RocketMesh with all surfaces and fins.
    """
    mesh = RocketMesh()
    mesh.total_length = rocket_def.body_length
    mesh.body_radius = rocket_def.body_diameter / 2.0
    mesh.cg_position = rocket_def.cg_from_nose
    mesh.cp_position = rocket_def.cp_from_nose

    # --- Nose cone ---
    nose_r = rocket_def.body_diameter / 2.0
    nose_len = rocket_def.nose_length
    if nose_len > 0:
        x_prof, r_prof = _nose_profile(
            rocket_def.nose_shape, nose_len, nose_r, n_points=30)
        X, Y, Z = _revolve_profile(x_prof, r_prof, n_theta)
        mesh.add_surface(X, Y, Z, color='#4682B4')  # steel blue

    # --- Body tubes ---
    # Build body from nose base to end of rocket
    # We need to handle transitions too - collect all axial sections
    axial_offset = nose_len

    # Check if we have detailed transition data from .ork parsing
    transitions = getattr(rocket_def, '_transitions', [])

    # Simple approach: reconstruct body profile from known geometry
    # Collect (position, diameter) points along the rocket
    profile_points = []

    if nose_len > 0:
        profile_points.append((nose_len, rocket_def.body_diameter))

    # If we have transition data, use it
    if transitions:
        # Sort by position
        sorted_trans = sorted(transitions, key=lambda t: t['position_from_nose'])
        for tr in sorted_trans:
            pos = tr['position_from_nose']
            profile_points.append((pos, tr['fore_diameter']))
            profile_points.append((pos + tr['length'], tr['aft_diameter']))

    # End of rocket
    profile_points.append((rocket_def.body_length, rocket_def.body_diameter))

    # Sort and deduplicate
    profile_points.sort(key=lambda p: p[0])

    # Build body tube segments between profile points
    # Simple cylinder segments between each pair of diameter changes
    if len(profile_points) >= 2:
        # Merge close points and build continuous profile
        x_body = [profile_points[0][0]]
        r_body = [profile_points[0][1] / 2.0]

        for pos, diam in profile_points[1:]:
            if pos > x_body[-1] + 1e-6:
                x_body.append(pos)
                r_body.append(diam / 2.0)
            elif abs(pos - x_body[-1]) < 1e-6 and abs(diam / 2.0 - r_body[-1]) > 1e-6:
                # Same position, different diameter: transition point
                x_body.append(pos)
                r_body.append(diam / 2.0)

        x_body = np.array(x_body)
        r_body = np.array(r_body)

        # Refine: add intermediate points for smooth rendering
        x_fine = []
        r_fine = []
        for i in range(len(x_body) - 1):
            n_seg = max(3, int((x_body[i+1] - x_body[i]) / 0.005))
            xs = np.linspace(x_body[i], x_body[i+1], n_seg, endpoint=(i == len(x_body) - 2))
            rs = np.linspace(r_body[i], r_body[i+1], n_seg, endpoint=(i == len(x_body) - 2))
            x_fine.extend(xs)
            r_fine.extend(rs)

        x_fine = np.array(x_fine)
        r_fine = np.array(r_fine)

        X, Y, Z = _revolve_profile(x_fine, r_fine, n_theta)
        mesh.add_surface(X, Y, Z, color='#A9A9A9')  # dark gray

    # --- Fins ---
    fin_count = rocket_def.fin_count
    fin_pos = rocket_def.fin_position_from_nose
    if fin_pos <= 0:
        # Estimate: fins at the aft end
        fin_pos = rocket_def.body_length - rocket_def.fin_root_chord

    body_r = rocket_def.body_diameter / 2.0

    # Determine body radius at fin position (may be different if there's a transition)
    fin_body_r = body_r
    if transitions:
        for tr in transitions:
            tr_start = tr['position_from_nose']
            tr_end = tr_start + tr['length']
            if tr_start <= fin_pos <= tr_end:
                frac = (fin_pos - tr_start) / tr['length'] if tr['length'] > 0 else 0
                fin_body_r = (tr['fore_diameter'] + frac * (tr['aft_diameter'] - tr['fore_diameter'])) / 2.0
                break

    for i in range(fin_count):
        angle = 2 * np.pi * i / fin_count
        verts = _fin_vertices(
            root_chord=rocket_def.fin_root_chord,
            tip_chord=rocket_def.fin_tip_chord,
            span=rocket_def.fin_span,
            sweep=rocket_def.fin_sweep,
            position_from_nose=fin_pos,
            body_radius=fin_body_r,
            angle_rad=angle,
        )
        mesh.add_fin(verts, color='#B22222')  # firebrick

    return mesh


def build_mesh_from_ork(ork_data, n_theta: int = 24) -> RocketMesh:
    """Build a RocketMesh directly from OrkData for higher fidelity.

    Uses the full component list rather than the simplified RocketDefinition.
    """
    mesh = RocketMesh()

    # --- Nose cone ---
    nc = ork_data.nose_cone
    if nc:
        nose_r = nc.base_diameter / 2.0
        if nc.length > 0 and nose_r > 0:
            x_prof, r_prof = _nose_profile(nc.shape, nc.length, nose_r, n_points=40)
            X, Y, Z = _revolve_profile(x_prof, r_prof, n_theta)
            mesh.add_surface(X, Y, Z, color='#4682B4')

    # --- Body tubes ---
    for bt in ork_data.body_tubes:
        if bt.length > 0 and bt.outer_diameter > 0:
            r = bt.outer_diameter / 2.0
            x_prof = np.array([bt.position_from_nose, bt.position_from_nose + bt.length])
            r_prof = np.array([r, r])
            X, Y, Z = _revolve_profile(x_prof, r_prof, n_theta)
            mesh.add_surface(X, Y, Z, color='#A9A9A9')

    # --- Transitions ---
    for tr in ork_data.transitions:
        if tr.length > 0:
            r_fore = tr.fore_diameter / 2.0
            r_aft = tr.aft_diameter / 2.0
            n_pts = max(10, int(tr.length / 0.002))
            x_prof = np.linspace(tr.position_from_nose,
                                 tr.position_from_nose + tr.length, n_pts)
            r_prof = np.linspace(r_fore, r_aft, n_pts)
            X, Y, Z = _revolve_profile(x_prof, r_prof, n_theta)
            mesh.add_surface(X, Y, Z, color='#808080')

    # --- Fins ---
    for fs in ork_data.fin_sets:
        # Determine body radius at fin position
        body_r = ork_data.reference_diameter / 2.0
        for bt in ork_data.body_tubes:
            bt_start = bt.position_from_nose
            bt_end = bt_start + bt.length
            if bt_start <= fs.position_from_nose <= bt_end:
                body_r = bt.outer_diameter / 2.0
                break
        for tr in ork_data.transitions:
            tr_start = tr.position_from_nose
            tr_end = tr_start + tr.length
            if tr_start <= fs.position_from_nose <= tr_end:
                frac = (fs.position_from_nose - tr_start) / tr.length if tr.length > 0 else 0
                body_r = (tr.fore_diameter + frac * (tr.aft_diameter - tr.fore_diameter)) / 2.0
                break

        for i in range(fs.fin_count):
            angle = 2 * np.pi * i / fs.fin_count
            verts = _fin_vertices(
                root_chord=fs.root_chord,
                tip_chord=fs.tip_chord,
                span=fs.span,
                sweep=fs.sweep_length,
                position_from_nose=fs.position_from_nose,
                body_radius=body_r,
                angle_rad=angle,
            )
            mesh.add_fin(verts, color='#B22222')

    # Store key dimensions
    mesh.total_length = ork_data.total_length
    mesh.body_radius = ork_data.reference_diameter / 2.0
    mesh.cg_position = ork_data.cg_from_nose
    mesh.cp_position = 0.0  # not available from OrkData alone

    return mesh
