"""Parse OpenRocket .ork files to extract rocket definition parameters.

.ork files are ZIP archives containing XML with all rocket design data
in SI units: body tubes, nose cone, fins, transitions, mass, and motor
thrust curves.
"""
import zipfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


@dataclass
class NoseConeData:
    shape: str = "ogive"
    length: float = 0.0  # m
    base_diameter: float = 0.0  # m
    thickness: float = 0.0  # m
    mass: float = 0.0  # kg
    shape_parameter: float = 0.0
    position_from_nose: float = 0.0  # m (always 0 for nose cone)


@dataclass
class BodyTubeData:
    name: str = ""
    length: float = 0.0  # m
    outer_diameter: float = 0.0  # m
    inner_diameter: float = 0.0  # m
    mass: float = 0.0  # kg
    position_from_nose: float = 0.0  # m


@dataclass
class FinSetData:
    name: str = ""
    fin_count: int = 3
    root_chord: float = 0.0  # m
    tip_chord: float = 0.0  # m
    span: float = 0.0  # m
    sweep_length: float = 0.0  # m
    thickness: float = 0.0  # m
    mass: float = 0.0  # kg (total for all fins)
    cross_section: str = "square"
    position_from_nose: float = 0.0  # m (fin root leading edge from nose tip)


@dataclass
class TransitionData:
    name: str = ""
    length: float = 0.0  # m
    fore_diameter: float = 0.0  # m
    aft_diameter: float = 0.0  # m
    mass: float = 0.0  # kg
    position_from_nose: float = 0.0  # m


@dataclass
class MassComponentData:
    name: str = ""
    mass: float = 0.0  # kg
    position: float = 0.0  # m from nose tip (absolute)
    position_from_nose: float = 0.0  # m (computed absolute position)


@dataclass
class MotorData:
    manufacturer: str = ""
    designation: str = ""
    configid: str = ""  # links to motorconfiguration
    diameter: float = 0.0  # m
    length: float = 0.0  # m
    total_mass: float = 0.0  # kg
    propellant_mass: float = 0.0  # kg
    delays: str = ""
    thrust_curve_times: list = field(default_factory=list)  # s
    thrust_curve_forces: list = field(default_factory=list)  # N


@dataclass
class OrkData:
    """All data extracted from an .ork file."""
    name: str = ""
    nose_cone: Optional[NoseConeData] = None
    body_tubes: list = field(default_factory=list)
    fin_sets: list = field(default_factory=list)
    transitions: list = field(default_factory=list)
    mass_components: list = field(default_factory=list)
    motors: list = field(default_factory=list)
    default_motor_configid: str = ""
    # Computed/aggregated
    total_length: float = 0.0  # m
    reference_diameter: float = 0.0  # m (max body diameter)
    cg_from_nose: float = 0.0  # m (dry CG position from nose tip)


def parse_ork(filepath: str | Path) -> OrkData:
    """Parse an OpenRocket .ork file and return extracted data.

    Args:
        filepath: Path to the .ork file.

    Returns:
        OrkData with all extracted rocket parameters.
    """
    filepath = Path(filepath)
    xml_content = _extract_xml(filepath)
    root = ET.fromstring(xml_content)
    return _parse_root(root)


def _extract_xml(filepath: Path) -> str:
    """Extract XML content from .ork ZIP archive, or read as plain XML."""
    try:
        with zipfile.ZipFile(filepath, "r") as zf:
            # Look for the XML file inside the ZIP
            for name in zf.namelist():
                if name.endswith(".ork") or name.endswith(".xml"):
                    return zf.read(name).decode("utf-8")
            # If no matching name, try the first file
            return zf.read(zf.namelist()[0]).decode("utf-8")
    except zipfile.BadZipFile:
        # Might be plain XML (older .ork format)
        return filepath.read_text(encoding="utf-8")


def _parse_root(root: ET.Element) -> OrkData:
    """Parse the root XML element."""
    data = OrkData()

    # Find the rocket element
    rocket_el = root.find(".//rocket")
    if rocket_el is None:
        raise ValueError("No <rocket> element found in .ork file")

    name_el = rocket_el.find("name")
    if name_el is not None and name_el.text:
        data.name = name_el.text

    # Find default motor configuration
    for mc_el in rocket_el.findall("motorconfiguration"):
        if mc_el.get("default") == "true":
            data.default_motor_configid = mc_el.get("configid", "")
            break

    # Parse all components recursively, tracking positions
    _parse_components(rocket_el, data, stack_pos=0.0, parent_start=0.0, parent_length=0.0)

    # Compute aggregated values
    _compute_aggregates(data)

    return data


def _get_position_info(el: ET.Element) -> tuple:
    """Get position offset and type from an element.

    OpenRocket stores position as:
        <position type="top|bottom|middle|absolute">value</position>
    or:
        <axialoffset method="top|bottom|middle|absolute">value</axialoffset>

    Returns:
        (offset_value, position_type) tuple.
    """
    # Try <position> first
    pos_el = el.find("position")
    if pos_el is not None and pos_el.text:
        pos_type = pos_el.get("type", "top")
        try:
            text = pos_el.text.strip()
            if text.startswith("auto"):
                text = text.split()[-1]
            return float(text), pos_type
        except ValueError:
            pass

    # Try <axialoffset>
    ax_el = el.find("axialoffset")
    if ax_el is not None and ax_el.text:
        pos_type = ax_el.get("method", "top")
        try:
            text = ax_el.text.strip()
            if text.startswith("auto"):
                text = text.split()[-1]
            return float(text), pos_type
        except ValueError:
            pass

    return 0.0, "top"


def _resolve_position(offset: float, pos_type: str,
                      parent_start: float, parent_length: float,
                      component_length: float = 0.0) -> float:
    """Resolve a component's absolute position from nose tip.

    Args:
        offset: Position offset value from .ork file.
        pos_type: Position type (top, bottom, middle, absolute).
        parent_start: Parent component's start position from nose (m).
        parent_length: Parent component's length (m).
        component_length: This component's length (m), for bottom reference.

    Returns:
        Absolute position of component's front from nose tip (m).
    """
    if pos_type == "absolute":
        return offset
    elif pos_type == "top":
        return parent_start + offset
    elif pos_type == "bottom":
        # Offset from parent's bottom edge, component_length used to get front
        return parent_start + parent_length + offset - component_length
    elif pos_type == "middle":
        return parent_start + parent_length / 2.0 + offset - component_length / 2.0
    else:
        return parent_start + offset


def _parse_components(parent: ET.Element, data: OrkData,
                      stack_pos: float = 0.0,
                      parent_start: float = 0.0,
                      parent_length: float = 0.0):
    """Recursively parse component tree, tracking positions.

    Structural components (nose, body tubes, transitions) stack sequentially.
    Sub-components (fins, mass components) are positioned relative to parent.

    Args:
        parent: Parent XML element.
        data: OrkData to populate.
        stack_pos: Current stacking position for sequential structural
                   components (m from nose tip).
        parent_start: Parent component's start position from nose (m).
        parent_length: Parent component's length (m).
    """
    current_stack = stack_pos

    for el in parent:
        tag = el.tag.lower()

        if tag == "nosecone":
            data.nose_cone = _parse_nosecone(el)
            data.nose_cone.position_from_nose = 0.0
            current_stack = data.nose_cone.length
            _parse_components(el, data, stack_pos=current_stack,
                              parent_start=0.0,
                              parent_length=data.nose_cone.length)

        elif tag == "bodytube":
            bt = _parse_bodytube(el)
            bt.position_from_nose = current_stack
            data.body_tubes.append(bt)
            _parse_components(el, data, stack_pos=current_stack + bt.length,
                              parent_start=current_stack,
                              parent_length=bt.length)
            current_stack += bt.length

        elif tag in ("trapezoidfinset", "freeformfinset", "ellipticalfinset"):
            fs = _parse_finset(el)
            # Resolve fin position relative to parent
            offset, pos_type = _get_position_info(el)
            fs.position_from_nose = _resolve_position(
                offset, pos_type, parent_start, parent_length,
                component_length=fs.root_chord)
            data.fin_sets.append(fs)

        elif tag == "transition":
            tr = _parse_transition(el)
            tr.position_from_nose = current_stack
            data.transitions.append(tr)
            _parse_components(el, data, stack_pos=current_stack + tr.length,
                              parent_start=current_stack,
                              parent_length=tr.length)
            current_stack += tr.length

        elif tag in ("masscomponent", "parachute", "shockcord",
                      "centeringring", "engineblock", "bulkhead",
                      "tubecoupler", "launchlug", "railbutton"):
            mc = _parse_mass_component(el, tag)
            # Resolve mass component position relative to parent
            offset, pos_type = _get_position_info(el)
            mc.position_from_nose = _resolve_position(
                offset, pos_type, parent_start, parent_length,
                component_length=0.0)
            data.mass_components.append(mc)

        elif tag in ("innertube",):
            # Inner tubes may contain motors, recurse with parent position
            _parse_components(el, data, stack_pos=current_stack,
                              parent_start=parent_start,
                              parent_length=parent_length)

        elif tag == "motormount":
            _parse_components(el, data, stack_pos=current_stack,
                              parent_start=parent_start,
                              parent_length=parent_length)

        elif tag == "motor":
            motor = _parse_motor(el)
            data.motors.append(motor)

        elif tag in ("subcomponents", "stage"):
            _parse_components(el, data, stack_pos=current_stack,
                              parent_start=parent_start,
                              parent_length=parent_length)

        else:
            # Recurse into unknown elements that might contain components
            _parse_components(el, data, stack_pos=current_stack,
                              parent_start=parent_start,
                              parent_length=parent_length)


def _get_float(el: ET.Element, tag: str, default: float = 0.0) -> float:
    """Get a float value from a child element.

    Handles OpenRocket's 'auto <value>' format (e.g. radius="auto 0.0287").
    """
    child = el.find(tag)
    if child is not None and child.text:
        text = child.text.strip()
        # Handle "auto <value>" format
        if text.startswith("auto"):
            text = text.split()[-1]
        try:
            return float(text)
        except ValueError:
            pass
    return default


def _get_text(el: ET.Element, tag: str, default: str = "") -> str:
    """Get text value from a child element."""
    child = el.find(tag)
    if child is not None and child.text:
        return child.text.strip()
    return default


def _get_int(el: ET.Element, tag: str, default: int = 0) -> int:
    """Get an integer value from a child element."""
    child = el.find(tag)
    if child is not None and child.text:
        try:
            return int(child.text)
        except ValueError:
            pass
    return default


def _parse_nosecone(el: ET.Element) -> NoseConeData:
    nc = NoseConeData()
    nc.shape = _get_text(el, "shape", "ogive").lower()
    nc.length = _get_float(el, "length")
    nc.base_diameter = _get_float(el, "aftradius") * 2  # radius -> diameter
    if nc.base_diameter == 0:
        nc.base_diameter = _get_float(el, "aftouterdiameter")
    nc.thickness = _get_float(el, "thickness")
    nc.mass = _get_float(el, "mass")
    nc.shape_parameter = _get_float(el, "shapeparameter")

    # Check for mass override
    override = _get_float(el, "overridemass")
    if override > 0:
        nc.mass = override

    return nc


def _parse_bodytube(el: ET.Element) -> BodyTubeData:
    bt = BodyTubeData()
    bt.name = _get_text(el, "name")
    bt.length = _get_float(el, "length")
    bt.outer_diameter = _get_float(el, "radius") * 2
    if bt.outer_diameter == 0:
        bt.outer_diameter = _get_float(el, "outerradius") * 2
    bt.inner_diameter = _get_float(el, "innerradius") * 2
    if bt.inner_diameter == 0:
        bt.inner_diameter = bt.outer_diameter - 2 * _get_float(el, "thickness")
    bt.mass = _get_float(el, "mass")

    override = _get_float(el, "overridemass")
    if override > 0:
        bt.mass = override

    return bt


def _parse_finset(el: ET.Element) -> FinSetData:
    fs = FinSetData()
    fs.name = _get_text(el, "name")
    fs.fin_count = _get_int(el, "fincount", 3)
    fs.root_chord = _get_float(el, "rootchord")
    fs.tip_chord = _get_float(el, "tipchord")
    fs.span = _get_float(el, "height")  # OR uses "height" for span
    if fs.span == 0:
        fs.span = _get_float(el, "span")
    fs.sweep_length = _get_float(el, "sweeplength")
    fs.thickness = _get_float(el, "thickness")
    fs.mass = _get_float(el, "mass")
    fs.cross_section = _get_text(el, "crosssection", "square")

    # Handle freeform fins: extract geometry from <finpoints>
    finpoints_el = el.find("finpoints")
    if finpoints_el is not None:
        points = []
        for pt in finpoints_el.findall("point"):
            x = float(pt.get("x", "0"))
            y = float(pt.get("y", "0"))
            points.append((x, y))
        if points:
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            # Root chord = x-extent along body at y=0
            root_pts = [p[0] for p in points if abs(p[1]) < 1e-6]
            if len(root_pts) >= 2:
                fs.root_chord = max(root_pts) - min(root_pts)
            else:
                fs.root_chord = max(xs) - min(xs)
            # Span = max height
            fs.span = max(ys) if ys else 0.0
            # Tip chord from points at max span
            max_y = max(ys)
            tip_pts = [p[0] for p in points if abs(p[1] - max_y) < 1e-6]
            if len(tip_pts) >= 2:
                fs.tip_chord = max(tip_pts) - min(tip_pts)
            elif len(tip_pts) == 1:
                fs.tip_chord = 0.0  # pointed tip
            # Sweep = x-offset of leading edge at max span
            leading_x_at_tip = min(p[0] for p in points if abs(p[1] - max_y) < 1e-6)
            fs.sweep_length = leading_x_at_tip - min(xs)

    override = _get_float(el, "overridemass")
    if override > 0:
        fs.mass = override

    return fs


def _parse_transition(el: ET.Element) -> TransitionData:
    tr = TransitionData()
    tr.name = _get_text(el, "name")
    tr.length = _get_float(el, "length")
    tr.fore_diameter = _get_float(el, "foreradius") * 2
    if tr.fore_diameter == 0:
        tr.fore_diameter = _get_float(el, "foreouterdiameter")
    tr.aft_diameter = _get_float(el, "aftradius") * 2
    if tr.aft_diameter == 0:
        tr.aft_diameter = _get_float(el, "aftouterdiameter")
    tr.mass = _get_float(el, "mass")

    override = _get_float(el, "overridemass")
    if override > 0:
        tr.mass = override

    return tr


def _parse_mass_component(el: ET.Element, tag: str) -> MassComponentData:
    mc = MassComponentData()
    mc.name = _get_text(el, "name", tag)
    mc.mass = _get_float(el, "mass")
    mc.position = _get_float(el, "position")

    override = _get_float(el, "overridemass")
    if override > 0:
        mc.mass = override

    return mc


def _parse_motor(el: ET.Element) -> MotorData:
    motor = MotorData()
    motor.configid = el.get("configid", "")
    motor.manufacturer = _get_text(el, "manufacturer")
    motor.designation = _get_text(el, "designation")
    motor.diameter = _get_float(el, "diameter")
    motor.length = _get_float(el, "length")
    motor.delays = _get_text(el, "delay")

    # Parse thrust curve data points
    for tc_el in el.findall("thrustcurvepoint"):
        if tc_el.text:
            parts = tc_el.text.strip().split(",")
            if len(parts) >= 2:
                try:
                    t = float(parts[0])
                    thrust = float(parts[1])
                    motor.thrust_curve_times.append(t)
                    motor.thrust_curve_forces.append(thrust)
                except ValueError:
                    continue

    # Also check for thrust-data element (older format)
    td_el = el.find("thrust-data")
    if td_el is not None:
        for dp in td_el.findall("eng-data") or td_el.findall("datapoint"):
            t = _get_float(dp, "t", -1)
            f = _get_float(dp, "f", -1)
            if t >= 0 and f >= 0:
                motor.thrust_curve_times.append(t)
                motor.thrust_curve_forces.append(f)

    # Motor mass info
    motor.total_mass = _get_float(el, "initialmass")
    if motor.total_mass == 0:
        motor.total_mass = _get_float(el, "mass")
    motor.propellant_mass = _get_float(el, "propellantmass")
    if motor.propellant_mass == 0:
        empty = _get_float(el, "emptymass")
        if empty > 0 and motor.total_mass > 0:
            motor.propellant_mass = motor.total_mass - empty

    return motor


def _compute_aggregates(data: OrkData):
    """Compute total length, reference diameter, CG, etc."""
    # Total length from nose + body tubes + transitions
    total = 0.0
    if data.nose_cone:
        total += data.nose_cone.length

    for bt in data.body_tubes:
        total += bt.length

    for tr in data.transitions:
        total += tr.length

    data.total_length = total

    # Reference diameter = max body diameter
    diameters = []
    if data.nose_cone and data.nose_cone.base_diameter > 0:
        diameters.append(data.nose_cone.base_diameter)
    for bt in data.body_tubes:
        if bt.outer_diameter > 0:
            diameters.append(bt.outer_diameter)

    data.reference_diameter = max(diameters) if diameters else 0.0

    # Compute dry CG from component positions and masses
    total_mass = 0.0
    moment_sum = 0.0

    if data.nose_cone and data.nose_cone.mass > 0:
        # Nose cone CG approximately at 50% of its length
        cg_pos = data.nose_cone.position_from_nose + data.nose_cone.length * 0.5
        total_mass += data.nose_cone.mass
        moment_sum += data.nose_cone.mass * cg_pos

    for bt in data.body_tubes:
        if bt.mass > 0:
            cg_pos = bt.position_from_nose + bt.length * 0.5
            total_mass += bt.mass
            moment_sum += bt.mass * cg_pos

    for tr in data.transitions:
        if tr.mass > 0:
            cg_pos = tr.position_from_nose + tr.length * 0.5
            total_mass += tr.mass
            moment_sum += tr.mass * cg_pos

    for fs in data.fin_sets:
        if fs.mass > 0:
            # Fin CG approximately at 40% of root chord from LE
            cg_pos = fs.position_from_nose + fs.root_chord * 0.4
            total_mass += fs.mass
            moment_sum += fs.mass * cg_pos

    for mc in data.mass_components:
        if mc.mass > 0:
            total_mass += mc.mass
            moment_sum += mc.mass * mc.position_from_nose

    if total_mass > 0:
        data.cg_from_nose = moment_sum / total_mass
    else:
        data.cg_from_nose = total / 2.0  # fallback: 50% of length
