"""Motor database: parse RASP .eng files and look up motors by designation.

RASP .eng format (standard for rocket motor data exchange):
    ; comment lines start with semicolon
    <designation> <diameter_mm> <length_mm> <delays> <propellant_mass_kg> <total_mass_kg> <manufacturer>
    <time_s> <thrust_N>
    <time_s> <thrust_N>
    ...
    ;  (blank line or end of file ends the motor)

The motors/ directory in the project root is searched for .eng files.
"""
import os
from pathlib import Path
from dataclasses import dataclass, field


@dataclass
class EngMotor:
    """Motor data from a .eng file."""
    designation: str = ""
    manufacturer: str = ""
    diameter_mm: float = 0.0
    length_mm: float = 0.0
    delays: str = ""
    propellant_mass: float = 0.0  # kg
    total_mass: float = 0.0  # kg
    thrust_times: list = field(default_factory=list)
    thrust_forces: list = field(default_factory=list)


def parse_eng(filepath: str | Path) -> list[EngMotor]:
    """Parse a RASP .eng file, which may contain multiple motors.

    Returns:
        List of EngMotor objects.
    """
    filepath = Path(filepath)
    motors = []
    current = None

    with open(filepath) as f:
        for line in f:
            line = line.strip()

            # Skip comments and blank lines
            if not line or line.startswith(";"):
                if current and current.thrust_times:
                    motors.append(current)
                    current = None
                continue

            parts = line.split()

            # Header line: designation diam length delays prop_mass total_mass manufacturer
            if len(parts) >= 7 and not _is_data_line(parts):
                if current and current.thrust_times:
                    motors.append(current)
                current = EngMotor()
                current.designation = parts[0]
                current.diameter_mm = float(parts[1])
                current.length_mm = float(parts[2])
                current.delays = parts[3]
                current.propellant_mass = float(parts[4])
                current.total_mass = float(parts[5])
                current.manufacturer = parts[6]

            # Data line: time thrust
            elif len(parts) >= 2 and current is not None:
                try:
                    t = float(parts[0])
                    thrust = float(parts[1])
                    current.thrust_times.append(t)
                    current.thrust_forces.append(thrust)
                except ValueError:
                    pass

    # Don't forget the last motor
    if current and current.thrust_times:
        motors.append(current)

    return motors


def _is_data_line(parts: list[str]) -> bool:
    """Check if a split line looks like time/thrust data vs a header."""
    try:
        float(parts[0])
        float(parts[1])
        # If first two are valid floats, it's data unless there are 7+ parts
        # and the first value has letters (designation like "G80T")
        if any(c.isalpha() for c in parts[0]):
            return False
        return True
    except ValueError:
        return False


def find_motor(designation: str,
               search_dirs: list[str | Path] = None) -> EngMotor | None:
    """Search for a motor by designation in .eng files.

    Args:
        designation: Motor designation (e.g. "G80T", "F67C").
        search_dirs: Directories to search for .eng files.
                     Defaults to motors/ in the project root.

    Returns:
        EngMotor if found, None otherwise.
    """
    if search_dirs is None:
        # Default: look in motors/ relative to project root
        project_root = Path(__file__).parent.parent.parent.parent
        search_dirs = [project_root / "motors"]

    designation_upper = designation.upper()

    for search_dir in search_dirs:
        search_dir = Path(search_dir)
        if not search_dir.exists():
            continue

        for eng_file in search_dir.glob("*.eng"):
            try:
                motors = parse_eng(eng_file)
                for m in motors:
                    if m.designation.upper() == designation_upper:
                        return m
            except Exception:
                continue

    return None
