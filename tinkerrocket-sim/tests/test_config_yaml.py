"""The checked-in YAML config must not advertise keys nothing reads (#170).

Both config files had accumulated whole sections of real-looking keys that no
code path consumed — sensor specs, controller gains, simulation timing, and a
`gust_sigma_mps` that promised a turbulence model the sim did not have. Several
contradicted the values actually in force, so anyone who trusted the file was
misled rather than merely ignored.

These tests pin the invariant: every key present is a key some reader consumes.
Adding a new section without a reader fails here.
"""
from pathlib import Path

import pytest
import yaml

from tinkerrocket_sim.rocket.definition import from_yaml
from tinkerrocket_sim.simulation.rocketpy_passive import _load_site_config

CONFIG_DIR = Path(__file__).resolve().parent.parent / "config"
SIM_CONFIG = CONFIG_DIR / "sim_config.yaml"
LAUNCH_SITE = CONFIG_DIR / "launch_site.yaml"

# The sections rocket.definition._apply_yaml_overrides actually reads.
CONSUMED_SIM_SECTIONS = {"fin_tabs", "mass_overrides", "aerodynamics"}
# rocketpy_passive._load_site_config reads only this one.
CONSUMED_SITE_SECTIONS = {"launch_site"}


def _load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def test_config_files_parse():
    assert _load(SIM_CONFIG) is not None
    assert _load(LAUNCH_SITE) is not None


def test_sim_config_has_no_unread_sections():
    """A section here with no reader is a promise the code does not keep."""
    assert set(_load(SIM_CONFIG)) == CONSUMED_SIM_SECTIONS


def test_launch_site_has_no_unread_sections():
    assert set(_load(LAUNCH_SITE)) == CONSUMED_SITE_SECTIONS


def _all_keys(node):
    """Every mapping key anywhere in a parsed YAML tree."""
    if isinstance(node, dict):
        for k, v in node.items():
            yield k
            yield from _all_keys(v)
    elif isinstance(node, list):
        for v in node:
            yield from _all_keys(v)


@pytest.mark.parametrize("path", [SIM_CONFIG, LAUNCH_SITE])
def test_no_wind_or_gust_keys_masquerading_as_config(path):
    """gust_sigma_mps sat in launch_site.yaml unread for the life of the sim.
    Wind and turbulence are configured on SimConfig (wind_speed,
    gust_w20_mps), never in these files. Checked against the parsed tree —
    prose pointing readers at the real knobs is fine."""
    keys = list(_all_keys(_load(path)))
    assert not [k for k in keys if "gust" in k or "wind" in k]


def _perturb(node):
    """Copy a parsed YAML tree with every numeric leaf changed."""
    if isinstance(node, dict):
        return {k: _perturb(v) for k, v in node.items()}
    if isinstance(node, bool):
        return not node
    if isinstance(node, (int, float)):
        return node * 3.0 + 1.0
    return node


def _fingerprint(obj, depth=0):
    """Flatten an object's scalar attributes so two of them can be diffed."""
    if depth > 3 or not hasattr(obj, "__dict__"):
        return {}
    out = {}
    for name, value in vars(obj).items():
        if isinstance(value, (int, float, str, bool)) or value is None:
            out[name] = value
        else:
            for sub, sv in _fingerprint(value, depth + 1).items():
                out[f"{name}.{sub}"] = sv
    return out


@pytest.mark.parametrize("section", sorted(CONSUMED_SIM_SECTIONS))
def test_every_sim_config_section_actually_reaches_the_rocket(section, tmp_path):
    """Behavioral, not declarative: perturb one section's values and require
    the RocketDefinition to change.

    Asserting `rd.Cd == cfg["aerodynamics"]["Cd"]` would NOT do this — the
    checked-in values happen to equal the dataclass defaults, so that
    assertion passes even with the reader deleted. Perturbing proves a reader
    exists and applies the value, so deleting one fails here.
    """
    cfg = _load(SIM_CONFIG)
    perturbed = dict(cfg)
    perturbed[section] = _perturb(cfg[section])

    path = tmp_path / "perturbed.yaml"
    path.write_text(yaml.safe_dump(perturbed))

    base = _fingerprint(from_yaml(SIM_CONFIG))
    after = _fingerprint(from_yaml(path))
    changed = [k for k in base if base[k] != after.get(k)]
    assert changed, (
        f"perturbing the '{section}' section changed nothing in the "
        f"RocketDefinition — it has no live reader"
    )


def test_launch_site_values_reach_the_environment(tmp_path):
    """Same reasoning: use values that differ from the loader's defaults, so
    the assertion fails if the file is ignored."""
    path = tmp_path / "site.yaml"
    path.write_text(yaml.safe_dump({"launch_site": {
        "latitude_deg": 12.5, "longitude_deg": -77.25, "elevation_m": 1234.0,
    }}))
    site = _load_site_config(path)
    assert site["latitude_deg"] == 12.5
    assert site["longitude_deg"] == -77.25
    assert site["elevation_m"] == 1234.0

    # ...and the checked-in file still parses into that same shape.
    real = _load_site_config(LAUNCH_SITE)
    for key in ("latitude_deg", "longitude_deg", "elevation_m"):
        assert real[key] == _load(LAUNCH_SITE)["launch_site"][key]


@pytest.mark.parametrize("content", [
    "",                                          # empty file
    "# only a comment\n",                        # comment-only -> safe_load None
    "launch_site:\n",                            # key present, body null
    "launch_site:\n  # latitude_deg: 38.0\n",    # body commented out
    "other_section:\n  a: 1\n",                  # key absent entirely
])
def test_site_loader_falls_back_to_defaults(content, tmp_path):
    """Every degenerate shape must fall back, not raise.

    Two distinct traps here: safe_load returns None for an empty file, and a
    present-but-null block parses to {'launch_site': None} — where a .get()
    default does NOT apply, because the key exists. Commenting out the body
    while leaving the key is the natural editing accident, and this file now
    carries a long prose comment under that key.
    """
    p = tmp_path / "site.yaml"
    p.write_text(content)
    site = _load_site_config(p)
    assert site["latitude_deg"] == 38.0
    assert site["longitude_deg"] == -122.0
    assert site["elevation_m"] == 0.0


def test_site_loader_handles_a_missing_file(tmp_path):
    site = _load_site_config(tmp_path / "does_not_exist.yaml")
    assert site["latitude_deg"] == 38.0
