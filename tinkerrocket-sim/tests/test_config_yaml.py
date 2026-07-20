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


def test_sim_config_values_reach_the_rocket_definition():
    """The sections we kept must genuinely round-trip, not just parse."""
    cfg = _load(SIM_CONFIG)
    rd = from_yaml(SIM_CONFIG)
    assert rd.fin_tabs.Kt_ref == cfg["fin_tabs"]["Kt_ref"]
    assert rd.fin_tabs.n_tabs == cfg["fin_tabs"]["n_tabs"]
    assert rd.I_roll_launch == cfg["mass_overrides"]["I_roll_launch"]
    assert rd.Cd == cfg["aerodynamics"]["Cd"]


def test_launch_site_values_reach_the_environment():
    cfg = _load(LAUNCH_SITE)["launch_site"]
    site = _load_site_config(LAUNCH_SITE)
    for key in ("latitude_deg", "longitude_deg", "elevation_m"):
        assert site[key] == cfg[key]


def test_site_loader_survives_an_empty_file(tmp_path):
    """yaml.safe_load returns None for an empty or comment-only file, which
    used to reach .get() and raise. Deleting a top-level block is exactly the
    edit that makes this reachable."""
    for content in ("", "# only a comment\n"):
        p = tmp_path / "site.yaml"
        p.write_text(content)
        site = _load_site_config(p)
        assert site["latitude_deg"] == 38.0
        assert site["elevation_m"] == 0.0


def test_site_loader_handles_a_missing_file(tmp_path):
    site = _load_site_config(tmp_path / "does_not_exist.yaml")
    assert site["latitude_deg"] == 38.0
