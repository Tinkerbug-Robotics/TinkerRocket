#!/usr/bin/env python3
"""Run a passive (uncontrolled) RocketPy flight simulation.

Usage:
    python scripts/run_passive_sim.py [--ork rockets/tinkerrocket_mini.ork]
    python scripts/run_passive_sim.py --yaml-only

If --ork is provided, loads rocket definition from the .ork file.
Otherwise uses default parameters from sim_config.yaml.
"""
import argparse
import sys
from pathlib import Path

# Add src to path for development
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from tinkerrocket_sim.rocket.definition import from_ork, from_yaml, RocketDefinition
from tinkerrocket_sim.rocket.fin_tab_aero import FinTabAeroModel
from tinkerrocket_sim.simulation.rocketpy_passive import run_passive_flight
from tinkerrocket_sim.plotting.flight_plots import plot_passive_flight

import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description="Run passive flight simulation")
    parser.add_argument("--ork", type=str, default=None,
                        help="Path to OpenRocket .ork file")
    parser.add_argument("--config", type=str,
                        default="config/sim_config.yaml",
                        help="Path to simulation config YAML")
    parser.add_argument("--site", type=str,
                        default="config/launch_site.yaml",
                        help="Path to launch site config")
    parser.add_argument("--yaml-only", action="store_true",
                        help="Use only YAML config (no .ork file)")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Simulation duration in seconds")
    parser.add_argument("--save", type=str, default=None,
                        help="Save plot to file")
    args = parser.parse_args()

    project_root = Path(__file__).parent.parent
    config_path = project_root / args.config
    site_path = project_root / args.site

    # Build rocket definition
    if args.ork:
        ork_path = Path(args.ork)
        if not ork_path.is_absolute():
            ork_path = project_root / ork_path
        print(f"Loading rocket from: {ork_path}")
        rocket_def = from_ork(ork_path, config_path)
    elif args.yaml_only:
        print("Using YAML-only configuration")
        rocket_def = from_yaml(config_path)
    else:
        print("Using default rocket definition")
        rocket_def = RocketDefinition()
        from tinkerrocket_sim.rocket.definition import _apply_yaml_overrides
        if config_path.exists():
            _apply_yaml_overrides(rocket_def, config_path)

    # Print rocket summary
    print(f"\n--- Rocket: {rocket_def.name} ---")
    print(f"  Body diameter: {rocket_def.body_diameter*1000:.1f} mm")
    print(f"  Body length:   {rocket_def.body_length*1000:.1f} mm")
    print(f"  Dry mass:      {rocket_def.dry_mass*1000:.1f} g")
    print(f"  Fin count:     {rocket_def.fin_count}")
    print(f"  Fin span:      {rocket_def.fin_span*1000:.1f} mm")
    print(f"  Cd:            {rocket_def.Cd}")
    print(f"  I_roll:        {rocket_def.I_roll_launch:.4e} kg-m^2")

    if rocket_def.motor.burn_time > 0:
        print(f"\n--- Motor: {rocket_def.motor.designation} ---")
        print(f"  Burn time:     {rocket_def.motor.burn_time:.2f} s")
        print(f"  Total mass:    {rocket_def.motor.total_mass*1000:.1f} g")
        print(f"  Propellant:    {rocket_def.motor.propellant_mass*1000:.1f} g")
        peak = max(rocket_def.motor.thrust_forces) if len(rocket_def.motor.thrust_forces) > 0 else 0
        print(f"  Peak thrust:   {peak:.1f} N")

    # Print fin tab model
    ft = FinTabAeroModel(
        Kt_ref=rocket_def.fin_tabs.Kt_ref,
        V_ref=rocket_def.fin_tabs.V_ref,
        n_tabs=rocket_def.fin_tabs.n_tabs,
    )
    print(f"\n--- Fin Tab Model ---")
    print(f"  Kt_ref:  {ft.Kt_ref:.4e} N-m/deg at {ft.V_ref} m/s")
    print(f"  Tabs:    {ft.n_tabs}")
    print(f"  At 95 m/s, 10 deg: {ft.torque(95, 10)*1000:.1f} mN-m total")
    print(f"  At 50 m/s, 10 deg: {ft.torque(50, 10)*1000:.1f} mN-m total")

    # Run simulation
    print(f"\n--- Running passive flight ({args.duration}s) ---")
    try:
        result = run_passive_flight(
            rocket_def,
            launch_site_config=site_path,
            sim_duration=args.duration,
        )
    except Exception as e:
        print(f"Simulation failed: {e}")
        print("This may be due to incomplete rocket/motor definition.")
        print("Try providing a complete .ork file with --ork flag.")
        return

    print(f"\n--- Results ---")
    print(f"  Apogee:          {result.apogee_m:.1f} m")
    print(f"  Max speed:       {result.max_speed_mps:.1f} m/s")
    print(f"  Max Mach:        {result.max_mach:.3f}")
    print(f"  Max accel:       {result.max_acceleration_g:.1f} g")
    print(f"  Flight time:     {result.flight_time_s:.1f} s")

    # Plot
    title = f"Passive Flight: {rocket_def.name}"
    fig = plot_passive_flight(result, title=title, save_path=args.save)
    plt.show()


if __name__ == "__main__":
    main()
