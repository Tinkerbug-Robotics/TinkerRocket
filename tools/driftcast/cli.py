"""
Command-line interface for the reverse drift cast tool.

Usage:
    python -m driftcast --launch-lat 32.943 --launch-lon -106.92 \
                        --landing-lat 32.945 --landing-lon -106.918 \
                        --apogee 10000 --drogue-rate 60 --main-rate 18 \
                        --main-deploy 700 --time "2026-03-15T14:00:00Z"
"""

import argparse
import sys

from .driftcast import compute_guidance_point, print_result
from .wind_api import print_wind_profile


def main():
    parser = argparse.ArgumentParser(
        description="Reverse Drift Cast: compute apogee guidance point for a desired landing location.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Example:
    python -m driftcast \\
        --launch-lat 32.9430 --launch-lon -106.9200 \\
        --landing-lat 32.9450 --landing-lon -106.9180 \\
        --apogee 10000 \\
        --drogue-rate 60 --main-rate 18 --main-deploy 700 \\
        --time "2026-03-15T14:00:00Z"
""",
    )

    parser.add_argument("--launch-lat", type=float, required=True,
                        help="Launch pad latitude (decimal degrees)")
    parser.add_argument("--launch-lon", type=float, required=True,
                        help="Launch pad longitude (decimal degrees)")
    parser.add_argument("--landing-lat", type=float, required=True,
                        help="Desired landing latitude (decimal degrees)")
    parser.add_argument("--landing-lon", type=float, required=True,
                        help="Desired landing longitude (decimal degrees)")
    parser.add_argument("--apogee", type=float, required=True,
                        help="Apogee altitude AGL (feet)")
    parser.add_argument("--drogue-rate", type=float, required=True,
                        help="Drogue parachute descent rate (feet/second)")
    parser.add_argument("--main-rate", type=float, required=True,
                        help="Main parachute descent rate (feet/second)")
    parser.add_argument("--main-deploy", type=float, required=True,
                        help="Main deployment altitude AGL (feet)")
    parser.add_argument("--time", type=str, required=True,
                        help="Launch time UTC (ISO 8601, e.g. 2026-03-15T14:00:00Z)")
    parser.add_argument("--max-steering", type=float, default=15.0,
                        help="Maximum steering angle off vertical (degrees, default: 15)")
    parser.add_argument("--ground-elev", type=float, default=None,
                        help="Ground elevation ASL (feet). Default: from API.")
    parser.add_argument("--show-winds", action="store_true",
                        help="Print wind profile table")
    parser.add_argument("--plot", action="store_true",
                        help="Generate matplotlib plot")
    parser.add_argument("--html-map", type=str, default=None,
                        help="Output path for interactive HTML map (folium)")

    args = parser.parse_args()

    print(f"Fetching winds for ({args.landing_lat:.4f}, {args.landing_lon:.4f}) "
          f"at {args.time}...")

    result = compute_guidance_point(
        launch_lat=args.launch_lat,
        launch_lon=args.launch_lon,
        landing_lat=args.landing_lat,
        landing_lon=args.landing_lon,
        apogee_agl_ft=args.apogee,
        drogue_rate_fps=args.drogue_rate,
        main_rate_fps=args.main_rate,
        main_deploy_agl_ft=args.main_deploy,
        launch_time_utc=args.time,
        max_steering_deg=args.max_steering,
        ground_elev_ft=args.ground_elev,
    )

    if args.show_winds:
        print()
        print_wind_profile(result.wind_profile)
        print()

    print_result(result)

    if args.plot:
        try:
            from .viz import plot_guidance, plot_wind_profile as plot_wp
            plot_guidance(
                launch_pt=(args.launch_lat, args.launch_lon),
                landing_pt=(args.landing_lat, args.landing_lon),
                guidance_pt=(result.guidance_lat, result.guidance_lon),
                descent_track=result.descent_track,
            )
            plot_wp(result.wind_profile)
        except ImportError as e:
            print(f"Warning: could not import viz module: {e}", file=sys.stderr)

    if args.html_map:
        try:
            from .viz import generate_html_map
            generate_html_map(
                launch_pt=(args.launch_lat, args.launch_lon),
                landing_pt=(args.landing_lat, args.landing_lon),
                guidance_pt=(result.guidance_lat, result.guidance_lon),
                descent_track=result.descent_track,
                output_path=args.html_map,
            )
            print(f"HTML map saved to: {args.html_map}")
        except ImportError as e:
            print(f"Warning: folium not installed: {e}", file=sys.stderr)


if __name__ == "__main__":
    main()
