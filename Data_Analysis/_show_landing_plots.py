#!/usr/bin/env python3
"""One-shot helper: render RIM-66 + Eagle Claw landing-predictor plots and
open both as interactive matplotlib windows.
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("MacOSX")
import matplotlib.pyplot as plt

_HERE = Path(__file__).parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "tinkerrocket-sim" / "src"))

from plot_landing_predictor import collect_trajectories, plot_flight
from landing_predictor import _find_binary

BASE = Path("/Users/christianpedersen/Documents/Hobbies/ModelRockets/TestFlights/2026_05_17")
PLOTS = _HERE.parent / "plots"
PLOTS.mkdir(exist_ok=True)

print("Rendering RIM-66...")
d1 = collect_trajectories(_find_binary(BASE / "RIM-66_65mm "),
                           [5, 15, 35.3, 36.8, 41.3, 46], -4.0)
plot_flight(d1, "RIM-66 (2026-05-17) — wind-only predictor, no drag",
            PLOTS / "landing_pred_RIM-66_65mm.png")

print("Rendering Eagle Claw...")
d2 = collect_trajectories(_find_binary(BASE / "Eagle Claw"),
                           [4, 8, 12.3, 14, 16], -4.0)
plot_flight(d2, "Eagle Claw (2026-05-17) — wind-only predictor, no drag",
            PLOTS / "landing_pred_Eagle_Claw.png")

print("Opening windows — close them to exit.")
plt.show()
