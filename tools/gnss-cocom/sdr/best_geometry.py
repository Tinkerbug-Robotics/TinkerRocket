#!/usr/bin/env python3
"""Find the time of day with the most GPS satellites above the horizon.

Worth an explicit answer because "add more satellites" is not available the way
it sounds: gps-sdr-sim is GPS L1 C/A only, so the count is set by how much of a
31-satellite constellation happens to be overhead. It varies by a couple either
way over a day; it does not go to twenty. The real lever is bands and
constellations, which needs a different generator.
"""
import subprocess, sys, datetime as dt
from pathlib import Path

nav = sys.argv[1]
day = sys.argv[2]                      # YYYY/MM/DD
lats = [float(x) for x in sys.argv[3].split(",")]
hours = [int(x) for x in sys.argv[4].split(",")]
lon = sys.argv[5] if len(sys.argv) > 5 else "-119"
sim = Path(__file__).resolve().parent / "bin" / "gps-sdr-sim"

def count(lat, hh):
    t = f"{day},{hh:02d}:30:00"
    r = subprocess.run([str(sim), "-e", nav, "-l", f"{lat},{lon},1200",
                        "-d", "1", "-b", "8", "-o", "/dev/null", "-t", t, "-p"],
                       capture_output=True, text=True)
    out = (r.stdout + r.stderr).replace("\r", "\n")
    return sum(1 for l in out.splitlines()
               if l[:2].strip().isdigit() and len(l.split()) >= 4)

print(f"GPS satellites above the horizon, {day}, lon {lon}")
print("Ephemeris: " + Path(nav).name + "\n")
print("  lat  " + " ".join(f"{h:02d}h" for h in hours) + "   min  max  mean")
best = (0, None)
for lat in lats:
    row = [count(lat, h) for h in hours]
    mean = sum(row) / len(row)
    print(f"  {lat:>4.0f}  " + " ".join(f"{n:>3d}" for n in row) +
          f"   {min(row):>3d}  {max(row):>3d}  {mean:>4.1f}")
    for h, n in zip(hours, row):
        if n > best[0]:
            best = (n, f"lat {lat:.0f}, {h:02d}:30")
print(f"\nbest single epoch: {best[0]} satellites at {best[1]}")
