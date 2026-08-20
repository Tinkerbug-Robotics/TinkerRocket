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

nav, lat, lon = sys.argv[1], sys.argv[2], sys.argv[3]
day = sys.argv[4]                      # YYYY/MM/DD
sim = Path(__file__).resolve().parent / "bin" / "gps-sdr-sim"

print(f"GPS satellites above the horizon at {lat},{lon} on {day}\n")
best = (0, None)
for hh in range(0, 24):
    t = f"{day},{hh:02d}:30:00"
    r = subprocess.run([str(sim), "-e", nav, "-l", f"{lat},{lon},1200",
                        "-d", "1", "-b", "8", "-o", "/dev/null", "-t", t, "-p"],
                       capture_output=True, text=True)
    out = (r.stdout + r.stderr).replace("\r", "\n")
    svs = [l.split() for l in out.splitlines()
           if l[:2].strip().isdigit() and len(l.split()) >= 4]
    n = len(svs)
    if n == 0:
        continue
    elevs = sorted(float(s[2]) for s in svs)
    bar = "#" * n
    print(f"  {hh:02d}:30  {n:2d} SV  {bar:<14} lowest {elevs[0]:5.1f} deg, "
          f"highest {elevs[-1]:5.1f} deg")
    if n > best[0]:
        best = (n, t)
print(f"\nbest: {best[0]} satellites at {best[1]}")
print("Ephemeris coverage limits this as much as geometry does -- a partial-day\n"
      "broadcast file has few satellites at the hours it was thin on.")
