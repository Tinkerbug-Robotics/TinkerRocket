# Radiated setup, Faraday cage — 2026-08-20

Flight computer (ESP32-P4, u-blox SAM-M10Q) in a cage with a whip antenna fed
from the HackRF through 100 dB of attenuation. FC built with
`-DTR_GNSS_COCOM_DIAG=1`; observed over its own USB console, no serial tap.

## Cage floor

With nothing transmitting: **0 satellites above 30 dBHz, peak 19 dBHz.** Real
GPS does leak in but far too weakly to fix, so anything the receiver does under
injection is ours.

## Gain sweep, static scenario at the equator

    gain   sats>=30   peak C/N0   median   fix epochs (of ~45)
       0         14          43       42        30
      12         15          48       47        34
      24         14          48       48        34
      36         16          48       48        34
      47         14          48       48        35

It works at every gain, including 0, and plateaus from 12 upward — the
receiver's AGC saturating. **Use gain 12**: the bottom of the plateau, so there
is margin in both directions.

This is far more generous than the link budget predicted (which wanted 39-45 at
100 dB). A cage is a resonant cavity rather than free space, so energy is
concentrated instead of spreading; the calculation in the README is the right
way to pick a starting point and the wrong way to pick a final one.

## Confirmation that the lock is ours

    injected   0.00000 N  -119.00000 E   1200 m
    reported   0.00002 N  -119.00001 E   1186 m   FIX_3D, ok=1, 13 SV

About 2 m horizontally. Time to first fix ~11 s — far quicker than the SkyTraq
bench receiver's 43-186 s, and with no seeded restart needed at all.

## Notes for the next session

- 13-16 satellites tracked against 14 transmitted: the u-blox is using nearly
  everything on offer, which the SkyTraq rarely managed.
- Nothing measured on the PX1125R transfers. u-blox enforces its own limits
  near 500 m/s and 80 km, documented as independent. Re-measure.
- The FC warns `[ORIENT] pad gravity 94.9 deg off nose with MANUAL orientation
  +X` — unrelated to GNSS, but it will skew anything orientation-dependent.
