# SignalSim files for the rig

[SignalSim](https://github.com/globsky/SignalSim) (commit c83ae92) writes multi-GNSS IF files: GPS, Galileo and
BeiDou on L1, and GPS L5, Galileo E5a and BeiDou B2a on L5, where gps-sdr-sim only does GPS L1. It has **no
license**, so none of its code is in this repository. The bench keeps a patched build in
`~/Projects/ModelRockets/bench-backups/signalsim-builds/2026-09-30_gps-almanac-fix/`: the source with local changes,
the Release build every current file came from, the build from before the almanac fix, and the patches. This folder
holds only our side: the configs, the scripts that write them, and the tools around the files.

## Building SignalSim

The bench-backups build is the one to use: `gen.sh` and `../regen_c8.py` find it by default, and `SIGNALSIM`
points them at another. To rebuild from upstream c83ae92 on macOS (arm64, clang, CMake Release), two sets of local
changes are needed. The bench-backups README has the detail.

- **Build fixes.** Add missing standard headers, a missing `return`, and virtual destructors on the navigation-bit
  and trajectory-segment base classes. Without the destructors the arm64 build traps at exit and loses the end of
  the file.
- **The GPS almanac.** Fill the GPS almanac pages, put the health of SV 26-32 into subframe 4 page 25, and give
  pages with no almanac the dummy SV ID 0. Upstream sends every GPS almanac page as unhealthy filler, and a SkyTraq
  PX1105R then stops searching those satellites across cold starts (bench-verified 2026-09-30).

SignalSim is deterministic: `rand()` is never seeded and the build runs single-threaded, so the same binary and
config give the same bytes. `../regen_c8.py` relies on that.

## Making a file

    ./gen.sh hotshot_all_2026_45_w_p180          # configs/<name>.json -> ../c8/signalsim_<name>.C8, logs/gen_<name>.log
    ../regen_c8.py build signalsim_traveler_all_2026_45_w_p180_cofs.C8   # any manifest file, carrier shift included

Configs hold short relative paths (`EphData/...`, `c8/...`): SignalSim truncates long JSON strings and reads the
file in 255-byte lines, so the JSON stays indented. `gen.sh` runs from this folder and makes `EphData/` (the rig's
day-230 multi-GNSS broadcast file, from `../results/`) and a `c8` link to `../c8` on first use.

## The formats

| Format | Rate, centre | Carries |
|---|---|---|
| narrow (`_n`) | 8.184 Msps at 1575.42 MHz | GPS L1 C/A and Galileo E1, cleanly; BeiDou B1C too (the `gpsgalb1c` files) |
| wide (`_w`) | 18.48 Msps at 1568.286 MHz | adds BeiDou B1I; L1/E1 sit 7.1 MHz off centre near the HackRF's band edge, where Galileo suffers: keep the 20 MHz baseband filter |
| L5 | 18.48 Msps at 1176.45 MHz | GPS L5, Galileo E5a, BeiDou B2a |

Every file starts on the rig's date and place: 2026-08-18, latitude 0, longitude 119 W, 1200 m. The flights ignite at
08:40:00 GPST, as the gps-sdr-sim files do.

**Carrier.** SignalSim files lack the rig's carrier correction for the HackRF, which is -22 Hz on the narrow files
and -23 Hz on the wide ones. Without it, the PX1105R's GPS pseudoranges slide 4.2 m/s against the code. There are two
ways to apply it. `carrier_shift.py` bakes it into the file (the `_cofs` files). `hackrf_tx_ram -C -23`
(`TX_CARRIER_HZ` in `../px_run_ram.py`) applies it while transmitting, which is how the uncorrected wide files fly.

## Files

| File | What it is |
|---|---|
| `configs/` | The exact configs behind every SignalSim file in `../c8_manifest.json`, plus the rest of the 2026 set |
| `make_2026_configs.py` | The 2026-dated static and traveler configs, narrow and wide. Turns a vertical flight CSV into a pad segment plus 0.1 s vertical-acceleration segments |
| `make_wide_p180.py`, `make_wide_p180_levels.py` | The wide traveler cut to a 180 s pad and T+360, so it fits the page cache, and the same at other levels |
| `make_wide_hotshot.py` | The hotshot in the same wide 180 s-pad format, at the levels given |
| `make_l5_config.py`, `make_l5w_config.py` | The static L5-band file, and its warm-start twin that starts 160 s after the hotshot L1 file |
| `make_b1c_config.py` | The static narrow file with BeiDou B1C added, for the stage-0 software receiver |
| `make_b1c_flights.py` | The hotshot and the traveler of the wide 180 s-pad files, narrow and with BeiDou on B1C: every signal the stage-0 software receiver tracks |
| `gen.sh` | Runs IFdataGen on configs |
| `carrier_shift.py` | Carrier-only frequency shift of an IQ8 file, with code timing untouched |
| `sat_summary.py` | How clean a PX1105R flight of a SignalSim file was, per constellation, from the capture and the generation log |

Re-running the `make_*` scripts (traveler and hotshot ones need `../scenarios/`, which `../make_flights.py` and
`../pad_scenario.py` rebuild byte for byte) writes `configs/` back byte for byte.
