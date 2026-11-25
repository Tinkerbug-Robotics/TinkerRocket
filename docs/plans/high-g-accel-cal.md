# The high-g accelerometer's six-face calibration, aligned to the low-g

**Written 2026-10-18, branch `fc/highg-accel-cal`** (from origin/main 08a5ce0f).
The ISM6HG256X has two accelerometers: a ±16 g low-g and a ±256 g high-g. The
flight loop flies the low-g and switches to the high-g only while a low-g axis
is near its rail (15.5 g, #1191). Until now the high-g got one correction: the
pad calibration's bias, high-g minus low-g at the pad's attitude. Its own
offset apart from that attitude was never measured, and neither were its
scale and its misalignment against the low-g.

This extends the six-face calibration (#1629) to the high-g. Not on hardware
yet. Whether it goes into the Goblin's 2026-10-24 image is the user's call
(see "Bench plan").

## The model

One correction for the high-g, in the board frame (after the ISM6's Z
rotation, before the board-to-rocket mounting), m/s²:

    high = M * h - b - o
    low  =     l     - o

- `l`, `h`: the low-g and high-g readings, from the counts at the configured
  full scales.
- `M`: the alignment, a 3x3 (scale on the diagonal, cross-axis off it) that
  makes the high-g read as the low-g does.
- `b`: the high-g's offset against the raw low-g.
- `o`: the low-g's six-face offset, the same as before.

The six faces fit `M` and `b` to the raw low-g by least squares: `l = M h - b`
on every face, one row at a time. Each row has four unknowns and six
equations. The low-g is the reference because it is the more accurate channel
on the bench. The fit also means the high-g takes over at the low-g's rail
with no step. The low-g's own scale is not applied (only its offset is), so
the high-g follows the low-g's scale too. That keeps the switch seamless. The
Beetle's low-g scales came out within 0.1 %.

The high-g's own gravity fit is the low-g's sphere fit run on the high-g. It
is shown beside the alignment as a check. The two are compared:

- **Offset:** the high-g's zero-g that the alignment implies, `M^-1 (b + o)`,
  against the sphere fit's offset.
- **Scale:** `M_ii` against the low-g's scale over the high-g's.

The difference counts as a disagreement past 3 sigma, the sigma coming from
the faces' noise with floors of 0.5 mg and 0.02 %. **A disagreement is
reported, not refused: the alignment to the low-g is saved either way**, as
the user asked. What makes the two disagree is something neither model has,
such as an even-order nonlinearity. The tests use one of 4 mg at 1 g.

A save is refused (the screen won't send it, and the FC won't store it) when:

- a scale is outside 0.97-1.03 (each channel's sensitivity is trimmed to
  ±1 %);
- a cross-axis term is past 0.03 (1.7°);
- an offset is past 11 m/s². The high-g's zero-g is within ±1000 mg, the
  low-g's within ±65 mg.

The screen and the script also refuse a fit that leaves any face more than
3 mg from the low-g. A still 3 s face leaves about 0.3 mg rms, so 3 mg means
the board moved or warmed up between faces.

**The pad calibration** keeps `M` and measures `b` again, as
`mean(M h) - mean(l)` over its still window. With `M` the identity this is
exactly the old bias, so nothing changes for a board without a six-face
high-g fit. The high-g's offset drifts (about 2 mg/°C; the V9's moved 40-70 mg
between two bench days), while `M` is a property of the part. So the six faces
set `M` once per board, and the pad calibration refreshes `b` on the day.

**Sim:** the alignment is off with the low-g offset (`setLowGOffsetEnabled`),
because the sim's counts are the truth's.

**The OC:** its converter, which feeds the app's telemetry `hx/hy/hz`, applies
the bias but not `M` (nor the low-g offset, as before). Telemetry high-g is for
display only. The flight computer is what flies.

## Storage and compatibility

| Where | What |
|---|---|
| NVS `hg_cal` (new) | `ver` = `HG_CAL_NVS_SCHEMA_VERSION` (1), `done`, `m` (9 floats, row-major). Without it, `M` = identity. |
| NVS `cal` (as before) | `hgbx/hgby/hgbz` = `b`, `gx/gy/gz`; new `hgsrc` (1 = the pad calibration or a profile push, 2 = the six faces) |
| NVS `acc_cal` | untouched (ver 1): an older image still reads the low-g offset |

**Migration:** today's `hgb*` bias is the `M` = identity case of the model, so
it carries over as it is: no rewrite, nothing to do. An older image ignores
`hg_cal` and flies the high-g on its bias alone.

| Link | Change |
|---|---|
| 0xD1 (ACC_CAL_STATUS_MSG) | 40 → 114 bytes: `AccelCalStatusData` unchanged, then `AccelCalHighGStatus` (the face window's high-g mean and SD, `hg_flags`, `hg_bias_src`, the stored `M`, the `b` in use) |
| BLE 80 → OC → FC 0x85/0x86 | `HighGCalApplyData` (49 B): op 1 store `M` and `b`, op 0 clear `M` (`b` kept until a pad calibration) |
| PAD_CONTEXT_MSG | `PCX_HG` (7): `M` and whether it was applied, beside `PCX_CAL`/`PCX_ACC`; the parser applies it |

Old/new pairs, all handled:

| App | OC | FC | Result |
|---|---|---|---|
| new | new | new | low-g offset and high-g alignment |
| new | old | new | The old OC relays 40 bytes and drops cmd 80. The screen says "The faces came without the high-g reading. Update the flight computer and the out computer…", and the offset saves as before. |
| new | new | old | 40-byte frames, cmd 80 unanswered: the same message |
| old | any | new | The old app reads the first 40 bytes (it checks `>=`): its six-face screen is unchanged |

The FC's config read needs an exact payload length (`readConfigFrame`). So
the high-g has its own command and does not lengthen 76's payload, which an
older FC would never find.

**Profile push:** a rocket profile that holds this board's sensor cal pushes
its gyro and `b` on connect (cmd 62). After a confirmed high-g save, the app
writes the new `b` into that profile, so the push does not restore the older
pad-cal bias under the new `M`. A second phone with an older profile can still
push an older `b`. The next pad calibration replaces it, as it does today.

## How to calibrate

In the app: Settings → General → Calibration → Accelerometer Calibration (Six
Faces). Or use `tools/accel_cal_six_face.py` from the Terminal panel; the
phone app must be disconnected.

1. Power the flight computer and let it warm up for ~5 minutes: the high-g's
   offset moves ~2 mg/°C.
2. Measure the six faces in one go, hands off during each 3 s window. Any
   order works. After the sixth, the screen saves by itself: it sends the
   offset (76), reads it back, then the alignment (80), and reads that back
   too. Both lines should be green. Save Again sends it once more, and a face
   measured again saves the new solve.
3. Read the High-g Accelerometer section:
   - each axis's scale and offset with ±1 sigma;
   - the cross-axis term;
   - the fit to the low-g, which should be well under 1 mg;
   - the agreement line;
   - "Passed: saved with the offset" or "Not saved: <why>".
4. Before a flight, run the pad calibration as usual. It keeps the alignment
   and measures the high-g's offset for the day; the stored section then shows
   its source as "measured by the pad calibration".

`accel_cal_six_face.py --check` measures any attitude after a save and prints
the corrected high-g less the calibrated low-g. It checks the fit on an
attitude the fit never saw, such as a 45° tilt.

## Precision

Measured on the Goblin's V9 at rest (`flight_20261017_092613.bin`, 3780 Hz):

- high-g: 57 mg per sample (7 counts of 7.8 mg, well dithered);
- low-g: 1.9 mg per sample.

The high-g's noise density is about 1.3 mg/√Hz. Its datasheet says 1.0, for
the sibling ISM6HGK256X; st.com did not serve the ISM6HG256X PDF. The density
does not depend on the data rate, so a window's mean is set by its length, not
the IMU rate. A 3 s mean came to 0.24-0.45 mg on the log, against 0.54 mg for
white noise.

The table below assumes white noise plus 0.2 mg of drift per face. Its numbers
are the fit's own reported sigma; Monte Carlo over 200 trials gave the same to
within 5 %.

| Face window | High-g mean, 1σ | Scale, 1σ | Cross-axis, 1σ | Offset, 1σ |
|---|---|---|---|---|
| 1 s | 0.93 mg | 0.070 % | 0.040° | 0.41 mg |
| **3 s (as built)** | **0.54 mg** | **0.042 %** | **0.024°** | **0.25 mg** |
| 10 s | 0.29 mg | 0.026 % | 0.015° | 0.15 mg |
| 30 s | 0.17 mg | 0.019 % | 0.011° | 0.11 mg |
| 60 s | 0.12 mg | 0.017 % | 0.010° | 0.10 mg |

What this means:

- **The 3 s windows are enough.** A ±256 g part sees only ±1 g on the bench,
  but its noise is low enough that 3 s faces give the scale to 0.04 % and the
  axes to 0.02°.
- **Longer windows buy little.** Past ~30 s the drift floor dominates: the
  offset moves while the board is handled.
- **The part limits the result more than the fit does.** The high-g's
  sensitivity moves ±0.01 %/°C typical, so a launch day 20 °C colder than the
  bench is 0.2 %. Its nonlinearity is 2 % of full scale typical (sibling
  datasheet). The six faces measure the scale at ±1 g; how well that holds at
  20-100 g is what the in-flight overlap fit below would show.
- **What the alignment fixes in flight.** It is the high-g's scale and
  cross-axis against the low-g, of order 1 %. On the synthetic board in
  `test_high_g_cal` the old bias alone was 1 g off at 120 g; with `M` the
  error is a few counts. The offset matters least: a 1 mg error is 0.007 %
  at 15 g.

## Bench plan (the Goblin's V9)

The V9 ("RC V9", FC USB serial = MAC **80:F1:B2:D0:94:A7**) is the Goblin's
flight board for 2026-10-24. Its flight image comes from `fc/v9-raw-primary`,
with 393fed6c as the fallback. This branch is cut from main, so a bench image
from it is not the flight image. To fly the high-g calibration, this branch
would be merged into the flight branch, and that is the user's decision.

Rules: ask before every flash. Before any esptool call, check the port's USB
serial with
`ioreg -p IOUSB -l -w0 | grep -E '"USB Serial Number"|"USB Product Name"'`.
It must read 80:F1:B2:D0:94:A7 for the FC; record the OC's MAC the first time
it is seen. Never open a console with DTR=RTS=0, which resets an S3.

1. **Record what is stored now** (current image, BLE from the Terminal panel):
   - `accel_cal_six_face.py --name "RC V9" --read` for the low-g offset;
   - the app's sensor-cal readback (63) for the gyro and the high-g bias.

   This is the baseline to restore. Optional, on the user's OK: an NVS
   partition backup with `esptool read-flash`.
2. **Flash** the FC (USB, MAC checked), then the OC (USB, or OTA through the
   app).
   - Boot log: `NVS high-g alignment: none -- the high-g flies its bias alone`,
     plus the existing `NVS HG bias` and `NVS low-g offset` lines.
   - With nothing new stored, the high-g flies the old arithmetic: it is the
     same code path, bit-identical on the host against origin/main.
3. **No regression with nothing stored:**
   - READY, telemetry live, the pad calibration's result as before;
   - with the six-face screen open and before any save, `Stored Offset` shows
     the old offset and "No high-g alignment stored", with the high-g bias
     "measured by the pad calibration" (or "source not recorded" for a bias
     written by an older image).
4. **Six faces** (warm-up first). In the Goblin the board turns with the
   rocket, or it can be done out of the airframe. Expect:
   - fit to the low-g under 1 mg rms;
   - scales within ±1.5 %;
   - offsets of order ±150 mg against the low-g (the V9's raw high-g minus
     low-g was 11, -39 and -68 mg on top of its stored bias on 10-17);
   - "agrees".

   The save follows the sixth face by itself, and both read-backs are green.
5. **Power-cycle the FC** (cmd 8 off for 15 s, then on). The boot log shows
   `NVS high-g alignment: diag …` and the screen "High-g alignment in use".
6. **`--check` on new attitudes:** 45° tilts and an upside-down. Each should
   read within ~1-2 mg of the low-g on every axis.
7. **Pad calibration** with the rocket still. Then:
   - the FC log shows `High-g aligned: … (the six-face alignment, kept)`;
   - the bias source reads "measured by the pad calibration";
   - the new bias is within a few mg of the six-face one (same temperature);
   - `--check` still reads ~1 mg.
8. **Optional:**
   - **Clear** (the screen's Clear the Stored Calibration…): the low-g offset
     gone, "No high-g alignment stored", the bias kept. Then redo step 4.
   - **Downgrade to the flight image:** the low-g offset is still read (its
     namespace is untouched), and the high-g flies its bias alone.

Not checkable on the bench: the high-g's scale above 1 g. That is the
follow-up below.

## Follow-up: an in-flight fit in the 2-16 g overlap

During the boost both channels see 2-15 g on the thrust axis, sample by
sample, before the switch. That is the only range where the high-g's scale
can be checked against the low-g at flight accelerations, and where the 2 %FS
nonlinearity would show.

**Post-flight, first (no risk):**

- The log has both channels' raw counts for every sample (`ISM6HG256_MSG`).
- A script over the boost window fits `l = M h - b` per axis on the samples
  where every low-g axis is under ~12 g, so the low-g's own nonlinearity near
  its rail stays out. The thrust axis is above 2 g there.
- It low-passes both channels first, to ~50 Hz. Their analog bandwidths
  differ (low-g 0.6 kHz, high-g 2 kHz), so under motor vibration their phases
  differ, and an unfiltered fit would read that phase difference as scale.
- It reports:
  - the in-flight scale against the six-face one;
  - the residual against acceleration (curvature = nonlinearity);
  - the step at the switch with the six-face `M` and without it.
- One Goblin or Beetle flight with a six-face fit stored would answer whether
  `M` at ±1 g holds at 15 g.

**On board, later, and only as a monitor:**

- Accumulate the normal-equation sums (`Σ h hᵀ`, `Σ h lᵀ`, `Σ h`, `Σ l`, n)
  on low-passed samples while 2 g < |a| and the low-g is below 12 g, before
  the first switch.
- Log the fit at burnout. Do not apply it: a few flights should show it
  stable first, and a boost is short and noisy enough that a bad fit is easy.
- Applying it in flight would also change the attitude filter's input
  mid-boost. That is a design question for the filter work, not this
  calibration.
