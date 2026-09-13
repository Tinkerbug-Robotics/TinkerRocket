# Bench-captured telemetry JSON (#624)

Real frames captured off the TELEMETRY characteristic on the bench via the
BenchSeamTest wire tap (`BenchJson` logcat tag).

**Re-captured 2026-09-13** on the current bench: **Rocket Computer V9**
fw `4b02a923-v9+20260912-1459` (BLE 9C:13:9E:28:AB:5E) + **BaseStation V4**
fw `e1a4bee-v2+20260910-1020` (BLE E0:72:A1:CA:E1:2E), rocket rail ON, no
GNSS fix indoors.

The previous capture was 2026-07-27 on a **V6** board with two *dirty*
firmware builds — a different board generation, from trees that correspond to
no commit, six weeks and many protocol changes stale. Both boards now carry
tagged firmware. The direct capture also gained three config kinds the V6 run
never emitted: `config_servo`, `config_guid`, `config_roll`.
This is the freshness net for the one protocol surface the emitter can't
generate — telemetry JSON is firmware *behavior* (TR_BLE_To_APP.cpp), not
a struct (plan §2.1 known hole).

- `rocket_direct.jsonl` — direct BLE link (MTU 512, untrimmed superset):
  one frame of each config kind heard + 5 telemetry frames.
- `bs_relay.jsonl` — base-station link: relayed-rocket telemetry with the
  BS battery fields (`bsoc`/`bvol`/`bcur`) + config kinds.

Privacy: `lat`/`lon` are quantized to 2 decimals (~1 km). Applied
mechanically by the harvest step, not by inspection — the 2026-09-13 capture
had no GNSS fix so every coordinate was already 0.0, and a rule that only
holds when the data happens to be empty is not a rule.

Re-capture procedure (protocol checklist step when telemetry JSON
changes): both boards on, iOS app force-quit, then per board
`./gradlew :core:ble:connectedDebugAndroidTest
  -Pandroid.testInstrumentationRunnerArguments.bench=1
  -Pandroid.testInstrumentationRunnerArguments.benchAddr=<MAC>`
and harvest `adb logcat -d -s BenchJson`.

Consumers: parser tests may load these as lenient-decode smoke input;
they are REFERENCE captures, not byte-pinned goldens — the firmware
adds fields freely (additive-only contract).
