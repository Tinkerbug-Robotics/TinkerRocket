# Bench-captured telemetry JSON (#624)

Real frames captured off the TELEMETRY characteristic on the bench
(2026-07-27, Rocket Computer V6 fw 34742135-dirty + BaseStation V4 fw
7d7fd0b-dirty) via the BenchSeamTest wire tap (`BenchJson` logcat tag).
This is the freshness net for the one protocol surface the emitter can't
generate — telemetry JSON is firmware *behavior* (TR_BLE_To_APP.cpp), not
a struct (plan §2.1 known hole).

- `rocket_direct.jsonl` — direct BLE link (MTU 512, untrimmed superset):
  one frame of each config kind heard + 5 telemetry frames.
- `bs_relay.jsonl` — base-station link: relayed-rocket telemetry with the
  BS battery fields (`bsoc`/`bvol`/`bcur`) + config kinds.

Privacy: `lat`/`lon` are quantized to 2 decimals (~1 km).

Re-capture procedure (protocol checklist step when telemetry JSON
changes): both boards on, iOS app force-quit, then per board
`./gradlew :core:ble:connectedDebugAndroidTest
  -Pandroid.testInstrumentationRunnerArguments.bench=1
  -Pandroid.testInstrumentationRunnerArguments.benchAddr=<MAC>`
and harvest `adb logcat -d -s BenchJson`.

Consumers: parser tests may load these as lenient-decode smoke input;
they are REFERENCE captures, not byte-pinned goldens — the firmware
adds fields freely (additive-only contract).
