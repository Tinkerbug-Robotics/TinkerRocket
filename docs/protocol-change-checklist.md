# Protocol-Change Checklist

The ordered touch sequence for ANY wire-protocol change now that two apps ship
against the firmware (android-port plan §2.5). CI guards most steps; the ones
it cannot see are marked ⚠ MANUAL. Finish with `tools/preflight_protocol.sh`.

## A. The ordered sequence

1. **`RocketComputerTypes.h`** — struct/constant change. Append-only within
   versioned structs; bump `format_version`/length gates; update the in-header
   `static_assert` size pin and (app-visible structs, #386) `offsetof` pins.
2. **`tests_cpp/test_rocket_computer_types.cpp`** — update the hard-coded
   size/offset/uniqueness pins (they exist to force this stop).
3. **Firmware dispatch/consumers** — OC `main.cpp`, BS `main.cpp`, FC
   `main.cpp` if FC-consumed; `TR_BLE_To_APP.cpp` if the telemetry/config JSON
   vocabulary changes. ⚠ MANUAL: new telemetry keys must be priority-RANKED
   into the tiered MTU-budgeted builder, never appended (#282).
4. **Regenerate goldens** —
   `cmake --build tests_cpp/build --target regen-wire-fixtures`; review the
   diff (it IS the wire change, human-readable); update the affected entries'
   `notes` in `manifest.json`; commit bytes + sidecars. The freshness gtest
   fails until you do.
5. **iOS** — `BLEDevice.swift` encoder/decoder; `SensorTypes.swift` /
   `TelemetryData.swift` / `MagCalStatus.swift` as applicable; XCTests green.
   Add a hand-written intent test when the change has semantics (sentinels,
   gates), not just bytes.
6. **Android** — `:core:protocol` encoder/decoder + `BleCommandId.kt`; JVM
   golden walk green (`./gradlew test` in `TinkerRocketAndroid/`).
7. **`tools/check_ble_command_ids.py`** passes — automatic in CI (wire-codes);
   update `ALLOWED_DIVERGENCE` (with a comment) only when a feature ships on
   one app first, and file the parity-ledger entry (step 9).
8. **`Data_Analysis/*.py`** — ⚠ MANUAL: the parsers hardcode `FMT_`/msg
   lengths (#227); sweep them on any struct-size change. Also
   `replay_flight_ekf.py` if NonSensor/EKF fields moved (#529).
9. **`docs/android-parity-ledger.md`** — ⚠ MANUAL: if the change lands on one
   platform before the other, add a dated entry; remove it when the twin
   catches up.
10. **Bench validation** before the next flight (⚠ MANUAL — pause for a go,
    per the standing hardware-test rule).

## B. Change-type → extra surfaces

| Change | Extra surfaces to touch |
|---|---|
| New BLE command | OC or BS dispatch (independent namespaces — never reuse 56–58/61–63 history or 70–72), both apps' senders + `BleCommandId.kt`, checker |
| Grown log struct | New length gate in ALL decoders (C, Swift, Kotlin, Python), fixture regen, CSV column append (comma-free names! #514), EKF replay |
| New telemetry JSON key | Ranked insertion in the builder (#282), lenient-optional decode in both apps (ints via flexInt; floats/strings strict — mirror iOS exactly), fixture for the trimmed form |
| New file_ops frame | Discriminator must not collide with `{`/`[`/0xAA/0xCA–0xCF; both apps' demux ladders; full-frame fixture (discriminator byte included) |
| Config field | The six-way lockstep: `config.h` default == RocketProfile default in BOTH apps (a wrong default silently re-tunes the rocket on connect), NVS, readback JSON key + sentinel semantics, gtest |

## C. One-command local gate

```
tools/preflight_protocol.sh
```

runs: checker → C++ build+ctest (includes fixture freshness) → Android JVM
tests. iOS tests still need Xcode (`xcodebuild test` or CI).
