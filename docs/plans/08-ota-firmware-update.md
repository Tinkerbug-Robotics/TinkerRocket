# OTA firmware update via iOS app

**Issue:** [#8](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/8)
**Status:** Phase 1 — design. No firmware or iOS code lands in Phase 1.
**Last updated:** 2026-05-28

Enable over-the-air firmware updates for all three computers (Flight Computer, Out Computer, Base Station) directly from the iOS app over BLE. Today, every firmware change requires opening the rocket, plugging in USB-C, and running `idf.py flash` — fine on the bench, painful at the field.

This document is the contract that Phases 2–4 implement against.

## Phasing

| Phase | Scope | Why this order |
|---|---|---|
| 1 | Design doc covering all three targets | Lock the protocol, partition layout, identity contract, and FC relay strategy before any code lands. |
| 2 | Base Station implementation | BS is the lowest-risk target — handheld, easy USB recovery, not airborne. Shake the protocol down here first. |
| 3 | Out Computer implementation | Same protocol, ported. Mostly mechanical once Phase 2 is solid. |
| 4 | Flight Computer (relayed via OC) | Hardest. FC has no BLE; firmware must traverse OC → existing OC↔FC wires → FC. See §7 for the chosen split-bus design. |

## Resolved design decisions

| # | Decision |
|---|---|
| 1 | Partition layout: keep 3 MB app slots, add a second 3 MB `ota_1` slot. Drop SPIFFS entirely on OC and FC (never mounted in source today). BS keeps a 2 MB SPIFFS for SD-fallback logging. |
| 2 | BLE transport: reuse the existing File Transfer characteristic (made writable from the central) for image chunks; reuse the existing Command characteristic with three new command codes (70/71/72) for control; reuse the existing File Operations characteristic for status replies. No new GATT characteristics. |
| 3 | Integrity: full-image SHA-256 sent in `OTA_BEGIN`, verified by firmware before `esp_ota_set_boot_partition`. |
| 4 | Rollback: enable `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`. Newly-booted firmware calls `esp_ota_mark_app_valid_cancel_rollback()` only after its first successful telemetry round-trip; anything earlier (panic, hang, BLE-stack init failure) auto-rolls back to `ota_0`. |
| 5 | Identity: add `"fw"` field to the existing `config_identity` JSON. Format: `<git_short_sha>+<build_yyyymmdd-hhmm>`. iOS uses it for pre-flash display and post-reboot verification. |
| 6 | iOS source for `.bin` files: native `.fileImporter` (Files / iCloud / AirDrop). No bundled-in-app firmware, no HTTPS distribution in v1. |
| 7 | FC relay path: cooperative FC-side OTA using the existing 5 wires — I2C (control plane) + I2S reconfigured OC→FC at high BCLK (data plane). FC pauses sensor operations during the transfer window. See §7. |
| 8 | Out of scope: secure boot v2 (separate hardening issue); HTTPS firmware distribution; OTA over LoRa; parallel multi-device flash. |

---

## 1. Partition layout

All three boards are 8 MB. New `partitions.csv`:

### Base Station

```
# Name,   Type, SubType, Offset,  Size
nvs,      data, nvs,     0x9000,  0x5000
otadata,  data, ota,     0xe000,  0x2000
ota_0,    app,  ota_0,   0x10000, 0x300000
ota_1,    app,  ota_1,   0x310000,0x300000
spiffs,   data, spiffs,  0x610000,0x1F0000
```

Total ends at `0x800000` (8 MB). BS keeps 2 MB SPIFFS because [base_station/main/main.cpp:2676](../../tinkerrocket-idf/projects/base_station/main/main.cpp:2676) mounts it as an SD-card fallback log store; 2 MB is enough for an emergency log when no SD is present.

### Out Computer

```
# Name,   Type, SubType, Offset,  Size
nvs,      data, nvs,     0x9000,  0x5000
otadata,  data, ota,     0xe000,  0x2000
ota_0,    app,  ota_0,   0x10000, 0x300000
ota_1,    app,  ota_1,   0x310000,0x300000
coredump, data, coredump,0x610000,0x10000
```

Total ends at `0x620000` (6.125 MB used of 8 MB). SPIFFS removed — `grep` confirms OC never mounts a SPIFFS partition. Coredump partition retained. ~1.9 MB trailing space available for future partitions (e.g., dedicated OTA staging area for Phase 4 if NAND staging proves awkward).

### Flight Computer

```
# Name,   Type, SubType, Offset,  Size
nvs,      data, nvs,     0x9000,  0x5000
otadata,  data, ota,     0xe000,  0x2000
ota_0,    app,  ota_0,   0x10000, 0x300000
ota_1,    app,  ota_1,   0x310000,0x300000
```

Total ends at `0x610000` (6.06 MB used of 8 MB). SPIFFS removed. ~2 MB trailing space available.

### Headroom

Current binaries (May 2026): BS = 763 KB, FC = 558 KB. The 3 MB app slot leaves ~4× headroom — sufficient for the medium-term feature roadmap. Revisit only if a single binary crosses ~2 MB.

### NVS offset unchanged

NVS stays at `0x9000` on every board, so existing NVS contents (unit name, network ID, rocket ID, mag-cal data, etc.) survive the partition-table change. No migration step needed.

---

## 2. BLE OTA protocol

Reuses existing characteristics defined in [TR_BLE_To_APP.h:213](../../tinkerrocket-idf/components/TR_BLE_To_APP/TR_BLE_To_APP.h):

- **Service** `4fafc201-1fb5-459e-8fcc-c5c9c331914b` (unchanged)
- **Command** `cba1d466-344c-4be3-ab3f-189f80dd7518` (write — extended with new command codes)
- **File Operations** `8d53dc1d-1db7-4cd3-868b-8a527460aa84` (notify — extended with `ota_status` JSON replies)
- **File Transfer** `1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d` (currently read+notify; extended to also be writable)

### 2.1 Control commands (Command characteristic)

| Code | Name | Payload | Firmware action |
|------|------|---------|-----------------|
| 70 | `OTA_BEGIN` | `[target:1][total_size:4 LE][sha256:32]` (37 bytes) | `esp_ota_begin()` on `ota_1`; reset SHA-256 state; notify `ota_status: ready` |
| 71 | `OTA_FINISH` | none | Finalize SHA; compare to expected; if match: `esp_ota_set_boot_partition()` + schedule 500ms-deferred `esp_restart()`; if mismatch: `esp_ota_abort()`, notify `verify_failed` |
| 72 | `OTA_ABORT` | none | `esp_ota_abort()`; clear state; notify `aborted` |

(Codes 70/71/72 picked to clear the existing 1-60 range used for LoRa/servo/PID/etc. command dispatch in main.cpp. The originally-proposed 10/11/12 collided with `sendLoRaConfig` and friends; corrected before any code shipped.)

`target` field in `OTA_BEGIN`:
- `0` = app on this device (BS or OC self-update; Phase 2 and Phase 3)
- `1` = Flight Computer relay (OC only; Phase 4)

A new `OTA_BEGIN` while a session is already active aborts the prior session first.

### 2.2 Image stream (File Transfer characteristic)

Today this characteristic is read+notify (device → phone). For OTA we add the **write** property (`BLE_GATT_CHR_F_WRITE`) so the phone can stream chunks in.

Per-chunk write payload — **identical framing to the existing download direction**:

```
[offset:4 LE][length:2 LE][flags:1][data:length]
```

`flags` bit 0 = EOF marker (set on the final chunk).

The phone writes with response (`writeValue(_:for:type: .withResponse)`) — the BLE stack's per-write callback gives delivery acknowledgment. On callback failure the phone retries the same chunk. No application-layer per-chunk ACK protocol is needed.

Chunk size matches the existing download path: `MTU - 10` bytes of payload (so ~502 B at 512 MTU, ~175 B at 185 MTU).

### 2.3 Status replies (File Operations characteristic)

Firmware notifies JSON status as state changes. All replies use the `"type":"ota_status"` discriminator (existing convention — peer-existing types include `config_identity`, `config_rocket`, etc.).

```json
{"type":"ota_status","state":"ready","slot":1}
{"type":"ota_status","state":"writing","bytes":131072}
{"type":"ota_status","state":"verify_failed","err":"sha_mismatch"}
{"type":"ota_status","state":"verify_failed","err":"write_failed"}
{"type":"ota_status","state":"aborted"}
{"type":"ota_status","state":"ready_to_boot","fw":"abc1234+20260601-1430"}
```

`writing` is rate-limited to ≤ 2 Hz to avoid drowning the notify queue mid-flash.

Each JSON message must stay under `MTU - 3` bytes — already enforced for the existing config JSONs by the guard at [TR_BLE_To_APP.cpp:800](../../tinkerrocket-idf/components/TR_BLE_To_APP/TR_BLE_To_APP.cpp:800). All `ota_status` messages above fit easily within 185-byte MTU.

### 2.4 Session lifecycle (happy path)

```
phone                                            device
  │  OTA_BEGIN(target, size, sha256)               │
  │ ────────────────────────────────────────────►  │  esp_ota_begin(ota_1)
  │  ◄──────────────  {state:"ready"}              │
  │                                                │
  │  chunk[0..N]   (FileTransfer write)            │
  │ ────────────►                                  │  esp_ota_write
  │  ◄────────────  {state:"writing", bytes:…}     │  (throttled to 2 Hz)
  │                          …                     │
  │  chunk[N], EOF flag set                        │
  │ ────────────►                                  │
  │                                                │
  │  OTA_FINISH                                    │
  │ ────────────────────────────────────────────►  │  SHA verify, set_boot_partition
  │  ◄────────────  {state:"ready_to_boot", fw:…}  │
  │                                                │  esp_restart() after 500 ms
  │  (BLE disconnect)                              │
  │  reconnect → read identity → compare fw        │
```

### 2.5 Error paths

| Trigger | Firmware response | Phone reaction |
|---|---|---|
| SHA mismatch in `OTA_FINISH` | `verify_failed`, `esp_ota_abort`, no boot change | Surface error, allow retry from picker |
| Out-of-order or out-of-range chunk offset | `verify_failed: bad_offset`, `esp_ota_abort` | Treat as fatal; abort + restart from scratch |
| Phone-side disconnect mid-stream | (device times out after 30s of no chunks, calls `esp_ota_abort`) | On reconnect, send `OTA_ABORT` defensively then start fresh |
| Power loss mid-flash | (no change — `ota_1` is half-written but unused; bootloader still boots `ota_0`) | Restart OTA from scratch |
| `OTA_BEGIN` while a session is active | `esp_ota_abort` prior session, start new | (initiated by phone; phone considers prior session lost) |

### 2.6 Throughput estimate

At 502 B chunks and ~15 ms between write-with-response acks (BLE 4.2 typical):
- 800 KB BS binary → ~24 s
- 1.5 MB worst-case → ~45 s
- 3 MB hypothetical → ~90 s

Acceptable for a manual-trigger workflow. Phase 4's FC relay uses a faster path (high-speed I2S between OC and FC — see §7), so the OC↔FC leg is not the bottleneck even with the BLE leg in front of it.

---

## 3. Identity / version contract

Add `"fw"` field to the existing `config_identity` JSON, built at [out_computer/main/main.cpp:2058](../../tinkerrocket-idf/projects/out_computer/main/main.cpp:2058) and the analogous BS / FC spots.

Format: `<git_short_sha>+<build_yyyymmdd-hhmm>`, e.g. `"fw":"dce9599+20260527-2103"`.

Source — compile-time macro injected by each project's `CMakeLists.txt`:

```cmake
execute_process(
    COMMAND git rev-parse --short HEAD
    OUTPUT_VARIABLE TR_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
string(TIMESTAMP TR_BUILD_DATE "%Y%m%d-%H%M")
add_compile_definitions(TR_FW_VERSION="${TR_GIT_SHA}+${TR_BUILD_DATE}")
```

JSON change:

```c
snprintf(id_buf, sizeof(id_buf),
         "{\"type\":\"config_identity\""
         ",\"uid\":\"%s\""
         ",\"un\":\"%s\""
         ",\"nid\":%u"
         ",\"rid\":%u"
         ",\"dt\":\"%s\""
         ",\"fw\":\"%s\"}",
         unit_id_hex, unit_name,
         (unsigned)network_id, (unsigned)rocket_id,
         config::DEVICE_TYPE, TR_FW_VERSION);
```

Size impact: ~30 bytes per identity payload. Stays well under any MTU (per [feedback_ble_json_size.md](../../../.claude/projects/-Users-christianpedersen-Documents-Hobbies-ModelRockets-Code/memory/feedback_ble_json_size.md)).

The 13-character `<sha>+<date>` form is also short enough to display verbatim in iOS UI.

---

## 4. Rollback safety

Enable `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` in all three sdkconfig defaults.

Sequence:

1. Phone sends `OTA_BEGIN` → chunks → `OTA_FINISH`.
2. Firmware verifies SHA, calls `esp_ota_set_boot_partition(ota_1)`, schedules `esp_restart()` 500 ms later (lets the final `ready_to_boot` status notification flush).
3. On reboot, bootloader sees `ota_1` is marked `PENDING_VERIFY` in `otadata`; boots it.
4. New firmware comes up. BLE stack starts. First client connects, requests identity, identity sent successfully.
5. **Only then** does firmware call `esp_ota_mark_app_valid_cancel_rollback()`. Image becomes permanent.
6. If any step in (4) fails — panic in startup, BLE stack init failure, hang — device reboots. Bootloader sees `PENDING_VERIFY` still set, automatically rolls back to `ota_0`.

The "first client round-trip" gate is deliberately strong: it proves not just that the firmware boots, but that BLE + identity-builder are intact. The user reconnecting and seeing the *old* firmware version signals rollback occurred.

### Migration for boards on single-slot factory image

Today's deployed boards run a single `app0/ota_0` image without an `ota_1` partition. After the new partition table is flashed (via USB once), the first OTA-installed image bootstraps the dual-slot contract. NVS offset stays at `0x9000`, so unit name / network ID / mag-cal data survive. No data migration code needed.

---

## 5. iOS UI

New `FirmwareUpdateView` reachable from each device's settings screen.

- **File picker**: `.fileImporter(allowedContentTypes: [.data])`. On selection, validate by extension (`.bin`) and parse first 256 bytes for an ESP32 app header magic byte (0xE9). Read entire file into memory (worst case ~3 MB — fine on iOS).
- **Pre-flash display**: file name, file size, computed SHA-256 of file, current device `firmwareVersion`. "Flash <device name>" button (disabled if not connected).
- **Flash state machine** (`OTASession` class):

  ```
  idle → uploading → verifying → rebooting → reconnecting → verified
                          │           │            │             │
                          └───────────┴────────────┴─► failed (with error)
                                                  └─► rollback_detected
  ```

- **Progress UI**: mirrors the existing `FileManagerView` linear progress bar at [FileManagerView.swift:63](../../TinkerRocketApp/Views/FileManagerView.swift:63). Show % + bytes-of-bytes + estimated remaining time.
- **Reconnect timeout**: 60 s. Beyond that, surface "Device did not reconnect — try power-cycling and re-pairing." (Recovery is via USB.)
- **Version verification**: on reconnect, read `config_identity`. If `fw` matches the expected (computed from the firmware file's git-sha header, or asked at pick-time — TBD in Phase 2), show "Updated to <version>". If it matches the pre-flash version, show "Rollback detected — flash did not take, device is on old firmware."
- **Cancel button**: maps to `OTA_ABORT` cmd 72. Returns to file-picker state.

### Targeting

Initial Phase 2 implementation gates `FirmwareUpdateView` on `device.isBaseStation`. Phase 3 removes the gate. Phase 4 adds a "target" picker (this device / Flight Computer) when the connected device is an OC.

---

## 6. Rollback safety vs. secure boot

This design does *not* enable secure boot v2 / signed images. Anyone with physical USB access can flash arbitrary firmware. That's an intentional v1 tradeoff:

- Secure boot v2 requires burning eFuses — irreversible. We're not ready to commit to a signing key custody policy yet.
- The threat model for a hobbyist rocket is "I might brick it", not "an adversary will field-flash malicious firmware".
- Adding secure boot later is a separate hardening pass with its own ergonomics work (signing tool in CI, key rotation policy, USB-flash disabled).

Tracked as a follow-up; not blocking this issue.

---

## 7. Flight Computer relay strategy (Phase 4)

The FC has no BLE radio. Reaching it requires iOS → BLE → OC → wires → FC. The OC↔FC interface is 5 GPIOs split across two buses, both already in active use:

| Wires | Bus | Today's role | Rate |
|---|---|---|---|
| FC 23 / 27 / 28 | **I2S** (DOUT / BCLK / WS) | FC → OC high-frequency telemetry stream | 22 050 sample/s × 4 B ≈ 88 KB/s |
| FC 41 / 42 | **I2C** (SDA / SCL) | OC ↔ FC command/config | 400 kHz |

There is **no hardware reset or IO0 line** between OC and FC, so the ESP32 ROM bootloader path (`esptool` SLIP over UART) is not on the table without a hardware change. Whatever we ship must be cooperative — the FC firmware itself receives the new image and writes it to `ota_1`.

### Chosen design — split-bus cooperative OTA

Use both buses, each for what it's best at, mirroring the BLE-side split:

| Plane | Bus | Direction | Carries |
|---|---|---|---|
| **Control** | I2C (41/42) | bidirectional | `OTA_BEGIN(target=1, size, sha256)`, `OTA_FINISH`, `OTA_ABORT`, and `ota_status` reply frames. Reuses the existing `WireFormat.h` SOF/type/len/payload/CRC framing — new message types added. |
| **Data** | I2S (23/27/28), reconfigured | OC → FC | Image chunks. OC becomes I2S master TX, FC becomes I2S slave RX (direction reversed from normal). BCLK cranked up well past the audio-rate default. |

**Why this rather than I2C-only**: I2C at 400 kHz tops out around 30 KB/s after framing overhead, so a 600 KB FC image would take ~20 minutes. I2S can run BCLK in the multi-MHz range; even a modest 2.5 MHz BCLK on 16-bit stereo slots gives 10 Mbps raw ≈ 1.25 MB/s effective. A 600 KB image transfers in under a second of wall time; flash erase + write dominates total OTA time (~5-10 s including the I2C handshake and SHA verify).

**Why this rather than reconfiguring I2S pins to SPI**: We'd gain nothing material — I2S with DMA already drives the same physical pins at comparable rates, and we already have a working component (`TR_I2S_Stream`) that handles channel setup, DMA, and framing. SPI would mean writing a new slave-side ring buffer driver for no throughput win.

### Operating sequence

1. iOS → BLE → OC: `OTA_BEGIN(target=1, size, sha256)` arrives.
2. OC sends `OTA_BEGIN` to FC over I2C. FC pauses sensor reads and its I2S TX stream.
3. Both sides tear down the I2S channel and re-init it with **OC = master TX, FC = slave RX**, at the higher OTA BCLK. The TR_I2S_Stream component grows a `beginMasterTx`/`beginSlaveRx` variant pair for this — most of the channel/DMA setup is already there.
4. OC streams the image as raw bytes over the I2S data line; FC's `i2s_recv_cb_t` callback hands buffers to an `OTAReceiver` instance (same component as Phases 2/3) which calls `esp_ota_write` and updates the running SHA-256.
5. OC sends `OTA_FINISH` over I2C. FC finalizes SHA, compares, calls `esp_ota_set_boot_partition(ota_1)`, replies `ready_to_boot` over I2C, then reboots.
6. After FC reboot, the I2S channel is torn down + reverted to normal direction (FC = master TX) by both sides. Sensor stream resumes.
7. The same `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` contract from §4 applies on the FC: the new image marks itself valid only after publishing its first I2S frame back to OC.

### Rollback signal

FC has no direct phone-home path. The OC observes a watchdog window after the FC reboot — if FC doesn't resume I2S TX within (say) 10 s, OC notifies the phone over BLE as `{state:"verify_failed","err":"fc_no_resume"}`. The bootloader will already have auto-rolled back to `ota_0`; the FC will then come up on its previous firmware and the I2S stream resumes shortly after. OC reports the actual `fw` it observes from the FC's next `config_identity` response (relayed up over BLE).

### Pin assignment confirmed (no schematic check needed)

The original §7 listed "schematic check" as a Phase 1 follow-up. That's resolved: pin map is confirmed from [flight_computer/main/config.h:287-296](../../tinkerrocket-idf/projects/flight_computer/main/config.h:287). No additional hardware reset lines exist; the cooperative design above is the path.

### Phase 4 dependencies / things to settle when we get there

- **Maximum I2S BCLK** achievable between OC (ESP32-S3) and FC (ESP32-P4) over the existing PCB traces — bench-measure during Phase 4 bringup, back off if signal integrity is marginal. Doc target: 2.5 MHz, allow up to 10 MHz if traces handle it.
- **FC OTA mode entry** — how does FC firmware know to drop into OTA mode? Options:
  - Reactive: OTA_BEGIN over I2C interrupts sensor loop on a dedicated low-priority task.
  - Pre-armed: phone sends an "arm OTA" command earlier, FC enters a quieter mode, then OTA_BEGIN proceeds.
  - Default to reactive unless bench testing shows I2S-driver-teardown-while-running is unsafe.
- **Failure to enter OTA mode** — what if FC is unresponsive over I2C at OTA_BEGIN time? OC times out, phone surfaces "FC didn't acknowledge — recover via USB."

These are Phase 4 specifics, not Phase 1 design decisions.

---

## 8. Verification per phase

### Phase 1 (this doc)

- Read-through with maintainer. Adjust per feedback.

### Phase 2 (BS implementation)

1. `idf.py fullclean && idf.py build` on `base_station/` — confirm binary still fits in 3 MB slot. (Per [feedback_idf_fullclean_when_suspect.md](../../../.claude/projects/-Users-christianpedersen-Documents-Hobbies-ModelRockets-Code/memory/feedback_idf_fullclean_when_suspect.md): always `fullclean` when partition table changes.)
2. `idf.py partition-table` — confirm new layout parses.
3. `idf.py flash monitor` — clean boot; BLE up; identity JSON includes `"fw"` field; SPIFFS fallback still works when SD is absent.
4. Push a known-good firmware via a small Python `bleak` script (faster iteration than iOS for protocol debugging): observe `esp_restart`, reconnect, confirm identity `fw` matches.
5. Negative tests:
   - Corrupt one chunk → `verify_failed: sha_mismatch` → no flash damage.
   - Disconnect mid-upload → next session starts cleanly.
   - Power loss mid-flash → `otadata` keeps booting `ota_0`.
6. Rollback test: install a deliberately broken image (panic in `app_main` after 5 s). One panic → automatic rollback to `ota_0`. BLE comes back on old firmware.
7. iOS end-to-end on a physical iPhone: build firmware → AirDrop bin to phone → pick in app → watch flash → reconnect → confirm new `fw` shown.

### Phase 3 (OC implementation)

Same matrix as Phase 2, on OC. Additionally: smoke-test the I2C command-forward link to FC (existing app→FC commands like servo config, sim start should be unaffected).

### Phase 4 (FC relay)

Defined when we get there. Depends on §7 outcome.

---

## 9. Out of scope (explicit)

- Secure boot v2 / signed images (separate hardening issue)
- HTTPS firmware distribution from a release server
- OTA over LoRa
- Parallel multi-device flash (one device at a time)
- A/B-test framework for trying experimental firmware (just use git branches)
