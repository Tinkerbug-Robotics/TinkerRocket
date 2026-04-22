# Custom append-only NAND flight-log layer

**Issue:** [#50](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/50)
**Status:** Stage 1 complete (191 tests passing) — Stage 2 pending
**Last updated:** 2026-04-22

Replace LittleFS on the flight-logging hot path with a deterministic append-only NAND layer. Preserves the iOS app's existing BLE file-management flow (cmd 2 list / cmd 3 delete / cmd 4 download).

Root cause & context are in [#50](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/50); this doc is the implementation plan.

## Resolved design decisions

| # | Decision |
|---|---|
| 1 | Pre-allocation size: **32 MB** (256 blocks) per flight. ~5 min at ~100 KB/s. |
| 2 | Overflow: **extend** the range by 64 blocks, absorb ~128–320 ms stall, one-shot warn. |
| 3 | Deletion: **release blocks for reuse**. Persistent bitmap becomes 3-state (free / allocated / bad), 2 bits × 1024 = 256 B in NVS. |
| 4 | Legacy LFS files: **wipe on first boot** of new firmware. No union-with-LFS in cmd 2/4. |
| 5 | Metadata location: **dedicated NAND blocks 1020–1023** (dual-copy + CRC16). |

See the [resolution comment on #50](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/50#issuecomment-4299300538) for discussion.

## Stages

- **Stage 1** — new `TR_FlightLog` component + unit tests (isolated library, feature-flag gated).
- **Stage 2** — wire into flush task; legacy LFS wipe-on-first-boot; flip feature flag.
- **Stage 3** — re-back BLE cmd 2/3/4 on the new index.
- **Stage 4** — bench validation on hardware (reproduce the 15+ flight stall scenario from #47).

---

## Stage 1 — `TR_FlightLog` component scaffold + unit tests

### Goals
- Pure library with no ESP-IDF runtime dependencies in unit tests (mockable NAND interface).
- No changes to the hot path. The existing [TR_LogToFlash](../../tinkerrocket-idf/components/TR_LogToFlash) path is untouched until Stage 2.
- Gated with `CONFIG_TR_FLIGHTLOG_ENABLED` (default off).

### New files

Mirror in both `libraries/` and `tinkerrocket-idf/components/` per project convention.

```
tinkerrocket-idf/components/TR_FlightLog/
├── CMakeLists.txt
├── include/
│   ├── TR_FlightLog.h           # public API
│   ├── TR_FlightLog_types.h     # FlightIndexEntry, PageHeader, BlockState
│   └── TR_NandBackend.h         # abstract NAND interface for testability
├── TR_FlightLog.cpp             # core implementation
└── TR_NandBackend_esp.cpp       # real NAND backend (wraps existing nand* helpers)
```

### Core APIs

`TR_FlightLog.h`:

```cpp
class TR_FlightLog {
public:
    struct Config {
        uint16_t prealloc_blocks     = 256;   // 32 MB
        uint16_t extend_blocks       = 64;    // overflow step (~200 ms stall)
        uint16_t flight_region_start = 32;    // first block owned by this layer
        uint16_t flight_region_end   = 1020;
        uint16_t metadata_blocks[4]  = {1020, 1021, 1022, 1023};
    };

    esp_err_t begin(TR_NandBackend& nand, const Config& cfg);

    // Pre-launch: pick + erase a free contiguous range. May stall ~770 ms. Returns flight_id.
    esp_err_t prepareFlight(uint32_t& flight_id_out);

    // Hot path: deterministic page write. Called by flush task. No allocation, no metadata.
    esp_err_t writePage(const uint8_t* page, size_t len);

    // Post-flight: append index entry. May stall.
    esp_err_t finalizeFlight(const char* filename, uint32_t final_bytes);

    // BLE-facing read surface
    size_t    listFlights(FlightIndexEntry* out, size_t max, size_t page, size_t per_page);
    esp_err_t readFlightPage(const char* filename, uint32_t offset, uint8_t* buf, size_t len, size_t& out_len);
    esp_err_t deleteFlight(const char* filename);    // releases blocks for reuse
    esp_err_t renameFlight(const char* old_name, const char* new_name);  // timestamp apply
};
```

`TR_NandBackend.h`:

```cpp
class TR_NandBackend {
public:
    virtual esp_err_t readPage(uint32_t block, uint32_t page_in_block, uint8_t* buf) = 0;
    virtual esp_err_t programPage(uint32_t block, uint32_t page_in_block, const uint8_t* buf) = 0;
    virtual esp_err_t eraseBlock(uint32_t block) = 0;
    virtual bool      isBlockBad(uint32_t block) = 0;
    virtual esp_err_t markBlockBad(uint32_t block) = 0;
};
```

The real backend wraps the existing `nandReadPage` / `nandProgramPage` / `nandEraseBlock` / `markBlockBad` helpers from [TR_LogToFlash.cpp](../../tinkerrocket-idf/components/TR_LogToFlash/TR_LogToFlash.cpp).

### Block state bitmap (2 bits/block)

| State | Bits | Meaning |
|---|---|---|
| `FREE` | `00` | Unused, erase on claim |
| `ALLOCATED` | `01` | Owned by a flight in `flights.idx` |
| `BAD` | `10` | Skip always |
| (reserved) | `11` | — |

256 bytes total. Stored in NVS namespace `"map"` (same as today). Loaded in `begin()`. Persisted on state transitions (delete, finalize, mark-bad).

### Metadata region layout (blocks 1020–1023)

- **1020, 1021**: active + shadow for `flights.idx` (whole-block writes, CRC16 at end, monotonic sequence number in header).
- **1022, 1023**: active + shadow for block-state bitmap snapshot (NVS is authoritative; these are backup).

On read, pick whichever of {active, shadow} has the higher sequence number **and** a valid CRC. On write, target the older copy first, then update the other.

### Test strategy — host tests via [tests_cpp/](../../tests_cpp) (GoogleTest)

The existing infrastructure (GoogleTest via FetchContent, `host_shim/` for Arduino-compat) is reused. All tests run on the host, no hardware required. A hardware bench phase comes in Stage 4.

Tests are organized into **five** executables so regressions can be localized. The central primitive is `FakeNandBackend` — an in-memory 128 MB array with simulated program/erase semantics (a page bit can only go 1→0 without erase; erase resets a whole block to 0xFF; configurable bad-block injection; configurable program-fail injection; optional timing model for latency tests).

#### 1. `test_tr_flightlog_core` — functional correctness

Happy-path and edge cases for the new layer against `FakeNandBackend`.

1. `prepareFlight` picks a free range, skips bad blocks, erases it.
2. `writePage` bounds-checks, rejects out-of-range, advances sequentially.
3. `writePage` crossing a bad block skips to next good page (page-program fail path).
4. `finalizeFlight` writes a valid index entry with matching `final_bytes` + CRC.
5. Round-trip: prepare → 1000 page writes → finalize → `listFlights` → `readFlightPage` → data matches.
6. Delete releases blocks (bitmap marks FREE); next `prepareFlight` can reuse them.
7. Overflow: prepare a range, write past end, extend succeeds, stall is measured and reported.
8. Rename updates index entry, preserves blocks.
9. `listFlights` pagination order-stable, skips deleted entries.

#### 2. `test_tr_flightlog_recovery` — brownout + corruption

Every test here uses a deliberate crash/corruption injection so the recovery path is exercised.

1. Brownout mid-write: no `finalizeFlight` called. Reboot. Scanner finds pages matching `flight_id`, reconstructs partial index entry. Worst-case data loss ≤ 1 page.
2. Brownout during `finalizeFlight` (index half-written): next boot detects invalid CRC on active metadata copy, falls back to shadow.
3. Both metadata copies corrupted: layer returns a defined error, no crash, no silent data destruction.
4. Page CRC mismatch on read: `readFlightPage` returns error for that page, continues for others.
5. Power loss during `deleteFlight` partway through bitmap update: next boot sees either pre- or post-delete state, never a half-state.

#### 3. `test_tr_flightlog_wire_format` — iOS-facing compatibility guard

**This is the key regression-proofing test.** Golden-file tests pinned to the current BLE wire format so the new backend cannot silently change what iOS sees.

The encoder logic needs a small refactor (Stage 3) to separate wire-format encoding from BLE characteristic writes, so these tests can call the encoders directly. Stage 1 prepares by **capturing golden outputs** from the current implementation into `tests_cpp/fixtures/ble_wire_format/`:

- `cmd2_list_empty.bin` — JSON bytes for empty list
- `cmd2_list_3_files.bin` — JSON bytes for 3 files, page 0
- `cmd2_list_pagination.bin` — JSON for page 1 of a 20-file list
- `cmd4_chunk_first.bin` — first download chunk (offset=0, flags=0)
- `cmd4_chunk_mid.bin` — middle chunk
- `cmd4_chunk_last.bin` — final chunk (EOF flag set)
- `cmd3_delete_request.bin` — request-side parsing test

Tests assert the new encoders produce **byte-identical** output for equivalent inputs. Golden files are checked in.

#### 4. `test_tr_flightlog_property` — fuzz / invariants

Randomized stress tests that run many cycles to shake out state-machine bugs.

1. **Bitmap round-trip**: randomly generate block-state maps, serialize → NVS → deserialize. Bit-exact. 10,000 iterations.
2. **Index entry round-trip**: randomly generate entries, write → read → compare. 10,000 iterations.
3. **Allocator stress**: random sequence of prepare/write/finalize/delete, assert no overlap between active flights, no free block marked allocated. 1,000 cycles.
4. **Partial write fuzz**: at random points in a flight, truncate FakeNandBackend writes; assert scanner recovers consistent state. 500 cycles.
5. **CRC collision sanity**: 1,000 random page payloads with deliberate single-bit flips → CRC catches every one.

#### 5. `test_tr_flightlog_timing` — determinism guard

Uses `FakeNandBackend` with a simulated timing model (page-program ≈ 300 µs, erase ≈ 3 ms, SPI overhead modeled). Validates the performance guarantees of the design.

1. `writePage` under steady state completes in < 1 ms (simulated). 10,000 iterations, histogram asserted.
2. `writePage` across a bad-block skip completes in < 2 ms.
3. `prepareFlight` (256-block erase) completes in < 1 s.
4. Overflow extend (64-block erase) completes in < 400 ms.
5. `writePage` call-rate throughput sustains ≥ 200 KB/s (double the observed ~100 KB/s write rate — headroom for bursts).

These are *simulated* timings, not hardware measurements — they catch algorithmic regressions (e.g. someone adds an accidental O(n) metadata scan to the hot path) before they reach hardware. Stage 4 does real hardware measurements.

### CI wiring

Existing [`.github/workflows/cpp-tests.yml`](../../.github/workflows/cpp-tests.yml) runs `cmake -S tests_cpp -B tests_cpp/build && cmake --build tests_cpp/build && ctest --test-dir tests_cpp/build --output-on-failure` on every push/PR that touches `tinkerrocket-idf/components/**` or `tests_cpp/**`. New targets added to [tests_cpp/CMakeLists.txt](../../tests_cpp/CMakeLists.txt) (following the `test_ekf` pattern) are picked up automatically via `gtest_discover_tests`. No workflow changes needed.

### Build wiring

- `TR_FlightLog` added to both component dirs but not yet referenced by [flight_computer CMakeLists](../../tinkerrocket-idf/projects/flight_computer/main/CMakeLists.txt). Gate with `CONFIG_TR_FLIGHTLOG_ENABLED` (default off).
- [tests_cpp/CMakeLists.txt](../../tests_cpp/CMakeLists.txt) gains a new target `test_tr_flightlog`.

### Out of Stage 1 scope

- Wiring into the flush task → Stage 2.
- BLE cmd 2/3/4 re-backing → Stage 3.
- Bench validation on hardware → Stage 4.
- Legacy LFS wipe-on-first-boot → Stage 2 when the feature flag flips.

### Order of work

Tests land **with** each piece of functionality (not at the end). Each step below includes its tests.

1. `TR_NandBackend.h` interface + `FakeNandBackend` skeleton (used by every later test).
2. Block-state bitmap (load/save/query/update) + tests in `test_tr_flightlog_core` and `test_tr_flightlog_property`.
3. `FlightIndexEntry` + metadata dual-copy read/write + tests in `test_tr_flightlog_core`, `test_tr_flightlog_recovery`, and `test_tr_flightlog_property`.
4. `prepareFlight` (range pick + erase) + tests in `test_tr_flightlog_core` and `test_tr_flightlog_timing`.
5. `writePage` hot path + bad-block skip + tests in `test_tr_flightlog_core` and `test_tr_flightlog_timing`.
6. `finalizeFlight` + `listFlights` + `readFlightPage` + tests in `test_tr_flightlog_core`.
7. `deleteFlight` + `renameFlight` + tests in `test_tr_flightlog_core` and `test_tr_flightlog_property`.
8. Brownout scanner + tests in `test_tr_flightlog_recovery`.
9. Golden BLE wire-format fixtures captured from current implementation; `test_tr_flightlog_wire_format` stub in place (full assertions activate in Stage 3 once encoders are extracted).
10. Real `TR_NandBackend_esp.cpp` wrapper (no hardware tests yet — just compiles).

**Estimate:** 1.5–2 days (bumped from 1–1.5 to cover the expanded test suite).

---

## Stage 2 — wire into flush task (TBD)

Flip `CONFIG_TR_FLIGHTLOG_ENABLED`. Replace [TR_LogToFlash.cpp](../../tinkerrocket-idf/components/TR_LogToFlash/TR_LogToFlash.cpp) flush-task write with `TR_FlightLog::writePage`. Add first-boot LFS wipe. Pre-launch hook calls `prepareFlight`. Post-flight hook calls `finalizeFlight`.

## Stage 3 — BLE handler re-backing (TBD)

[TR_BLE_To_APP.cpp](../../tinkerrocket-idf/components/TR_BLE_To_APP/TR_BLE_To_APP.cpp) cmd 2 / cmd 3 / cmd 4 handlers switch their backing source to the new index. Wire format unchanged.

## Stage 4 — bench validation (TBD)

Reproduce the 15+ flight stall scenario from [#47](https://github.com/Tinkerbug-Robotics/TinkerRocket/issues/47). Expect zero stalls >40 ms, zero dropped frames regardless of chip fill.
