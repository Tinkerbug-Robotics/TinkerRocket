# Handoff: out_computer board_v9.h — remove the phantom MRAM

**Issue:** #822 (and its twin #826). Tracker: #819.
**Repo:** `Tinkerbug-Robotics/TinkerRocket`, branch off `main`.
**Status going in:** nothing started. This is a fresh task.

---

## The task in one line

Split a real `out_computer/main/board/board_v9.h` out of `board_v8.h` with
`MRAM_CS = -1`, so V9/V10 hardware stops addressing an MRAM that was deleted from
the board — **without changing behaviour on the physical V8 bench boards.**

---

## Why this exists

The out computer has only `board_v7.h` and `board_v8.h`.
`out_computer/main/config.h:20-31` aliases the V9 flag to the V8 header:

```c
// TR_BOARD_V9=1 selects the SAME board_v8.h. Unlike the FC — where V9 moved
// three pyro pins and needed its own header — the S3's pin map is unchanged
// from V8 through V10 ... If a future revision does move an S3 pin, split
// board_v9.h out here and switch this branch to it.
#if TR_BOARD_V8 || TR_BOARD_V9
#include "board/board_v8.h"
```

That reasoning is **pin-correct but presence-blind**, and the pin half was
re-verified during the 2026-08-18 review: every S3 GPIO is identical V8 → V10.
But MRAM is not a moved pin, it is a **deleted part**, and an alias cannot express
"same pins, one fewer device."

`board_v8.h:49` therefore applies to V9/V10 too:

```c
static constexpr int MRAM_CS = 34;       // MRAM_CS    (CONFIRMED; -1 if unfitted)
```

`-1 if unfitted` is exactly the V9/V10 case, and nothing selects it.

### The hardware fact

MRAM (U12, MR25H10) was **deliberately deleted** during the V9 design, replaced by
the ESP32-S3RH2's in-package PSRAM. Recorded in
`hardware/rocket-computer/prefab-review-2026-07-30.md:1202`:

> *I18. MRAM deletion fully implemented: no U12/MRAM in schematic, BOM, or PCB*

Confirmed independently from the netlist: S3 (U15) **GPIO34 is
`unconnected-(U15-GPIO34-Pad39)`**, and `net 'MRAM'` returns nothing. Also
confirmed by `rocket-computer/prefab-review-2026-08-05.md:1490`.

### What goes wrong today

`TR_LogToFlash.cpp:45` enables the MRAM ring on the CS pin alone, with **no device
probe**:

```c
if (cfg.mram_cs >= 0) { use_mram_ = true; ring_size_ = cfg.mram_size; ... }
```

So on V9/V10 the 128 KB flight-log ring and the in-flight-reboot snapshot are both
backed by an unconnected pad. The code even anticipates it at
`TR_LogToFlash.cpp:74-79`: *"the RAM-ring branch has never been flown — but it
becomes live the moment a board ships without the MRAM part."*

---

## Hard constraint

**Isolate the change to V9/V10.** The physical bench boards are **V8 and DO have
MRAM** (`bench-board-flash-flags`: FC `30:ED:A0:E3:6A:5B`, OC
`E0:72:A1:CA:E4:24`, both built `-DTR_BOARD_V8=1`). `board_v8.h` must keep
`MRAM_CS = 34` and must not change behaviour.

**Corollary the plan must account for: this change cannot be bench-validated on
the hardware we have.** Setting `MRAM_CS = -1` only takes effect under
`-DTR_BOARD_V9=1`, and flashing that onto the V8 bench OC would be testing the
wrong map. Options are (a) build-only verification plus host tests, (b) a
temporary local edit to `board_v8.h` to exercise the RAM-ring path on the bench
without committing it, or (c) wait for V9/V10 hardware. Decide deliberately and
say which in the PR.

---

## The two sharp edges — resolve these before writing the header

### 1. `-1` activates a code path that has never flown

Today: `use_mram_ = true`, `ring_size_ = 131072` (writes go nowhere).
After: `use_mram_ = false`, `ring_size_ = cfg.ring_buffer_size` = **65536**
(`TR_LogToFlash.h:19`), heap-allocated from internal RAM.

Questions to answer:
- Does 64 KB of internal RAM allocate reliably on the OC at that point in boot?
  A failed `heap_caps_malloc` makes `begin()` return false — what does the caller
  do, and does the operator find out?
- The ring is the **pre-launch buffer**. What does halving 128 KB → 64 KB cost in
  seconds of pre-launch history at the current logging rate? `prelaunchCap()` and
  `ring_prelaunch_cap_` are the relevant machinery.
- `main.cpp:5195` mentions *"64 KB when the RAM-only fallback ring is in use even
  with MRAM wired"* — read that comment in full, it may already answer this.

### 2. Reboot recovery and the #274 dirty marker have nowhere to live

Both are MRAM-region features:
- Snapshot: `SNAPSHOT_REGION_BASE = MRAM_SIZE - 1024` (`config.h:57-58`), written
  at `main.cpp:2869` via `mramRawWrite`, read back at `main.cpp:2883`.
- Dirty marker: `MRAM_DIRTY_MARKER_ADDR = MRAM_SIZE - 16` (`config.h:63`), wired at
  `main.cpp:5637`, consumed at `main.cpp:5702` via `hasPendingMramRecovery()`.

`mramRawWrite`/`mramRawRead` both **return false immediately when `use_mram_` is
false** (`TR_LogToFlash.cpp:846`, `:874`). So with `-1`:
- Snapshot writes become no-ops that *report failure* instead of silently
  succeeding into the void — arguably better, but check nothing treats the
  `false` as an error worth logging at 10 Hz through the whole flight.
- The snapshot read returns false, so the FC's `GET_FLIGHT_SNAPSHOT` gets **no
  response at all** rather than garbage. Check the FC's handling: `main.cpp:2877-2890`
  says the FC's CRC+magic check rejects bad data, but "no reply" is a different
  path — see the FC's `masterRead` timeout behaviour.
- **Do not silently convert one silent failure into another.** In-flight reboot
  recovery is already non-functional on V9/V10; this change should make that
  *visible* (a clear boot log line, and ideally the `SH_*` storage health bit),
  not merely differently-broken. See #364, which tracks recovery being single-shot.

---

## Also worth folding in

- **`GPS_PWR_PIN = -1` carries a stale TODO** in `board_v8.h:38`
  (*"TODO: confirm whether V8 has one"*). On V9/V10 the GNSS rail is switched by
  the **P4** (`GPS_ACT`, FC GPIO15 → U27), not the S3, so `-1` is correct and the
  TODO can be closed in the new header with that reasoning.
- **`main.cpp:7288` drives `MRAM_CS` low on power-down** as part of the back-feed
  loop. With `-1` that must not index a negative GPIO — check the guard.
  Related: #822's sibling finding that the same loop drives the shared NAND SPI
  bus as outputs on a still-powered NAND (in the #834 medium roundup).
- **A device probe is the real fix.** `MRAM_CS = -1` fixes V9/V10 by
  configuration; a JEDEC-ID/status probe in `TR_LogToFlash::begin()` would make a
  wrong header fail loudly on *any* board. Consider whether that belongs here or
  as a follow-up — it is the thing that would have caught this class of bug
  without anyone noticing the board file.

---

## CI gap to close in the same PR

`.github/workflows/firmware-build.yml:27` builds `out_computer` with **no board
flag**, so CI compiles the V7 map — a revision nobody has. The FC already builds
all three maps explicitly because a wrong pyro map is silent. The OC deserves the
same now that its maps genuinely differ:

```yaml
- project: out_computer
  board: V8
  flags: -B build_v8 -DTR_BOARD_V8=1
- project: out_computer
  board: V9
  flags: -B build_v9 -DTR_BOARD_V9=1
```

Without this, a broken `board_v9.h` would never be compiled by CI at all.

---

## Verification checklist

- [ ] `idf.py -B build_v8 -DTR_BOARD_V8=1 build` — unchanged behaviour, MRAM still 34
- [ ] `idf.py -B build_v9 -DTR_BOARD_V9=1 build` — new header, MRAM_CS = -1
- [ ] Confirm at runtime/inspection that `use_mram_` is false under V9 and the RAM
      ring allocates
- [ ] `cmake --build tests_cpp/build && ctest` — 749/749 at time of writing
- [ ] `python3 tools/check_dispatch_admits.py`, `check_ble_command_ids.py`
- [ ] **Regenerate docs BEFORE committing** — `gen_section_index.py` embeds line
      numbers and goes stale on any `main.cpp` edit, including a comment-only one.
      This failed CI three times on 2026-08-19. Also `gen_protocol_reference.py`
      and `check_docs.py`.
- [ ] Guidance submodule: `git submodule update --init --checkout
      tinkerrocket-idf/components/TR_GuidancePN` in a fresh worktree, or the FC
      silently builds against the stub.
- [ ] ESP-IDF v6.0 (`~/esp/esp-idf-v6.0`), not the 5.3.2 at `~/esp/esp-idf`.

## Definition of done

`board_v9.h` exists, V8 is byte-for-byte unchanged in behaviour, CI builds both OC
maps, and the PR states plainly which of the two sharp edges above were resolved
versus deferred — especially whether reboot recovery on V9/V10 is now *visibly*
unavailable rather than quietly so.
