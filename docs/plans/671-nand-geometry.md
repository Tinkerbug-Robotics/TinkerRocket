# #671 — Runtime NAND geometry from RDID

**Problem:** `TR_LogToFlash` and `TR_FlightLog` hardcode the bench chip's geometry
(F35SQB004G: 4096 B pages × 64 × 2048 blocks = 512 MB) in two deliberately
duplicated constant sets. The fabbed V9 places a GD5F2GQ5UE (2048 B pages,
2048 blocks = 256 MB) and the mini a GD5F1GQ5UE (2048 B pages, 1024 blocks =
128 MB). On those parts every 4096-byte page program overruns a 2048+128 B page
buffer and block addressing is 2× off — flight logging is broken as shipped.
Third member of the wrong-by-configuration class (#492, #822).

## Part table (each ID cross-validated in two independent drivers)

| Part | Board | MID | DID | Page | Pages/blk | Blocks | Source |
|---|---|---|---|---|---|---|---|
| F35SQB004G | V8 bench | 0xCD | 0x53 | 4096 | 64 | 2048 | vendored `nand_foresee.c` (validated on the bench part, #492); Foresee datasheet |
| GD5F2GQ5UE | V9/V10 | 0xC8 | 0x52 | 2048 | 64 | 2048 | Linux `gigadevice.c` (`NAND_MEMORG(1,2048,128,64,2048,40,1,1,1)`); vendored `nand_gigadevice.c` |
| GD5F1GQ5UE | mini | 0xC8 | 0x51 | 2048 | 64 | 1024 | Linux `gigadevice.c` (`…,64,1024,20,1,1,1`); vendored `nand_gigadevice.c` |

All three are **single-plane** (no plane-select bit in the column address), all
64 pages/block, so the command sequences are identical — only page size and
block count vary. The GD Q5 READID stream has a dummy byte before MID/DID,
which `nandReadId()` already accounts for.

## Design

1. **`NandGeometry` struct + ID table live in `TR_LogToFlash`** (the raw-SPI
   driver that owns RDID). Resolved in `nandInit()` immediately after the
   existing RDID read:
   - table hit → that part's geometry, logged at INFO with the part name;
   - unknown ID → **legacy fallback** (4096/64/2048 — i.e. today's behaviour,
     so an unlisted future 4 Gbit part or an odd bench chip keeps working) +
     ERROR log naming the ID so it gets added to the table;
   - dead bus → legacy fallback, existing dead-bus handling unchanged.
2. **Compile-time constants become compile-time MAXIMA** (`NAND_PAGE_SIZE_MAX
   = 4096`, `NAND_BLOCK_COUNT_MAX = 2048`) used only to size static buffers
   (page_buf, bad-block bitmaps). All arithmetic moves to runtime fields
   (`page_size_`, `pages_per_block_`, `block_size_`, `block_count_`).
3. **Geometry flows to `TR_FlightLog` through `TR_NandBackend`** (new virtual
   accessors: `pageSize()`, `pagesPerBlock()`, `blockCount()`). The duplicate
   constant set in `TR_FlightLog_types.h` becomes maxima; `Config`'s
   region/metadata defaults become sentinels derived at `begin()` from the
   backend (`flight_region_end = blocks − 4`, metadata = last 4 blocks) unless
   the caller overrode them.
4. **The write-sink payload becomes `page_size − 16`** (4080 → 2032 on GD
   parts) end to end: ring drain slicing, `PageHeader` CRC span, `writeFrame`,
   `readFlightPage`, `readFileChunk`.
5. **Factory-bad-marker probe column = `page_size`** (first spare byte), not a
   hardcoded 4096.
6. **V8 is behaviourally frozen**: DID 0xCD/0x53 resolves to exactly the old
   constants, and the unknown-ID fallback equals them too.

## Out of scope

- The IDF `spi_nand_flash` component (base station only; per #492 it is off
  the rocket flight path and already understands all three parts).
- Any on-chip data migration: geometry follows the chip, and no chip ever
  changes geometry in place.

## Verification

- Host tests parameterized over both geometries (fake backend advertises
  4096×2048 and 2048×1024/2048 variants); ID-table unit tests incl. fallback.
- OC V8/V9 + mini + FC + BS builds.
- Bench: RDID line on the V8 bench board must log `F35SQB004G` and behave
  byte-identically; V9 first article per #847.

## Implementation record (2026-08-20)

Landed as designed, with these decisions made during implementation:

- **Retired constant names**: `NAND_PAGE_SIZE`/`NAND_PAGES_PER_BLK`/
  `NAND_BLOCK_SIZE`/`NAND_BLOCK_COUNT` no longer exist in either component —
  only `*_MAX` buffer-sizing constants and the runtime `NandGeometry` — so all
  139 use sites the survey mapped became compile errors and were consciously
  converted. Nothing could be missed silently.
- **Deferred NVS blob check**: `loadBadBlocksFromNVS()` runs before RDID, so
  it now records the stored blob length and `nandInit()` finalizes
  `bad_block_map_blob_ok_` once the expected per-chip size is known.
- **Sync cadence became byte-denominated** (`SYNC_INTERVAL_BYTES = 256 KB`):
  64 pages on the legacy part (byte-identical) and 128 on the GD5F parts,
  preserving the tuned metadata-commit batching per byte logged.
- **`drainMramToSink` chunk** is the shared runtime `sinkPayloadSize()` and
  chunks via `page_buf` — the 4080-byte constexpr stack array is gone.
- **Config sentinels**: `flight_region_end = 0` / all-zero `metadata_blocks`
  derive from the chip at `begin()`; both firmwares fill them explicitly from
  `logger.nandGeometry()` because the auto-evict target and bitmap-store bind
  need the numbers pre-begin.
- **Factory-marker column** is the runtime page size (was hardcoded 4096,
  which on 2 KB parts aliased to main-array byte 0 — factory-bad blocks would
  have entered service).
- **Host tests run the full suite at all three geometries** via
  `TR_TEST_NAND_GEOMETRY` (ctest registers legacy/gd2g/gd1g); the fake
  backend takes geometry in its constructor; `test_nand_geometry.cpp` pins
  the ID table, the fallback, and dead-bus behaviour.
- **0xCC storage stats** now put the runtime `block_size_kb` on the wire
  (the struct was already self-describing; the app scales by the field).

Sharp edge accepted and documented in code: a V9/mini chip that was ever
formatted or written under the old wrong geometry will fail CRC/mount checks
once and re-format/rescan — expected one-time behaviour on first corrected
boot, impossible on V8 (byte-identical path).

## Adversarial review (2026-08-20, 28-agent workflow)

Six lenses (SPI/addressing, LFS/flush, FlightLog, call sites, V8 byte-identity
hunk audit, test quality) + two-skeptic verification per finding. 11 findings,
4 confirmed, 7 refuted. All four confirmed are fixed in this change:

1. **[major] Trusted-map gate blind to scan-time geometry** — a V9 chip once
   booted with pre-#671 firmware persists a same-length (256 B) map scanned
   with the factory-marker column aliasing into the main array; the length
   check alone would trust it forever. Fixed with an NVS `gpage` stamp
   (scan-time page size, missing = 4096) folded into the gate: V8 maps stay
   trusted with no spurious rescan, a poisoned GD5F map forces the one healing
   rescan.
2. **[minor] metadata blocks could overlap the flight region** on mixed
   explicit/sentinel configs — overlap gate added + test.
3. **[minor] Absent map blob + matching chip id → perpetual boot rescan**
   (a zero-bad-block scan never dirtied, so the blob was never written) —
   `!blob_ok` now always dirties so the post-scan persist writes the blob.
4. **[minor] gd1g test variant silently exercised the fresh-seed path**
   instead of restore (MAX-length seed blob) — seeds runtime length now.

The geometry matrix separately caught `persistBitmap()` saving the MAX length
(broke reboot restore + brownout recovery on 1024-block chips only — invisible
at both 2048-block geometries).
