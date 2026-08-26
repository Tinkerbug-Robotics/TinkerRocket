# Contributing to TinkerRocket

Contributions are welcome — firmware, the iOS app, the simulator, hardware, or docs.
This page covers what you need to know before your first pull request.

## Before your first PR: the CLA

TinkerRocket asks contributors to agree to a [Contributor License Agreement](CLA.md)
once. You keep the copyright in your work; you grant us a licence to use it, including
the right to relicense the project later.

To sign, comment on your first pull request with:

```
I have read the TinkerRocket CLA and I agree to it.
Signed-off-by: Your Name <your.email@example.com>
```

One signature covers everything you contribute afterwards. If you would rather not
sign, you are still very welcome to file issues, report flight anomalies, and discuss
designs — that is genuinely useful and needs no agreement.

## Getting set up

| What | You need |
|------|----------|
| Firmware | [ESP-IDF v6.0](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/) — **every** project needs v6; CI builds on `espressif/idf:v6.0.1` |
| iOS app | Xcode 16+ |
| Simulator | Python 3.10+ |
| Host tests | CMake 3.16+ |
| Hardware | KiCad 10.0.3 |

A plain, non-recursive clone is enough. The proportional-navigation guidance law is a
private submodule; everything else builds and runs without it, against a header-only
no-op stub. If you see `PN Guidance: NOT COMPILED IN (stub active)` at boot, that is
expected and not a problem.

## Making a change

Branch, commit, open a PR — the same for hardware as for firmware.

```bash
git switch -c my-change
# ... work ...
git commit
```

Write commit messages that explain **why**, not just what. The existing history is a
reasonable guide: most of the value in this repo's messages is the reasoning that would
otherwise be lost.

## What CI checks

The workflows you can run locally, each path-filtered to what it covers — except `docs.yml`,
which runs on everything. (The full set is larger; the README's CI table lists all of them.)

| Workflow | Runs on |
|----------|---------|
| `cpp-tests.yml` | changes to components, `tests_cpp/`, integration tests |
| `firmware-build.yml` | full ESP-IDF build of all six firmware projects, once per board revision |
| `sim-tests.yml` | the simulator and the sources it binds to |
| `unit-tests.yml` | `tests/unit/` — Python ports of firmware logic, plus the base-station log reader |
| `ios-tests.yml` | the iOS app |
| `flight-report-tests.yml` | flight-report tooling |
| `wire-codes.yml` | duplicate BLE command numbers |
| `docs.yml` | **every push and PR** — generated docs and prose consistency |

Run the equivalents locally before pushing:

```bash
cmake -S tests_cpp -B tests_cpp/build -DCMAKE_BUILD_TYPE=Debug
cmake --build tests_cpp/build -j$(nproc)
ctest --test-dir tests_cpp/build --output-on-failure
```

```bash
python3 -m pytest tests/unit/ tests/test_roll_profile_semantics.py
```

```bash
python3 tools/check_docs.py
python3 tools/gen_section_index.py --check
python3 tools/gen_protocol_reference.py --check
```

## Things that will trip you up

These are real traps that have each cost someone a debugging session.

### Adding a BLE command or message type

The dispatch is a flat first-match `if (ble_cmd == N)` chain. Assign a number twice and
the **second handler becomes dead code** — no compiler error, no failing test. That
shipped once already.

1. Pick an unused number *in that device's space* — check the
   [protocol reference](docs/architecture/generated/protocol-reference.md), not memory.
   The Out Computer and Base Station spaces are independent and may overlap.
2. Put a comment on the branch. It becomes the description in the generated reference.
3. For a new FC↔OC message type, add it to the registry in
   `tests_cpp/test_rocket_computer_types.cpp` and bump the count — a code missing from
   that registry is one nothing checks for collisions.
4. If it carries a struct, add a `static_assert` on its size.
5. Regenerate: `python3 tools/gen_protocol_reference.py`

Note the gap no guard closes: **two branches can each take the same free number**, pass
CI independently, and collide only at merge. If someone else is adding commands at the
same time, agree on numbers first.

### Editing documentation

Some docs are generated and must not be hand-edited — anything under
`docs/architecture/generated/`. Change the source, then re-run the generator. CI fails if
a committed file disagrees with its source.

Prose is checked too: links must resolve, the ESP-IDF version must match CI, the workflow
list must be complete, and struct sizes quoted in the README must match the header.

### Editing KiCad files

**Never let git three-way-merge a board file.** `.kicad_pcb` and `.kicad_sch` are text, so
git will merge them happily and produce a corrupt or subtly wrong board. See
[`hardware/README.md`](hardware/README.md), which covers this and three other
hardware-specific rules in detail.

### Building the flight computer

There is no default board revision, and the build fails without one:

```bash
idf.py -B build_v9 -DTR_BOARD_V9=1 build
```

`TR_BOARD_V9=1` is the board in `hardware/rocket-computer/` (its title block reads V9, V10
at HEAD). `TR_BOARD_V8=1` is the older bench boards, whose PCB was never committed;
`TR_BOARD_V7=1` is the legacy board.

The flag is mandatory here — and only here — because the failure mode is pyrotechnic and
silent. V8 and V9 disagree on the ARM pin (5 vs 16) and swap the FIRE pins of channels 2
and 3, while keeping all four continuity pins identical. A V9 board flashed with the V8
map boots clean, reports the correct channel armed and continuous, then fires the *other*
channel's connector — or, since ARM is never driven, fires nothing at all through a 1 kΩ
bleed that still lights a test LED. The boot log prints the map it was built with
(`[BOARD] pin map:` / `[BOARD] pyro:`); check it against the board before arming anything.

The out computer takes `-DTR_BOARD_V9=1` too, though on that MCU it selects the same pins
as V8 — pass it anyway so both halves of a pair are built with one flag.

### Building the base station

The build flag does not match the board number. Current hardware is **V6**, built as
`TR_BS_BOARD=3` — see [base-station.md](docs/architecture/base-station.md) for the
flag-to-silkscreen mapping, which is the one place that tracks it:

```bash
idf.py -B build_v3 -DTR_BS_BOARD=3 build
```

A plain `build/` defaults to board 2 — a superseded revision. Flashing it onto a V6
hard-hangs at boot with `LoRa init FAILED!`, before BLE init and before the main
loop (#837 item 15).

### `sdkconfig` overrides `sdkconfig.defaults`

`sdkconfig` is generated and untracked, and it wins. Editing the defaults file does
nothing while a stale `sdkconfig` sits beside it, and a symbol that no longer exists fails
silently. When a config change appears to have no effect, delete `sdkconfig` and rebuild.

## Safety-critical areas

TinkerRocket fires pyrotechnic devices. Changes touching pyro, deployment, recovery, or
flight-state logic get more scrutiny and may be asked for bench evidence before merge.
That is not distrust — a mistake in those paths cannot be recovered in flight.

If you are changing one of these, say what you tested and how in the PR description.

## Understanding the system

Each codebase has an architecture page covering its task model, data paths, state
machines, and the decisions that are not visible from reading the code. Each ends with a
**Gotchas** section of things that have cost bench time.

- [Flight Computer](docs/architecture/flight-computer.md)
- [Out Computer](docs/architecture/out-computer.md)
- [Base Station](docs/architecture/base-station.md)
- [iOS App](docs/architecture/ios-app.md)
- [Protocols](docs/architecture/protocols.md)

Start with the [index](docs/architecture/README.md).

## Licensing

Software contributions are licensed under GPL-3.0-or-later, hardware contributions under
CERN-OHL-S v2. See [LICENSE](LICENSE), [hardware/LICENSE](hardware/LICENSE), and the
License section of the [README](README.md#license).
