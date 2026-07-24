# Architecture

How each part of TinkerRocket actually works — the task model, the data paths, the
state machines, and the decisions that are not obvious from reading the code.

For what the system *is* and how to build it, start at the [top-level
README](../../README.md). These pages are the layer below that: one per codebase,
written to be read start to finish.

## Pages

| Page | Covers | Status |
|---|---|---|
| [Flight Computer](flight-computer.md) | ESP32-P4 — sensors, EKF, control laws, pyro | written |
| [Out Computer](out-computer.md) | ESP32-S3 — storage, radios, power rail, BLE | written |
| [Base Station](base-station.md) | ESP32-S3 — LoRa receive, multi-rocket tracker, CSV log | written |
| [iOS App](ios-app.md) | SwiftUI — fleet model, telemetry, config, flight logs | written |
| Protocols | frame formats, message types, LoRa framing, BLE commands | not written |

The Out Computer page was the pilot. It sets the template — audience, depth, diagram
style, and the "Gotchas" section that carries the hard-won details — for the rest.

## The generated maps

The three firmware entry points are single files of 5,000–7,500 lines each. There is no
module structure to navigate by, so the source carries its own map: `// SECTION:`
banners mark each logical region, and
[`tools/gen_section_index.py`](../../tools/gen_section_index.py) turns them into a table
of line ranges with links.

The iOS app is the opposite shape — 65 files that already carry Swift's native
`// MARK: -` markers. The same tool reads those, plus each file's type declarations, and
emits a module map. No Swift source was modified to make that work; adding a second
banner convention on top of a convention Xcode already understands would have been noise.

Generated maps live in [`generated/`](generated/) and are **not** hand-edited.

```bash
python3 tools/gen_section_index.py
```

CI runs the same script with `--check` and fails if a committed map disagrees with its
source, so the map cannot silently go stale. Adding a section means adding a banner to
the source and re-running — nothing in the tool needs editing.

All three firmwares are bannered. The generator skips any source with no banners yet, so
adding a fourth is one entry in `TARGETS` (C++, banner-based) or `SWIFT_TARGETS`
(directory tree, MARK-based).

## Conventions

- **Audience is layered.** The body of each page assumes general embedded and rocketry
  familiarity but not knowledge of this project. The "Gotchas" section at the end is
  where the expensive, non-obvious details live.
- **Write "Base Station" out in full.** Never abbreviate it — the two-letter form means
  something else to every reader. "FC" and "OC" are fine once introduced; the code uses
  the short form freely and that is left alone.
- **Diagrams are Mermaid**, rendered natively by GitHub. No build step, no image
  assets to keep in sync.
- **No line-number links in prose.** They rot. Prose links to a file or to a named
  section; the generated map is the only place line numbers appear, and it is
  regenerated.
- **Issue numbers are kept.** `#317`, `#524`, and friends are load-bearing — they are
  how you find the bench session that produced a given rule.

## Related

- [`docs/plans/`](../plans/) — design plans written *before* the work. Intent and
  sequence, not kept in sync with the final implementation. These pages describe what
  the code does now; plans describe what someone meant to do.
