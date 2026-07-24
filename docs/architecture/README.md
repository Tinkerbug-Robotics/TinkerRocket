# Architecture

How each part of TinkerRocket actually works — the task model, the data paths, the
state machines, and the decisions that are not obvious from reading the code.

For what the system *is* and how to build it, start at the [top-level
README](../../README.md). These pages are the layer below that: one per codebase,
written to be read start to finish.

## Pages

| Page | Covers | Status |
|---|---|---|
| [Out Computer](out-computer.md) | ESP32-S3 — storage, radios, power rail, BLE | written |
| Flight Computer | ESP32-P4 — sensors, EKF, control laws, pyro | not written |
| Base Station | ESP32-S3 — LoRa receive, multi-rocket tracker, SD log | not written |
| iOS App | SwiftUI — fleet model, telemetry, config, flight logs | not written |
| Protocols | frame formats, message types, LoRa framing, BLE commands | not written |

The Out Computer page is the pilot. It sets the template — audience, depth, diagram
style, and the "Gotchas" section that carries the hard-won details — for the rest.

## The section map

The three firmware entry points are single files of 5,000–7,500 lines each. There is no
module structure to navigate by, so the source carries its own map: `// SECTION:`
banners mark each logical region, and
[`tools/gen_section_index.py`](../../tools/gen_section_index.py) turns them into a table
of line ranges with links.

Generated maps live in [`generated/`](generated/) and are **not** hand-edited.

```bash
python3 tools/gen_section_index.py
```

CI runs the same script with `--check` and fails if a committed map disagrees with its
source, so the map cannot silently go stale. Adding a section means adding a banner to
the source and re-running — nothing in the tool needs editing.

Currently bannered: Out Computer. The Flight Computer and Base Station are skipped by
the generator until they get banners.

## Conventions

- **Audience is layered.** The body of each page assumes general embedded and rocketry
  familiarity but not knowledge of this project. The "Gotchas" section at the end is
  where the expensive, non-obvious details live.
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
