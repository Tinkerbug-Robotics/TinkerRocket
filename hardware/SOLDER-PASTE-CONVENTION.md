# Solder paste and stencil — the convention

Two rules, both decided 2026-09-09 against measured evidence rather than
general guidance. Every board's `FABRICATION-NOTES.md` points here rather than
restating them.

- **#959** — stencil thickness, and the area-ratio floor.
- **#906** — how much of a pad gets paste.

---

## 1. Stencil thickness: size to the table, not to the part

**100 µm (4 mil), flat, laser cut, electropolished and nano-coated**, unless a
board states otherwise. Two boards do, and both say why in their own notes:

| board | foil | why |
|---|---|---|
| `rocket-computer-mini` | **80 µm** | its 24-ball WLCSP flash is AR 0.64 at 100 µm |
| `gnss-sam10m8-18mm-hv` | **120 µm** | u-blox requirement (UBX-22020019 R02 §4.4.1), not an area-ratio limit |

**Area ratio ≥ 0.66** (IPC-7525) at the ordered thickness:

```
AR = aperture area / (aperture perimeter × foil thickness)
```

Computed from the **aperture**, never from the pad. Apertures carry KiCad's
`solder_paste_margin` and `solder_paste_margin_ratio`, and some are drawn as
polygons on the paste layer rather than derived from pads — a pad-only scan
misses those, which is exactly what made two rows of the #959 analysis 13 µm
optimistic until they were re-scanned. There is deliberately **no automated
checker**: a partial one would give false confidence on the case that has
already bitten.

---

## 2. Coverage: full pad by default. Windowpane only for a named reason.

**The default is 100 % — the pad as drawn.** Reduce it only when you can name
the mechanism you are reducing it for:

- a **large, square-ish pad with a long flux-escape path**, where trapped
  volatiles have nowhere to go and would void the joint. Long and narrow does
  not qualify: a 1 mm-wide strip already has a short escape path across its
  width.
- a **real bridging risk** — pads close enough that the extra volume would
  short them. On 2.2 mm pitch with 1.2 mm gaps, it is not a risk to trade
  against.

### Why this is not the usual "windowpane anything over 2 mm²"

That rule was proposed on #906 and the evidence went the other way.

**Coverage sets standoff.** Height ≈ coverage × foil thickness × ~0.5 metal
fraction. Taking the LC86G's LGA pads to the SAM-M10Q's 62 % would cut standoff
from ~50 µm to ~31 µm — **less** forgiving of module warpage, on precisely the
heavy modules whose joints keep failing. Reducing paste there moves in the
wrong direction.

**And the part that actually failed had correct paste.** The SAM-M10Q cold
joints on the V9 first article happened on a footprint already at a textbook
2 × 2 windowpane, 61.9 % coverage, AR 1.34. #906's own body says it: *"the
aperture sizing on this board is not the bug."* The follow-up analysis ranked
**thermal mass** first and the solid ground tie a weak lever, because in a
profiled oven the plane is being heated too — the failure is joints under a
dense ceramic patch reaching liquidus late, which is a soak and
time-above-liquidus problem, not an aperture problem.

So: **do not spend standoff to fix a thermal problem.** If a heavy module is
cold, profile to the joint before touching the artwork.

### Consequences already accepted (#906)

- **Contact bottom-side preheat is off the table** on `rocket-computer-mini`:
  the shadow under U5 stays populated (38 parts including the radio module), so
  the 357 mm² clear window is not available. Oven convection from below is
  unaffected. Heat is bought back through the stack instead — one via per
  ground pad takes the vertical path to ~3.9× bare board against 2.2× as drawn.
- **The design targets the oven case only.** Rework cannot be assumed. Two
  things follow: filled-and-capped via protection stops being a cost adder and
  becomes a **build gate**, because an unplugged barrel under an LGA pad drains
  a joint nobody can see or fix; and first articles are a **qualification run**,
  with the mechanism confirmed off witness pads, X-ray or cross-section before
  committing to a batch. The SAM-M10Q was recovered by reworking U1. The mini's
  U5 cannot be.

---

## What is still open

Asking the assembler what foil went on the 2026-08-09 build. It is the only way
to attach an absolute solder volume to the #906 cold joints, and it is a
question for a person rather than a repo change.
