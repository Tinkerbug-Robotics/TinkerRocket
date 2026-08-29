#!/usr/bin/env python3
"""Every symbol the schematic puts on the board must exist in the layout.

Why this exists: on 2026-08-22, rocket-computer's schematic carried Q11
(AONR21321, the flight-battery reverse-polarity MOSFET — `on_board yes`,
`dnp no`, footprint assigned) and the PCB did not have it at all.  Its drain
net VBAT_J8 therefore reached nothing but J8 pad 2 and a floating 43.8 mm²
pour, i.e. the flight battery's positive terminal was not connected to the
board.

`kicad-cli pcb drc --schematic-parity` did NOT catch it — it reported zero
parity issues, because it reconciles footprints that ARE on the board rather
than noticing one that is absent.  Per docs/board-versioning.md that DRC pass
is the only gate on hardware/, so nothing stood between this and a fab run.

Deliberately pure-Python: no KiCad on the runner, so it can gate every push.
It answers one question — does the set of board-bound schematic symbols equal
the set of PCB footprints — which is exactly the question that went unasked.
"""
from __future__ import annotations

import sys
from pathlib import Path


def sexpr(text: str):
    """Minimal s-expression reader. KiCad files are plain nested lists."""
    tokens, i, n = [], 0, len(text)
    stack, cur = [], []
    while i < n:
        c = text[i]
        if c == '(':
            stack.append(cur)
            cur = []
            i += 1
        elif c == ')':
            done = cur
            cur = stack.pop() if stack else []
            cur.append(done)
            i += 1
        elif c == '"':
            j = i + 1
            buf = []
            while j < n:
                if text[j] == '\\':
                    buf.append(text[j + 1]); j += 2; continue
                if text[j] == '"':
                    break
                buf.append(text[j]); j += 1
            cur.append(''.join(buf))
            i = j + 1
        elif c.isspace():
            i += 1
        else:
            j = i
            while j < n and not text[j].isspace() and text[j] not in '()"':
                j += 1
            cur.append(text[i:j])
            i = j
    return cur


def walk(node):
    if isinstance(node, list):
        yield node
        for child in node:
            yield from walk(child)


def head(node) -> str:
    return node[0] if node and isinstance(node[0], str) else ''


def field(node, name, default=None):
    for child in node:
        if isinstance(child, list) and head(child) == name and len(child) > 1:
            return child[1]
    return default


def schematic_refs(board_dir: Path) -> tuple[set[str], dict[str, str]]:
    """References the schematic says belong on the board, and their lib_ids."""
    refs: set[str] = set()
    libs: dict[str, str] = {}
    for sch in sorted(board_dir.glob('*.kicad_sch')):
        for node in walk(sexpr(sch.read_text(errors='replace'))):
            if head(node) != 'symbol':
                continue
            lib_id = field(node, 'lib_id')
            if not lib_id:
                continue                      # a library definition, not an instance
            if lib_id.startswith('power:'):
                continue                      # power flags have no footprint by design
            if field(node, 'on_board') == 'no':
                continue                      # deliberately schematic-only
            for inner in walk(node):
                if head(inner) == 'instances':
                    for ref_node in walk(inner):
                        if head(ref_node) == 'reference' and len(ref_node) > 1:
                            r = ref_node[1]
                            if r and not r.startswith('#'):   # #PWR / #FLG
                                refs.add(r)
                                libs[r] = lib_id
    return refs, libs


def pcb_refs(board_dir: Path) -> set[str]:
    refs: set[str] = set()
    for pcb in sorted(board_dir.glob('*.kicad_pcb')):
        for node in walk(sexpr(pcb.read_text(errors='replace'))):
            if head(node) != 'footprint':
                continue
            for prop in node:
                if (isinstance(prop, list) and head(prop) == 'property'
                        and len(prop) > 2 and prop[1] == 'Reference'):
                    if prop[2]:
                        refs.add(prop[2])
    return refs


# Boards not gated, and why.  Everything else is gated by DEFAULT, so a new
# board is covered the day it lands rather than the day someone remembers.
EXEMPT = {
    'rocket-computer-mini':
        'layout in progress — never tagged or fabbed, ~40 parts still unplaced '
        '(U32 the ESP32-S3 among them). Gate it once the layout is complete.',
    'rocket-computer':
        'pack-fire hold-up (2026-08-26) + supervised-arm rework 3 (2026-08-28) '
        'await the V10 layout pass: 20 symbols are schematic-only (C130, '
        'C134/C136-C140, D16, R120, R125-R128, R132/R133, U40/U42/U44/U45/U46). '
        'THIS BOARD FLIES — re-gate it the moment the rework is placed, and do '
        'not let anything else ride in behind the exemption. Tracked in #966.',
}


def main() -> int:
    root = Path(__file__).resolve().parent.parent / 'hardware'
    boards = sorted(d for d in root.iterdir()
                    if d.is_dir() and any(d.glob('*.kicad_pcb')))
    failed = False
    for board in boards:
        sch, libs = schematic_refs(board)
        pcb = pcb_refs(board)
        missing = sorted(sch - pcb)
        if board.name in EXEMPT:
            note = f"{len(missing)} unplaced" if missing else "complete"
            print(f"skip {board.name}: not gated ({note}) — {EXEMPT[board.name]}")
            continue
        if missing:
            failed = True
            print(f"FAIL {board.name}: {len(missing)} symbol(s) marked for the board "
                  f"in the schematic, absent from the layout")
            for r in missing:
                print(f"       {r}  ({libs.get(r, '?')})")
        else:
            print(f"  ok {board.name}: {len(sch)} board symbols all present in the layout")
    if failed:
        print("\nA symbol marked for the board has no footprint in the layout, so its "
              "nets are unrouted copper. Place it, or set `on_board no` if it is "
              "deliberately schematic-only. See #833 for how this ships otherwise.")
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
