#!/usr/bin/env python3
"""Every symbol the schematic puts on the board must exist in the layout, and
every footprint in the layout must be linked to its own symbol.

Why this exists: on 2026-08-22, rocket-computer's schematic carried Q11
(AONR21321, the flight-battery reverse-polarity MOSFET — `on_board yes`,
`dnp no`, footprint assigned) and the PCB did not have it at all.  Its drain
net VBAT_J8 therefore reached nothing but J8 pad 2 and a floating 43.8 mm²
pour, i.e. the flight battery's positive terminal was not connected to the
board.

`kicad-cli pcb drc --schematic-parity` did NOT catch it — it reported zero
parity issues, because it reconciles footprints that ARE on the board rather
than noticing one that is absent.  Per docs/board-versioning.md that DRC pass
was the only gate on hardware/, so nothing stood between this and a fab run.

Being present is not being linked.  Update PCB from Schematic does not find a
footprint's symbol by reference: it follows the footprint's `(path …)`, the
uuids of the sheet symbols from the root sheet down and then the symbol's own.
On 2026-09-28 tinker-base's J2 had no path at all, and R13 and R16 had paths
that began with the root sheet's own uuid, which a footprint path never
carries.  `--schematic-parity` and the reference check here both passed,
because both look footprints up by reference.  The owner's next Update PCB
deleted all three and dropped fresh copies off the board edge, every track to
them left dangling.

Deliberately pure-Python: no KiCad on the runner, so it can gate every push.
It answers two questions — does every board-bound schematic symbol have a
footprint, and does every footprint's path lead back to a symbol with the
footprint's reference — which are exactly the questions that went unasked.

Usage:
    python3 tools/check_board_parity.py              # every board under hardware/
    python3 tools/check_board_parity.py DIR [DIR …]  # these board directories
"""
from __future__ import annotations

import argparse
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


def prop(node, name, default=''):
    """The value of a `(property "<name>" "<value>" …)` child."""
    for child in node:
        if (isinstance(child, list) and head(child) == 'property'
                and len(child) > 2 and child[1] == name):
            return child[2]
    return default


def board_bound(symbol) -> bool:
    """A placed symbol that the schematic sends to the board."""
    lib_id = field(symbol, 'lib_id')
    if not lib_id:
        return False                          # a library definition, not an instance
    if lib_id.startswith('power:'):
        return False                          # power flags have no footprint by design
    return field(symbol, 'on_board') != 'no'  # `no` is deliberately schematic-only


def schematic_refs(board_dir: Path) -> tuple[set[str], dict[str, str]]:
    """References the schematic says belong on the board, and their lib_ids."""
    refs: set[str] = set()
    libs: dict[str, str] = {}
    for sch in sorted(board_dir.glob('*.kicad_sch')):
        for node in walk(sexpr(sch.read_text(errors='replace'))):
            if head(node) != 'symbol' or not board_bound(node):
                continue
            for inner in walk(node):
                if head(inner) == 'instances':
                    for ref_node in walk(inner):
                        if head(ref_node) == 'reference' and len(ref_node) > 1:
                            r = ref_node[1]
                            if r and not r.startswith('#'):   # #PWR / #FLG
                                refs.add(r)
                                libs[r] = field(node, 'lib_id')
    return refs, libs


def symbol_paths(root_sch: Path) -> tuple[str, dict[str, str]]:
    """The root sheet's uuid, and the footprint path of every board-bound
    symbol in the hierarchy under it, mapped to the symbol's reference.

    A footprint path is the uuids of the sheet symbols from the root sheet
    down, then the symbol's own uuid.  The root file's uuid is not part of it,
    although every `instances` path in the schematic starts with it.  A
    multi-unit symbol has one uuid per unit, and any of them links.
    """
    root = ''
    paths: dict[str, str] = {}
    pending = [(root_sch, ())]
    while pending:
        sch, sheets = pending.pop()
        top = sexpr(sch.read_text(errors='replace'))[0]
        root = root or field(top, 'uuid', '')
        instance = '/'.join(('', root, *sheets))  # this sheet as `instances` spells it
        for node in top:
            if not isinstance(node, list):
                continue
            if head(node) == 'sheet':
                pending.append((sch.parent / prop(node, 'Sheetfile'),
                                (*sheets, field(node, 'uuid'))))
            elif head(node) == 'symbol' and board_bound(node):
                for inner in walk(node):
                    if head(inner) == 'path' and len(inner) > 1 and inner[1] == instance:
                        r = field(inner, 'reference', '')
                        if r and not r.startswith('#'):
                            paths['/'.join(('', *sheets, field(node, 'uuid')))] = r
    return root, paths


def pcb_refs(board_dir: Path) -> tuple[set[str], list[tuple[str, str]]]:
    """References in the layout, and each footprint whose path does not lead
    back to a board-bound symbol with its reference, with the reason."""
    refs: set[str] = set()
    unlinked: list[tuple[str, str]] = []
    for pcb in sorted(board_dir.glob('*.kicad_pcb')):
        root, links = symbol_paths(pcb.with_suffix('.kicad_sch'))
        names = set(links.values())
        for node in walk(sexpr(pcb.read_text(errors='replace'))):
            if head(node) != 'footprint':
                continue
            attr = next((a for a in node if isinstance(a, list) and head(a) == 'attr'), [])
            if 'board_only' in attr:
                continue                      # a logo: no symbol, nor can it stand in for one
            ref = prop(node, 'Reference')
            if ref:
                refs.add(ref)
            path = field(node, 'path', '')
            if links.get(path) == ref:
                continue
            if ref not in names:
                why = "no board symbol has this reference"
            elif not path:
                why = "no path"
            elif path.startswith(f'/{root}/'):
                why = "path starts with the root sheet's uuid, which a footprint path never carries"
            elif path in links:
                why = f"path leads to {links[path]}'s symbol"
            else:
                why = f"path {path} leads to no symbol"
            unlinked.append((ref or '?', why))
    return refs, sorted(unlinked)


# Boards not gated, and why.  Everything else is gated by DEFAULT, so a new
# board is covered the day it lands rather than the day someone remembers.
# Empty from 2026-09-22, when the V10 rework was placed (275 of 275 board
# symbols) and the mini fabbed and fully placed (229 of 229), until the path
# check arrived on 2026-09-28.  An entry here is a hole in the gate on a board
# that may fly, so add one only with its reason and the condition that
# removes it.
EXEMPT = {
    'base-station': "retired; R13/R16 are the TPS63020 feedback divider that "
                    "0f721aa6 took out of the schematic, and the V6 layout was never "
                    "synced. Remove after an Update PCB on it, plus the short FB "
                    "trace 0f721aa6 describes",
}


def board_arg(arg: str) -> Path:
    d = Path(arg)
    if not any(d.glob('*.kicad_pcb')):
        raise argparse.ArgumentTypeError(f"no .kicad_pcb in {arg}")
    return d


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('boards', nargs='*', type=board_arg, metavar='DIR',
                    help='board directories to check (default: every board under hardware/)')
    args = ap.parse_args()
    root = Path(__file__).resolve().parent.parent / 'hardware'
    if args.boards:
        # A board anywhere — a scratch copy, another checkout, an old commit
        # extracted with `git archive` — labelled as given.
        boards = [(d, str(d)) for d in args.boards]
    else:
        # Retired boards live one level down in hardware/legacy/ and are still gated:
        # moving a board out of the product line must not quietly drop its check.
        candidates = [*root.iterdir(), *(root / 'legacy').iterdir()]
        boards = sorted(((d, d.relative_to(root).as_posix()) for d in candidates
                         if d.is_dir() and any(d.glob('*.kicad_pcb'))),
                        key=lambda b: b[1])
    any_missing = any_unlinked = False
    for board, label in boards:
        sch, libs = schematic_refs(board)
        pcb, unlinked = pcb_refs(board)
        missing = sorted(sch - pcb)
        if board.name in EXEMPT:
            notes = []
            if missing:
                notes.append(f"{len(missing)} unplaced")
            if unlinked:
                notes.append(f"{len(unlinked)} unlinked")
            print(f"skip {label}: not gated ({', '.join(notes) or 'complete'})"
                  f" — {EXEMPT[board.name]}")
            continue
        if missing:
            any_missing = True
            print(f"FAIL {label}: {len(missing)} symbol(s) marked for the board "
                  f"in the schematic, absent from the layout")
            for r in missing:
                print(f"       {r}  ({libs.get(r, '?')})")
        if unlinked:
            any_unlinked = True
            print(f"FAIL {label}: {len(unlinked)} footprint(s) not linked by path "
                  f"to a symbol with their reference")
            for r, why in unlinked:
                print(f"       {r}  {why}")
        if not missing and not unlinked:
            print(f"  ok {label}: {len(sch)} board symbols all present in the layout "
                  f"and linked by path")
    if any_missing:
        print("\nA symbol marked for the board has no footprint in the layout, so its "
              "nets are unrouted copper. Place it, or set `on_board no` if it is "
              "deliberately schematic-only. See #833 for how this ships otherwise.")
    if any_unlinked:
        print("\nUpdate PCB matches a footprint to its symbol by path, not by "
              "reference, so it drops a fresh copy of an unlinked part off the board edge "
              "and can delete the placed one, leaving its tracks dangling. Re-link first: "
              "one Update PCB with 'Re-link footprints to schematic symbols based on their "
              "reference designators' ticked rewrites the paths and keeps the placement "
              "and routing. A footprint with no board symbol at all is a leftover: delete "
              "it, or tick its 'Not in schematic' attribute if it is artwork.")
    return 1 if any_missing or any_unlinked else 0


if __name__ == '__main__':
    sys.exit(main())
