#!/usr/bin/env python3
"""
Generate the wire-protocol reference from the sources that define it.

Every table in docs/architecture/generated/protocol-reference.md is extracted,
never transcribed. That is the whole point: the wire surface is exactly where a
hand-maintained document goes wrong quietly. A command number copied into a
table and then reassigned in firmware reads as authoritative and is a lie, and
this project has already paid for that class of mistake (#132/#148 shipped two
handlers on one command number).

Sources of truth, in the order they appear in the output:

  * Frame format ......... TR_I2C_Interface.h (SOF bytes)
  * FC<->OC message types  the curated registry in
                           tests_cpp/test_rocket_computer_types.cpp, resolved
                           against RocketComputerTypes.h for values/comments.
                           The registry is used rather than grepping the header
                           because the header's constant block also holds
                           payload sentinels (IMU_ORIENT_AUTO = 0xFF is not a
                           message type), and the registry is CI-enforced for
                           uniqueness.
  * Struct wire sizes .... static_assert(sizeof(X) == N) in the header --
                           compiler-enforced, so it cannot drift at all
  * BLE command spaces ... the `ble_cmd ==` dispatch chains in the OC and Base
                           Station firmware, with each branch's own comment as
                           its description
  * Flags and bitfields .. named constants in the header

Cross-check: any constant >= 0xA0 in the header's message-type block that is
NOT in the gtest registry is reported, because that is either a missing
registry entry (the uniqueness check silently stops covering it) or a sentinel
that belongs in KNOWN_NON_MESSAGE below.

Usage:
    python3 tools/gen_protocol_reference.py            # write the reference
    python3 tools/gen_protocol_reference.py --check    # exit 1 if stale
"""

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
REPO_URL = "https://github.com/Tinkerbug-Robotics/TinkerRocket"
BRANCH = "main"

HEADER = REPO / "tinkerrocket-idf/components/TR_RocketComputerTypes/RocketComputerTypes.h"
REGISTRY = REPO / "tests_cpp/test_rocket_computer_types.cpp"
I2C_HDR = REPO / "tinkerrocket-idf/components/TR_I2C_Interface/TR_I2C_Interface.h"
OUT = REPO / "docs/architecture/generated/protocol-reference.md"

DISPATCHES = [
    ("app to Out Computer", REPO / "tinkerrocket-idf/projects/out_computer/main/main.cpp"),
    ("app to Base Station", REPO / "tinkerrocket-idf/projects/base_station/main/main.cpp"),
]

# Constants that live in the message-type block but are payload values, not
# message types. Anything else missing from the registry is a real finding.
KNOWN_NON_MESSAGE = {"IMU_ORIENT_AUTO"}

CONST_RE = re.compile(
    r"^static constexpr uint8_t\s+(\w+)\s*=\s*(0[xX][0-9A-Fa-f]+|\d+)\s*;"
    r"(?:\s*//\s*(.*))?"
)
# Values are either a literal or a bit-shift expression -- the flag constants
# are written `(1u << 3)`, which a literal-only pattern silently skips (it did,
# and the telemetry-flags table came out empty).
ANY_U8_RE = re.compile(
    r"static\s+constexpr\s+uint8_t\s+(\w+)\s*=\s*"
    r"(0[xX][0-9A-Fa-f]+|\d+|\(\s*1u?\s*<<\s*\d+\s*\))\s*;"
    r"(?:\s*//\s*(.*))?"
)
SHIFT_RE = re.compile(r"\(\s*1u?\s*<<\s*(\d+)\s*\)")


def parse_u8(text):
    """Value of a uint8 constant: literal or `(1u << N)`."""
    m = SHIFT_RE.match(text)
    if m:
        return 1 << int(m.group(1))
    return int(text, 0)
SIZE_RE = re.compile(r"static_assert\(\s*sizeof\((\w+)\)\s*==\s*(\d+)")
MT_RE = re.compile(r"MT\(\s*([A-Z_][A-Z0-9_]*)\s*\)")
BLE_CMD_RE = re.compile(r"ble_cmd\s*==\s*(0[xX][0-9A-Fa-f]+|\d+|[A-Za-z_]\w*)")
NUMERIC_RE = re.compile(r"\A(?:0[xX][0-9A-Fa-f]+|\d+)\Z")
MSG_BLOCK_MARK = "### Message Types from In ESP32 ###"


def blob(path, line=None):
    rel = path.relative_to(REPO)
    anchor = f"#L{line}" if line else ""
    return f"{REPO_URL}/blob/{BRANCH}/{rel}{anchor}"


def sentence(text, limit=110):
    """First sentence of a comment, trimmed for a table cell."""
    text = re.sub(r"\s+", " ", text).strip().rstrip(".")
    text = text.replace("|", "\\|")
    if len(text) > limit:
        cut = text[:limit].rsplit(" ", 1)[0]
        return cut + "…"
    return text


def header_constants():
    """All uint8 constants in the header: name -> (value, comment)."""
    out = {}
    for line in HEADER.read_text().splitlines():
        m = ANY_U8_RE.search(line)
        if m:
            out[m.group(1)] = (parse_u8(m.group(2)), (m.group(3) or "").strip())
    return out


def message_block_constants():
    """uint8 constants declared inside the '### Message Types ###' block."""
    lines = HEADER.read_text().splitlines()
    start = next((i for i, l in enumerate(lines) if MSG_BLOCK_MARK in l), None)
    if start is None:
        sys.exit(f"could not find {MSG_BLOCK_MARK!r} in {HEADER}")
    out = {}
    for line in lines[start:]:
        m = CONST_RE.match(line)
        if m:
            out[m.group(1)] = (int(m.group(2), 0), (m.group(3) or "").strip())
    return out


def registry_names():
    """The curated MT(...) message-type list from the host test."""
    text = REGISTRY.read_text()
    blk = re.search(r"const MsgType codes\[\]\s*=\s*\{(.*?)\n\s*\};", text, re.S)
    if not blk:
        sys.exit(f"could not find the MsgType registry in {REGISTRY}")
    seen, names = set(), []
    for n in MT_RE.findall(blk.group(1)):
        if n not in seen:
            seen.add(n)
            names.append(n)
    return names


def direction(name, comment):
    """Infer link direction from the constant's own comment."""
    for arrow, label in (("OC→FC", "OC → FC"), ("FC→OC", "FC → OC"),
                         ("OC->FC", "OC → FC"), ("FC->OC", "FC → OC"),
                         ("OC→self", "OC → log")):
        if arrow in comment:
            return label
    if name.endswith("_PENDING") or name.endswith("_CMD"):
        return "OC → FC"
    if name.endswith("_MSG"):
        return "FC → OC"
    return "—"


def ble_commands(path, consts):
    """[(value, token, description)] for one device's dispatch chain."""
    lines = path.read_text().splitlines()
    rows, unresolved = [], []
    for i, line in enumerate(lines):
        m = BLE_CMD_RE.search(line)
        if not m:
            continue
        tok = m.group(1)
        val = int(tok, 0) if NUMERIC_RE.match(tok) else (
            consts[tok][0] if tok in consts else None)
        if val is None:
            unresolved.append(tok)
            continue
        rows.append((val, tok, branch_description(lines, i, tok)))
    rows.sort(key=lambda r: r[0])
    return rows, unresolved


def branch_description(lines, i, token):
    """Comment describing a dispatch branch: inside it, else above it."""
    j = i + 1
    while j < len(lines) and lines[j].strip() in ("", "{"):
        j += 1
    body = []
    while j < len(lines) and lines[j].strip().startswith("//"):
        body.append(lines[j].strip().lstrip("/").strip())
        j += 1
    if body:
        return sentence(" ".join(body))
    k, above = i - 1, []
    while k >= 0 and lines[k].strip().startswith("//"):
        above.insert(0, lines[k].strip().lstrip("/").strip())
        k -= 1
    if above:
        return sentence(" ".join(above))
    # Fall back to humanising a self-describing constant name.
    if not NUMERIC_RE.match(token):
        pretty = re.sub(r"^(BLE_BS_CMD_|BLE_CMD_|LORA_CMD_)", "", token)
        return pretty.replace("_", " ").capitalize()
    return ""


def flag_table(consts, prefix, exclude=()):
    rows = []
    for name, (val, comment) in consts.items():
        if name.startswith(prefix) and name not in exclude:
            rows.append((val, name, sentence(comment)))
    rows.sort(key=lambda r: r[0])
    return rows


def render():
    consts = header_constants()
    msg_block = message_block_constants()
    names = registry_names()
    i2c_text = I2C_HDR.read_text()
    sof = re.findall(r"static constexpr uint8_t SOF\d\s*=\s*(0[xX][0-9A-Fa-f]+)", i2c_text)

    o = []
    a = o.append
    a("<!-- GENERATED by tools/gen_protocol_reference.py -- do not edit by hand. -->")
    a("<!-- Regenerate with: python3 tools/gen_protocol_reference.py -->")
    a("")
    a("# Wire protocol reference")
    a("")
    a("Every table on this page is extracted from source, not transcribed. See")
    a(f"[`tools/gen_protocol_reference.py`]({blob(REPO / 'tools/gen_protocol_reference.py')})")
    a("for which file each one comes from. For how the links are used, see")
    a("[Protocols](../protocols.md).")
    a("")

    # ---- framing -----------------------------------------------------------
    a("## Frame format")
    a("")
    a("Inter-board frames (FC↔OC over I2C and I2S, and the on-flash log) share one")
    a("framing:")
    a("")
    sof_txt = " ".join(s.upper().replace("0X", "0x") for s in sof)
    a("```")
    a(f"[{sof_txt}] [Type] [Length] [Payload] [CRC16_MSB] [CRC16_LSB]")
    a(f"{'(start of frame)'.center(len(sof_txt) + 2)}  (1)     (1)     (0-N)     (1)         (1)")
    a("```")
    a("")
    a(f"Start-of-frame bytes from [`TR_I2C_Interface.h`]({blob(I2C_HDR)}).")
    a("")

    # ---- message types -----------------------------------------------------
    missing_value = [n for n in names if n not in consts]
    a("## FC ↔ OC message types")
    a("")
    a(f"{len(names)} codes. The dispatch on both ends is a flat first-match chain, so")
    a("two handlers sharing a value means the second is silently dead — which is why")
    a("this list is CI-enforced for uniqueness.")
    a("")
    a("| Code | Name | Direction | Notes |")
    a("|------|------|-----------|-------|")
    undocumented = 0
    for n in names:
        if n not in consts:
            a(f"| ? | `{n}` | — | **not found in the header** |")
            continue
        val, comment = consts[n]
        if not comment:
            undocumented += 1
        a(f"| `0x{val:02X}` | `{n}` | {direction(n, comment)} | {sentence(comment)} |")
    a("")
    if undocumented:
        a(f"> {undocumented} of these {len(names)} codes carry no comment in the header,")
        a("> so the Notes column is blank for them. Direction is inferred from the")
        a("> `_PENDING` / `_CMD` / `_MSG` suffix in that case, which is a convention,")
        a("> not a guarantee. A trailing `// OC→FC: what it does` on the constant")
        a("> fills the row in.")
        a("")

    stray = sorted(k for k, (v, _) in msg_block.items()
                   if v >= 0xA0 and k not in set(names) and k not in KNOWN_NON_MESSAGE)
    if stray:
        a("> **Not in the registry:** " + ", ".join(f"`{s}`" for s in stray))
        a("> — these are declared in the message-type block but absent from the")
        a("> uniqueness registry, so nothing checks them for collisions. Either add")
        a("> them to the registry or, if they are payload sentinels, to")
        a("> `KNOWN_NON_MESSAGE` in the generator.")
        a("")

    # ---- struct sizes ------------------------------------------------------
    sizes = SIZE_RE.findall(HEADER.read_text())
    a("## Wire struct sizes")
    a("")
    a("From `static_assert` in the header, so these are compiler-enforced: a struct")
    a("that changes size fails the build rather than corrupting a log silently.")
    a("")
    a("| Struct | Bytes |")
    a("|--------|-------|")
    for name, n in sorted(sizes, key=lambda s: (-int(s[1]), s[0])):
        a(f"| `{name}` | {n} |")
    a("")

    # ---- BLE commands ------------------------------------------------------
    a("## BLE command spaces")
    a("")
    a("The app talks to **either** an Out Computer or a Base Station, so the two")
    a("spaces are independent and values may overlap: command 50 is mag-cal start to")
    a("the Out Computer and relay-to-rocket to the Base Station. Both are guarded")
    a(f"for internal uniqueness by [`tools/check_ble_command_ids.py`]({blob(REPO / 'tools/check_ble_command_ids.py')}).")
    a("")
    for label, path in DISPATCHES:
        if not path.exists():
            continue
        rows, unresolved = ble_commands(path, consts)
        a(f"### {label}")
        a("")
        a(f"{len(rows)} commands.")
        a("")
        a("| Cmd | Constant | Description |")
        a("|-----|----------|-------------|")
        for val, tok, desc in rows:
            const = "" if NUMERIC_RE.match(tok) else f"`{tok}`"
            a(f"| {val} | {const} | {desc} |")
        a("")
        undoc = [str(v) for v, _, d in rows if not d]
        if undoc:
            a(f"> {len(undoc)} of these have no comment in the dispatch and so no")
            a(f"> description here: {', '.join(undoc)}. Adding a comment to the branch")
            a("> fills this table in.")
            a("")
        if unresolved:
            a(f"> Unresolved tokens (not literals, not in the header): "
              f"{', '.join(f'`{u}`' for u in unresolved)}")
            a("")

    # ---- flags -------------------------------------------------------------
    a("## Telemetry flags")
    a("")
    a("`NonSensorData.flags` — set by the FC, carried to the log and the ground.")
    a("")
    a("| Bit | Name | Meaning |")
    a("|-----|------|---------|")
    for val, name, comment in flag_table(consts, "NSF_"):
        bit = val.bit_length() - 1 if val and (val & (val - 1)) == 0 else None
        a(f"| {bit if bit is not None else '—'} | `{name}` | {comment} |")
    a("")

    nsf2 = flag_table(consts, "NSF2_")
    if nsf2:
        a("`NonSensorData.apogee_flags` — apogee-detector votes and health bits.")
        a("")
        a("| Bit | Name | Meaning |")
        a("|-----|------|---------|")
        for val, name, comment in nsf2:
            bit = val.bit_length() - 1 if val and (val & (val - 1)) == 0 else None
            a(f"| {bit if bit is not None else '—'} | `{name}` | {comment} |")
        a("")

    a("## Sensor health field")
    a("")
    a("Two bits per sensor at the shifts below, each holding a `SensorHealthState`")
    a("(`SH_NA` 0, `SH_OK` 1, `SH_DEGRADED` 2, `SH_BAD` 3).")
    a("")
    rows = flag_table(consts, "SH_",
                      exclude={"SH_NA", "SH_OK", "SH_DEGRADED", "SH_BAD"})
    # The four pyro channels share one array declaration, which the scalar
    # constant pattern does not see; without this the table silently omits
    # shifts 12-18 and reads as if nothing lives there.
    m = re.search(r"SH_PYRO_SHIFT\[\d+\]\s*=\s*\{([^}]*)\}", HEADER.read_text())
    if m:
        for ch, v in enumerate(int(x) for x in re.findall(r"\d+", m.group(1))):
            rows.append((v, f"SH_PYRO_SHIFT[{ch}]", f"pyro channel {ch + 1}"))
    rows.sort(key=lambda r: r[0])
    a("| Shift | Name | Covers |")
    a("|-------|------|--------|")
    for val, name, comment in rows:
        a(f"| {val} | `{name}` | {comment} |")
    a("")

    # ---- LoRa --------------------------------------------------------------
    a("## LoRa constants")
    a("")
    a("| Name | Value | Notes |")
    a("|------|-------|-------|")
    for name in sorted(consts):
        if name.startswith("LORA_") and not name.startswith("LORA_FACTORY_"):
            val, comment = consts[name]
            a(f"| `{name}` | `0x{val:02X}` ({val}) | {sentence(comment)} |")
    a("")
    a("Factory rendezvous parameters — the channel both ends fall back to, and the")
    a("one every Base Station boot forces regardless of NVS:")
    a("")
    a("| Name | Value |")
    a("|------|-------|")
    for name in sorted(consts):
        if name.startswith("LORA_FACTORY_"):
            a(f"| `{name}` | {consts[name][0]} |")
    a("")

    return "\n".join(o) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true",
                    help="verify the committed reference matches the sources")
    args = ap.parse_args()

    for p in (HEADER, REGISTRY, I2C_HDR):
        if not p.exists():
            print(f"ERROR: missing source {p}", file=sys.stderr)
            return 1

    want = render()
    have = OUT.read_text() if OUT.exists() else None

    if args.check:
        if have != want:
            print(f"  X  {OUT.relative_to(REPO)}: STALE", file=sys.stderr)
            print("\nProtocol reference is out of date with the sources.\n"
                  "Run:  python3 tools/gen_protocol_reference.py\n"
                  "and commit the result.", file=sys.stderr)
            return 1
        print(f"  ok {OUT.relative_to(REPO)}")
        return 0

    if have == want:
        print(f"  ok {OUT.relative_to(REPO)}: unchanged")
    else:
        OUT.parent.mkdir(parents=True, exist_ok=True)
        OUT.write_text(want)
        print(f"  ->  {OUT.relative_to(REPO)}: written")
    return 0


if __name__ == "__main__":
    sys.exit(main())
