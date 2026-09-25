#!/usr/bin/env python3
"""Assemble the two reports from the editable text, the receiver data and the figures.

    report.html        GNSS Receiver Limits: where each receiver stops publishing
    boost_report.html  GNSS Under Boost: what the boost itself does to tracking

Three inputs, each owned by whoever should own it:

    report_text.html, boost_report_text.html   the words   -- edit freely
    report_head.html   the stylesheet, shared by both     -- edit rarely
    results/receivers.json + results/figures/   the measurements

The prose used to live in an f-string in this file, which meant a stray brace in
a sentence broke the build and anyone editing the page had to work inside Python
quoting. It is now a plain HTML file, read as text and never string-formatted,
so braces, quotes and percent signs in the copy need no escaping.

Placeholders in the text files are substituted, not formatted. Either page may
use any of them; each must appear on at least one page:

    {{GATE_TABLE_ROWS}}    rows of the comparison table
    {{RECEIVER_SECTIONS}}  a panel per receiver, blurb + measured figures + plots
    {{FIG_BLOCK_DIAGRAM}}  }
    {{FIG_M8T_ALTRAMP}}    }  inline SVG from results/figures/
    {{FIG_DIP}}            }
    {{FIG_BOOST_RATE}}     }
    {{FIG_BOOST_DOPPLER}}  }
    {{FIG_LC86_MODES}}     }
    {{FIG_LC86_KNEE}}      }

Per-receiver blurbs are marked in report_text.html as

    <!--#blurb px1125r--> ... <!--/blurb-->

    ./build_report.py            regenerate both pages
    ./build_report.py --check    verify inputs without writing
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIG = HERE / "results" / "figures"
TEXT = HERE / "report_text.html"
HEAD = HERE / "report_head.html"
DATA = HERE / "results" / "receivers.json"
OUT = HERE / "report.html"

# (words, page, <title>). The title goes into the head, where the first 8 kB of
# the page -- all a gallery reads -- can see it. Per-receiver blurbs and figure
# notes live in the first file only.
PAGES = [
    (TEXT, OUT, "GNSS Receiver Limits"),
    (HERE / "boost_report_text.html", HERE / "boost_report.html", "GNSS Under Boost"),
]

sys.path.insert(0, str(HERE))
from receiver_table import bracket        # noqa: E402

CAUSE_LABEL = {"not cocom": "not COCOM", "dyn model": "dynamic model",
               "nav mode": "navigation mode", "mute": "all output muted"}

# Which figures illustrate each receiver, and the headline shown above them.
PLOTS = {
    "px1125r":  ("spaceshot.svg", "gentle_alt.svg"),
    "sam_m10q": ("ublox_m10_spaceshot.svg", "ublox_m10_gentle_alt.svg"),
    "zed_f9p":  ("zed_f9p_spaceshot.svg", "zed_f9p_gentle_alt.svg"),
    "neo_m8t":  ("neo_m8t_spaceshot.svg", "neo_m8t_gentle_alt.svg"),
    "air530":   ("air530_spaceshot.svg", "air530_gentle_alt.svg"),
    "quescan_m10": ("quescan_m10_spaceshot.svg", "quescan_m10_gentle_alt.svg"),
    "beitian_bn182": ("beitian_bn182_spaceshot.svg", "beitian_bn182_gentle_alt.svg"),
    "lc86g_normal": ("lc86g_normal_spaceshot.svg", "lc86g_normal_gentle_alt.svg"),
    "lc86g_balloon": ("lc86g_balloon_spaceshot.svg", "lc86g_balloon_gentle_alt.svg"),
}
ORDER = ["px1125r", "sam_m10q", "quescan_m10", "beitian_bn182", "zed_f9p", "neo_m8t",
         "air530", "lc86g_normal", "lc86g_balloon"]


def scoped(svg: str, scope: str) -> str:
    """Confine an inline figure's <style> rules to that figure.

    A <style> inside inline SVG is not scoped to the SVG: every rule applies to
    the whole page. The figures reuse short class names (.lbl, .gl, .note) with
    different values, so whichever figure came last restyled all the others.
    Each rule is prefixed with the figure's own class inside :where(), which
    leaves its specificity exactly as it was.
    """
    m = re.search(r'<svg\b[^>]*>', svg)
    if not m:
        return svg
    tag = m.group(0)
    tag = (tag.replace('class="', f'class="{scope} ', 1) if 'class="' in tag
           else tag[:4] + f' class="{scope}"' + tag[4:])

    def rule(r):
        sels = ", ".join(f":where(svg.{scope}) {s.strip()}" for s in r.group(1).split(","))
        return f"{sels}{{{r.group(2)}}}"

    def style(s):
        if "@" in s.group(1):        # an at-rule would need real parsing
            raise SystemExit(f"  !! {scope}: at-rule in a figure's <style>, cannot scope it")
        return f"<style>{re.sub(r'([^{}]+)\{([^}]*)\}', rule, s.group(1))}</style>"

    svg = svg[:m.start()] + tag + svg[m.end():]
    return re.sub(r'<style>(.*?)</style>', style, svg, flags=re.S)


def fig(name: str) -> str:
    p = FIG / name
    if not p.exists():
        return f"<!-- missing {name} -->"
    return scoped(p.read_text().strip(), "fig-" + re.sub(r'[^a-z0-9]+', '-', p.stem.lower()))


def blurbs(text: str) -> dict:
    """Per-receiver prose, keyed by receiver id, from the marked blocks."""
    return {m.group(1): m.group(2).strip() for m in
            re.finditer(r'<!--#blurb (\w+)-->(.*?)<!--/blurb-->', text, re.S)}


def fignotes(text: str) -> dict:
    """Optional note printed under a receiver's plots, keyed by receiver id.

    For things visible in one part's figures that would be noise in the shared
    caption -- a bench artifact in one run, say. A receiver with nothing to say
    simply has no block.
    """
    return {m.group(1): m.group(2).strip() for m in
            re.finditer(r'<!--#fignote (\w+)-->(.*?)<!--/fignote-->', text, re.S)}


def receiver_sections(d, text) -> str:
    by_id = {r["id"]: r for r in d["receivers"]}
    notes = blurbs(text)
    fnotes = fignotes(text)
    out = []
    for rid in ORDER:
        r = by_id[rid]
        f1, f2 = PLOTS[rid]
        fn = fnotes.get(rid)
        note = (f'\n    <div class="prose">\n      <p class="note">{fn}</p>\n    </div>'
                if fn else "")
        vel = (f"none to {r['velocity_fix_max_mps']:.0f} m/s"
               if r.get("velocity_gate_present") is False else
               bracket(r.get("velocity_fix_max_mps"),
                       r.get("velocity_blocked_min_mps"), "m/s"))
        alt = bracket(r.get("altitude_fix_max_km"),
                      r.get("altitude_blocked_min_km"), "km", "{:.2f}")
        if alt == "--" and r.get("altitude_note"):
            alt = r["altitude_note"]       # say why there is no bracket
        cause = (r.get("altitude_gate_cause") or "cocom").strip().lower()
        if cause != "cocom" and alt != "--":
            alt += f" <em>({CAUSE_LABEL.get(cause, cause)})</em>"
        out.append(f'''
    <div class="prose">
      <h3>{r['part']}</h3>
      <p class="note">{r['protocol']} &middot; {r['path']} &middot; {r['rf']}</p>
      <p>{notes.get(rid, '')}</p>
      <p><strong>Velocity gate</strong> {vel} &nbsp;&middot;&nbsp;
         <strong>Altitude gate</strong> {alt} &nbsp;&middot;&nbsp;
         <strong>Satellites</strong> median {r['sats_median']},
         5th percentile {r['sats_p05']}</p>
    </div>

    <figure>
      <div class="panels" style="grid-template-columns:1fr">
        <div class="panel">
          <h4>{r['part']} &mdash; 15 g boost</h4>
          <p>82.5 km apogee, 1343 m/s peak &mdash; both limits exceeded</p>
{fig(f1)}
        </div>
        <div class="panel">
          <h4>{r['part']} &mdash; 3 g boost</h4>
          <p>same apogee reached slowly &mdash; the run that brackets the thresholds</p>
{fig(f2)}
        </div>
      </div>
      <figcaption>{r['part']}: the two standard profiles. Shaded bands mark where
        the injected trajectory exceeds <em>this receiver's own</em> measured
        thresholds &mdash; red for speed, amber for altitude &mdash; so they sit
        where its lock strip should change rather than at a constant 515 m/s and
        80 km. Read the lock strip against those bands, and the satellite bar
        underneath to confirm the receiver was still tracking.</figcaption>
    </figure>
{note}
''')
    return "".join(out)


def build(text_path: Path, title: str):
    """One page: (html, placeholders it used, unknown ones left in it, all of them)."""
    d = json.loads(DATA.read_text())
    text = text_path.read_text()
    # strip the editing instructions; keep any other comments the author wrote
    text = re.sub(r'<!--\s*=+\s*\n.*?=+\s*\n-->', '', text, flags=re.S)
    text = re.sub(r'<!--#blurb \w+-->.*?<!--/blurb-->', '', text, flags=re.S)
    text = re.sub(r'<!--#fignote \w+-->.*?<!--/fignote-->', '', text, flags=re.S)

    table = subprocess.run([sys.executable, str(HERE / "receiver_table.py"), "--html"],
                           capture_output=True, text=True, check=True).stdout.strip()
    rows = table.split("\n", 1)[1].rsplit("\n", 1)[0]
    fills = {
        "{{GATE_TABLE_ROWS}}": rows,
        "{{RECEIVER_SECTIONS}}": receiver_sections(d, TEXT.read_text()),
        "{{FIG_BLOCK_DIAGRAM}}": fig("rig_block_diagram.svg"),
        "{{FIG_M8T_ALTRAMP}}": fig("neo_m8t_t2_altramp.svg"),
        "{{FIG_DIP}}": fig("air530_dip_periodicity.svg"),
        "{{FIG_BOOST_RATE}}": fig("boost_rate.svg"),
        "{{FIG_BOOST_DOPPLER}}": fig("boost_doppler.svg"),
        "{{FIG_LC86_MODES}}": fig("lc86g_modes.svg"),
        "{{FIG_LC86_KNEE}}": fig("lc86g_knee.svg"),
    }
    used = {k for k in fills if k in text}
    for k, v in fills.items():
        text = text.replace(k, v)
    left = re.findall(r'\{\{[A-Z_]+\}\}', text)
    head = HEAD.read_text().replace("{{TITLE}}", title)
    return head + "\n" + text.strip() + "\n", used, left, set(fills)


def tag_balance(text: str):
    """Per-section counts of the container tags, so an edit cannot silently
    unbalance the page.

    Deleting a paragraph is easy; deleting the </div> that closed the block it
    lived in is easy too, and the result still looks like valid text in an
    editor. It shows up much later as a section swallowing everything after it.
    """
    import re as _re
    out = []
    for m in _re.finditer(r'<section>(.*?)</section>', text, _re.S):
        body = m.group(1)
        h = _re.search(r'<h2><span class="n">(\d+)</span>([^<]*)', body)
        name = f"{h.group(1)} {h.group(2).strip()}" if h else "(unnamed)"
        for tag in ("div", "figure", "table", "p"):
            o = len(_re.findall(rf'<{tag}[\s>]', body))
            c = body.count(f"</{tag}>")
            if o != c:
                out.append((name, tag, o, c))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true",
                    help="verify the inputs without writing either page")
    args = ap.parse_args()

    bad = False
    used_any, every = set(), set()
    built = []
    for text_path, out, title in PAGES:
        html, used, left, every = build(text_path, title)
        used_any |= used
        for name, tag, o, c in tag_balance(text_path.read_text()):
            print(f"  !! section '{name}': {o} <{tag}> but {c} </{tag}> "
                  f"-- {text_path.name} has an unclosed or stray tag")
            bad = True
        for k in left:
            print(f"  !! {k} in {text_path.name} is not a placeholder this script fills")
            bad = True
        if s := html.count("<!-- missing "):
            print(f"  !! {s} figure(s) referenced in {text_path.name} but not found "
                  f"in results/figures/")
        built.append((text_path, out, html))
    for k in sorted(every - used_any):
        print(f"  !! {k} is on neither page -- that content will not appear")
        bad = True

    for text_path, out, html in built:
        if args.check:
            print(f"  {text_path.name} {text_path.stat().st_size} bytes")
        else:
            out.write_text(html)
            print(f"  {out.name} rebuilt: {len(html)} bytes")
    if args.check:
        print(f"  {len(blurbs(TEXT.read_text()))} receiver blurbs")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
