#!/usr/bin/env python3
"""Where the LC86G's tracking loops give up: each channel against its Doppler rate.

One axis, the Doppler rate each satellite actually presented through the burn --
measured, by the NEO-M8T's RXM-RAWX on the byte-identical scenario file
(results/doppler_ref_*.json) -- and one row per flight. Each dot is one of the
LC86G's channels, from its own RTCM MSM7 in Balloon mode:

    held   the channel tracked through the burn (a 1-2 s drop at ignition and
           back is still held -- it re-locked while the acceleration was on)
    lost   gone for the rest of the burn
    slip   held, but its carrier-phase lock counter reset at ignition

The satellites split cleanly at 13.5 g: every one at or below 128 Hz/s held,
every one at or above 171 Hz/s was lost. That is the knee.

    ./plot_lc86_knee.py            writes results/figures/lc86g_knee.svg
"""

from __future__ import annotations

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
RES = HERE / "results"

# From the LC86G's MSM7 (msm_channels.py on results/lc86g_balloon_*.log.gz) and
# its GSV elevations on the pad. Outcome per flight; elevation in degrees.
ELEV = {5: 28, 6: 31, 11: 70, 12: 9, 14: 17, 15: 20, 19: 12, 20: 27, 21: 38,
        22: 26, 24: 51, 29: 5, 30: 11}
OUTCOME = {
    "spaceshot": {12: "held", 19: "held", 29: "held", 30: "slip",
                  5: "lost", 6: "lost", 11: "lost", 14: "lost", 15: "lost",
                  20: "lost", 21: "lost", 22: "lost", 24: "lost"},
    "gentle_alt": {p: "held" for p in ELEV} | {11: "slip", 21: "slip", 24: "slip"},
}
ROWS = [("spaceshot", "13.5 g burn", "spaceshot"),
        ("gentle_alt", "2 g burn", "gentle_alt")]

W, H = 900, 268
PAD_L, PAD_R = 150, 30
X_MAX = 560.0
KNEE = (128.0, 171.0)             # last held, first lost at 13.5 g
FOUR_G_ZENITH = 206.0             # 4 g x 9.807 x 5.255 Hz per m/s


def x(v):
    return PAD_L + (W - PAD_L - PAD_R) * min(v, X_MAX) / X_MAX


def rates(scen):
    d = json.loads((RES / f"doppler_ref_{scen}.json").read_text())
    return {int(k.split(":")[1]): v["rate_hzs"] for k, v in d["sats"].items()}


def dot(cx, cy, outcome, tip):
    fill = {"held": "var(--fix, #0E7C66)", "slip": "var(--fix, #0E7C66)",
            "lost": "none"}[outcome]
    stroke = "var(--nolock, #9B3535)" if outcome == "lost" else "var(--surface, #FFFFFF)"
    out = [f'<g><title>{tip}</title>']
    # 24 px hit target, invisible, so the tooltip does not need a pixel-perfect hover
    out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="12" fill="transparent"/>')
    if outcome == "slip":
        out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="8.5" fill="none" '
                   f'stroke="var(--ink-2, #4A5261)" stroke-width="1.5"/>')
    out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="5" fill="{fill}" '
               f'stroke="{stroke}" stroke-width="2"/>')
    out.append('</g>')
    return "".join(out)


def build() -> str:
    o = [f'<svg viewBox="0 0 {W} {H}" xmlns="http://www.w3.org/2000/svg" role="img" '
         'aria-label="Each LC86G channel plotted at the Doppler rate its satellite '
         'presented through the burn. At 13.5 g every channel at or below 128 Hz '
         'per second held and every one at or above 171 held lost. At 2 g every '
         'channel held; the three steepest slipped carrier phase at ignition.">',
         '<style>.ax{stroke:var(--rule-strong,#C3CAD5);stroke-width:1}'
         '.lbl{font-family:var(--f-mono,monospace);font-size:10px;fill:var(--ink-3,#79808F);'
         'paint-order:stroke;stroke:var(--surface,#FFFFFF);stroke-width:3px}'
         '.row{font-family:var(--f-display,sans-serif);font-size:12px;font-weight:600;'
         'fill:var(--ink-2,#4A5261)}'
         '.note{font-family:var(--f-mono,monospace);font-size:10px;fill:var(--ink-2,#4A5261)}'
         '</style>']
    top, row_h = 26, 70
    bottom = top + row_h * len(ROWS)
    # knee band and the 4 g reference, behind everything
    o.append(f'<rect x="{x(KNEE[0]):.1f}" y="{top-8}" width="{x(KNEE[1])-x(KNEE[0]):.1f}" '
             f'height="{bottom-top+8}" fill="var(--shade-b, rgba(41,69,126,0.24))"/>')
    o.append(f'<text class="note" x="{(x(KNEE[0])+x(KNEE[1]))/2:.1f}" y="{top-12}" '
             f'text-anchor="middle">knee 128-171 Hz/s</text>')
    o.append(f'<line x1="{x(FOUR_G_ZENITH):.1f}" y1="{top-8}" x2="{x(FOUR_G_ZENITH):.1f}" '
             f'y2="{bottom}" stroke="var(--ink-3,#79808F)" stroke-width="1" '
             f'stroke-dasharray="3 3"/>')
    o.append(f'<text class="note" x="{x(FOUR_G_ZENITH)+5:.1f}" y="{top+2}">'
             f'rated 4 g, overhead: 206 Hz/s</text>')

    for i, (scen, label, _cap) in enumerate(ROWS):
        yc = top + row_h * i + row_h / 2
        o.append(f'<text class="row" x="{PAD_L-14}" y="{yc+4:.1f}" '
                 f'text-anchor="end">{label}</text>')
        o.append(f'<line class="ax" x1="{PAD_L}" y1="{yc:.1f}" x2="{W-PAD_R}" '
                 f'y2="{yc:.1f}"/>')
        r = rates(scen)
        placed = []                       # (x, lane) for collision avoidance
        for prn, outcome in sorted(OUTCOME[scen].items(),
                                   key=lambda kv: (r.get(kv[0]) or 1e9)):
            rate = r.get(prn)
            cx = x(rate) if rate is not None else x(X_MAX) - 6
            lane = 0
            while any(abs(cx - px) < 12 and pl == lane for px, pl in placed):
                lane = -lane if lane > 0 else -lane + 1
            placed.append((cx, lane))
            cy = yc + lane * 13
            rtxt = f"{rate:.0f} Hz/s" if rate is not None else "not measurable (steepest)"
            tip = (f"G{prn:02d} at {ELEV[prn]}&#176; elevation: {rtxt} -- "
                   + {"held": "held", "lost": "lost for the rest of the burn",
                      "slip": "held; carrier phase slipped at ignition"}[outcome])
            o.append(dot(cx, cy, outcome, tip))
            # Label only the channels that bound the knee, and the one off-scale.
            # Anchored away from each other: the pair sits either side of the
            # band, 43 Hz/s apart, and centred labels ran into one string.
            if scen == "spaceshot" and prn in (19, 14, 11):
                txt, dx, anchor = {19: ("G19 12&#176; held", -6, "end"),
                                   14: ("G14 17&#176; lost", 6, "start"),
                                   11: ("G11 70&#176;", 0, "end")}[prn]
                o.append(f'<text class="lbl" x="{cx+dx:.1f}" y="{cy+22:.1f}" '
                         f'text-anchor="{anchor}">{txt}</text>')

    # x axis
    o.append(f'<line class="ax" x1="{PAD_L}" y1="{bottom+6}" x2="{W-PAD_R}" y2="{bottom+6}"/>')
    for v in range(0, 501, 100):
        o.append(f'<text class="lbl" x="{x(v):.1f}" y="{bottom+20}" '
                 f'text-anchor="middle">{v}</text>')
    o.append(f'<text class="lbl" x="{x(X_MAX)-2:.1f}" y="{bottom+20}" '
             f'text-anchor="end">off scale</text>')
    o.append(f'<text class="lbl" x="{PAD_L}" y="{bottom+36}">measured Doppler rate '
             f'through the burn (Hz/s)</text>')

    # legend, stacked
    ly = bottom + 56
    for i, (outcome, label) in enumerate((
            ("held", "held through the burn"),
            ("lost", "lost for the rest of the burn"),
            ("slip", "held, carrier phase slipped at ignition"))):
        lx = PAD_L + i * 230
        o.append(dot(lx + 6, ly, outcome, label))
        o.append(f'<text class="lbl" x="{lx+20}" y="{ly+4}">{label}</text>')
    o.append("</svg>")
    return "\n".join(o)


def main() -> int:
    out = RES / "figures" / "lc86g_knee.svg"
    out.write_text(build() + "\n")
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
