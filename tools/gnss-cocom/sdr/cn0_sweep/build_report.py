#!/usr/bin/env python3
"""The C/N0 report page in four parts (intro, PX1105R, NEO-M8T, conclusions), from report_text.json: EYEBROW, LEDE and
PARTS, each part {id, title, blocks}. Blocks render in order; types: h3 / h4 (text, optional id), p, lede (a part's
opening paragraph), findings, notes, key (GPS/BeiDou/Galileo colours), table {head, rows}, figure {file, name, alt,
caption}, levels {flights: [[name, boost JSON]], sky: {G|C|E: [median, above 60] or null}}, drops (a boost_traces_wide.py
JSON), flight_table (sweep_table.py JSON), runs ([[label, png], ...] or {group, items, caption}: a tab set per group).
Writes page/index.html (template
report_template.html) and copies every image from figures/ beside it (JSON inputs from data/) under the names the page uses.
    build_report.py"""
import html
import json
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA, FIG, OUT = HERE / "data", HERE / "figures", HERE / "page"
OUT.mkdir(exist_ok=True)
NAME = {"G": "GPS", "C": "BeiDou", "E": "Galileo"}
e = html.escape
show_rules = []


def levels(b):
    sky = b["sky"]
    head = ("<thead><tr><th>Run</th>" + "".join(
        f"<th class=num>{NAME[c]}</th><th class=num>vs median</th><th class=num>vs &gt;60&deg;</th>" for c in "GCE")
        + "</tr></thead>")
    rows = []
    for name, fn in b["flights"]:
        rows.append(f"<tr class=group><th colspan=10 scope=rowgroup>{e(name)}</th></tr>")
        for r in json.load(open(DATA / fn)):
            pc = r["pad_cn0"]
            cells = []
            for c in "GCE":
                if c not in pc:
                    cells.append("<td class=num>&ndash;</td><td class=num></td><td class=num></td>")
                elif sky.get(c):
                    cells.append(f"<td class=num>{pc[c]:.0f}</td><td class='num delta'>{pc[c] - sky[c][0]:+.0f}</td>"
                                 f"<td class='num delta'>{pc[c] - sky[c][1]:+.0f}</td>")
                else:
                    cells.append(f"<td class=num>{pc[c]:.0f}</td><td class='num delta'>&ndash;</td>"
                                 f"<td class='num delta'>&ndash;</td>")
            rows.append(f"<tr><th scope=row>{e(r['label'])}</th>{''.join(cells)}</tr>")
    return f"<div class=\"tablewrap\"><table>{head}<tbody>{''.join(rows)}</tbody></table></div>"


def drops(fn):
    out = []
    for r in json.load(open(DATA / fn)):
        first = True
        n_sys = sum(1 for c in "GCE" if any(q["sys"] == c for q in r["recs"]))
        for c in "GCE":
            qs = sorted((q for q in r["recs"] if q["sys"] == c), key=lambda q: (q["lock_lost"], q["lock_t"]))
            if not qs:
                continue
            lost = sorted((q for q in qs if q["lock_lost"]), key=lambda q: q["lock_t"])
            held = [q for q in qs if not q["lock_lost"]]
            chips = "".join(f"<span class='chip {c}{' back' if q['lock_end'] else ''}'>{c}{q['prn']:02d} "
                            f"<b>{q['lock_t']:.1f}</b> s <b>{q['lock_rate']:.0f}</b> Hz/s"
                            f"{' &#8634;' if q['lock_end'] else ''}</span>" for q in lost)
            hold = "".join(f"<span class='chip held {c}'>{c}{q['prn']:02d} {q['el']:.0f}&deg;</span>" for q in held)
            lockend = sum(q["lock_end"] for q in qs)
            rawheld = sum(q["raw_end"] for q in qs)
            head = ""
            if first:
                sub = f"<span class=sub>fix stops T+{r['fix_end']:.1f}</span>" if r.get("fix_end") is not None else ""
                head = f"<th scope=row rowspan={n_sys}>{e(r['label'])}{sub}</th>"
            out.append(f"<tr>{head}<td class='sys {c}'>{NAME[c]}</td><td class=num>{lockend}/{len(qs)}</td>"
                       f"<td class=num>{rawheld}/{len(qs)}</td><td class=chips>{chips or '<span class=none>none</span>'}"
                       f"</td><td class=chips>{hold or '<span class=none>none</span>'}</td></tr>")
            first = False
    head = ("<thead><tr><th>Run</th><th>System</th><th class=num>Locked at burnout</th><th class=num>Delivering</th>"
            "<th>First lock loss (s, Hz/s)</th><th>Never lost lock</th></tr></thead>")
    return f"<div class=\"tablewrap\"><table>{head}<tbody>{''.join(out)}</tbody></table></div>"


def flight_table(fn):
    t = json.load(open(DATA / fn))
    head = "".join(f"<th scope=col>{e(r['label'])}</th>" for r in t["rows"])
    body = "".join(f"<tr><th scope=row>{e(name)}</th>" + "".join(f"<td>{e(r[k])}</td>" for r in t["rows"]) + "</tr>"
                   for name, k in t["keys"])
    return f"<div class=\"tablewrap\"><table><thead><tr><th></th>{head}</tr></thead><tbody>{body}</tbody></table></div>"


def table(b):
    head = "".join(f"<th scope=col>{h}</th>" for h in b["head"])
    body = "".join("<tr>" + f"<th scope=row>{row[0]}</th>" + "".join(f"<td>{c}</td>" for c in row[1:]) + "</tr>"
                   for row in b["rows"])
    return f"<div class=\"tablewrap\"><table class=facts><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table></div>"


def figure(f):
    shutil.copy(FIG / f["file"], OUT / f["name"])
    return (f"<figure><div class=\"chart\"><img src=\"{f['name']}\" alt=\"{e(f['alt'])}\" loading=lazy></div>"
            f"<figcaption>{f['caption']}</figcaption></figure>")


RUN_CAPTION = ("speed and altitude (truth and the receiver's own), vertical acceleration, raw-measurement raster, "
               "satellites per epoch and fix, pseudorange and Doppler errors; boost close-up on the right.")


def runs(v):
    """A tab set of run plots: a list (group "run", images run0.png ...) or {group, items, caption}; each group is its
    own radio set, images <group><i>.png."""
    group, items, cap = ("run", v, RUN_CAPTION) if isinstance(v, list) else (v["group"], v["items"],
                                                                            v.get("caption", RUN_CAPTION))
    tabs, panes = [], []
    for i, (label, png) in enumerate(items):
        name = f"{group}{i}.png"
        shutil.copy(FIG / png, OUT / name)
        tabs.append(f"<input type=radio name={group} id={group}{i} {'checked' if i == 0 else ''}>"
                    f"<label for={group}{i}>{e(label)}</label>")
        panes.append(f"<figure class='runfig {group}-{i}'><img src='{name}' alt='Full-run plot, {e(label)}' loading=lazy>"
                     f"<figcaption>{e(label)}: {cap}</figcaption></figure>")
        show_rules.append(f"#{group}{i}:checked ~ .panes .{group}-{i} {{ display: block; }}")
    return f"<div class=\"runs\">{''.join(tabs)}<div class=\"panes\">{''.join(panes)}</div></div>"


def block(b):
    kind = next(k for k in b if k != "id")
    v = b[kind]
    anchor = f" id=\"{b['id']}\"" if b.get("id") else ""
    if kind in ("h3", "h4"):
        return f"<{kind}{anchor}>{v}</{kind}>"
    if kind == "p":
        return f"<p>{v}</p>"
    if kind == "lede":
        return f"<p class=\"partlede\">{v}</p>"
    if kind == "findings":
        return "<ul class=\"findings\">" + "".join(f"<li>{x}</li>" for x in v) + "</ul>"
    if kind == "notes":
        return "<ul class=\"notes\">" + "".join(f"<li>{x}</li>" for x in v) + "</ul>"
    if kind == "key":
        return "<div class=\"key\"><span class=\"kg\">GPS</span><span class=\"kc\">BeiDou</span><span class=\"ke\">Galileo</span></div>"
    if kind == "table":
        return table(v)
    if kind == "figure":
        return figure(v)
    if kind == "levels":
        return levels(v)
    if kind == "drops":
        return drops(v)
    if kind == "flight_table":
        return flight_table(v)
    if kind == "runs":
        return runs(v)
    raise ValueError(f"unknown block {kind}")


text = json.loads((HERE / "report_text.json").read_text())
parts, toc = [], []
for n, part in enumerate(text["PARTS"], 1):
    body = "\n".join(block(b) for b in part["blocks"])
    parts.append(f"<section class=\"part\" id=\"{part['id']}\" aria-labelledby=\"h-{part['id']}\">\n"
                 f"<div class=\"partno\">Part {n}</div>\n<h2 id=\"h-{part['id']}\">{part['title']}</h2>\n{body}\n</section>")
    toc.append(f"<a href=\"#{part['id']}\"><span>{n}</span>{part['title']}</a>")
page = (HERE / "report_template.html").read_text()
for k, v in {"{{EYEBROW}}": text["EYEBROW"], "{{LEDE}}": text["LEDE"], "{{TOC}}": "\n".join(toc),
             "{{PARTS}}": "\n\n".join(parts), "{{SHOW_RULES}}": "\n".join(show_rules)}.items():
    page = page.replace(k, v)
assert "{{" not in page, "unfilled placeholder"
(OUT / "index.html").write_text(page)
print(f"wrote {OUT / 'index.html'} ({len(page) / 1e3:.0f} kB), {len(parts)} parts, {len(show_rules)} runs")
