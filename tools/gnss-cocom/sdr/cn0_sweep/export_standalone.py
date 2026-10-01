#!/usr/bin/env python3
"""One self-contained HTML file of the published report, for sending: page/index.html with every image inlined
as a data: URI, wrapped in a full document (doctype, charset, viewport -- the artifact host adds these itself), and the
footer's internal worktree/scratchpad pointer replaced by a plain credit. Fonts still come from Google Fonts when online
(system fallbacks offline).   export_standalone.py OUT.html"""
import base64
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PAGE = HERE / "page"
out = Path(sys.argv[1])
src = (PAGE / "index.html").read_text()


def inline(m):
    q, name = m.group(1), m.group(2)
    data = base64.b64encode((PAGE / name).read_bytes()).decode()
    return f"src={q}data:image/png;base64,{data}{q}"


body = re.sub(r"src=(['\"])([\w.-]+\.png)\1", inline, src)
left = re.findall(r"src=['\"](?!data:)[^'\"]+['\"]", body)
assert not left, f"not inlined: {left}"
body = re.sub(r"<footer>.*?</footer>", "<footer>Bench data from the TinkerRocket COCOM rig, 2026-09-30 and 10-01.</footer>",
              body, flags=re.S)
head_end = body.index("</style>") + len("</style>")
doc = ("<!doctype html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
       "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
       "<style>:root { color-scheme: light; } img { max-width: 100%; }</style>\n"
       + body[:head_end] + "\n</head>\n<body>\n" + body[head_end:].lstrip() + "\n</body>\n</html>\n")
out.write_text(doc)
print(f"wrote {out} ({len(doc) / 1e6:.1f} MB, {doc.count('data:image/png')} images inlined)")
