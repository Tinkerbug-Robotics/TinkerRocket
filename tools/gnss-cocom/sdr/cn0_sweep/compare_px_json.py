#!/usr/bin/env python3
"""Regenerated boost-chart JSON against the published one: every run's per-satellite lock/delivery results (the
fields the report's tables and comparison charts use) must be identical; only the new error fields may differ.
    compare_px_json.py PREV.json NEW.json"""
import json
import sys

prev, new = (json.load(open(f)) for f in sys.argv[1:3])
KEYS = ("lost", "t", "rate", "lock_lost", "lock_t", "lock_rate", "lock_end", "raw_end", "el")
diffs = 0
for a, b in zip(prev, new):
    if a["label"] != b["label"] or a.get("fix_end") != b.get("fix_end") or a["pad_cn0"] != b["pad_cn0"]:
        print("run header differs:", a["label"], b["label"])
        diffs += 1
    ra = {(q["sys"], q["prn"]): q for q in a["recs"]}
    rb = {(q["sys"], q["prn"]): q for q in b["recs"]}
    if set(ra) != set(rb):
        print(a["label"], "satellite sets differ")
        diffs += 1
    for k in ra:
        for f in KEYS:
            if k in rb and ra[k].get(f) != rb[k].get(f):
                print(a["label"], k, f, ra[k].get(f), "->", rb[k].get(f))
                diffs += 1
        if k in rb and [p[:5] for p in ra[k]["series"]] != [p[:5] for p in rb[k]["series"]]:
            print(a["label"], k, "series differ")
            diffs += 1
print(f"{len(prev)} runs compared, {diffs} differences")
