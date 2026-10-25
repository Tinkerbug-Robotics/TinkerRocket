#!/usr/bin/env python3
"""A local stand-in for the parts of GitHub the flight log uses.

Exercises publishing and browsing without a real account or token: the page's
GitHub client is pointed here with one localStorage key, and every call it
makes — the API, raw.githubusercontent.com, and GitHub Pages — is answered
from memory.

    python3 dev/mock_github.py                  # API + raw on :8651, Pages on :8652
    # then, in the tool's console:
    localStorage.setItem("tr.dev.github", JSON.stringify({
      api: "http://127.0.0.1:8651", raw: "http://127.0.0.1:8651/raw",
      web: "http://127.0.0.1:8651/web", pages: "http://127.0.0.1:8652/{owner}/"}))

Tokens are "test-token-<login>". Pages runs on its own port so a public report
gets an origin of its own, as it does on <login>.github.io. One flyer, "alice",
is seeded with a published flight whose report tries to read the page that
frames it — it should fail, and says so on screen.

Not a faithful GitHub: one branch, flat trees, no auth beyond the token's
name. Enough to drive the client through every path it takes.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import re
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

REPO = "tinkerrocket-flight-log"

blobs: dict[str, bytes] = {}
trees: dict[str, dict[str, str]] = {}      # tree sha -> {path: blob sha}
commits: dict[str, dict] = {}              # commit sha -> {tree, parents, message}
heads: dict[str, str | None] = {}          # owner -> head commit sha (None: empty repo)
lock = threading.Lock()
pages_on = True


def sha(kind: str, data: bytes) -> str:
    return hashlib.sha1(kind.encode() + b"\0" + data).hexdigest()


def put_blob(data: bytes) -> str:
    s = sha("blob", data)
    blobs[s] = data
    return s


def put_tree(files: dict[str, str]) -> str:
    s = sha("tree", json.dumps(sorted(files.items())).encode())
    trees[s] = dict(files)
    return s


def put_commit(tree: str, parents: list[str], message: str) -> str:
    s = sha("commit", json.dumps([tree, parents, message, len(commits)]).encode())
    commits[s] = {"tree": tree, "parents": parents, "message": message}
    return s


def head_files(owner: str) -> dict[str, str] | None:
    h = heads.get(owner)
    return trees[commits[h]["tree"]] if h else None


def seed_alice() -> None:
    report = b"""<!doctype html><meta charset=utf-8><title>alice report</title>
<body style="font:16px system-ui;padding:2rem">
<h1>Alice's seeded report</h1>
<p>Origin: <b id=o></b></p>
<p>Reading the framing page's storage: <b id=r></b></p>
<script>
document.getElementById("o").textContent = location.origin;
try {
  var t = parent.localStorage.getItem("tr.github");
  document.getElementById("r").textContent = "READ IT: " + t;
  document.getElementById("r").style.color = "red";
} catch (e) {
  document.getElementById("r").textContent = "blocked (" + e.name + ")";
  document.getElementById("r").style.color = "green";
}
</script>"""
    index = {
        "schema": 1, "pilot": {"login": "alice"}, "updated": "2026-10-20T12:00:00Z",
        "flights": [{
            "schema": 1, "id": "20261018-151200-zebra", "rocket": "Zebra", "motor": "G74-6",
            "location": {"name": "Snow Ranch, CA", "source": "manual", "lat": 38.2501, "lon": -121.0102, "alt_m": 20},
            "date": {"utc": "2026-10-18T15:12:00Z", "source": "gnss", "local": "2026-10-18 08:12", "tz": "America/Los_Angeles"},
            "computer": {"board": "B2", "board_asserted": False, "firmware": "41b2ab92", "firmware_dirty": False},
            "results": {"apogee_m": 301.2, "max_speed_mps": 77.5, "max_accel_g": 14.2, "flight_time_s": 61.0, "landing_distance_m": 140},
            "outcome": "partial", "notes": "Chute fouled <b>slightly</b>; <img src=x onerror=alert(1)> should show as text.",
            "files": {"report": "report.html", "data": ["flight_20261018_151200.bin", "../escape.bin"]},
            "source_file": "flight_20261018_151200.bin",
            "created": "2026-10-20T12:00:00Z", "updated": "2026-10-20T12:00:00Z",
        }, {
            "id": "../../etc", "rocket": "should never show",
        }],
    }
    files = {
        "README.md": put_blob(b"# alice's TinkerRocket flight log\n"),
        "flightlog.json": put_blob(json.dumps(index, indent=2).encode()),
        "flights/20261018-151200-zebra/report.html": put_blob(report),
        "flights/20261018-151200-zebra/data/flight_20261018_151200.bin": put_blob(b"\x00" * 1024),
    }
    heads["alice"] = put_commit(put_tree(files), [], "seed")


class Handler(BaseHTTPRequestHandler):
    server_version = "mock-github"

    def log_message(self, fmt, *args):  # quieter: method, path, status
        print(f"{self.command} {self.path[:100]} -> {args[1] if len(args) > 1 else ''}", flush=True)

    def cors(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Authorization, Content-Type, X-GitHub-Api-Version, Accept")
        self.send_header("Access-Control-Allow-Methods", "GET, HEAD, POST, PATCH, PUT, DELETE, OPTIONS")

    def reply(self, status: int, body=None, ctype="application/json"):
        data = b"" if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
        self.send_response(status)
        self.cors()
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(data)

    def body(self):
        n = int(self.headers.get("Content-Length") or 0)
        return json.loads(self.rfile.read(n) or b"{}")

    def login(self):
        m = re.match(r"Bearer test-token-([A-Za-z0-9-]+)$", self.headers.get("Authorization", ""))
        return m.group(1) if m else None

    def do_OPTIONS(self):
        self.reply(204)

    def do_HEAD(self):
        self.do_GET()

    # ---- Pages (its own port, its own origin) ----
    def pages(self):
        if not pages_on:
            return self.reply(404, b"Pages off", "text/plain")
        m = re.match(r"^/([^/]+)/(.*)$", unquote(urlparse(self.path).path))
        files = head_files(m.group(1)) if m else None
        if not files or m.group(2) not in files:
            return self.reply(404, b"Not found", "text/plain")
        path = m.group(2)
        ctype = "text/html; charset=utf-8" if path.endswith(".html") else (
            "application/json" if path.endswith(".json") else "application/octet-stream")
        self.reply(200, blobs[files[path]], ctype)

    def do_GET(self):
        if self.server.server_port == self.server.pages_port:
            return self.pages()
        url = urlparse(self.path)
        path = unquote(url.path)
        with lock:
            if path == "/user":
                who = self.login()
                return self.reply(200, {"login": who, "avatar_url": ""}) if who else self.reply(401, {"message": "Bad credentials"})
            if path == "/search/repositories":
                items = [{"name": REPO, "private": False, "owner": {"login": o, "avatar_url": ""},
                          "pushed_at": "2026-10-20T12:00:00Z", "html_url": f"http://127.0.0.1:8651/web/{o}/{REPO}"}
                         for o, h in heads.items() if h]
                return self.reply(200, {"total_count": len(items), "items": items})
            m = re.match(rf"^/raw/([^/]+)/{REPO}/HEAD/(.+)$", path)
            if m:
                files = head_files(m.group(1))
                if not files or m.group(2) not in files:
                    return self.reply(404, b"404: Not Found", "text/plain")
                return self.reply(200, blobs[files[m.group(2)]], "text/plain; charset=utf-8")
            m = re.match(rf"^/repos/([^/]+)/{REPO}(/.*)?$", path)
            if not m or m.group(1) not in heads:
                return self.reply(404, {"message": "Not Found"})
            owner, rest = m.group(1), m.group(2) or ""
            if rest == "":
                return self.reply(200, {"name": REPO, "private": False, "default_branch": "main",
                                        "permissions": {"push": self.login() == owner}})
            if rest == "/git/ref/heads/main":
                if not heads[owner]:
                    return self.reply(409, {"message": "Git Repository is empty."})
                return self.reply(200, {"object": {"sha": heads[owner]}})
            m2 = re.match(r"^/git/commits/([0-9a-f]+)$", rest)
            if m2 and m2.group(1) in commits:
                return self.reply(200, {"sha": m2.group(1), "tree": {"sha": commits[m2.group(1)]["tree"]}})
            m2 = re.match(r"^/contents/(.+)$", rest)
            if m2:
                files = head_files(owner)
                if not files or m2.group(1) not in files:
                    return self.reply(404, {"message": "Not Found"})
                return self.reply(200, blobs[files[m2.group(1)]], "application/octet-stream")
            return self.reply(404, {"message": "Not Found"})

    def write_guard(self):
        m = re.match(rf"^/repos/([^/]+)/{REPO}(/.*)$", unquote(urlparse(self.path).path))
        if not m or m.group(1) not in heads:
            self.reply(404, {"message": "Not Found"})
            return None
        if self.login() != m.group(1):
            self.reply(403, {"message": "Resource not accessible by personal access token"})
            return None
        return m.group(1), m.group(2)

    def do_PUT(self):
        with lock:
            g = self.write_guard()
            if not g:
                return
            owner, rest = g
            m = re.match(r"^/contents/(.+)$", rest)
            b = self.body()
            files = dict(head_files(owner) or {})
            files[m.group(1)] = put_blob(base64.b64decode(b["content"]))
            parents = [heads[owner]] if heads[owner] else []
            heads[owner] = put_commit(put_tree(files), parents, b["message"])
            self.reply(201, {"commit": {"sha": heads[owner]}})

    def do_POST(self):
        with lock:
            g = self.write_guard()
            if not g:
                return
            owner, rest = g
            b = self.body()
            if rest == "/git/blobs":
                return self.reply(201, {"sha": put_blob(base64.b64decode(b["content"]))})
            if rest == "/git/trees":
                files = dict(trees[b["base_tree"]])
                for e in b["tree"]:
                    if e["sha"] is None:
                        files.pop(e["path"], None)
                    else:
                        assert e["sha"] in blobs, e
                        files[e["path"]] = e["sha"]
                return self.reply(201, {"sha": put_tree(files)})
            if rest == "/git/commits":
                return self.reply(201, {"sha": put_commit(b["tree"], b["parents"], b["message"])})
            self.reply(404, {"message": "Not Found"})

    def do_PATCH(self):
        with lock:
            g = self.write_guard()
            if not g:
                return
            owner, rest = g
            b = self.body()
            if rest != "/git/refs/heads/main":
                return self.reply(404, {"message": "Not Found"})
            if commits[b["sha"]]["parents"] != [heads[owner]]:
                return self.reply(422, {"message": "Update is not a fast forward"})
            heads[owner] = b["sha"]
            print(f"  {owner}: {commits[b['sha']]['message']!r} -> {sorted(head_files(owner))}", flush=True)
            self.reply(200, {"object": {"sha": b["sha"]}})


def main() -> None:
    global pages_on
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8651)
    ap.add_argument("--pages-port", type=int, default=8652)
    ap.add_argument("--repo", action="append", default=["flyer"],
                    help="a login whose (empty) repository exists; repeatable")
    ap.add_argument("--no-pages", action="store_true", help="answer every Pages request 404")
    args = ap.parse_args()
    pages_on = not args.no_pages
    for login in args.repo:
        heads.setdefault(login, None)
    seed_alice()
    servers = []
    for port in (args.port, args.pages_port):
        srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
        srv.pages_port = args.pages_port
        servers.append(srv)
        threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f"mock GitHub on :{args.port} (API, /raw), Pages on :{args.pages_port}; repos: {sorted(heads)}", flush=True)
    threading.Event().wait()


if __name__ == "__main__":
    main()
