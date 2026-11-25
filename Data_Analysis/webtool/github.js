/* Public flight logs, kept in each flyer's own GitHub repository.

   The site is static and has no server, so a public log needs somewhere that
   anyone can read and only its owner can write. A public repository named
   tinkerrocket-flight-log is that: the flyer owns it, GitHub serves it to
   anyone with CORS, and deleting the repository takes the log down. Nothing
   about it needs an account with us.

   Reading needs no sign-in. A log's index comes from raw.githubusercontent.com
   and the list of flyers from GitHub's repository search, both open to an
   unauthenticated browser. Writing uses a fine-grained token the flyer makes
   for that one repository, pasted once and kept in this browser's
   localStorage — it never goes anywhere but api.github.com.

   For development the endpoints can be pointed at a local stand-in:
     localStorage.setItem("tr.dev.github", JSON.stringify({api, raw}))
   (Data_Analysis/webtool/dev/mock_github.py is one.) */

import { REPO_NAME, INDEX_PATH } from "./flightlog.js";

const TOKEN_KEY = "tr.github";

export function endpoints() {
  let dev = null;
  try { dev = JSON.parse(localStorage.getItem("tr.dev.github") || "null"); } catch (err) { /* none */ }
  return {
    api: (dev && dev.api) || "https://api.github.com",
    raw: (dev && dev.raw) || "https://raw.githubusercontent.com",
    web: (dev && dev.web) || "https://github.com",
    pages: (dev && dev.pages) || null,
  };
}

// ---------- setup links ----------

export function newRepoUrl(owner) {
  const p = new URLSearchParams({
    name: REPO_NAME,
    description: "My TinkerRocket flight log",
    visibility: "public",
  });
  if (owner) p.set("owner", owner);
  return `https://github.com/new?${p}`;
}

/* GitHub pre-fills a fine-grained token form from these parameters. The
   repository itself cannot be pre-selected; the flyer picks "Only select
   repositories" and this one. Contents write is the whole grant — the token
   cannot touch any other repository, settings, or the account. */
export function newTokenUrl(owner) {
  const p = new URLSearchParams({
    name: "TinkerRocket flight log",
    description: `Lets the TinkerRocket flight analysis page publish to ${REPO_NAME}.`,
    expires_in: "366",
    contents: "write",
  });
  if (owner) p.set("target_name", owner);
  return `https://github.com/settings/personal-access-tokens/new?${p}`;
}

// ---------- the saved connection ----------

export function savedConnection() {
  try {
    const c = JSON.parse(localStorage.getItem(TOKEN_KEY) || "null");
    return c && c.owner && c.token ? c : null;
  } catch (err) {
    return null;
  }
}

export function saveConnection(owner, token) {
  localStorage.setItem(TOKEN_KEY, JSON.stringify({ owner, token }));
}

export function forgetConnection() {
  localStorage.removeItem(TOKEN_KEY);
}

// ---------- public reads ----------

export class NotFound extends Error {}

/* Every flyer with a public log. GitHub's search allows ten of these a minute
   without a token, which is plenty for a page that runs it once per visit. */
export async function searchPilots() {
  const { api } = endpoints();
  const q = encodeURIComponent(`${REPO_NAME} in:name fork:false`);
  const resp = await fetch(`${api}/search/repositories?q=${q}&sort=updated&per_page=100`, {
    headers: { Accept: "application/vnd.github+json" },
  });
  if (!resp.ok) throw new Error(`GitHub search failed (HTTP ${resp.status})`);
  const body = await resp.json();
  return (body.items || [])
    .filter((r) => r.name.toLowerCase() === REPO_NAME && !r.private)
    .map((r) => ({
      login: r.owner.login,
      avatar: r.owner.avatar_url,
      updated: r.pushed_at || r.updated_at,
      repoUrl: r.html_url,
    }));
}

export function rawUrl(owner, path) {
  const { raw } = endpoints();
  const enc = path.split("/").map(encodeURIComponent).join("/");
  return `${raw}/${encodeURIComponent(owner)}/${REPO_NAME}/HEAD/${enc}`;
}

export function repoUrl(owner) {
  return `${endpoints().web}/${encodeURIComponent(owner)}/${REPO_NAME}`;
}

/* Where GitHub Pages serves the flyer's repository.

   A public report is shown from here and nowhere else. It is someone else's
   HTML, with scripts, and rendered same-origin with this page it could read
   everything this page keeps — the saved flights, and the viewer's own token.
   <login>.github.io is a different site from ours (github.io is on the public
   suffix list), so a report there runs walled off from this page, and it has
   a real https address, which is what the 3D globe needs.

   Computed from the login, never read from the log: an index naming its own
   viewer URL could point the frame anywhere. */
export function pagesUrl(owner, path = "") {
  const dev = endpoints().pages;
  const base = dev
    ? dev.replace("{owner}", encodeURIComponent(owner))
    : `https://${owner.toLowerCase()}.github.io/${REPO_NAME}/`;
  return base + path.split("/").map(encodeURIComponent).join("/");
}

export function pagesSettingsUrl(owner) {
  return `https://github.com/${encodeURIComponent(owner)}/${REPO_NAME}/settings/pages`;
}

/* Whether Pages is serving the repository yet. Pages answers cross-origin
   reads, so this is a plain fetch — of the index once a flight is published,
   or of the site root before then. A repository made with a README and Pages
   switched on serves that README as its root page until the first publish
   adds .nojekyll; after it, the root has no page and the index is there. */
export async function pagesLive(owner) {
  const ok = (path) => fetch(pagesUrl(owner, path), { cache: "no-store" })
    .then((r) => (r.ok ? true : Promise.reject(new Error(String(r.status)))));
  try {
    return await Promise.any([ok(INDEX_PATH), ok("")]);
  } catch (err) {
    return false;
  }
}

/* Whether a public repository by the log's name exists for this login,
   asked without a token (GitHub allows sixty such reads an hour). "missing"
   also covers a private one, which reads as absent to everyone but its
   owner — and which no one else could read the log from either. */
export async function repoState(owner) {
  try {
    const resp = await fetch(`${endpoints().api}/repos/${encodeURIComponent(owner)}/${REPO_NAME}`, {
      headers: { Accept: "application/vnd.github+json" },
    });
    if (resp.status === 404) return "missing";
    if (!resp.ok) return "unknown";
    const repo = await resp.json();
    return repo.private ? "missing" : "ok";
  } catch (err) {
    return "unknown";
  }
}

/* A flyer's public index. raw.githubusercontent.com caches for five minutes,
   so a flight published a moment ago may not show here yet; the owner's own
   view reads through the API instead and is always current. */
export async function readPublicIndex(owner) {
  const resp = await fetch(rawUrl(owner, INDEX_PATH), { cache: "no-cache" });
  if (resp.status === 404) throw new NotFound(`${owner} has no public flight log`);
  if (!resp.ok) throw new Error(`Reading ${owner}'s log failed (HTTP ${resp.status})`);
  return resp.json();
}

export async function readPublicBlob(owner, path) {
  const resp = await fetch(rawUrl(owner, path));
  if (resp.status === 404) throw new NotFound(`${path} is not in ${owner}'s log`);
  if (!resp.ok) throw new Error(`Reading ${path} failed (HTTP ${resp.status})`);
  return resp.blob();
}

// ---------- the owner's writes ----------

export class GitHubError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
  }
}

export class Publisher {
  constructor(owner, token) {
    this.owner = owner;
    this.token = token;
    this.base = `${endpoints().api}/repos/${encodeURIComponent(owner)}/${REPO_NAME}`;
  }

  async call(method, url, body, accept = "application/vnd.github+json") {
    const resp = await fetch(url.startsWith("http") ? url : this.base + url, {
      method,
      cache: "no-store",
      headers: {
        Accept: accept,
        Authorization: `Bearer ${this.token}`,
        "X-GitHub-Api-Version": "2022-11-28",
        ...(body ? { "Content-Type": "application/json" } : {}),
      },
      body: body ? JSON.stringify(body) : undefined,
    });
    if (!resp.ok) {
      let message = `HTTP ${resp.status}`;
      try { message = (await resp.json()).message || message; } catch (err) { /* not JSON */ }
      throw new GitHubError(resp.status, message);
    }
    return accept.includes("raw") ? resp : resp.status === 204 ? null : resp.json();
  }

  /* Who the token belongs to and whether it can write the log. Throws with
     a message meant for the flyer, because each failure has a different fix. */
  async check() {
    let user;
    try {
      user = await this.call("GET", `${endpoints().api}/user`);
    } catch (err) {
      if (err.status === 401) throw new Error("GitHub did not accept that token. Check it was copied whole and has not expired.");
      throw err;
    }
    if (user.login.toLowerCase() !== this.owner.toLowerCase()) {
      throw new Error(`That token belongs to ${user.login}, not ${this.owner}.`);
    }
    let repo;
    try {
      repo = await this.call("GET", "");
    } catch (err) {
      if (err.status === 404) {
        throw new Error(`No repository ${this.owner}/${REPO_NAME} that this token can see. Create it (step 1), and give the token access to it.`);
      }
      throw err;
    }
    if (repo.private) throw new Error(`${this.owner}/${REPO_NAME} is private, so nobody else could read the log. Make it public on GitHub.`);
    if (repo.permissions && repo.permissions.push === false) {
      throw new Error("The token can read the repository but not write to it. Give it Contents: Read and write.");
    }
    this.branch = repo.default_branch || "main";
    return { login: user.login, avatar: user.avatar_url, branch: this.branch };
  }

  /* The index straight from the API, never a cached copy: it is about to be
     edited, and an edit made against a stale index would drop a flight. */
  async readIndex() {
    try {
      const resp = await this.call("GET", `/contents/${INDEX_PATH}`, null, "application/vnd.github.raw+json");
      return await resp.json();
    } catch (err) {
      if (err.status === 404) return null;
      throw err;
    }
  }

  /* One commit carrying every change: {path: Blob | string | null}, null
     deleting the path. The Git Data API rather than the contents endpoint,
     because a flight is several files and a reader should never see the index
     pointing at a report that is not there yet. */
  async commit(changes, message) {
    if (!this.branch) await this.check();
    let head = await this.headSha();
    if (!head) {
      // A repository created without a README has no commit to build on, and
      // the Git Data API refuses to work on one. The contents endpoint does
      // not, so it makes the first commit.
      await this.call("PUT", "/contents/README.md", {
        message: "Start the flight log",
        content: await base64(new Blob(["# TinkerRocket flight log\n"])),
        branch: this.branch,
      });
      head = await this.headSha();
    }
    const base = await this.call("GET", `/git/commits/${head}`);
    const tree = [];
    for (const [path, content] of Object.entries(changes)) {
      if (content == null) {
        tree.push({ path, mode: "100644", type: "blob", sha: null });
        continue;
      }
      const blob = await this.call("POST", "/git/blobs", {
        content: await base64(typeof content === "string" ? new Blob([content]) : content),
        encoding: "base64",
      });
      tree.push({ path, mode: "100644", type: "blob", sha: blob.sha });
    }
    const newTree = await this.call("POST", "/git/trees", { base_tree: base.tree.sha, tree });
    const commit = await this.call("POST", "/git/commits", {
      message, tree: newTree.sha, parents: [head],
    });
    // Not forced: if another browser published in the meantime this fails
    // rather than discarding its flight, and the caller starts over from the
    // index as it now stands.
    await this.call("PATCH", `/git/refs/heads/${encodeURIComponent(this.branch)}`, { sha: commit.sha });
    return commit.sha;
  }

  async headSha() {
    try {
      const ref = await this.call("GET", `/git/ref/heads/${encodeURIComponent(this.branch)}`);
      return ref.object.sha;
    } catch (err) {
      if (err.status === 404 || err.status === 409) return null; // empty repository
      throw err;
    }
  }
}

/* Read the index, change it, commit it with its files — and if another
   browser moved the branch underneath, read it again and redo the change.
   `edit(index)` returns {index, changes, message}. */
export async function updateLog(publisher, edit, attempts = 3) {
  for (let i = 0; ; i++) {
    const current = await publisher.readIndex();
    const { index, changes, message } = await edit(current);
    // The first publish also turns Jekyll off. Pages would otherwise run every
    // report through it, which is slow on a 9 MB file and pointless on one
    // that is already the finished page.
    const first = current ? {} : { ".nojekyll": "" };
    try {
      await publisher.commit({ ...first, ...changes, [INDEX_PATH]: JSON.stringify(index, null, 2) + "\n" }, message);
      return index;
    } catch (err) {
      const raced = err.status === 422 || err.status === 409;
      if (!raced || i + 1 >= attempts) throw err;
    }
  }
}

function base64(blob) {
  return new Promise((resolve, reject) => {
    const r = new FileReader();
    r.onload = () => resolve(String(r.result).replace(/^data:[^,]*,/, ""));
    r.onerror = () => reject(r.error);
    r.readAsDataURL(blob);
  });
}
