/* The flight log's views: the entry form after a report, My flights, the
   public-log set-up, and browsing other flyers' logs.

   Everything is built with createElement and textContent. A public log is
   another person's JSON, and nothing in it may become markup here. */

import * as store from "./store.js";
import * as gh from "./github.js";
import {
  OUTCOMES, REPO_NAME, applyDetails, cleanId, cleanLogin, computerText, emptyIndex,
  flightDir, flightPaths, fmtValue, localStamp, locationSuggestions, motorSuggestions, newEntry,
  parseMotor, placeName, publicEntry, readme, removeFromIndex, sortEntries, toCSV, upsertIndex,
} from "./flightlog.js";
import { showLocalReport } from "./reports.js";

// Where a published README sends people. Fixed rather than taken from
// location, so a log published while testing on localhost still points at the
// real tool.
const PUBLIC_SITE = "https://tinkerbug-robotics.github.io/TinkerRocket/";

const el = (id) => document.getElementById(id);

function h(tag, attrs, ...kids) {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs || {})) {
    if (v == null || v === false) continue;
    if (k === "class") n.className = v;
    else if (k.startsWith("on") && typeof v === "function") n.addEventListener(k.slice(2), v);
    else if (v === true) n.setAttribute(k, "");
    else n.setAttribute(k, v);
  }
  for (const kid of kids.flat(Infinity)) {
    if (kid == null || kid === false) continue;
    n.append(kid instanceof Node ? kid : String(kid));
  }
  return n;
}

function external(href, text) {
  return h("a", { href, target: "_blank", rel: "noopener" }, text);
}

// ---------- units ----------

function imperial() {
  try { return localStorage.getItem("tr-units") === "imperial"; } catch (err) { return false; }
}

/* The report's own metric/imperial switch, same key, so a flyer who reads
   reports in feet reads the log in feet too. */
function fillUnitToggles(root, rerender) {
  for (const box of root.querySelectorAll("[data-units]")) {
    box.replaceChildren(
      ...[["metric", "m"], ["imperial", "ft"]].map(([key, label]) => h("button", {
        type: "button",
        class: (key === "imperial") === imperial() ? "on" : "",
        onclick: () => {
          try { localStorage.setItem("tr-units", key); } catch (err) { /* private window */ }
          rerender();
        },
      }, label))
    );
  }
}

function dateText(e) {
  const d = e.date || {};
  if (d.local) return d.local.slice(0, 10);
  if (d.utc) return d.utc.slice(0, 10);
  return "";
}

function outcomeChip(key) {
  const o = OUTCOMES.find((x) => x.key === key);
  return o ? h("span", { class: `outcome ${o.key}` }, o.label) : "";
}

// ---------- tables ----------

function logTable(entries, { href, pilot = null, publicCol = false }) {
  if (!entries.length) return h("p", { class: "empty" }, "No flights yet.");
  const imp = imperial();
  const head = h("tr", null,
    h("th", null, "Date"),
    pilot ? h("th", null, "Flyer") : null,
    h("th", null, "Rocket"),
    h("th", null, "Motor"),
    h("th", null, "Location"),
    h("th", null, "Computer"),
    h("th", { class: "num" }, "Apogee"),
    h("th", { class: "num" }, "Max speed"),
    h("th", null, "Outcome"),
    publicCol ? h("th", null, "Public") : null,
  );
  const rows = sortEntries(entries).map((e) => {
    const link = href(e);
    const tr = h("tr", { onclick: (ev) => { if (!ev.target.closest("a")) location.hash = link; } },
      h("td", { class: "nowrap" }, h("a", { href: link }, dateText(e) || "—")),
      pilot ? h("td", null, pilot(e)) : null,
      h("td", null,
        h("div", { class: "rocket" }, e.rocket || ""),
        e.notes ? h("div", { class: "note", title: e.notes }, e.notes) : null),
      h("td", { class: "nowrap" }, e.motor || ""),
      h("td", { class: "place" }, (e.location && e.location.name) || ""),
      h("td", { class: "nowrap" }, computerText(e.computer)),
      h("td", { class: "num" }, fmtValue(e.results && e.results.apogee_m, "m", imp)),
      h("td", { class: "num" }, fmtValue(e.results && e.results.max_speed_mps, "m/s", imp)),
      h("td", null, outcomeChip(e.outcome)),
      publicCol ? h("td", null, e.published
        ? h("span", { class: "pub on" }, "✓ public")
        : h("span", { class: "pub" }, "—")) : null,
    );
    return tr;
  });
  return h("div", { class: "table-wrap" },
    h("table", { class: "log" }, h("thead", null, head), h("tbody", null, rows)));
}

// ---------- location ----------

async function reverseGeocode(pad) {
  // Nominatim's usage policy: an identifying Referer (the browser sends ours),
  // and at most one request a second — one per report is far inside that.
  const url = new URL("https://nominatim.openstreetmap.org/reverse");
  url.search = new URLSearchParams({
    format: "jsonv2", lat: String(pad.lat), lon: String(pad.lon), zoom: "14", addressdetails: "1",
    "accept-language": navigator.language || "en",
  });
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), 8000);
  try {
    const resp = await fetch(url, { signal: ctl.signal });
    if (!resp.ok) return null;
    return placeName(await resp.json()) || null;
  } catch (err) {
    return null;
  } finally {
    clearTimeout(timer);
  }
}

function osmLink(pad) {
  const lat = pad.lat.toFixed(5);
  const lon = pad.lon.toFixed(5);
  return `https://www.openstreetmap.org/?mlat=${lat}&mlon=${lon}#map=15/${lat}/${lon}`;
}

// ---------- the entry form ----------

/* One form for a new flight and for editing a saved one. `facts` is what the
   log said (read-only here); `values` the flyer's columns so far. */
function entryForm(form, { values, facts, entries, submitLabel, extras, onSubmit }) {
  const v = values || {};
  const f = facts || {};
  const state = { locationSource: v.locationSource || null, touched: !!v.locationName };

  const rocket = h("input", { type: "text", name: "rocket", value: v.rocket || "", required: true });

  const motorList = h("datalist", { id: `motors-${Math.random().toString(36).slice(2)}` },
    motorSuggestions(entries).map((m) => h("option", { value: m })));
  const motor = h("input", {
    type: "text", name: "motor", value: v.motor || "", list: motorList.id,
    placeholder: "e.g. H130W-8",
  });
  const motorHint = h("div", { class: "hint" });
  const describeMotor = () => {
    const m = parseMotor(motor.value);
    motorHint.textContent = !motor.value.trim() ? "" : m
      ? `${m.impulse} impulse class${m.delay ? ` · ${m.delay === "P" ? "plugged" : m.delay + " s delay"}` : ""}`
      : "";
  };
  motor.addEventListener("input", describeMotor);
  describeMotor();

  const loc = h("input", {
    type: "text", name: "location", value: v.locationName || "", placeholder: "Club or field",
  });
  const chips = h("div", { class: "chips" });
  const locHint = h("div", { class: "hint" });
  const sayWhere = () => {
    const src = state.locationSource;
    locHint.textContent = src === "log" ? "Suggested from your earlier flights here — type to change."
      : src === "map" ? "Suggested from the GNSS pad position — type a club or field name to change."
      : "";
  };
  loc.addEventListener("input", () => {
    state.touched = true;
    state.locationSource = loc.value.trim() ? "manual" : null;
    for (const c of chips.children) c.classList.remove("on");
    sayWhere();
  });
  const suggestions = [];
  const drawChips = () => {
    chips.replaceChildren(...suggestions.map((s) => h("button", {
      type: "button",
      class: `chip${loc.value === s.name ? " on" : ""}`,
      title: s.detail,
      onclick: () => {
        loc.value = s.name;
        state.locationSource = s.source;
        state.touched = true;
        drawChips();
        sayWhere();
      },
    }, s.name, h("span", { class: "muted" }, s.detail))));
  };
  const offer = (list) => {
    for (const s of list) if (!suggestions.some((x) => x.name === s.name)) suggestions.push(s);
    if (!state.touched && suggestions[0]) {
      loc.value = suggestions[0].name;
      state.locationSource = suggestions[0].source;
    }
    drawChips();
    sayWhere();
  };
  const pad = f.pad || (v.pad && v.pad.lat != null ? v.pad : null);
  if (pad) {
    offer(locationSuggestions(pad, entries, null));
    // Only asked while the field is empty: a saved flight that already has a
    // name does not send its pad to OpenStreetMap every time it is opened.
    if (!v.locationName) {
      reverseGeocode(pad).then((name) => {
        if (name) offer(locationSuggestions(pad, entries, name).filter((s) => s.source === "map"));
      });
    }
  }
  sayWhere();

  const outcome = h("div", { class: "seg", role: "radiogroup" },
    OUTCOMES.map((o) => h("label", { class: o.key },
      h("input", { type: "radio", name: "outcome", value: o.key, checked: v.outcome === o.key }),
      o.label)));

  const notes = h("textarea", {
    name: "notes", rows: "3", placeholder: "How it went: recovery, damage, what to change next time",
  });
  notes.value = v.notes || "";

  const imp = imperial();
  const r = f.results || f;
  const comp = f.computer || f;
  const stamp = f.date ? f.date : localStamp(f.date_utc, Intl.DateTimeFormat().resolvedOptions().timeZone);
  const when = stamp && (stamp.local || stamp.utc);
  const fromName = (f.date ? f.date.source : f.date_source) === "filename";
  const factSpans = [
    when ? h("span", { title: fromName ? "From the file name (the log had no GNSS time)" : "From the GNSS clock at liftoff" },
      h("b", null, when)) : null,
    comp.board ? h("span", {
      title: comp.board_asserted
        ? "As the firmware image was built for; the board had no revision of its own stored"
        : "The board's own stored revision",
    }, "Computer ", h("b", null, comp.board)) : (f.firmware || comp.firmware) ? h("span", {
      title: "The board started writing its revision into the log with settings frame v9",
    }, "Computer ", h("span", { class: "muted" }, "not recorded")) : null,
    comp.firmware ? h("span", {
      title: comp.firmware_dirty ? "Built with uncommitted changes" : "Firmware git commit",
    }, "Firmware ", h("b", null, comp.firmware + (comp.firmware_dirty ? "+" : ""))) : null,
    r.apogee_m != null ? h("span", null, "Apogee ", h("b", null, fmtValue(r.apogee_m, "m", imp))) : null,
    r.max_speed_mps != null ? h("span", null, "Max speed ", h("b", null, fmtValue(r.max_speed_mps, "m/s", imp))) : null,
    r.flight_time_s != null ? h("span", null, "Flight ", h("b", null, `${r.flight_time_s.toFixed(1)} s`)) : null,
  ].filter(Boolean);

  const status = h("span", { class: "status" });
  const submit = h("button", { type: "submit" }, submitLabel);

  form.replaceChildren(
    h("div", { class: "form-grid" },
      h("label", null, "Rocket"), rocket,
      h("label", null, "Motor"), h("div", null, motor, motorList, motorHint),
      h("label", null, "Location"), h("div", null, loc, chips, locHint,
        pad ? h("div", { class: "hint" },
          `Pad ${pad.lat.toFixed(5)}, ${pad.lon.toFixed(5)} · `, external(osmLink(pad), "map")) : null),
      h("label", null, "Outcome"), h("div", null, outcome),
      h("label", null, "Notes"), notes,
      factSpans.length ? h("label", null, "From the log") : null,
      factSpans.length ? h("div", { class: "facts" }, factSpans) : null,
      f.error ? h("label", null, "From the log") : null,
      f.error ? h("div", { class: "hint" }, `Could not read the log's own details (${f.error}); fill in what you know.`) : null,
      extras ? h("label", null, "") : null,
      extras ? extras.node : null,
    ),
    h("div", { class: "form-actions" }, submit, status),
  );

  form.onsubmit = async (ev) => {
    ev.preventDefault();
    const checked = form.querySelector('input[name="outcome"]:checked');
    const details = {
      rocket: rocket.value,
      motor: motor.value,
      locationName: loc.value,
      locationSource: loc.value.trim() ? state.locationSource || "manual" : null,
      outcome: checked ? checked.value : null,
      notes: notes.value,
    };
    submit.disabled = true;
    status.className = "status";
    status.textContent = "Saving…";
    try {
      const said = await onSubmit(details, extras ? extras.read() : {}, (msg) => { status.textContent = msg; });
      status.className = "status ok";
      status.replaceChildren(...(Array.isArray(said) ? said : [said || "Saved"]));
    } catch (err) {
      status.className = "status err";
      status.textContent = String(err.message || err);
    } finally {
      submit.disabled = false;
    }
  };
}

// ---------- after a report ----------

let navCount = () => {};
export function onCountChange(fn) { navCount = fn; }

async function refreshCount() {
  try { navCount((await store.listFlights()).length); } catch (err) { navCount(null); }
}

/* The flight's files worth keeping: the log, its sidecar, and base-station
   logs. A flight's .csv is left out — it is exported from the .bin, nothing
   reads it, and it is often the largest file dropped. */
function keptFiles(files) {
  const stems = new Set(files.filter((f) => /\.bin$/i.test(f.name)).map((f) => f.name.replace(/\.bin$/i, "")));
  return files.filter((f) => !(/\.csv$/i.test(f.name) && stems.has(f.name.replace(/\.csv$/i, ""))));
}

function mb(bytes) {
  return `${(bytes / 1e6).toFixed(1)} MB`;
}

export async function offerEntry({ html, facts, files, rocketName }) {
  const panel = el("log-panel");
  panel.hidden = false;
  const entries = await store.listFlights().catch(() => []);
  const conn = gh.savedConnection();
  const keep = keptFiles(files);
  const keepBytes = keep.reduce((n, f) => n + f.blob.size, 0);
  const tz = Intl.DateTimeFormat().resolvedOptions().timeZone;

  // The same flight saved before — same file, same liftoff — is updated in
  // place rather than saved twice, and its answers come back into the form.
  const draft = newEntry(facts, { rocket: rocketName }, new Date(), tz);
  const prior = entries.find((e) => e.source_file && e.source_file === draft.source_file
    && (e.date && e.date.utc) === draft.date.utc);

  const keepBox = h("input", { type: "checkbox", checked: true });
  const pubBox = h("input", { type: "checkbox", checked: !!conn });
  const dataBox = h("input", { type: "checkbox", checked: true });
  const dataRow = h("label", null, dataBox, "Include the flight's files, so others can analyze it themselves");
  const syncData = () => {
    dataBox.disabled = !keepBox.checked || !pubBox.checked;
    dataRow.style.opacity = dataBox.disabled ? ".5" : "";
  };
  keepBox.addEventListener("change", syncData);
  pubBox.addEventListener("change", syncData);
  syncData();

  const extras = {
    node: h("div", { class: "checks" },
      h("label", null, keepBox, `Keep the flight's files with it (${mb(keepBytes)}) — needed to publish them later`),
      conn
        ? [h("label", null, pubBox, "Publish to my public log (", h("b", null, conn.owner), ")"), dataRow]
        : h("span", { class: "hint" }, "Want others to see it? ",
          h("a", { href: "#/flights" }, "Set up a public log"), " in My flights."),
    ),
    read: () => ({
      keepData: keepBox.checked,
      publish: !!conn && pubBox.checked,
      includeData: keepBox.checked && dataBox.checked,
    }),
  };

  entryForm(el("log-form"), {
    values: prior ? {
      rocket: prior.rocket, motor: prior.motor, locationName: prior.location.name,
      locationSource: prior.location.source, outcome: prior.outcome, notes: prior.notes,
    } : { rocket: rocketName },
    facts,
    entries,
    submitLabel: prior ? "Update saved flight" : "Save to my flights",
    extras,
    onSubmit: async (details, opts, say) => {
      const all = await store.listFlights();
      const before = all.find((e) => e.id === (prior && prior.id))
        || all.find((e) => e.source_file === draft.source_file && e.date.utc === draft.date.utc);
      let entry = newEntry(facts, details, new Date(), tz);
      if (before) {
        entry.id = before.id;
        entry.created = before.created;
        entry.published = before.published || null;
        await store.deleteFlight(before.id);
      }
      entry.files.report = "report.html";
      entry.files.data = opts.keepData ? keep.map((f) => f.name) : [];
      entry.keepData = opts.keepData;
      await store.saveFlight(entry, [
        { name: "report.html", blob: new Blob([html], { type: "text/html" }) },
        ...(opts.keepData ? keep.map((f) => ({ name: `data/${f.name}`, blob: f.blob })) : []),
      ]);
      store.persist();
      refreshCount();
      const open = h("a", { href: `#/flights/${entry.id}` }, "open it in My flights");
      if (opts.publish || entry.published) {
        say("Saved. Publishing…");
        entry = await publishFlight(entry, { includeData: opts.includeData, replaceReport: true }, say);
        return ["Saved and published — ", open, "."];
      }
      return ["Saved — ", open, "."];
    },
  });
}

export function hideEntry() {
  el("log-panel").hidden = true;
}

// ---------- publishing ----------

function publisher() {
  const conn = gh.savedConnection();
  if (!conn) throw new Error("Connect a public log first (My flights → Public flight log).");
  return new gh.Publisher(conn.owner, conn.token);
}

function explain(err) {
  if (err instanceof gh.GitHubError) {
    if (err.status === 401) return new Error("GitHub no longer accepts the saved token — it may have expired. Reconnect in My flights.");
    if (err.status === 403) return new Error(`GitHub refused the write (${err.message}). The token needs Contents: Read and write on ${REPO_NAME}.`);
    if (err.status === 404) return new Error(`GitHub cannot find ${REPO_NAME} for this token. Check the token has access to it.`);
  }
  return err;
}

/* Put one saved flight in the public log, or bring its public copy up to
   date. The report and files go up only when the public copy lacks them — an
   edit to the notes should not re-send 9 MB. */
async function publishFlight(entry, { includeData, replaceReport = false }, say = () => {}) {
  const pub = publisher();
  const dir = flightDir(entry.id);
  const shareData = includeData && (entry.files.data || []).length > 0;
  const out = publicEntry(entry);
  if (!shareData) out.files.data = [];
  try {
    await pub.check();
    await gh.updateLog(pub, async (index) => {
      const base = index || emptyIndex(pub.owner);
      const prev = base.flights.find((e) => e.id === entry.id);
      const changes = {};
      if (replaceReport || !prev || !(prev.files && prev.files.report)) {
        say("Uploading the report…");
        changes[`${dir}/report.html`] = await store.getBlob(entry.id, "report.html");
      }
      const had = new Set((prev && prev.files && prev.files.data) || []);
      if (shareData) {
        for (const name of out.files.data) {
          if (had.has(name)) continue;
          say(`Uploading ${name}…`);
          const blob = await store.getBlob(entry.id, `data/${name}`);
          if (!blob) throw new Error(`${name} is no longer saved in this browser.`);
          changes[`${dir}/data/${name}`] = blob;
        }
      }
      // Files the public copy has and this one no longer shares.
      const keep = new Set(flightPaths(out));
      if (prev) for (const p of flightPaths(prev)) if (!keep.has(p)) changes[p] = null;
      const next = upsertIndex(base, out);
      next.pilot = { ...(next.pilot || {}), login: pub.owner };
      changes["README.md"] = readme(next, PUBLIC_SITE);
      say("Committing…");
      return {
        index: next,
        changes,
        message: `${prev ? "Update" : "Add"} ${entry.rocket}${entry.date.local ? ` (${entry.date.local})` : ""}`,
      };
    });
  } catch (err) {
    throw explain(err);
  } finally {
    indexCache.delete(pub.owner);
  }
  const done = { ...entry, published: { owner: pub.owner, at: new Date().toISOString(), data: shareData } };
  await store.putFlight(done);
  return done;
}

async function unpublishFlight(entry) {
  const pub = publisher();
  try {
    await gh.updateLog(pub, async (index) => {
      const base = index || emptyIndex(pub.owner);
      const prev = base.flights.find((e) => e.id === entry.id);
      const changes = {};
      for (const p of flightPaths(prev || entry)) changes[p] = null;
      const next = removeFromIndex(base, entry.id);
      changes["README.md"] = readme(next, PUBLIC_SITE);
      return { index: next, changes, message: `Remove ${entry.rocket}${entry.date.local ? ` (${entry.date.local})` : ""}` };
    });
  } catch (err) {
    throw explain(err);
  } finally {
    indexCache.delete(pub.owner);
  }
  const done = { ...entry, published: null };
  await store.putFlight(done);
  return done;
}

function publicLink(owner, id) {
  return `${PUBLIC_SITE}#/pilots/${encodeURIComponent(owner)}/${id}`;
}

// ---------- My flights ----------

async function renderFlights() {
  const view = el("view-flights");
  let entries = [];
  try {
    entries = await store.listFlights();
  } catch (err) {
    el("flights-list").replaceChildren(h("p", { class: "status err" },
      "This browser would not open its saved-flights store (private windows often refuse)."));
  }
  fillUnitToggles(view, renderFlights);
  el("flights-list").replaceChildren(entries.length
    ? logTable(entries, { href: (e) => `#/flights/${e.id}`, publicCol: !!gh.savedConnection() })
    : h("p", { class: "empty" }, "No saved flights yet. Analyze a flight, then save it from the form under the progress list."));
  el("export-csv").onclick = () => {
    const blob = new Blob([toCSV(entries)], { type: "text/csv" });
    const a = h("a", { href: URL.createObjectURL(blob), download: "tinkerrocket-flight-log.csv" });
    a.click();
  };
  el("export-csv").disabled = !entries.length;
  renderPublicPanel(entries);
}

async function renderPublicPanel(entries) {
  const panel = el("public-panel");
  const conn = gh.savedConnection();
  if (!conn) {
    renderSetup(panel);
    return;
  }

  const unpublished = (entries || []).filter((e) => !e.published);
  const status = h("span", { class: "status" });
  const pages = h("span", { class: "hint" }, "Checking GitHub Pages…");
  const publishAll = h("button", {
    type: "button", class: "secondary", disabled: !unpublished.length,
    onclick: async () => {
      publishAll.disabled = true;
      try {
        for (const [i, e] of unpublished.entries()) {
          status.className = "status";
          status.textContent = `Publishing ${i + 1} of ${unpublished.length}: ${e.rocket}…`;
          await publishFlight(e, { includeData: (e.files.data || []).length > 0 });
        }
        renderFlights();
      } catch (err) {
        status.className = "status err";
        status.textContent = String(err.message || err);
        publishAll.disabled = false;
      }
    },
  }, unpublished.length
    ? `Publish ${unpublished.length} unpublished flight${unpublished.length === 1 ? "" : "s"}`
    : "Every flight is published");
  panel.replaceChildren(
    h("div", { class: "panel-head" },
      h("h2", null, "Public flight log"),
      h("a", { href: `#/pilots/${encodeURIComponent(conn.owner)}` }, "View it as others see it")),
    h("p", null, "Publishing as ", h("b", null, conn.owner), " to ",
      external(gh.repoUrl(conn.owner), `${conn.owner}/${REPO_NAME}`), ". ", pages),
    h("div", { class: "actions" },
      publishAll,
      h("button", {
        type: "button", class: "secondary",
        onclick: () => {
          if (!confirm("Forget the GitHub token in this browser? Your public log stays as it is.")) return;
          gh.forgetConnection();
          renderFlights();
        },
      }, "Disconnect"),
      status),
  );
  gh.pagesLive(conn.owner).then((live) => {
    pages.replaceChildren(live
      ? "GitHub Pages is serving your reports."
      : h("span", null, "GitHub Pages is not serving it yet, so others can see your log but cannot open reports. ",
        external(gh.pagesSettingsUrl(conn.owner), "Open the Pages settings"), " and set ",
        h("i", null, "Source: Deploy from a branch"), ", ", h("i", null, "Branch: main"), ", ",
        h("i", null, "/ (root)"), ", then Save. It goes live about a minute later. ",
        h("a", { href: "#/help/publish" },"Help")));
  });
}

/* A link that does nothing until it has somewhere real to go. The Pages
   settings and the token form both need the flyer's login in them; a link
   built before it is typed goes to a GitHub 404. */
function stepLink(text) {
  const a = h("a", { target: "_blank", rel: "noopener", class: "step-link", "aria-disabled": "true" }, text);
  return {
    node: a,
    set(href) {
      if (href) { a.href = href; a.removeAttribute("aria-disabled"); }
      else { a.removeAttribute("href"); a.setAttribute("aria-disabled", "true"); }
    },
  };
}

function stepState(node, state, text) {
  node.className = `step-state ${state}`;
  node.textContent = text;
}

/* Setting up a public log: the login first, because every link after it is
   the flyer's own; then what each step is for, so a flyer who has never made
   a token knows what they are handing over and why. */
function renderSetup(panel) {
  const user = h("input", {
    type: "text", placeholder: "GitHub username", "aria-label": "GitHub username",
    autocomplete: "username", spellcheck: "false",
  });
  const token = h("input", { type: "password", placeholder: "github_pat_…", "aria-label": "GitHub token", autocomplete: "off" });
  const status = h("span", { class: "status" });
  const repoLink = stepLink("Open GitHub's form");
  const pagesLink = stepLink("Open its Pages settings");
  const tokenLink = stepLink("Open the token form");
  const userState = h("span", { class: "step-state" });
  const repoState = h("span", { class: "step-state" });
  const pagesState = h("span", { class: "step-state" });
  repoLink.set(gh.newRepoUrl());

  let check = 0;
  const refresh = async () => {
    const login = cleanLogin(user.value);
    repoLink.set(gh.newRepoUrl(login));
    pagesLink.set(login && gh.pagesSettingsUrl(login));
    tokenLink.set(login && gh.newTokenUrl(login));
    for (const n of [userState, repoState, pagesState]) stepState(n, "", "");
    if (!user.value.trim()) return;
    if (!login) { stepState(userState, "bad", "That is not a GitHub username."); return; }
    const mine = ++check;
    stepState(repoState, "wait", "Checking…");
    const [repo, live] = await Promise.all([gh.repoState(login), gh.pagesLive(login)]);
    if (mine !== check) return;
    stepState(repoState, repo === "ok" ? "ok" : repo === "missing" ? "todo" : "",
      repo === "ok" ? `✓ ${login}/${REPO_NAME} is there` : repo === "missing" ? "Not created yet (or not public)" : "");
    stepState(pagesState, live ? "ok" : "todo", live ? "✓ Pages is on" : "Not on yet");
  };
  let timer = null;
  user.addEventListener("input", () => { clearTimeout(timer); timer = setTimeout(refresh, 500); });

  const connect = h("button", {
    type: "button",
    onclick: async () => {
      const login = cleanLogin(user.value);
      if (!login) { status.className = "status err"; status.textContent = "Enter your GitHub username in step 1."; return; }
      if (!token.value.trim()) { status.className = "status err"; status.textContent = "Paste the token from step 4."; return; }
      connect.disabled = true;
      status.className = "status";
      status.textContent = "Checking with GitHub…";
      try {
        const who = await new gh.Publisher(login, token.value.trim()).check();
        gh.saveConnection(who.login, token.value.trim());
        renderFlights();
      } catch (err) {
        status.className = "status err";
        status.textContent = String(err.message || err);
      } finally {
        connect.disabled = false;
      }
    },
  }, "Connect");

  panel.replaceChildren(
    h("div", { class: "panel-head" },
      h("h2", null, "Public flight log"),
      h("a", { href: "#/help/publish" }, "Step-by-step help")),
    h("p", null, "Publishing puts your flights where anyone can browse them, under Public logs and at a link you can share. ",
      "There is no TinkerRocket server or account: your log is kept on GitHub, in a repository that belongs to you. ",
      "Setting it up takes three things on GitHub, once:"),
    h("ul", { class: "explain" },
      h("li", null, h("b", null, "A repository"), " named ", h("code", null, REPO_NAME),
        ". It is where your log, reports and flight files are stored. It is yours; delete it and everything is gone."),
      h("li", null, h("b", null, "GitHub Pages"), ", GitHub's free web hosting for that repository. ",
        "It shows your reports to other people from your own address (", h("i", null, "you"), ".github.io), kept apart from this page."),
      h("li", null, h("b", null, "A token"), ", a key that lets this page write to that one repository and nothing else on your account. ",
        "It stays in this browser and is only ever sent to GitHub.")),
    h("p", { class: "hint" }, "Nothing is public until you press Publish on a flight. A published flight shows its report ",
      "(including the GPS track and launch site), the details you entered, and the flight's files if you choose to include them."),
    h("ol", { class: "setup" },
      h("li", null, h("b", null, "Your GitHub username. "), "No account? ", external("https://github.com/signup", "Sign up"),
        " — it is free.",
        h("div", { class: "setup-token" }, user), userState),
      h("li", null, h("b", null, "Create the repository. "), repoLink.node,
        ": keep it ", h("i", null, "Public"), ", turn on ", h("i", null, "Add README"),
        " (Pages needs something to serve), and press ", h("i", null, "Create repository"), ". ", repoState),
      h("li", null, h("b", null, "Turn on GitHub Pages. "), pagesLink.node, ": under ",
        h("i", null, "Build and deployment"), " set Source to ", h("i", null, "Deploy from a branch"), ", Branch to ",
        h("i", null, "main"), " and ", h("i", null, "/ (root)"), ", and Save. ", pagesState),
      h("li", null, h("b", null, "Make a token. "), tokenLink.node, ": the form is filled in for you — under ",
        h("i", null, "Repository access"), " choose ", h("i", null, "Only select repositories"), " and pick ",
        h("code", null, REPO_NAME), ", then ", h("i", null, "Generate token"),
        " and copy it (GitHub shows it only once). It lasts a year."),
      h("li", null, h("b", null, "Connect. "), "Paste the token here.",
        h("div", { class: "setup-token" }, token, connect),
        status),
    ),
  );
}

// ---------- one saved flight ----------

function metaList(e, { pilot } = {}) {
  const imp = imperial();
  const r = e.results || {};
  const c = e.computer || {};
  const rows = [
    ["Date", e.date && (e.date.local
      ? `${e.date.local}${e.date.tz ? ` (${e.date.tz})` : ""}`
      : e.date.utc || "")],
    pilot ? ["Flyer", pilot] : null,
    ["Motor", e.motor],
    ["Location", e.location && e.location.name],
    ["Pad", e.location && e.location.lat != null
      ? h("span", null, `${e.location.lat.toFixed(5)}, ${e.location.lon.toFixed(5)} · `, external(osmLink(e.location), "map"))
      : null],
    ["Computer", c.board ? `${c.board}${c.board_asserted ? " (as built for)" : ""}` : null],
    ["Firmware", c.firmware ? `${c.firmware}${c.firmware_dirty ? " (built with uncommitted changes)" : ""}` : null],
    ["Apogee", fmtValue(r.apogee_m, "m", imp)],
    ["Max speed", fmtValue(r.max_speed_mps, "m/s", imp)],
    ["Max thrust acceleration", r.max_accel_g != null ? `${r.max_accel_g.toFixed(1)} G` : null],
    ["Peak shock", r.peak_shock_g != null ? `${r.peak_shock_g.toFixed(0)} G` : null],
    ["Flight time", r.flight_time_s != null ? `${r.flight_time_s.toFixed(1)} s` : null],
    ["Landed", fmtValue(r.landing_distance_m, "m", imp) ? `${fmtValue(r.landing_distance_m, "m", imp)} from the pad` : null],
    ["Outcome", outcomeChip(e.outcome) || null],
    ["Notes", e.notes],
  ].filter((row) => row && row[1] != null && row[1] !== "");
  return h("dl", { class: "meta" }, rows.map(([k, v]) => [h("dt", null, k), h("dd", null, v)]));
}

function subtitle(e) {
  return [e.motor, e.location && e.location.name, e.date && (e.date.local || e.date.utc)]
    .filter(Boolean).join(" · ");
}

async function renderLocalFlight(id) {
  const view = el("view-flight");
  const e = await store.getFlight(id).catch(() => null);
  if (!e) {
    view.replaceChildren(h("a", { class: "back", href: "#/flights" }, "← My flights"),
      h("section", { class: "panel" }, h("p", null, "That flight is not saved in this browser.")));
    return;
  }
  const conn = gh.savedConnection();
  const status = h("span", { class: "status" });
  const frame = h("iframe", { class: "report-frame", sandbox: "allow-scripts allow-same-origin" });
  const openLink = h("a", { class: "button secondary", target: "_blank", rel: "noopener" }, "Open in tab");
  const download = h("a", { class: "button secondary" }, "Download report");
  const editPanel = h("section", { class: "panel", hidden: true },
    h("div", { class: "panel-head" }, h("h2", null, "Edit")), h("form", { autocomplete: "off" }));

  const run = (fn) => async () => {
    status.className = "status";
    status.textContent = "Working…";
    try {
      const msg = await fn((m) => { status.textContent = m; });
      status.className = "status ok";
      status.replaceChildren(...(Array.isArray(msg) ? msg : [msg || "Done"]));
    } catch (err) {
      status.className = "status err";
      status.textContent = String(err.message || err);
    }
  };

  const pubButtons = [];
  if (conn) {
    if (e.published) {
      const link = publicLink(e.published.owner, e.id);
      pubButtons.push(
        h("button", {
          type: "button", class: "secondary",
          onclick: run(async () => {
            await navigator.clipboard.writeText(link);
            return "Public link copied.";
          }),
        }, "Copy public link"),
        h("button", {
          type: "button", class: "secondary",
          onclick: run(async () => {
            if (!confirm("Remove this flight from your public log? It stays saved here.")) return "Kept.";
            await unpublishFlight(e);
            renderLocalFlight(id);
            return "Removed from the public log.";
          }),
        }, "Unpublish"),
      );
    } else {
      pubButtons.push(h("button", {
        type: "button",
        onclick: run(async (say) => {
          await publishFlight(e, { includeData: (e.files.data || []).length > 0 }, say);
          renderLocalFlight(id);
          return "Published.";
        }),
      }, "Publish"));
    }
  }

  view.replaceChildren(
    h("a", { class: "back", href: "#/flights" }, "← My flights"),
    h("section", { class: "panel" },
      h("div", { class: "panel-head" },
        h("h2", { class: "flight-title" }, e.rocket),
        h("span", { class: "units", "data-units": true }),
        e.published ? h("span", { class: "pub on" }, `✓ public (${e.published.owner})`) : null),
      h("p", { class: "flight-sub" }, subtitle(e)),
      metaList(e),
      h("div", { class: "actions" },
        h("button", {
          type: "button", class: "secondary",
          onclick: () => { editPanel.hidden = !editPanel.hidden; },
        }, "Edit details"),
        ...pubButtons,
        openLink, download,
        (e.files.data || []).length ? h("button", {
          type: "button", class: "secondary",
          onclick: run(async () => {
            for (const name of e.files.data) {
              const blob = await store.getBlob(e.id, `data/${name}`);
              if (blob) h("a", { href: URL.createObjectURL(blob), download: name }).click();
            }
            return `Downloading ${e.files.data.length} file(s).`;
          }),
        }, "Download flight files") : null,
        h("button", {
          type: "button", class: "secondary",
          onclick: run(async () => {
            const alsoPublic = e.published && conn
              && confirm("Also remove it from your public log? (Cancel keeps the public copy.)");
            if (!confirm(`Delete ${e.rocket} (${dateText(e)}) from this browser?`)) return "Kept.";
            if (alsoPublic) await unpublishFlight(e);
            await store.deleteFlight(e.id);
            refreshCount();
            location.hash = "#/flights";
            return "Deleted.";
          }),
        }, "Delete"),
        status),
    ),
    editPanel,
    h("section", { class: "panel" }, frame),
  );
  fillUnitToggles(view, () => renderLocalFlight(id));

  const entries = await store.listFlights().catch(() => []);
  entryForm(editPanel.querySelector("form"), {
    values: {
      rocket: e.rocket, motor: e.motor, locationName: e.location.name,
      locationSource: e.location.source, outcome: e.outcome, notes: e.notes, pad: e.location,
    },
    facts: { ...e, date_source: e.date && e.date.source },
    entries: entries.filter((x) => x.id !== e.id),
    submitLabel: e.published ? "Save and update public copy" : "Save",
    onSubmit: async (details, _opts, say) => {
      let next = applyDetails(e, details);
      await store.putFlight(next);
      if (next.published && gh.savedConnection()) {
        say("Saved. Updating the public copy…");
        next = await publishFlight(next, { includeData: !!next.published.data }, say);
      }
      setTimeout(() => renderLocalFlight(id), 600);
      return next.published ? "Saved and published." : "Saved.";
    },
  });

  const blob = await store.getBlob(e.id, "report.html");
  if (blob) await showLocalReport(blob, `${e.id}_report`, { frame, open: openLink, download });
  else frame.replaceWith(h("p", { class: "empty" }, "The report for this flight is no longer saved."));
}

// ---------- public logs ----------

let pilotsCache = null;
const indexCache = new Map();

/* A flyer's index. One's own comes through the API with the saved token:
   raw.githubusercontent.com caches for five minutes, and a flyer checking a
   flight they just published should see it. */
async function publicIndex(login) {
  if (!indexCache.has(login)) {
    const conn = gh.savedConnection();
    const own = conn && conn.owner.toLowerCase() === login.toLowerCase();
    const p = own
      ? new gh.Publisher(conn.owner, conn.token).readIndex().then((idx) => {
        if (!idx) throw new gh.NotFound(`${login} has not published a flight yet`);
        return idx;
      })
      : gh.readPublicIndex(login);
    indexCache.set(login, p);
    p.catch(() => indexCache.delete(login));
  }
  return indexCache.get(login);
}

/* Only what a well-formed entry would hold reaches the page: an id that is
   safe in a path, and file names that are plain names. */
function sane(e) {
  if (!e || typeof e !== "object" || !cleanId(e.id)) return null;
  const obj = (o) => (o && typeof o === "object" ? o : {});
  const str = (v) => (typeof v === "string" ? v.slice(0, 4000) : "");
  const num = (v) => (typeof v === "number" && isFinite(v) ? v : null);
  const name = (v) => typeof v === "string" && /^[\w.\-]{1,120}$/.test(v) && !/^\.+$/.test(v);
  const files = obj(e.files), loc = obj(e.location), r = obj(e.results), c = obj(e.computer), d = obj(e.date);
  const lat = num(loc.lat), lon = num(loc.lon);
  return {
    id: e.id,
    rocket: str(e.rocket),
    motor: str(e.motor),
    notes: str(e.notes),
    outcome: str(e.outcome),
    files: {
      report: name(files.report) ? files.report : null,
      data: Array.isArray(files.data) ? files.data.filter(name) : [],
    },
    location: { name: str(loc.name), lat: lat != null && lon != null ? lat : null, lon: lat != null && lon != null ? lon : null },
    results: Object.fromEntries(["apogee_m", "max_speed_mps", "max_accel_g", "peak_shock_g", "flight_time_s", "landing_distance_m"]
      .map((k) => [k, num(r[k])])),
    computer: { board: str(c.board), board_asserted: c.board_asserted === true,
      firmware: str(c.firmware), firmware_dirty: c.firmware_dirty === true },
    date: { utc: str(d.utc), local: str(d.local), tz: str(d.tz), source: str(d.source) },
  };
}

function saneFlights(index) {
  return ((index && Array.isArray(index.flights)) ? index.flights : []).map(sane).filter(Boolean);
}

async function renderPilots() {
  const box = el("pilots-list");
  el("pilot-lookup").onsubmit = (ev) => {
    ev.preventDefault();
    const login = cleanLogin(el("pilot-name").value);
    if (login) location.hash = `#/pilots/${encodeURIComponent(login)}`;
  };
  fillUnitToggles(el("view-pilots"), renderPilots);
  if (!pilotsCache) {
    box.replaceChildren(h("p", { class: "hint" }, "Looking for public logs…"));
    pilotsCache = gh.searchPilots().catch((err) => { pilotsCache = null; throw err; });
  }
  let pilots;
  try {
    pilots = await pilotsCache;
  } catch (err) {
    box.replaceChildren(h("p", { class: "status err" },
      `${err.message || err}. GitHub limits searches without sign-in to ten a minute; try again shortly, or open a log by username above.`));
    return;
  }
  if (!pilots.length) {
    box.replaceChildren(h("p", { class: "empty" },
      "No public logs found yet. GitHub can take a few minutes to list a new repository; a log can always be opened by username above."));
    el("recent-panel").hidden = true;
    return;
  }
  const cards = pilots.map((p) => {
    const what = h("div", { class: "what" }, "…");
    publicIndex(p.login).then((idx) => {
      const flights = saneFlights(idx);
      const last = sortEntries(flights)[0];
      what.textContent = `${flights.length} flight${flights.length === 1 ? "" : "s"}${last ? ` · last ${dateText(last)}` : ""}`;
    }, () => { what.textContent = "no log yet"; });
    return h("a", { class: "pilot", href: `#/pilots/${encodeURIComponent(p.login)}` },
      h("img", { src: p.avatar, alt: "", loading: "lazy", onerror: (ev) => { ev.target.style.visibility = "hidden"; } }),
      h("div", null, h("div", { class: "who" }, p.login), what));
  });
  box.replaceChildren(h("div", { class: "pilots" }, cards));

  // Every log is already being fetched for its card; the latest flights across
  // them come for free.
  const loaded = await Promise.allSettled(pilots.slice(0, 30).map((p) =>
    publicIndex(p.login).then((idx) => saneFlights(idx).map((e) => ({ ...e, _pilot: p.login })))));
  const recent = sortEntries(loaded.flatMap((r) => (r.status === "fulfilled" ? r.value : []))).slice(0, 25);
  el("recent-panel").hidden = !recent.length;
  el("recent-list").replaceChildren(logTable(recent, {
    href: (e) => `#/pilots/${encodeURIComponent(e._pilot)}/${e.id}`,
    pilot: (e) => h("a", { href: `#/pilots/${encodeURIComponent(e._pilot)}` }, e._pilot),
  }));
}

async function renderPilot(login) {
  const view = el("view-pilot");
  const back = h("a", { class: "back", href: "#/pilots" }, "← Public logs");
  view.replaceChildren(back, h("section", { class: "panel" }, h("p", { class: "hint" }, `Reading ${login}'s log…`)));
  let idx;
  try {
    idx = await publicIndex(login);
  } catch (err) {
    view.replaceChildren(back, h("section", { class: "panel" },
      h("p", null, err instanceof gh.NotFound
        ? `${login} has no public TinkerRocket flight log (no ${REPO_NAME} repository with a flightlog.json).`
        : String(err.message || err))));
    return;
  }
  const flights = saneFlights(idx);
  const exportBtn = h("button", {
    type: "button", class: "secondary",
    onclick: () => h("a", {
      href: URL.createObjectURL(new Blob([toCSV(flights)], { type: "text/csv" })),
      download: `${login}-flight-log.csv`,
    }).click(),
  }, "Export CSV");
  view.replaceChildren(
    back,
    h("section", { class: "panel" },
      h("div", { class: "pilot-head" },
        h("img", {
          class: "avatar", src: `${gh.endpoints().web}/${encodeURIComponent(login)}.png?size=112`, alt: "",
          onerror: (ev) => { ev.target.style.visibility = "hidden"; },
        }),
        h("div", { class: "spacer" },
          h("h2", { class: "flight-title" }, `${login}'s flight log`),
          h("div", { class: "hint" }, `${flights.length} flight${flights.length === 1 ? "" : "s"} · `,
            external(gh.repoUrl(login), `github.com/${login}/${REPO_NAME}`))),
        h("span", { class: "units", "data-units": true }),
        exportBtn),
      logTable(flights, { href: (e) => `#/pilots/${encodeURIComponent(login)}/${e.id}` })),
  );
  fillUnitToggles(view, () => renderPilot(login));
}

async function renderPublicFlight(login, id) {
  const view = el("view-flight");
  const back = h("a", { class: "back", href: `#/pilots/${encodeURIComponent(login)}` }, `← ${login}'s log`);
  let e;
  try {
    e = saneFlights(await publicIndex(login)).find((x) => x.id === id);
  } catch (err) {
    view.replaceChildren(back, h("section", { class: "panel" }, h("p", null, String(err.message || err))));
    return;
  }
  if (!e) {
    view.replaceChildren(back, h("section", { class: "panel" }, h("p", null, `That flight is not in ${login}'s public log.`)));
    return;
  }
  const status = h("span", { class: "status" });
  const reportPath = e.files.report ? `${flightDir(e.id)}/${e.files.report}` : null;
  const page = reportPath ? gh.pagesUrl(login, reportPath) : null;
  const downloadRaw = (path, name) => async () => {
    status.className = "status";
    status.textContent = `Downloading ${name}…`;
    try {
      const blob = await gh.readPublicBlob(login, path);
      h("a", { href: URL.createObjectURL(blob), download: name }).click();
      status.textContent = "";
    } catch (err) {
      status.className = "status err";
      status.textContent = String(err.message || err);
    }
  };
  const reportBox = h("section", { class: "panel" });
  view.replaceChildren(
    back,
    h("section", { class: "panel" },
      h("div", { class: "panel-head" },
        h("h2", { class: "flight-title" }, e.rocket || "Flight"),
        h("span", { class: "units", "data-units": true })),
      h("p", { class: "flight-sub" }, subtitle(e)),
      metaList(e, { pilot: h("a", { href: `#/pilots/${encodeURIComponent(login)}` }, login) }),
      h("div", { class: "actions" },
        page ? external(page, "Open report in tab") : null,
        reportPath ? h("button", { type: "button", class: "secondary", onclick: downloadRaw(reportPath, `${e.id}_report.html`) }, "Download report") : null,
        e.files.data.map((name) => h("button", {
          type: "button", class: "secondary", onclick: downloadRaw(`${flightDir(e.id)}/data/${name}`, name),
        }, name)),
        h("button", {
          type: "button", class: "secondary",
          onclick: async () => {
            await navigator.clipboard.writeText(publicLink(login, e.id));
            status.className = "status ok";
            status.textContent = "Link copied.";
          },
        }, "Copy link"),
        status),
    ),
    reportBox,
  );
  for (const a of view.querySelectorAll(".actions > a")) a.classList.add("button", "secondary");
  fillUnitToggles(view, () => renderPublicFlight(login, id));

  if (!page) {
    reportBox.replaceChildren(h("p", { class: "empty" }, "This flight was published without a report."));
    return;
  }
  // Rendered only from the flyer's own Pages origin; see pagesUrl().
  reportBox.replaceChildren(h("p", { class: "hint" }, "Loading the report…"));
  let live = false;
  try {
    live = (await fetch(page, { method: "HEAD", cache: "no-store" })).ok;
  } catch (err) { /* offline, or Pages off */ }
  if (!live) {
    reportBox.replaceChildren(h("p", null,
      `${login}'s GitHub Pages is not serving this report (yet). Pages publishes a minute or so after each change, `,
      "and has to be switched on once for the repository. The report can still be downloaded above."));
    return;
  }
  reportBox.replaceChildren(h("iframe", {
    class: "report-frame", src: page,
    sandbox: "allow-scripts allow-same-origin allow-popups allow-downloads",
  }));
}

// ---------- routing ----------

export function render(view, parts) {
  if (view === "flights") return renderFlights();
  if (view === "flight" && parts[0] === "flights") {
    const id = cleanId(parts[1]);
    return id ? renderLocalFlight(id) : renderFlights();
  }
  if (view === "pilots") return renderPilots();
  const login = cleanLogin(parts[1]);
  if (!login) return renderPilots();
  if (view === "pilot") return renderPilot(login);
  const id = cleanId(parts[2]);
  return id ? renderPublicFlight(login, id) : renderPilot(login);
}

refreshCount();
