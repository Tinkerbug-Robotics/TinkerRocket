/* The flight log: what an entry is, and the decisions made about it.

   Pure functions only — no DOM, no storage, no network — so every rule here is
   tested in Node (tests/js/flightlog.test.mjs) against the same file the page
   loads. The page, the local store and the GitHub client all build on these.

   An entry joins two kinds of fact:

     · what the log says about itself, read by flight_report.log_entry in the
       worker — the date, the pad position, which board and firmware flew, and
       the headline numbers. The flyer does not type these, because the two
       columns most often wrong in a hand-kept log are the board and the build;
     · what only the flyer knows — the motor, the field, how it went.

   The flyer's columns follow the hand-kept spreadsheet this replaces (Date,
   Rocket, Motor, Club/Location, Computer, Hardware, Max Alt, Max Speed,
   Outcome, Notes), so an exported CSV drops back into it. */

export const SCHEMA = 1;

// One repository per flyer, always this name, so a public log is found by its
// owner's GitHub login alone and GitHub's repository search lists every one.
export const REPO_NAME = "tinkerrocket-flight-log";
export const INDEX_PATH = "flightlog.json";

// A saved flight within this distance of a new pad is "the same field". Club
// ranges move the pads around by a few hundred metres between launches; two
// different fields are rarely closer than a few kilometres.
export const SAME_FIELD_KM = 2;

export const OUTCOMES = [
  { key: "success", label: "Success" },
  { key: "partial", label: "Partial" },
  { key: "failure", label: "Failure" },
];

export function slug(text) {
  const s = String(text || "")
    .normalize("NFKD")
    .replace(/[̀-ͯ]/g, "")
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-+|-+$/g, "");
  return s.slice(0, 40).replace(/-+$/, "") || "flight";
}

/* A stable id for one flight: when it flew, then what flew.

   Time first so a folder listing sorts chronologically, and to the second so
   two rockets on one rack launched a minute apart never collide. The rocket's
   slug is only for a human reading the repository. With no date at all (a log
   with neither a GNSS clock nor a dated filename) the file name stands in. */
export function flightId(facts, rocket) {
  const iso = facts && facts.date_utc;
  let stamp = null;
  if (iso) {
    const m = /^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})/.exec(iso);
    if (m) stamp = `${m[1]}${m[2]}${m[3]}-${m[4]}${m[5]}${m[6]}`;
  }
  if (!stamp) stamp = slug(String((facts && facts.file) || "flight").replace(/\.bin$/i, ""));
  return `${stamp}-${slug(rocket || (facts && facts.rocket))}`;
}

/* The date as the flyer saw it, fixed at save time.

   Stored rather than recomputed on view: a public log is read from every time
   zone, and an evening launch in California is already tomorrow in UTC. The
   flyer's own clock is the one their memory of the day matches. */
export function localStamp(isoUtc, timeZone) {
  if (!isoUtc) return null;
  // A filename date carries no zone; it is the board's clock, shown as is.
  const zoned = /Z$|[+-]\d{2}:\d{2}$/.test(isoUtc);
  if (!zoned) return { local: isoUtc.replace("T", " ").slice(0, 16), tz: null };
  const d = new Date(isoUtc);
  if (isNaN(d)) return null;
  const parts = new Intl.DateTimeFormat("en-CA", {
    timeZone, year: "numeric", month: "2-digit", day: "2-digit",
    hour: "2-digit", minute: "2-digit", hourCycle: "h23",
  }).formatToParts(d);
  const p = Object.fromEntries(parts.map((x) => [x.type, x.value]));
  return { local: `${p.year}-${p.month}-${p.day} ${p.hour}:${p.minute}`, tz: timeZone || null };
}

/* A new entry from the worker's facts and the flyer's answers. */
export function newEntry(facts, details, now = new Date(), timeZone = undefined) {
  const f = facts || {};
  const d = details || {};
  const rocket = (d.rocket || f.rocket || "").trim() || "Unnamed rocket";
  const when = localStamp(f.date_utc, timeZone) || { local: null, tz: null };
  return {
    schema: SCHEMA,
    id: flightId(f, rocket),
    rocket,
    motor: (d.motor || "").trim(),
    location: {
      name: (d.locationName || "").trim(),
      source: d.locationSource || (d.locationName ? "manual" : null),
      lat: f.pad ? f.pad.lat : null,
      lon: f.pad ? f.pad.lon : null,
      alt_m: f.pad ? f.pad.alt_m : null,
    },
    date: { utc: f.date_utc || null, source: f.date_source || null, local: when.local, tz: when.tz },
    computer: {
      board: f.board || null,
      board_asserted: f.board_asserted == null ? null : !!f.board_asserted,
      firmware: f.firmware || null,
      firmware_dirty: f.firmware_dirty == null ? null : !!f.firmware_dirty,
    },
    results: {
      apogee_m: num(f.apogee_m),
      max_speed_mps: num(f.max_speed_mps),
      max_accel_g: num(f.max_accel_g),
      peak_shock_g: num(f.peak_shock_g),
      flight_time_s: num(f.flight_time_s),
      landing_distance_m: num(f.landing_distance_m),
    },
    outcome: OUTCOMES.some((o) => o.key === d.outcome) ? d.outcome : null,
    notes: (d.notes || "").trim(),
    files: { report: null, data: [] },
    source_file: f.file || null,
    created: now.toISOString(),
    updated: now.toISOString(),
  };
}

function num(v) {
  return typeof v === "number" && isFinite(v) ? v : null;
}

/* The flyer's edits to an existing entry. Only their own columns move; what
   the log said stays as the log said it. */
export function applyDetails(entry, details, now = new Date()) {
  const e = structuredClone(entry);
  if ("rocket" in details) e.rocket = String(details.rocket || "").trim() || e.rocket;
  if ("motor" in details) e.motor = String(details.motor || "").trim();
  if ("locationName" in details) {
    e.location.name = String(details.locationName || "").trim();
    e.location.source = details.locationSource || (e.location.name ? "manual" : null);
  }
  if ("outcome" in details) e.outcome = OUTCOMES.some((o) => o.key === details.outcome) ? details.outcome : null;
  if ("notes" in details) e.notes = String(details.notes || "").trim();
  e.updated = now.toISOString();
  return e;
}

// ---------- where ----------

export function haversineKm(a, b) {
  const R = 6371.0088;
  const rad = Math.PI / 180;
  const dLat = (b.lat - a.lat) * rad;
  const dLon = (b.lon - a.lon) * rad;
  const h = Math.sin(dLat / 2) ** 2
    + Math.cos(a.lat * rad) * Math.cos(b.lat * rad) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.min(1, Math.sqrt(h)));
}

/* The name this flyer already gave the field, if they have flown here before.

   The nearest named pad within SAME_FIELD_KM wins. This is the suggestion that
   matters most: the map can say "Hopewell Township, NJ", but only the flyer's
   own log knows they call it "BARC". */
export function previousField(pad, entries, maxKm = SAME_FIELD_KM) {
  if (!pad || pad.lat == null || pad.lon == null) return null;
  let best = null;
  for (const e of entries || []) {
    const loc = e && e.location;
    if (!loc || !loc.name || loc.lat == null || loc.lon == null) continue;
    const km = haversineKm(pad, loc);
    if (km <= maxKm && (!best || km < best.km)) best = { name: loc.name, km, id: e.id };
  }
  return best;
}

/* A short place name from a Nominatim reverse-geocode answer.

   "Hopewell Township, NJ" rather than the full display_name, which runs to
   county and country and does not fit a table cell. US states come out as
   their two-letter code because that is how the hobby writes them; elsewhere
   the region's own name is kept. */
export function placeName(nominatim) {
  const a = (nominatim && nominatim.address) || {};
  const locality = a.city || a.town || a.village || a.hamlet || a.municipality
    || a.suburb || a.county || nominatim?.name || "";
  let region = "";
  const iso = a["ISO3166-2-lvl4"] || "";
  if (a.country_code === "us" && /^US-[A-Z]{2}$/.test(iso)) region = iso.slice(3);
  else region = a.state || a.region || a.country || "";
  return [locality, region].filter(Boolean).join(", ");
}

/* Every location suggestion for a pad, best first. */
export function locationSuggestions(pad, entries, mapName) {
  const out = [];
  const prev = previousField(pad, entries);
  if (prev) {
    out.push({
      name: prev.name, source: "log",
      detail: prev.km < 0.05 ? "your log, same pad" : `your log, ${fmtKm(prev.km)} away`,
    });
  }
  if (mapName && !out.some((s) => s.name.toLowerCase() === mapName.toLowerCase())) {
    out.push({ name: mapName, source: "map", detail: "OpenStreetMap" });
  }
  return out;
}

function fmtKm(km) {
  return km < 1 ? `${Math.round(km * 1000)} m` : `${km.toFixed(1)} km`;
}

// ---------- motor ----------

/* "H130W-8" -> { impulse: "H", thrust: 130, propellant: "W", delay: "8" }.

   Read loosely and never required: motors are written a dozen ways, and a
   designation this cannot parse is still a perfectly good note. The class
   letter is what a log is sorted and filtered by. */
export function parseMotor(text) {
  const m = /^\s*(?:\d+\s*x\s*)?(1\/[248]A|[A-O])\s*(\d+(?:\.\d+)?)\s*([A-Z]*)(?:\s*-\s*(\d+|P|M))?/i
    .exec(String(text || ""));
  if (!m) return null;
  return {
    impulse: m[1].toUpperCase(),
    thrust: Number(m[2]),
    propellant: (m[3] || "").toUpperCase(),
    delay: m[4] ? m[4].toUpperCase() : null,
  };
}

/* Motors this flyer has logged, most recent first, each once. */
export function motorSuggestions(entries) {
  const seen = new Set();
  const out = [];
  for (const e of sortEntries(entries)) {
    const m = (e.motor || "").trim();
    if (m && !seen.has(m.toLowerCase())) {
      seen.add(m.toLowerCase());
      out.push(m);
    }
  }
  return out;
}

// ---------- lists ----------

/* Newest flight first. The date the log carries, not when it was saved, so a
   backlog of old flights saved in one evening still reads in flight order. */
export function sortEntries(entries) {
  const key = (e) => (e.date && (e.date.utc || e.date.local)) || e.created || "";
  return [...(entries || [])].sort((a, b) => (key(a) < key(b) ? 1 : key(a) > key(b) ? -1 : 0));
}

export function computerText(c) {
  if (!c) return "";
  const parts = [];
  if (c.board) parts.push(c.board);
  if (c.firmware) parts.push(c.firmware + (c.firmware_dirty ? "+" : ""));
  return parts.join(" · ");
}

// Same conversions the report uses for these purposes (flight_report/units.py):
// altitude in feet, airspeed in mph, distance in feet.
const IMPERIAL = {
  m: { f: 3.28084, unit: "ft" },
  "m/s": { f: 2.236936, unit: "mph" },
};

export function fmtValue(v, unit, imperial, places = 0) {
  if (v == null || !isFinite(v)) return "";
  if (imperial && IMPERIAL[unit]) {
    return `${(v * IMPERIAL[unit].f).toFixed(places)} ${IMPERIAL[unit].unit}`;
  }
  return `${v.toFixed(places)} ${unit}`;
}

// ---------- CSV ----------

const CSV_COLUMNS = [
  ["Date", (e) => (e.date && (e.date.local || e.date.utc)) || ""],
  ["Rocket", (e) => e.rocket],
  ["Motor", (e) => e.motor],
  ["Club/Location", (e) => (e.location && e.location.name) || ""],
  ["Computer", (e) => (e.computer && e.computer.board) || ""],
  ["Firmware", (e) => (e.computer && e.computer.firmware
    ? e.computer.firmware + (e.computer.firmware_dirty ? " (uncommitted changes)" : "") : "")],
  ["Max Alt (m)", (e) => round(e.results && e.results.apogee_m, 1)],
  ["Max Speed (m/s)", (e) => round(e.results && e.results.max_speed_mps, 1)],
  ["Flight Time (s)", (e) => round(e.results && e.results.flight_time_s, 1)],
  ["Outcome", (e) => (OUTCOMES.find((o) => o.key === e.outcome) || {}).label || ""],
  ["Notes", (e) => e.notes],
  ["Pad Lat", (e) => (e.location && e.location.lat != null ? String(e.location.lat) : "")],
  ["Pad Lon", (e) => (e.location && e.location.lon != null ? String(e.location.lon) : "")],
];

function round(v, places) {
  return v == null || !isFinite(v) ? "" : v.toFixed(places);
}

function csvCell(v) {
  const s = v == null ? "" : String(v);
  return /[",\n\r]/.test(s) ? `"${s.replace(/"/g, '""')}"` : s;
}

export function toCSV(entries) {
  const rows = [CSV_COLUMNS.map((c) => c[0])];
  for (const e of sortEntries(entries)) rows.push(CSV_COLUMNS.map((c) => c[1](e)));
  return rows.map((r) => r.map(csvCell).join(",")).join("\r\n") + "\r\n";
}

// ---------- the public repository ----------

/* Layout of a public log:

     flightlog.json                     the index: every entry, nothing else
     README.md                          the same log as a table, for GitHub
     flights/<id>/report.html           the report, ready to open
     flights/<id>/data/<name>           the flight's own files, when shared

   The index is the one source of truth. README.md is regenerated from it on
   every publish so the repository reads sensibly to someone who lands on it
   from GitHub rather than from the tool. */
export function flightDir(id) {
  return `flights/${id}`;
}

export function emptyIndex(owner) {
  return { schema: SCHEMA, pilot: { login: owner }, updated: null, flights: [] };
}

export function upsertIndex(index, entry, now = new Date()) {
  const out = structuredClone(index || emptyIndex(""));
  const pub = publicEntry(entry);
  const i = out.flights.findIndex((e) => e.id === entry.id);
  if (i >= 0) out.flights[i] = pub;
  else out.flights.push(pub);
  out.flights = sortEntries(out.flights);
  out.updated = now.toISOString();
  return out;
}

export function removeFromIndex(index, id, now = new Date()) {
  const out = structuredClone(index);
  out.flights = out.flights.filter((e) => e.id !== id);
  out.updated = now.toISOString();
  return out;
}

/* The entry as published: local bookkeeping stripped. */
export function publicEntry(entry) {
  const e = structuredClone(entry);
  delete e.published;
  delete e.keepData;
  return e;
}

/* Every path a flight occupies in the repository, for publishing or removal. */
export function flightPaths(entry) {
  const dir = flightDir(entry.id);
  const paths = [];
  if (entry.files && entry.files.report) paths.push(`${dir}/${entry.files.report}`);
  for (const name of (entry.files && entry.files.data) || []) paths.push(`${dir}/data/${name}`);
  return paths;
}

export function readme(index, siteUrl) {
  const login = (index.pilot && index.pilot.login) || "";
  const lines = [
    `# ${login ? login + "'s " : ""}TinkerRocket flight log`,
    "",
    `Published from the [TinkerRocket flight analysis tool](${siteUrl}). ` +
      `Browse it there: [${siteUrl}#/pilots/${login}](${siteUrl}#/pilots/${login}).`,
    "",
    "| Date | Rocket | Motor | Location | Computer | Apogee (m) | Max speed (m/s) | Outcome | Report |",
    "|---|---|---|---|---|---:|---:|---|---|",
  ];
  const md = (s) => String(s || "").replace(/\|/g, "\\|").replace(/\n/g, " ");
  for (const e of sortEntries(index.flights)) {
    // To the tool's view of the flight, not the file: GitHub shows a linked
    // .html file as its source, which is 9 MB of script to someone who
    // clicked "report".
    const report = e.files && e.files.report ? `[report](${siteUrl}#/pilots/${login}/${e.id})` : "";
    lines.push(`| ${[
      md(e.date && (e.date.local || e.date.utc)),
      md(e.rocket), md(e.motor), md(e.location && e.location.name),
      md(computerText(e.computer)),
      round(e.results && e.results.apogee_m, 0),
      round(e.results && e.results.max_speed_mps, 0),
      md((OUTCOMES.find((o) => o.key === e.outcome) || {}).label),
      report,
    ].join(" | ")} |`);
  }
  lines.push("", "Delete this repository to take the log down.", "");
  return lines.join("\n");
}

/* A GitHub login, or null. Checked before it goes into a URL path. */
export function cleanLogin(text) {
  const s = String(text || "").trim().replace(/^@/, "");
  return /^[A-Za-z0-9](?:[A-Za-z0-9]|-(?=[A-Za-z0-9])){0,38}$/.test(s) ? s : null;
}

export function cleanId(text) {
  const s = String(text || "");
  return /^[a-z0-9][a-z0-9-]{0,80}$/.test(s) ? s : null;
}
