/* Tests for the web tool's flight log rules (Data_Analysis/webtool/flightlog.js).
 *
 * Run with:  node --test tests/js/
 *
 * The module is plain ESM with no DOM, so it is imported exactly as the page
 * imports it. Each case is a decision a flyer would notice going wrong: two
 * flights sharing an id, a field named after the wrong town, a CSV that will
 * not open in the spreadsheet it replaces, a public log that drops a flight.
 */

import { test } from "node:test";
import assert from "node:assert/strict";

import {
  applyDetails, cleanId, cleanLogin, emptyIndex, flightId, flightPaths, localStamp,
  locationSuggestions, motorSuggestions, newEntry, parseMotor, placeName, previousField,
  readme, removeFromIndex, slug, sortEntries, toCSV, upsertIndex,
} from "../../Data_Analysis/webtool/flightlog.js";

// What flight_report.log_entry.log_facts says about the Goblin's 10/24 flight.
const GOBLIN = {
  file: "flight_20261024_144610.bin", rocket: "01 Goblin WM 4in",
  date_utc: "2026-10-24T14:45:10Z", date_source: "gnss",
  pad: { lat: 39.468661, lon: -75.292815, alt_m: 33.9, fixes: 30 },
  board: "V9", board_asserted: true, firmware: "41b2ab92", firmware_dirty: false,
  apogee_m: 844.76, max_speed_mps: 398.62, max_accel_g: 11.46, flight_time_s: 58.51,
  landing_distance_m: 374.6,
};

const NOW = new Date("2026-10-25T12:00:00Z");

test("an id is the liftoff time to the second, then the rocket", () => {
  assert.equal(flightId(GOBLIN, "Goblin WM 4in"), "20261024-144510-goblin-wm-4in");
  // Two rockets off one rack a minute apart never collide.
  assert.notEqual(
    flightId({ ...GOBLIN, date_utc: "2026-10-24T14:46:10Z" }, "Goblin WM 4in"),
    flightId(GOBLIN, "Goblin WM 4in"));
});

test("a log with no date falls back to its file name, not to a shared default", () => {
  const a = flightId({ file: "flight_20260101_120000.bin" }, "X");
  const b = flightId({ file: "flight_20260101_130000.bin" }, "X");
  assert.notEqual(a, b);
  assert.ok(cleanId(a), a);
});

test("slugs survive accents, punctuation and empty names", () => {
  assert.equal(slug("Rolly Polly 54mm"), "rolly-polly-54mm");
  assert.equal(slug("Égalité / 2.0!"), "egalite-2-0");
  assert.equal(slug("///"), "flight");
});

test("the local date is the flyer's day, not UTC's", () => {
  // 9 pm in California is already the next day in UTC.
  const s = localStamp("2026-10-25T04:00:00Z", "America/Los_Angeles");
  assert.deepEqual(s, { local: "2026-10-24 21:00", tz: "America/Los_Angeles" });
  // A filename date has no zone and is shown as the board's clock had it.
  assert.deepEqual(localStamp("2026-07-05T17:45:32", "America/New_York"), { local: "2026-07-05 17:45", tz: null });
});

test("a new entry carries the log's facts and the flyer's answers, and nothing else", () => {
  const e = newEntry(GOBLIN, {
    rocket: " Goblin WM 4in ", motor: "J350W-L", locationName: "BARC",
    locationSource: "log", outcome: "success", notes: "  Nominal. ",
  }, NOW, "America/New_York");
  assert.equal(e.rocket, "Goblin WM 4in");
  assert.equal(e.notes, "Nominal.");
  assert.equal(e.date.local, "2026-10-24 10:45");
  assert.deepEqual(e.computer, { board: "V9", board_asserted: true, firmware: "41b2ab92", firmware_dirty: false });
  assert.equal(e.location.lat, 39.468661);
  assert.equal(e.results.apogee_m, 844.76);
  assert.equal(e.outcome, "success");
  assert.equal(newEntry(GOBLIN, { outcome: "great" }, NOW).outcome, null);
});

test("edits change the flyer's columns and leave the log's alone", () => {
  const e = newEntry(GOBLIN, { rocket: "Goblin", motor: "J350W" }, NOW, "UTC");
  const later = new Date("2026-10-26T00:00:00Z");
  const f = applyDetails(e, { motor: "J350W-L", notes: "Recovered intact", outcome: "partial" }, later);
  assert.equal(f.motor, "J350W-L");
  assert.equal(f.outcome, "partial");
  assert.deepEqual(f.computer, e.computer);
  assert.deepEqual(f.results, e.results);
  assert.equal(f.created, e.created);
  assert.equal(f.updated, later.toISOString());
  assert.equal(e.motor, "J350W", "the original is not mutated");
});

test("a pad near an earlier named flight is suggested that name, nearest first", () => {
  const saved = [
    { id: "a", location: { name: "BARC", lat: 39.4687, lon: -75.2930 } },       // ~20 m
    { id: "b", location: { name: "Far field", lat: 39.4800, lon: -75.2928 } },  // ~1.3 km
    { id: "c", location: { name: "", lat: 39.4686, lon: -75.2928 } },           // unnamed
    { id: "d", location: { name: "CENJARS", lat: 40.1631, lon: -74.0807 } },    // another state
  ];
  const prev = previousField(GOBLIN.pad, saved);
  assert.equal(prev.name, "BARC");
  assert.ok(prev.km < 0.05);
  assert.equal(previousField({ lat: 0.5, lon: 0.5 }, saved), null);
});

test("the map's name is offered after the flyer's own, and only once", () => {
  const saved = [{ id: "a", location: { name: "BARC", lat: 39.4687, lon: -75.2930 } }];
  const s = locationSuggestions(GOBLIN.pad, saved, "Hopewell Township, NJ");
  assert.deepEqual(s.map((x) => [x.name, x.source]), [["BARC", "log"], ["Hopewell Township, NJ", "map"]]);
  const same = locationSuggestions(GOBLIN.pad, [{ id: "z", location: { name: "Hopewell Township, NJ", lat: 39.4687, lon: -75.293 } }],
    "hopewell township, nj");
  assert.equal(same.length, 1);
});

test("Nominatim answers become short place names", () => {
  // The real answer for the Goblin's pad.
  assert.equal(placeName({
    name: "Hopewell Township",
    address: { town: "Hopewell Township", county: "Cumberland County", state: "New Jersey",
      "ISO3166-2-lvl4": "US-NJ", country: "United States", country_code: "us" },
  }), "Hopewell Township, NJ");
  assert.equal(placeName({ address: { village: "Kiel", state: "Schleswig-Holstein", country_code: "de" } }),
    "Kiel, Schleswig-Holstein");
  assert.equal(placeName({ address: { county: "Black Rock", state: "Nevada", "ISO3166-2-lvl4": "US-NV", country_code: "us" } }),
    "Black Rock, NV");
  assert.equal(placeName({}), "");
});

test("motor designations parse loosely and never throw", () => {
  assert.deepEqual(parseMotor("H130W-8"), { impulse: "H", thrust: 130, propellant: "W", delay: "8" });
  assert.deepEqual(parseMotor("g80t-7"), { impulse: "G", thrust: 80, propellant: "T", delay: "7" });
  assert.deepEqual(parseMotor("J350W-P"), { impulse: "J", thrust: 350, propellant: "W", delay: "P" });
  assert.deepEqual(parseMotor("1/2A3-4T"), { impulse: "1/2A", thrust: 3, propellant: "", delay: "4" });
  assert.equal(parseMotor("Estes"), null);
  assert.equal(parseMotor(null), null);
});

test("motor suggestions are this flyer's motors, newest first, each once", () => {
  const entries = [
    { motor: "G74-6", date: { utc: "2026-10-18T00:00:00Z" } },
    { motor: "H128W-14", date: { utc: "2026-07-05T00:00:00Z" } },
    { motor: "g74-6", date: { utc: "2026-06-01T00:00:00Z" } },
    { motor: "", date: { utc: "2026-10-20T00:00:00Z" } },
  ];
  assert.deepEqual(motorSuggestions(entries), ["G74-6", "H128W-14"]);
});

test("flights sort by when they flew, not when they were saved", () => {
  const sorted = sortEntries([
    { id: "old", date: { utc: "2025-09-27T15:00:00Z" }, created: "2026-10-25T00:00:00Z" },
    { id: "new", date: { utc: "2026-10-24T14:45:10Z" }, created: "2026-10-01T00:00:00Z" },
  ]);
  assert.deepEqual(sorted.map((e) => e.id), ["new", "old"]);
});

test("the CSV opens in the spreadsheet it replaces", () => {
  const e = newEntry(GOBLIN, {
    rocket: "Goblin", motor: "J350W", locationName: "BARC", outcome: "partial",
    notes: 'Chute "mostly" open,\nfin cracked',
  }, NOW, "America/New_York");
  const csv = toCSV([e]);
  const [header, row] = csv.split("\r\n");
  assert.ok(header.startsWith("Date,Rocket,Motor,Club/Location,Computer,Firmware,Max Alt (m),Max Speed (m/s)"));
  assert.ok(row.startsWith("2026-10-24 10:45,Goblin,J350W,BARC,V9,41b2ab92,844.8,398.6,58.5,Partial,"));
  assert.ok(csv.includes('"Chute ""mostly"" open,\nfin cracked"'));
});

test("publishing a flight twice replaces it, and removing it drops only that one", () => {
  const a = newEntry(GOBLIN, { rocket: "Goblin" }, NOW, "UTC");
  const b = newEntry({ ...GOBLIN, date_utc: "2026-10-24T18:49:11Z" }, { rocket: "Piercer Mini" }, NOW, "UTC");
  let idx = upsertIndex(emptyIndex("flyer"), { ...a, published: { owner: "flyer" }, keepData: true }, NOW);
  idx = upsertIndex(idx, b, NOW);
  idx = upsertIndex(idx, { ...a, notes: "edited" }, NOW);
  assert.equal(idx.flights.length, 2);
  assert.equal(idx.flights.find((e) => e.id === a.id).notes, "edited");
  assert.ok(!("published" in idx.flights[0]) && !("keepData" in idx.flights[0]), "local bookkeeping is not published");
  assert.deepEqual(removeFromIndex(idx, a.id, NOW).flights.map((e) => e.id), [b.id]);
});

test("a flight's repository paths are its report and its shared files", () => {
  const e = { id: "20261024-144510-goblin", files: { report: "report.html", data: ["flight_x.bin", "lora_y.bin"] } };
  assert.deepEqual(flightPaths(e), [
    "flights/20261024-144510-goblin/report.html",
    "flights/20261024-144510-goblin/data/flight_x.bin",
    "flights/20261024-144510-goblin/data/lora_y.bin",
  ]);
});

test("the README table cannot be broken by a pipe in the notes", () => {
  const e = newEntry(GOBLIN, { rocket: "A | B", locationName: "x|y" }, NOW, "UTC");
  e.files.report = "report.html";
  const md = readme(upsertIndex(emptyIndex("flyer"), e, NOW), "https://example.test/");
  const row = md.split("\n").find((l) => l.includes("A \\| B"));
  assert.ok(row, md);
  assert.equal(row.split(/(?<!\\)\|/).length, 11, row);
  assert.ok(row.includes(`(https://example.test/#/pilots/flyer/${e.id})`));
});

test("only real GitHub logins and well-formed ids reach a URL", () => {
  assert.equal(cleanLogin("@octo-cat"), "octo-cat");
  for (const bad of ["", "-x", "a--b", "a/b", "../x", "x".repeat(40)]) assert.equal(cleanLogin(bad), null, bad);
  assert.equal(cleanId("20261024-144510-goblin"), "20261024-144510-goblin");
  for (const bad of ["../../etc", "A", "-x", "x/y"]) assert.equal(cleanId(bad), null, bad);
});
