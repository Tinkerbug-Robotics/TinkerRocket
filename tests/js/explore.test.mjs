/* Tests for the Explore panel's plotting decisions (#752).
 *
 * Run with:  node --test tests/js/
 *
 * No npm, no package.json, no dependencies — `node --test` and `node:vm` are
 * both built in, and Node is preinstalled on GitHub runners. This repo has
 * never carried JS tooling and this does not start.
 *
 * The file under test is loaded through node:vm exactly as it ships, rather
 * than being converted to a module for testing. render.py inlines that same
 * text into a <script> tag, so there is no second copy to drift.
 *
 * Every case here is a bug that actually shipped, or the encoding invariant
 * that a bug would break. They were found by driving a browser by hand, which
 * is what these tests exist to replace.
 */

import { test } from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import vm from "node:vm";

const HERE = dirname(fileURLToPath(import.meta.url));
const SRC = join(HERE, "..", "..", "Data_Analysis", "flight_report", "static", "explore.js");

/* Evaluated in *this* realm, not a fresh vm context.
 *
 * A separate context gives the sandbox its own Array and Object, so every
 * value the module returns fails deepStrictEqual against a host-realm literal
 * — "same structure but not reference-equal". Same realm keeps the assertions
 * honest, and the file is still loaded verbatim rather than being rewritten as
 * a module for testing. */
function load() {
  vm.runInThisContext(readFileSync(SRC, "utf8"), { filename: SRC });
  assert.ok(globalThis.TRExplore, "explore.js did not define TRExplore");
  return globalThis.TRExplore;
}

/* A dataset shaped exactly like modules/explore.py emits. */
function dataset() {
  return {
    stepKinds: ["bool", "enum"],
    streams: {
      IMU: { t: [0, 0.1, 0.2, 0.3, 0.4], n: 5 },
      NAV: { t: [0, 0.25, 0.5], n: 3 },
    },
    channels: {
      "IMU.gyro_x": { stream: "IMU", label: "Gyro X", unit: "deg/s", kind: "float",
                      caution: "Rails at ±4000 dps.", shown: [],
                      y: [1, 2, 3, 4, 5] },
      "IMU.accel_x": { stream: "IMU", label: "Accel X", unit: "m/s²", kind: "float",
                       caution: "", shown: ["overview"], y: [9.8, 9.8, 20, 9.8, 9.8] },
      "NAV.health": { stream: "NAV", label: "Health", unit: "", kind: "enum",
                      caution: "0 is \"no verdict\", not healthy.", shown: [],
                      runs: [[0, 1], [1, 3]] },
      "NAV.armed": { stream: "NAV", label: "Armed", unit: "", kind: "bool",
                     caution: "", shown: [], const: 0 },
      "NAV.missing": { stream: "NAV", label: "Missing", unit: "", kind: "float",
                       caution: "", shown: [], y: [null, null, null] },
      "NAV.never": { stream: "NAV", label: "Never set", unit: "", kind: "enum",
                     caution: "", shown: [], const: null },
      "IMU.alt": { stream: "IMU", label: "Altitude", unit: "m", kind: "float",
                   caution: "", shown: [], y: [10, 20, 30, 40, 50] },
      // Displays as m/s, converts as ft/s — the vertical-rate idiom.
      "IMU.u_vel": { stream: "IMU", label: "Climb rate", unit: "m/s",
                     conv: "m/s:fps", kind: "float", caution: "", shown: [],
                     y: [0, 5, 10, 5, 0] },
      "IMU.speed": { stream: "IMU", label: "Speed", unit: "m/s", conv: "",
                     kind: "float", caution: "", shown: [], y: [1, 2, 3, 4, 5] },
    },
  };
}

const CONVERTIBLE = ["m", "km", "m/s", "m/s:fps", "m/s²", "s", "G", "kg", "°"];

test("the module defines its own palette", () => {
  // A missing COLORS global threw a ReferenceError on every Draw and nothing
  // rendered at all. It was a Python constant that had no JS counterpart.
  const E = load();
  assert.ok(Array.isArray(E.COLORS) && E.COLORS.length >= 10);
  assert.match(E.COLORS[0], /^#[0-9a-f]{6}$/i);
});

test("a plain series drops nulls and keeps its timestamps", () => {
  const E = load();
  const s = E.seriesFor(dataset(), "IMU.gyro_x");
  assert.deepEqual(s.x, [0, 0.1, 0.2, 0.3, 0.4]);
  assert.deepEqual(s.y, [1, 2, 3, 4, 5]);
});

test("a run-encoded channel expands to corners and holds to the end", () => {
  const E = load();
  const s = E.seriesFor(dataset(), "NAV.health");
  // Two runs plus the carry-to-end point. Without the last one the step stops
  // at t=0.25 and reads as the channel ending rather than holding its value.
  assert.deepEqual(s.x, [0, 0.25, 0.5]);
  assert.deepEqual(s.y, [1, 3, 3]);
});

test("a constant channel draws as two points, not one per frame", () => {
  const E = load();
  const s = E.seriesFor(dataset(), "NAV.armed");
  assert.deepEqual(s.x, [0, 0.5]);
  assert.deepEqual(s.y, [0, 0]);
});

test("channels that never carried a value return null", () => {
  const E = load();
  const data = dataset();
  assert.equal(E.seriesFor(data, "NAV.missing"), null);
  assert.equal(E.seriesFor(data, "NAV.never"), null);
  assert.equal(E.seriesFor(data, "NAV.nonexistent"), null);
});

test("one shared unit lets the axis claim it", () => {
  const E = load();
  const u = E.unitsFor(dataset(), ["IMU.alt"], { convertible: CONVERTIBLE });
  assert.equal(u.yUnit, "m");
  assert.equal(u.yBase, "Value");
  assert.equal(u.mixed, false);
});

test("a dimensionless channel counts as its own unit", () => {
  // The bug: gyro (deg/s) plus a 0-3 health code counted as "one unit", the
  // axis was labelled deg/s, and the code sat invisibly on the zero line of a
  // ±2000 axis with nothing saying why.
  const E = load();
  const u = E.unitsFor(dataset(), ["IMU.gyro_x", "NAV.health"],
                       { convertible: CONVERTIBLE });
  assert.equal(u.mixed, true, "mixing a unit with a dimensionless channel must warn");
  assert.equal(u.yUnit, null, "the axis must not claim a unit it does not share");
  assert.ok(u.names.includes("no unit"));
});

test("the axis never claims a unit the toggle cannot convert", () => {
  const E = load();
  // deg/s is a real unit but is identity-only; claiming it is fine, but a unit
  // absent from the conversion table must not reach the axis.
  const u = E.unitsFor(dataset(), ["IMU.gyro_x"], { convertible: CONVERTIBLE });
  assert.equal(u.yUnit, null);
  assert.equal(u.names.length, 1);
});

test("the axis base is never the unit name and never the internal bucket", () => {
  // Two shipped bugs: using the unit as the base produced "m (m)", and a
  // constant dimensionless channel leaked "no unit" onto the axis.
  const E = load();
  const withUnit = E.unitsFor(dataset(), ["IMU.alt"], { convertible: CONVERTIBLE });
  assert.equal(withUnit.yBase, "Value");
  assert.notEqual(withUnit.yBase, "m");

  const dimensionless = E.unitsFor(dataset(), ["NAV.armed"], { convertible: CONVERTIBLE });
  assert.equal(dimensionless.yBase, "Value");
  assert.notEqual(dimensionless.yBase, E.NO_UNIT);
  assert.equal(dimensionless.yUnit, null);
});

test("normalising maps to 0-1 and puts a flat series mid-lane", () => {
  const E = load();
  assert.deepEqual(E.normalise([0, 5, 10]), [0, 0.5, 1]);
  assert.deepEqual(E.normalise([7, 7, 7]), [0.5, 0.5, 0.5]);
});

test("normalised output claims no unit at all", () => {
  const E = load();
  const u = E.unitsFor(dataset(), ["IMU.alt"],
                       { convertible: CONVERTIBLE, normalised: true });
  assert.equal(u.yUnit, null);
  assert.match(u.yBase, /Normalised/);
});

test("build draws step kinds as steps and continuous ones as lines", () => {
  const E = load();
  const out = E.build(dataset(), ["IMU.gyro_x", "NAV.health"],
                      { convertible: CONVERTIBLE });
  assert.equal(out.traces.length, 2);
  assert.equal(out.traces[0].line.shape, "linear");
  assert.equal(out.traces[1].line.shape, "hv");
  // x/y start empty; the viewport renderer fills them from `full`.
  assert.deepEqual(out.traces[0].x, []);
  assert.equal(out.full[0].y.length, 5);
});

test("build skips empty channels and says so, rather than drawing nothing", () => {
  const E = load();
  const out = E.build(dataset(), ["IMU.alt", "NAV.missing"],
                      { convertible: CONVERTIBLE });
  assert.deepEqual(out.skipped, ["NAV.missing"]);
  assert.equal(out.traces.length, 1);
  assert.ok(out.notes.some((n) => n.includes("NAV.missing")));
});

test("a skipped channel does not drag a phantom unit into the axis decision", () => {
  const E = load();
  // NAV.missing is dimensionless and empty. If it still voted, a lone
  // altitude trace would come out "mixed" and lose its axis unit.
  const out = E.build(dataset(), ["IMU.alt", "NAV.missing"],
                      { convertible: CONVERTIBLE });
  assert.equal(out.yUnit, "m");
});

test("build carries the provenance cautions", () => {
  // A picker listing bare names invites the #514 mistake, so the warning
  // travels with the channel rather than living in another section.
  const E = load();
  const out = E.build(dataset(), ["IMU.gyro_x", "NAV.health"],
                      { convertible: CONVERTIBLE });
  assert.equal(out.cautions.length, 2);
  assert.ok(out.cautions[0].startsWith("IMU.gyro_x — "));
});

test("colours cycle rather than running out", () => {
  const E = load();
  const data = dataset();
  const keys = Object.keys(data.channels).filter((k) => E.seriesFor(data, k));
  const out = E.build(data, keys.concat(keys), { convertible: CONVERTIBLE });
  for (const tr of out.traces) {
    assert.match(tr.line.color, /^#[0-9a-f]{6}$/i);
  }
});

/* ---- channel vs channel (X–Y) ------------------------------------------ */

test("expand puts the compact encodings back to full length", () => {
  const E = load();
  const data = dataset();
  // const -> one entry per frame of its stream
  assert.deepEqual(E.expand(data, "NAV.armed"), [0, 0, 0]);
  // runs -> value held until the next run starts
  assert.deepEqual(E.expand(data, "NAV.health"), [1, 3, 3]);
  // plain y passes through
  assert.deepEqual(E.expand(data, "IMU.gyro_x"), [1, 2, 3, 4, 5]);
});

test("pairing is by index, so X and Y are the same instant", () => {
  const E = load();
  const p = E.pairFor(dataset(), "IMU.gyro_x", "IMU.alt");
  assert.deepEqual(p.x, [1, 2, 3, 4, 5]);
  assert.deepEqual(p.y, [10, 20, 30, 40, 50]);
  assert.deepEqual(p.t, [0, 0.1, 0.2, 0.3, 0.4]);
});

test("pairing drops an index where either side has no value", () => {
  const E = load();
  const data = dataset();
  data.channels["IMU.holey"] = {
    stream: "IMU", label: "Holey", unit: "", kind: "float", caution: "", shown: [],
    y: [1, null, 3, null, 5],
  };
  const p = E.pairFor(data, "IMU.holey", "IMU.alt");
  assert.deepEqual(p.x, [1, 3, 5]);
  assert.deepEqual(p.y, [10, 30, 50], "y must drop the same indices as x");
});

test("X–Y refuses to pair across streams rather than interpolating", () => {
  // The report is tested not to resample one sensor onto another's clock
  // (test_globe_tracks_are_independently_sampled). GNSS at 18 Hz against the
  // IMU at 963 has no honest pairing, so this must refuse and explain.
  const E = load();
  const out = E.buildScatter(dataset(), "IMU.gyro_x", ["NAV.health"], {});
  assert.equal(out.traces.length, 0);
  assert.match(out.error, /same stream/i);
  assert.match(out.error, /interpolat/i);
});

test("X–Y accepts channels that share a stream", () => {
  const E = load();
  const out = E.buildScatter(dataset(), "IMU.gyro_x", ["IMU.alt"], {});
  assert.equal(out.error, null);
  assert.equal(out.traces.length, 1);
  assert.equal(out.traces[0].mode, "markers");
  assert.deepEqual(out.traces[0].x, [1, 2, 3, 4, 5]);
  assert.equal(out.xUnit, "deg/s");
  assert.equal(out.yUnit, "m");
});

test("X–Y says the points are real samples, not interpolated", () => {
  const E = load();
  const out = E.buildScatter(dataset(), "IMU.gyro_x", ["IMU.alt"], {});
  assert.ok(out.notes.some((n) => /not interpolated/i.test(n)));
});

test("stride thins to a budget and reports how much it dropped", () => {
  const E = load();
  const xs = Array.from({ length: 1000 }, (_, i) => i);
  const r = E.stride(xs, xs, 100);
  assert.equal(r.shown, 100);
  assert.equal(r.total, 1000);
  assert.equal(r.x[0], 0);
  // Under budget, nothing is touched.
  const small = E.stride([1, 2, 3], [1, 2, 3], 100);
  assert.deepEqual(small.x, [1, 2, 3]);
  assert.equal(small.shown, small.total);
});

test("a thinned X–Y plot says so rather than passing for the whole set", () => {
  const E = load();
  const data = dataset();
  const n = 5000;
  data.streams.BIG = { t: Array.from({ length: n }, (_, i) => i * 0.01), n };
  data.channels["BIG.a"] = { stream: "BIG", label: "A", unit: "m", kind: "float",
                             caution: "", shown: [],
                             y: Array.from({ length: n }, (_, i) => i) };
  data.channels["BIG.b"] = { stream: "BIG", label: "B", unit: "m", kind: "float",
                             caution: "", shown: [],
                             y: Array.from({ length: n }, (_, i) => n - i) };
  const out = E.buildScatter(data, "BIG.a", ["BIG.b"], { budget: 500 });
  assert.ok(out.notes.some((n2) => /Showing .* of .* points/.test(n2)));
  assert.equal(out.traces[0].x.length, 500);
  // The full arrays are kept so zoom can redraw from real data.
  assert.equal(out.full[0].x.length, n);
});

test("zooming an X–Y plot filters on both axes, not just x", () => {
  // The reason X–Y cannot reuse the time-axis viewport code: lowerBound
  // binary-searches assuming x increases, which is false here.
  const E = load();
  const series = { x: [0, 5, 10, 15, 20], y: [0, 50, 0, 50, 0], t: [0, 1, 2, 3, 4] };
  const box = E.scatterView(series, [4, 16], [40, 60], 1000);
  assert.deepEqual(box.x, [5, 15]);
  assert.deepEqual(box.y, [50, 50]);
});

test("colour-by-time applies only to a single series", () => {
  const E = load();
  const one = E.buildScatter(dataset(), "IMU.gyro_x", ["IMU.alt"],
                             { colorByTime: true });
  assert.ok(Array.isArray(one.traces[0].marker.color), "expected a per-point colour array");
  assert.equal(one.traces[0].marker.showscale, true);

  const two = E.buildScatter(dataset(), "IMU.gyro_x", ["IMU.alt", "IMU.accel_x"],
                             { colorByTime: true });
  for (const tr of two.traces) {
    assert.equal(typeof tr.marker.color, "string",
                 "two colour-coded clouds cannot be told apart; use flat colours");
  }
});

test("X–Y carries cautions for both axes", () => {
  const E = load();
  const out = E.buildScatter(dataset(), "IMU.gyro_x", ["IMU.alt"], {});
  assert.ok(out.cautions.some((c) => c.startsWith("IMU.gyro_x — ")),
            "the X channel's caution must travel too");
});

test("zoom re-slices the colour array with the points it belongs to", () => {
  /* When a series is coloured by time, marker.color is a per-point array.
     Filtering x and y on zoom without filtering it zips a stale colour list
     against new coordinates — the plot then ships wrong colours rather than
     failing, which is the harder kind of bug to notice. */
  const E = load();
  const series = { x: [0, 5, 10, 15, 20], y: [0, 50, 0, 50, 0], t: [0, 1, 2, 3, 4] };
  const box = E.scatterView(series, [4, 16], [40, 60], 1000);
  assert.deepEqual(box.x, [5, 15]);
  assert.deepEqual(box.t, [1, 3], "time must survive the same filter as x and y");
  assert.equal(box.t.length, box.x.length);

  // And through the stride, not just the filter.
  const n = 1000;
  const big = {
    x: Array.from({ length: n }, (_, i) => i),
    y: Array.from({ length: n }, (_, i) => i),
    t: Array.from({ length: n }, (_, i) => i / 100),
  };
  const thinned = E.scatterView(big, null, null, 50);
  assert.equal(thinned.x.length, 50);
  assert.equal(thinned.t.length, 50);
  for (let i = 0; i < thinned.x.length; i++) {
    assert.equal(thinned.t[i], thinned.x[i] / 100, "colour desynced from its point");
  }
});

test("a series with no time still zooms", () => {
  const E = load();
  const box = E.scatterView({ x: [1, 2, 3], y: [1, 2, 3] }, [0, 10], null, 100);
  assert.deepEqual(box.x, [1, 2, 3]);
  assert.equal(box.t, null);
});

test("a vertical rate converts as ft/s, not mph", () => {
  /* units.py keeps a separate "m/s:fps" row for exactly this and says quoting
     a chute's sink rate in mph "reads as an error to anyone in the hobby". The
     catalog records it per channel; the panel has to carry it through, or the
     axis silently uses the airspeed idiom. */
  const E = load();
  const u = E.unitsFor(dataset(), ["IMU.u_vel"], { convertible: CONVERTIBLE });
  assert.equal(u.yUnit, "m/s", "the displayed unit is still m/s");
  assert.equal(u.yConv, "m/s:fps", "but it must convert by the ft/s row");
});

test("a horizontal speed keeps the plain m/s conversion", () => {
  const E = load();
  const u = E.unitsFor(dataset(), ["IMU.speed"], { convertible: CONVERTIBLE });
  assert.equal(u.yUnit, "m/s");
  assert.equal(u.yConv, "", "no override means convert by the unit itself");
});

test("X\u2013Y carries the conversion idiom on both axes", () => {
  const E = load();
  const out = E.buildScatter(dataset(), "IMU.alt", ["IMU.u_vel"], {});
  assert.equal(out.xUnit, "m");
  assert.equal(out.xConv, "", "altitude converts by its own unit");
  assert.equal(out.yUnit, "m/s");
  assert.equal(out.yConv, "m/s:fps", "climb rate on the Y axis must still be ft/s");
});
