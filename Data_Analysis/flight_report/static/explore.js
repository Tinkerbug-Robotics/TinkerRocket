/* Explore panel — the parts that decide *what* to draw.

   Split out of the report template so it can be tested without a browser. The
   template keeps the DOM wiring (populate the picker, handle Draw, hand the
   result to Plotly); everything here is a pure function of the embedded
   dataset, which is where the mistakes live:

     · expanding the const / runs / y encodings back into series, including the
       carry-to-end that stops a step chart reading as the channel stopping;
     · deciding whether the selected channels share a unit, which governs
       whether the y axis may claim one and whether the reader is warned;
     · naming the axis without repeating the unit or leaking an internal label.

   All three shipped wrong at least once and none of them needs a DOM.

   Assigned to globalThis rather than exported as a module: the report is a
   single self-contained file, so render.py inlines this text into a <script>
   tag. The tests load the same file through node:vm, so what is tested is
   exactly what ships. */
(function (root) {
  "use strict";

  /* Matplotlib's tab10, matching charts.COLORS on the Python side, so a series
     plotted here is the colour it would have had if someone had written a
     module for it. */
  var COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf"
  ];

  /* The bucket a dimensionless channel falls into. It exists so that mixing a
     gyro rate with a 0-3 health code counts as two units and earns a warning;
     it must never reach an axis label. */
  var NO_UNIT = "no unit";

  function isStep(data, ch) {
    return (data.stepKinds || []).indexOf(ch.kind) >= 0;
  }

  /* One channel as {x, y} in stored (SI) units, or null when it never carried
     a value.

     Runs and constants expand to their corner points, not to one sample per
     frame: a step line through the corners is the same picture as 34,000
     repeated values, and rebuilding the full array would undo the reason they
     were encoded that way. */
  function seriesFor(data, key) {
    var ch = data.channels[key];
    if (!ch) return null;
    var stream = data.streams[ch.stream];
    if (!stream || !stream.t.length) return null;
    var t = stream.t, last = t.length - 1;

    if (ch.const !== undefined) {
      if (ch.const === null) return null;
      return {x: [t[0], t[last]], y: [ch.const, ch.const]};
    }

    if (ch.runs) {
      var xs = [], ys = [];
      for (var i = 0; i < ch.runs.length; i++) {
        var idx = ch.runs[i][0], v = ch.runs[i][1];
        if (v === null) continue;
        xs.push(t[idx]);
        ys.push(v);
      }
      if (!xs.length) return null;
      // Carry the last value to the end of the stream, or the step stops early
      // and reads as the channel ending rather than holding its value.
      xs.push(t[last]);
      ys.push(ys[ys.length - 1]);
      return {x: xs, y: ys};
    }

    var ax = [], ay = [];
    for (var j = 0; j < ch.y.length; j++) {
      if (ch.y[j] === null) continue;
      ax.push(t[j]);
      ay.push(ch.y[j]);
    }
    return ax.length ? {x: ax, y: ay} : null;
  }

  /* Scale to 0-1. A flat series has no range to scale into, so it goes to the
     middle of the lane rather than to 0 or NaN. */
  function normalise(ys) {
    var lo = Infinity, hi = -Infinity;
    for (var i = 0; i < ys.length; i++) {
      if (ys[i] < lo) lo = ys[i];
      if (ys[i] > hi) hi = ys[i];
    }
    var span = hi - lo;
    if (!span) return ys.map(function () { return 0.5; });
    return ys.map(function (v) { return (v - lo) / span; });
  }

  /* Which units the selection spans, and whether the axis may claim one.

     The axis may only claim a unit when every series shares it AND the toggle
     knows how to convert it — otherwise switching to imperial would rescale
     values that are not lengths. */
  function unitsFor(data, keys, opts) {
    opts = opts || {};
    var convertible = opts.convertible || [];
    var seen = {};
    keys.forEach(function (key) {
      var ch = data.channels[key];
      if (!ch) return;
      seen[ch.unit || NO_UNIT] = true;
    });
    var names = Object.keys(seen).sort();
    var single = names.length === 1 ? names[0] : null;
    var yUnit = (!opts.normalised && single && single !== NO_UNIT
                 && convertible.indexOf(single) >= 0) ? single : null;
    return {
      names: names,
      yUnit: yUnit,
      // "Value", never the unit name: the caller appends the unit itself, so
      // using it as the base reads "m (m)".
      yBase: opts.normalised ? "Normalised (0–1)" : "Value",
      mixed: names.length > 1
    };
  }

  /* Everything the panel needs to plot a selection.

     Returns Plotly traces with empty x/y — the viewport renderer fills them
     from `full`, which is the whole series at full resolution. */
  function build(data, keys, opts) {
    opts = opts || {};
    var traces = [], full = [], skipped = [], cautions = [], notes = [];

    keys.forEach(function (key, i) {
      var ch = data.channels[key];
      var ser = seriesFor(data, key);
      if (!ser) { skipped.push(key); return; }

      var ys = opts.normalised ? normalise(ser.y) : ser.y;
      var step = isStep(data, ch);
      var mode = step ? "lines" : "lines+markers";
      var color = COLORS[i % COLORS.length];

      full.push({x: ser.x, y: ys, baseMode: mode});
      traces.push({
        type: "scatter", mode: mode, name: key, x: [], y: [], yaxis: "y",
        line: {width: 1.4, color: color, shape: step ? "hv" : "linear"},
        marker: {size: 4, color: color},
        hovertemplate: key + ": %{y:.4g}<br>%{x:.3f} s<extra></extra>"
      });
      if (ch.caution) cautions.push(key + " — " + ch.caution);
    });

    var units = unitsFor(data, keys.filter(function (k) {
      return skipped.indexOf(k) < 0;
    }), opts);

    if (units.mixed && !opts.normalised) {
      notes.push("These series do not share a unit (" + units.names.join(", ") +
                 "), so they share an axis that fits none of them — a " +
                 "small-valued one will sit flat against a large-valued one. " +
                 "Tick Normalise to compare their shapes instead.");
    }
    if (skipped.length) {
      notes.push("Skipped (never carried a value): " + skipped.join(", ") + ".");
    }

    return {
      traces: traces, full: full, skipped: skipped,
      notes: notes, cautions: cautions,
      yUnit: units.yUnit, yBase: units.yBase, unitNames: units.names
    };
  }

  root.TRExplore = {
    COLORS: COLORS,
    NO_UNIT: NO_UNIT,
    seriesFor: seriesFor,
    normalise: normalise,
    unitsFor: unitsFor,
    build: build
  };
})(typeof globalThis !== "undefined" ? globalThis : this);
