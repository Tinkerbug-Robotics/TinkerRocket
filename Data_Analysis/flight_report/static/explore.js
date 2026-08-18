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
    var seen = {}, convs = {};
    keys.forEach(function (key) {
      var ch = data.channels[key];
      if (!ch) return;
      var u = ch.unit || NO_UNIT;
      seen[u] = true;
      convs[u] = ch.conv || "";
    });
    var names = Object.keys(seen).sort();
    var single = names.length === 1 ? names[0] : null;
    // A channel may display one unit and convert by another row — a vertical
    // rate is "m/s" but belongs in ft/s. Claim the axis only if the row the
    // toggle would actually use is one it knows.
    var convKey = single ? (convs[single] || single) : null;
    var yUnit = (!opts.normalised && single && single !== NO_UNIT
                 && convertible.indexOf(convKey) >= 0) ? single : null;
    return {
      names: names,
      yUnit: yUnit,
      yConv: yUnit ? (convs[single] || "") : "",
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
      yUnit: units.yUnit, yConv: units.yConv,
      yBase: units.yBase, unitNames: units.names
    };
  }

  /* ---- channel vs channel ------------------------------------------------

     A second mode, deliberately narrower than the time-series one.

     Same stream only. Pairing two channels means answering "what was Y when X
     was this?", and across streams there is no answer without inventing one:
     GNSS logs at 18 Hz and the IMU at 963, so a pair would have to come from
     interpolating one onto the other's clock. The report is tested not to do
     that (test_globe_tracks_are_independently_sampled — resampling "would
     invent samples and smooth away precisely the disagreement this section
     exists to show"). Within a stream every channel shares one timestamp
     array, so index i of X and index i of Y are the same instant, exactly.

     Nothing here goes through the viewport renderer. `lowerBound` binary-
     searches assuming x increases, and an X–Y plot's x does not — altitude
     rises and falls, so a zoom would return an arbitrary slice and draw it
     without complaint. `envelope` is worse: it keeps each bucket's min and max
     y, which flattens the up-leg/down-leg split that is usually the whole
     reason to plot one channel against another. */

  /* A channel as a full-length array, one entry per frame of its stream —
     the const and runs encodings put back the way they were stored. Needed
     because pairing is by index and the compact forms have no index to pair. */
  function expand(data, key) {
    var ch = data.channels[key];
    if (!ch) return null;
    var stream = data.streams[ch.stream];
    if (!stream) return null;
    var n = stream.n, out, i;

    if (ch.const !== undefined) {
      out = new Array(n);
      for (i = 0; i < n; i++) out[i] = ch.const;
      return out;
    }
    if (ch.runs) {
      out = new Array(n);
      for (var r = 0; r < ch.runs.length; r++) {
        var from = ch.runs[r][0];
        var to = (r + 1 < ch.runs.length) ? ch.runs[r + 1][0] : n;
        for (i = from; i < to; i++) out[i] = ch.runs[r][1];
      }
      return out;
    }
    return ch.y;
  }

  /* Uniform stride to a point budget.

     Stride, not the min/max envelope used on the time axis: an envelope is
     defined against a single ordered axis and there is no such ordering here.
     Zooming re-filters from the full arrays, so detail stays reachable, and
     the caller states how many of how many points are on screen rather than
     letting a thinned cloud pass for the whole set. */
  function stride(xs, ys, budget) {
    var n = xs.length;
    if (n <= budget) return {x: xs, y: ys, shown: n, total: n};
    var step = n / budget, ox = [], oy = [];
    for (var b = 0; b < budget; b++) {
      var i = Math.floor(b * step);
      ox.push(xs[i]); oy.push(ys[i]);
    }
    return {x: ox, y: oy, shown: ox.length, total: n};
  }

  /* Index-aligned (x, y) for two channels of one stream, dropping any index
     where either side has no value. Also returns the frame time of each kept
     point, so the caller can colour by time. */
  function pairFor(data, xKey, yKey) {
    var xs = expand(data, xKey), ys = expand(data, yKey);
    if (!xs || !ys) return null;
    var t = data.streams[data.channels[xKey].stream].t;
    var ox = [], oy = [], ot = [];
    var n = Math.min(xs.length, ys.length);
    for (var i = 0; i < n; i++) {
      if (xs[i] === null || ys[i] === null) continue;
      ox.push(xs[i]); oy.push(ys[i]); ot.push(t[i]);
    }
    return ox.length ? {x: ox, y: oy, t: ot} : null;
  }

  var SCATTER_BUDGET = 6000;

  /* Traces for an X–Y plot, or `error` explaining why there are none. */
  function buildScatter(data, xKey, yKeys, opts) {
    opts = opts || {};
    var budget = opts.budget || SCATTER_BUDGET;
    var xCh = data.channels[xKey];
    if (!xCh) return {error: "Pick a channel for the X axis.", traces: []};
    if (!yKeys.length) return {error: "Pick at least one channel for the Y axis.", traces: []};

    var wrong = yKeys.filter(function (k) {
      var c = data.channels[k];
      return c && c.stream !== xCh.stream;
    });
    if (wrong.length) {
      return {
        error: "X–Y needs both channels from the same stream. " + xKey + " is " +
               xCh.stream + "; " + wrong.join(", ") + " is not. Streams log at " +
               "different rates, so pairing across them would mean interpolating " +
               "one onto the other's clock and inventing samples that were never " +
               "measured. Plot them against time instead to compare.",
        traces: []
      };
    }

    var traces = [], full = [], notes = [], cautions = [], skipped = [];
    var thinned = false, shownTotal = 0, pointTotal = 0;

    yKeys.forEach(function (yKey, i) {
      var pair = pairFor(data, xKey, yKey);
      if (!pair) { skipped.push(yKey); return; }
      var red = stride(pair.x, pair.y, budget);
      if (red.shown < red.total) thinned = true;
      shownTotal += red.shown;
      pointTotal += red.total;

      var marker = {size: 3, color: COLORS[i % COLORS.length], opacity: 0.7};
      if (opts.colorByTime && yKeys.length === 1) {
        // Only for a single series: a colourbar per trace would be unreadable,
        // and two colour-coded clouds cannot be told apart.
        var tRed = stride(pair.t, pair.y, budget);
        marker = {
          size: 3, opacity: 0.85, color: tRed.x, colorscale: "Viridis",
          showscale: true, colorbar: {title: {text: "t (s)"}, thickness: 12}
        };
      }
      full.push({x: pair.x, y: pair.y, t: pair.t});
      traces.push({
        type: "scatter", mode: "markers", name: yKey,
        x: red.x, y: red.y, marker: marker,
        hovertemplate: xKey + ": %{x:.4g}<br>" + yKey + ": %{y:.4g}<extra></extra>"
      });
      var yCh = data.channels[yKey];
      if (yCh && yCh.caution) cautions.push(yKey + " — " + yCh.caution);
    });

    if (!traces.length) {
      return {error: "Nothing to draw: " + (skipped.join(", ") || xKey) +
                     " never carried a value.", traces: []};
    }
    if (xCh.caution) cautions.unshift(xKey + " — " + xCh.caution);

    notes.push("Each point is one frame of " + xCh.stream + ": X and Y are the " +
               "same sample, not interpolated.");
    if (thinned) {
      notes.push("Showing " + shownTotal.toLocaleString() + " of " +
                 pointTotal.toLocaleString() + " points, evenly spaced. Zoom to " +
                 "redraw from the full data.");
    }
    if (skipped.length) {
      notes.push("Skipped (never carried a value): " + skipped.join(", ") + ".");
    }

    var kept = yKeys.filter(function (k) { return skipped.indexOf(k) < 0; });
    var yUnits = {}, yConvs = {};
    kept.forEach(function (k) {
      var c = data.channels[k];
      if (!c) return;
      var u = c.unit || NO_UNIT;
      yUnits[u] = true;
      yConvs[u] = c.conv || "";
    });
    var yNames = Object.keys(yUnits);
    var ySingle = yNames.length === 1 && yNames[0] !== NO_UNIT ? yNames[0] : null;

    return {
      traces: traces, full: full, notes: notes, cautions: cautions,
      skipped: skipped, error: null,
      xUnit: xCh.unit || null, xConv: xCh.conv || "", xBase: xKey,
      yUnit: ySingle, yConv: ySingle ? (yConvs[ySingle] || "") : "",
      yBase: yKeys.length === 1 ? yKeys[0] : "Value"
    };
  }

  /* Points inside the visible box, then strided — the zoom path for X–Y.

     Carries the frame time through the same filter and the same stride as the
     coordinates. It has to: when a series is coloured by time, `marker.color`
     is a per-point array, and re-slicing x and y without re-slicing it zips a
     stale colour list against new coordinates. That does not fail loudly — it
     draws the wrong colours, which is worse than drawing none. */
  function scatterView(series, xRange, yRange, budget) {
    budget = budget || SCATTER_BUDGET;
    var xs = [], ys = [], ts = [];
    var haveT = !!series.t;
    for (var i = 0; i < series.x.length; i++) {
      var x = series.x[i], y = series.y[i];
      if (xRange && (x < xRange[0] || x > xRange[1])) continue;
      if (yRange && (y < yRange[0] || y > yRange[1])) continue;
      xs.push(x); ys.push(y);
      if (haveT) ts.push(series.t[i]);
    }
    var n = xs.length;
    if (n <= budget) {
      return {x: xs, y: ys, t: haveT ? ts : null, shown: n, total: n};
    }
    var step = n / budget, ox = [], oy = [], ot = [];
    for (var b = 0; b < budget; b++) {
      var j = Math.floor(b * step);
      ox.push(xs[j]); oy.push(ys[j]);
      if (haveT) ot.push(ts[j]);
    }
    return {x: ox, y: oy, t: haveT ? ot : null, shown: ox.length, total: n};
  }

  root.TRExplore = {
    COLORS: COLORS,
    NO_UNIT: NO_UNIT,
    SCATTER_BUDGET: SCATTER_BUDGET,
    seriesFor: seriesFor,
    normalise: normalise,
    unitsFor: unitsFor,
    build: build,
    expand: expand,
    stride: stride,
    pairFor: pairFor,
    buildScatter: buildScatter,
    scatterView: scatterView
  };
})(typeof globalThis !== "undefined" ? globalThis : this);
