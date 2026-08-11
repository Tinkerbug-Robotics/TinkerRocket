/* Web Worker: runs the unmodified flight_report Python package under Pyodide.
 *
 * Protocol (worker -> main):
 *   {type:"status", stage, detail}          startup stages
 *   {type:"ready"}                          engine ready for runs
 *   {type:"progress", kind, index, total, detail}   per-run progress
 *   {type:"done", html}                     finished report HTML
 *   {type:"error", message}                 startup or run failure
 * (main -> worker):
 *   {type:"run", files:[{name, buffer}], rocketName, binName, level}
 * `metadata` is accepted but no longer sent — see the note on Flight.metadata.
 */

importScripts("https://cdn.jsdelivr.net/pyodide/v0.29.1/full/pyodide.js");

const post = (msg) => self.postMessage(msg);

let runSeq = 0;

// The Python entry point. Mirrors cli._process_one but reports progress to JS
// and returns the report HTML instead of writing it.
const RUNNER_PY = `
import json
import time

def run_flight(bin_path, level, metadata_json, progress):
    from flight_report.flight import Flight
    from flight_report.registry import modules_for, run_module
    from flight_report.render import render_report

    flight = Flight.from_bin(bin_path)
    flight.metadata = json.loads(metadata_json or "{}")
    progress("parse_start", 0, 0, "")
    t0 = time.time()
    flight.load()
    frames = flight.stats.get("total_frames", 0)
    if frames == 0:
        raise ValueError(
            f"No data frames found in {bin_path.rsplit('/', 1)[-1]} — "
            "this doesn't look like a TinkerRocket flight log."
        )
    progress("parse_done", 0, 0, json.dumps({
        "seconds": round(time.time() - t0, 1),
        "frames": frames,
    }))

    modules = modules_for(level)
    results = []
    total = len(modules)
    for i, (name, fn, _lvl) in enumerate(modules):
        progress("module_start", i, total, name)
        t0 = time.time()
        result = run_module(name, fn, flight)
        status = "error" if result.error else ("warn" if result.warnings else "ok")
        progress("module_done", i, total, json.dumps({
            "name": name,
            "status": status,
            "figures": len(result.figures),
            "seconds": round(time.time() - t0, 1),
        }))
        results.append(result)

    progress("render_start", 0, 0, "")
    return render_report(flight, results, level=level)

run_flight
`;

const pyodideReady = (async () => {
  post({ type: "status", stage: "runtime", detail: "Downloading Python runtime (one-time, cached by the browser)" });
  const pyodide = await loadPyodide();

  post({ type: "status", stage: "packages", detail: "Loading numpy, pandas, matplotlib, jinja2" });
  await pyodide.loadPackage(["numpy", "pandas", "matplotlib", "jinja2"]);

  post({ type: "status", stage: "source", detail: "Installing flight_report analysis package" });
  // no-cache revalidates instead of trusting heuristic freshness: the zip keeps
  // one URL across rebuilds, so a cached copy would silently run stale analysis.
  // Unchanged builds still 304 and reuse the cached bytes.
  const resp = await fetch("payload/flight_report_src.zip", { cache: "no-cache" });
  if (!resp.ok) throw new Error(`fetching flight_report_src.zip failed (HTTP ${resp.status}) — run build.py`);
  const buf = await resp.arrayBuffer();
  pyodide.FS.mkdirTree("/app/Data_Analysis");
  pyodide.unpackArchive(buf, "zip", { extractDir: "/app/Data_Analysis" });

  // Agg backend + sys.path BEFORE the first flight_report import (which
  // eagerly imports every analysis module).
  pyodide.runPython(`
import os, sys
os.environ["MPLBACKEND"] = "Agg"
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams["figure.max_open_warning"] = 0
sys.path.insert(0, "/app/Data_Analysis")
import flight_report
`);

  post({ type: "ready" });
  return pyodide;
})();

pyodideReady.catch((err) => post({ type: "error", message: "Startup failed: " + (err.message || err) }));

self.onmessage = async (ev) => {
  if (!ev.data || ev.data.type !== "run") return;

  let pyodide;
  try {
    pyodide = await pyodideReady;
  } catch {
    return; // startup error already posted
  }

  const { files, rocketName, binName, level, metadata } = ev.data;
  try {
    if (!binName) throw new Error("No .bin flight log among the selected files.");

    let safeName = String(rocketName || "").replace(/[/\0]/g, "-").trim();
    if (!safeName || /^\.+$/.test(safeName)) safeName = "My Rocket";

    // Fresh directory per run: the analysis discovers sidecars and lora_*.csv by
    // scanning the .bin's parent, so leftovers from an earlier run would silently
    // contaminate this report. The rocket name stays the immediate parent because
    // Flight.rocket_name is derived from it.
    const dir = "/data/flights/run" + runSeq++ + "/" + safeName;
    pyodide.FS.mkdirTree(dir);

    for (const f of files) {
      pyodide.FS.writeFile(dir + "/" + f.name, new Uint8Array(f.buffer));
    }

    const progress = (kind, index, total, detail) =>
      post({ type: "progress", kind, index, total, detail });

    const runFlight = pyodide.runPython(RUNNER_PY);
    try {
      const html = runFlight(dir + "/" + binName, level || "flight",
                             JSON.stringify(metadata || {}), progress);
      post({ type: "done", html, level: level || "flight" });
    } finally {
      runFlight.destroy();
    }
  } catch (err) {
    // Expected validation failures (a file that isn't a flight log) get the plain
    // message; genuine crashes keep their full Python traceback.
    let message = String(err.message || err);
    if (err && err.type === "ValueError") {
      message = message.trim().split("\n").pop().replace(/^ValueError:\s*/, "");
    }
    post({ type: "error", message });
  }
};
