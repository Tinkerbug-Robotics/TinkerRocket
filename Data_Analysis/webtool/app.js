/* Main-thread UI logic for the post-flight analysis web tool.

   A module: the flight log (logui.js and the modules under it) is imported
   here, and the analysis hands each finished report to it. */

import { showLocalReport } from "./reports.js";
import * as logui from "./logui.js";

const el = (id) => document.getElementById(id);

const startupList = el("startup-list");
const dropzone = el("dropzone");
const fileInput = el("file-input");
const rocketNameInput = el("rocket-name");
const sampleBtn = el("sample-btn");
const generateBtn = el("generate-btn");
const stagedBox = el("staged");
const stagedSummary = el("staged-summary");
const clearBtn = el("clear-btn");
const runPanel = el("run-panel");
const moduleList = el("module-list");
const runStatus = el("run-status");
const resultPanel = el("result-panel");
const reportFrame = el("report-frame");
const downloadBtn = el("download-btn");
const openBtn = el("open-btn");
const globeNote = el("globe-note");
const errorBox = el("error-box");

let engineReady = false;
let running = false; // a run is executing in the worker
let pending = false; // file reads / sample fetches in flight, before the run starts
let runStartMs = 0;
let runTimer = null;
let reportBlobUrl = null;
let currentBinStem = "flight";
let currentBinName = null;
let lastFacts = null;
let keptFiles = [];
let keptRocketName = "";

// Started on the first visit to the analyze view rather than on page load: a
// link to someone's public log should not download a Python runtime.
let worker = null;

// ---------- startup progress ----------

const STARTUP_STAGES = ["runtime", "packages", "source"];
const startupRows = {};
for (const stage of STARTUP_STAGES) {
  const li = document.createElement("li");
  li.className = "pending";
  li.textContent = stage;
  startupList.appendChild(li);
  startupRows[stage] = li;
}

function setStartupStage(stage, detail) {
  for (const s of STARTUP_STAGES) {
    const row = startupRows[s];
    if (s === stage) {
      row.className = "active";
      row.textContent = detail;
    } else if (STARTUP_STAGES.indexOf(s) < STARTUP_STAGES.indexOf(stage)) {
      row.className = "done";
    }
  }
}

function setEngineReady() {
  engineReady = true;
  for (const s of STARTUP_STAGES) startupRows[s].className = "done";
  el("startup-panel").classList.add("collapsed");
  el("engine-badge").textContent = "engine ready";
  el("engine-badge").className = "badge ready";
  updateControls();
}

function setEngineFailed(message) {
  errorBox.hidden = false;
  errorBox.textContent =
    message +
    "\nThe engine could not start. The first load needs internet access to download the Python runtime — reconnect and reload the page.";
  el("engine-badge").textContent = "engine failed";
  el("engine-badge").className = "badge failed";
  for (const s of STARTUP_STAGES) {
    if (startupRows[s].className !== "done") startupRows[s].className = "failed";
  }
}

// ---------- run progress ----------

function beginRun(label) {
  running = true;
  errorBox.hidden = true;
  resultPanel.hidden = true;
  logui.hideEntry();
  lastFacts = null;
  runPanel.hidden = false;
  moduleList.innerHTML = "";
  runStatus.textContent = `Analyzing ${label}…`;
  runStartMs = performance.now();
  if (runTimer) clearInterval(runTimer);
  runTimer = setInterval(() => {
    const s = ((performance.now() - runStartMs) / 1000).toFixed(0);
    el("run-elapsed").textContent = `${s}s`;
  }, 250);
  updateControls();
}

function endRun() {
  running = false;
  clearInterval(runTimer);
  runTimer = null;
  updateControls();
}

function addModuleRow(name) {
  const li = document.createElement("li");
  li.className = "active";
  li.dataset.name = name;
  li.innerHTML = `<span class="mod-status"></span><span class="mod-name"></span><span class="mod-detail"></span>`;
  li.querySelector(".mod-name").textContent = name;
  moduleList.appendChild(li);
  return li;
}

function finishModuleRow(info) {
  const li = moduleList.querySelector(`li[data-name="${CSS.escape(info.name)}"]`);
  if (!li) return;
  li.className = info.status; // ok | warn | error
  const mark = { ok: "✓", warn: "⚠", error: "✗" }[info.status] || "";
  li.querySelector(".mod-status").textContent = mark;
  const figs = info.figures === 1 ? "1 figure" : `${info.figures} figures`;
  li.querySelector(".mod-detail").textContent = `${figs} · ${info.seconds}s`;
}

function clearError() {
  errorBox.hidden = true;
  errorBox.textContent = "";
}

function showError(message) {
  errorBox.hidden = false;
  errorBox.textContent = message;
  // A failed run shouldn't hide a previous good report.
  if (reportBlobUrl) resultPanel.hidden = false;
}

// ---------- worker messages ----------

function startEngine() {
  if (worker) return;
  el("engine-badge").hidden = false;
  worker = new Worker("worker.js");
  worker.onmessage = onWorkerMessage;
  worker.onerror = onWorkerError;
}

function onWorkerMessage(ev) {
  const msg = ev.data;
  switch (msg.type) {
    case "status":
      setStartupStage(msg.stage, msg.detail);
      break;
    case "ready":
      setEngineReady();
      break;
    case "progress":
      handleProgress(msg);
      break;
    case "done": {
      endRun();
      runStatus.textContent = `Report ready in ${((performance.now() - runStartMs) / 1000).toFixed(1)}s`;
      showReport(msg.html);
      const facts = { ...(lastFacts || {}) };
      facts.file = facts.file || currentBinName;
      logui.offerEntry({ html: msg.html, facts, files: keptFiles, rocketName: keptRocketName });
      break;
    }
    case "error":
      if (!engineReady) {
        setEngineFailed(msg.message);
        break;
      }
      endRun();
      runStatus.textContent = "Failed";
      showError(msg.message);
      break;
  }
}

function onWorkerError(ev) {
  const text = `Worker error: ${ev.message || "unknown"} (${ev.filename}:${ev.lineno})`;
  if (!engineReady) {
    setEngineFailed(text);
    return;
  }
  endRun();
  runStatus.textContent = "Failed";
  showError(text);
}

function handleProgress(msg) {
  switch (msg.kind) {
    case "parse_start":
      runStatus.textContent = "Parsing binary flight log…";
      break;
    case "parse_done": {
      const info = JSON.parse(msg.detail);
      runStatus.textContent = `Parsed ${info.frames.toLocaleString()} frames in ${info.seconds}s — running analysis modules…`;
      break;
    }
    case "module_start":
      addModuleRow(msg.detail);
      break;
    case "module_done":
      finishModuleRow(JSON.parse(msg.detail));
      break;
    case "facts":
      lastFacts = JSON.parse(msg.detail);
      break;
    case "render_start":
      runStatus.textContent = "Rendering report (encoding figures and charts)…";
      break;
  }
}

// ---------- report display ----------

async function showReport(html) {
  resultPanel.hidden = false;
  reportBlobUrl = true;
  resultPanel.scrollIntoView({ behavior: "smooth", block: "start" });
  // Old URLs are intentionally never revoked mid-session: an "Open in tab" tab
  // may still resolve one on reload. All report URLs die with this page.
  const served = await showLocalReport(html, currentBinStem, {
    frame: reportFrame, open: openBtn, download: downloadBtn,
  });
  // Say so rather than letting the reader wonder why one section is black.
  globeNote.hidden = served;
}

// ---------- file intake ----------

// Files wait here until the reader presses Generate. Dropping used to start the
// run on the spot, which meant the rocket name — the one thing only they can
// supply, and which titles the report — could only be set by knowing to type it
// before dragging anything in.
let staged = [];

function describeStaged() {
  const bin = staged.find((f) => f.name.toLowerCase().endsWith(".bin"));
  const others = staged.length - (bin ? 1 : 0);
  const name = bin ? bin.name : `${staged.length} file(s)`;
  return others ? `${name} + ${others} more` : name;
}

function setStaged(entries) {
  staged = entries;
  stagedBox.hidden = !staged.length;
  if (staged.length) stagedSummary.textContent = describeStaged();
  updateControls();
}

function updateControls() {
  const enabled = engineReady && !running && !pending;
  sampleBtn.disabled = !enabled;
  fileInput.disabled = !enabled;
  generateBtn.disabled = !enabled || !staged.length;
  dropzone.classList.toggle("disabled", !enabled);
}

function runFiles(fileEntries, rocketName) {
  if (running) return;
  // #850: base-station logs are BINARY now (lora_*.bin), so ".bin" alone no
  // longer identifies the rocket flight. Counting both kinds made dropping a
  // flight together with its base-station log fail as "two flights" — the exact
  // combination the fuller report wants.
  const isBaseStationLog = (name) => /^lora_/i.test(name);
  const bins = fileEntries.filter(
    (f) => f.name.toLowerCase().endsWith(".bin") && !isBaseStationLog(f.name)
  );
  if (bins.length === 0) {
    showError(
      "Select at least the .bin flight log (add the matching .json/.csv and any lora_* log for a fuller report)."
    );
    return;
  }
  if (bins.length > 1) {
    showError(
      `Found ${bins.length} .bin flight logs (${bins.map((f) => f.name).join(", ")}) — drop one flight at a time.`
    );
    return;
  }
  const binEntry = bins[0];
  currentBinName = binEntry.name;
  keptRocketName = rocketName;
  // The buffers are transferred to the worker and gone from here after the
  // postMessage, so the log keeps its own copies: the dropped File where there
  // is one (a reference, not a copy), a Blob of the bytes otherwise.
  keptFiles = fileEntries.map((f) => ({ name: f.name, blob: f.blob || new Blob([f.buffer]) }));
  // One report level, so nothing to choose. Same filename the CLI writes, so a
  // downloaded report sits alongside one generated locally without colliding.
  const level = "flight";
  currentBinStem = binEntry.name.replace(/\.bin$/i, "") + "_report";
  beginRun(binEntry.name);
  const payload = fileEntries.map((f) => ({ name: f.name, buffer: f.buffer }));
  worker.postMessage(
    { type: "run", files: payload, rocketName, binName: binEntry.name, level },
    payload.map((f) => f.buffer)
  );
}

// Serializes the async intake phase (file reads / sample fetches) so a second
// gesture during that window can't start an interleaved run.
async function withIntakeLock(fn) {
  if (!engineReady || running || pending) return;
  pending = true;
  updateControls();
  try {
    await fn();
  } finally {
    pending = false;
    updateControls();
  }
}

function intakeFileList(fileList) {
  if (!fileList.length) return;
  withIntakeLock(async () => {
    let entries;
    try {
      entries = await Promise.all(
        Array.from(fileList).map(async (f) => ({ name: f.name, buffer: await f.arrayBuffer(), blob: f }))
      );
    } catch (err) {
      showError(
        "Couldn't read a dropped item — if you dropped the flight folder, open it and drop the files inside it instead."
      );
      return;
    }
    setStaged(entries);
    clearError();
  });
}

generateBtn.addEventListener("click", () => {
  if (!staged.length) return;
  // The buffers are transferred to the worker, so this is a one-shot: keep the
  // summary on screen for context but drop our claim on the memory.
  const entries = staged;
  setStaged([]);
  runFiles(entries, rocketNameInput.value);
});

clearBtn.addEventListener("click", () => {
  setStaged([]);
  clearError();
});

fileInput.addEventListener("change", () => {
  intakeFileList(fileInput.files);
  fileInput.value = "";
});

dropzone.addEventListener("dragover", (ev) => {
  ev.preventDefault();
  if (engineReady && !running && !pending) dropzone.classList.add("dragging");
});
dropzone.addEventListener("dragleave", () => dropzone.classList.remove("dragging"));
dropzone.addEventListener("drop", (ev) => {
  ev.preventDefault();
  dropzone.classList.remove("dragging");
  intakeFileList(ev.dataTransfer.files);
});
dropzone.addEventListener("click", (ev) => {
  if (ev.target.closest("label")) return; // the label opens the picker itself
  if (engineReady && !running && !pending) fileInput.click();
});

// ---------- sample flight ----------

sampleBtn.addEventListener("click", () => {
  withIntakeLock(async () => {
    try {
      // no-cache for the same reason as the source zip in worker.js: stable URLs
      // across rebuilds, so revalidate rather than trust heuristic freshness.
      const manifest = await (
        await fetch("payload/sample/manifest.json", { cache: "no-cache" })
      ).json();
      const entries = await Promise.all(
        manifest.files.map(async (name) => {
          const resp = await fetch(`payload/sample/${encodeURIComponent(name)}`, {
            cache: "no-cache",
          });
          if (!resp.ok) throw new Error(`fetching sample ${name} failed (HTTP ${resp.status})`);
          return { name, buffer: await resp.arrayBuffer() };
        })
      );
      rocketNameInput.value = manifest.rocket_name;
      runFiles(entries, manifest.rocket_name);
    } catch (err) {
      showError(`Loading sample flight failed: ${err.message || err}`);
    }
  });
});

updateControls();

// ---------- views ----------

const VIEWS = ["analyze", "flights", "flight", "pilots", "pilot"];

/* #/                        analyze a flight
   #/flights                 my saved flights
   #/flights/<id>            one saved flight
   #/pilots                  public logs
   #/pilots/<login>          one flyer's log
   #/pilots/<login>/<id>     one public flight — the link a flyer shares */
function route() {
  const parts = location.hash.replace(/^#\/?/, "").split("/").filter(Boolean).map((p) => {
    try { return decodeURIComponent(p); } catch (err) { return ""; }
  });
  let view = "analyze";
  if (parts[0] === "flights") view = parts[1] ? "flight" : "flights";
  else if (parts[0] === "pilots") view = parts[2] ? "flight" : parts[1] ? "pilot" : "pilots";
  for (const v of VIEWS) el(`view-${v}`).hidden = v !== view;
  const tab = view === "analyze" ? "analyze" : parts[0];
  for (const a of document.querySelectorAll("#nav a")) a.classList.toggle("current", a.dataset.view === tab);
  if (view === "analyze") startEngine();
  el("engine-badge").hidden = view !== "analyze";
  if (view !== "analyze") {
    logui.render(view, parts);
    window.scrollTo(0, 0);
  }
}

logui.onCountChange((n) => { el("nav-count").textContent = n ? `(${n})` : ""; });
window.addEventListener("hashchange", route);
route();
