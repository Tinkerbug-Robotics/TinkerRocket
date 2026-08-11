/* Main-thread UI logic for the post-flight analysis web tool. */

"use strict";

const el = (id) => document.getElementById(id);

const startupList = el("startup-list");
const dropzone = el("dropzone");
const fileInput = el("file-input");
const rocketNameInput = el("rocket-name");
const levelSelect = el("level");
const sampleBtn = el("sample-btn");
const runPanel = el("run-panel");
const moduleList = el("module-list");
const runStatus = el("run-status");
const resultPanel = el("result-panel");
const reportFrame = el("report-frame");
const downloadBtn = el("download-btn");
const openBtn = el("open-btn");
const errorBox = el("error-box");

let engineReady = false;
let running = false; // a run is executing in the worker
let pending = false; // file reads / sample fetches in flight, before the run starts
let runStartMs = 0;
let runTimer = null;
let reportBlobUrl = null;
let currentBinStem = "flight";

const worker = new Worker("worker.js");

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

function showError(message) {
  errorBox.hidden = false;
  errorBox.textContent = message;
  // A failed run shouldn't hide a previous good report.
  if (reportBlobUrl) resultPanel.hidden = false;
}

// ---------- worker messages ----------

worker.onmessage = (ev) => {
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
};

worker.onerror = (ev) => {
  const text = `Worker error: ${ev.message || "unknown"} (${ev.filename}:${ev.lineno})`;
  if (!engineReady) {
    setEngineFailed(text);
    return;
  }
  endRun();
  runStatus.textContent = "Failed";
  showError(text);
};

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
    case "render_start":
      runStatus.textContent = "Rendering report (encoding figures and charts)…";
      break;
  }
}

// ---------- report display ----------

function showReport(html) {
  resultPanel.hidden = false;

  // Load via blob URL, not srcdoc — a multi-MB srcdoc attribute stalls the renderer.
  // Old URLs are intentionally never revoked mid-session: an "Open full page" tab
  // may still resolve one on reload. All report URLs die with this page.
  reportBlobUrl = URL.createObjectURL(new Blob([html], { type: "text/html" }));
  reportFrame.src = reportBlobUrl;
  downloadBtn.href = reportBlobUrl;
  downloadBtn.download = `${currentBinStem}.html`;
  openBtn.href = reportBlobUrl;
  resultPanel.scrollIntoView({ behavior: "smooth", block: "start" });
}

// ---------- file intake ----------

function updateControls() {
  const enabled = engineReady && !running && !pending;
  sampleBtn.disabled = !enabled;
  fileInput.disabled = !enabled;
  levelSelect.disabled = !enabled;
  dropzone.classList.toggle("disabled", !enabled);
}

function runFiles(fileEntries, rocketName) {
  if (running) return;
  const bins = fileEntries.filter((f) => f.name.toLowerCase().endsWith(".bin"));
  if (bins.length === 0) {
    showError(
      "Select at least the .bin flight log (add the matching .json/.csv and any lora_*.csv for a fuller report)."
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
  const level = levelSelect.value;
  // Same filenames the CLI writes, so a downloaded report sits alongside one
  // generated locally without colliding or looking like a different artifact.
  currentBinStem =
    binEntry.name.replace(/\.bin$/i, "") +
    (level === "detailed" ? "_report_detailed" : "_report");
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
        Array.from(fileList).map(async (f) => ({ name: f.name, buffer: await f.arrayBuffer() }))
      );
    } catch (err) {
      showError(
        "Couldn't read a dropped item — if you dropped the flight folder, open it and drop the files inside it instead."
      );
      return;
    }
    runFiles(entries, rocketNameInput.value);
  });
}

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
