"""
Drift Cast Web GUI — Flask + Leaflet.js

Run:
    cd Code/tools
    python -m driftcast.app

Opens at http://localhost:5050
"""

import json
import traceback
from datetime import datetime, timezone, timedelta

from flask import Flask, request, jsonify, render_template_string

from .driftcast import compute_guidance_point

app = Flask(__name__)

# ---------------------------------------------------------------------------
# API endpoint
# ---------------------------------------------------------------------------

@app.route("/api/compute", methods=["POST"])
def api_compute():
    """Run the reverse drift cast and return JSON results."""
    try:
        data = request.get_json(force=True)

        result = compute_guidance_point(
            launch_lat=float(data["launch_lat"]),
            launch_lon=float(data["launch_lon"]),
            landing_lat=float(data["landing_lat"]),
            landing_lon=float(data["landing_lon"]),
            apogee_agl_ft=float(data["apogee_ft"]),
            drogue_rate_fps=float(data["drogue_rate_fps"]),
            main_rate_fps=float(data["main_rate_fps"]),
            main_deploy_agl_ft=float(data["main_deploy_ft"]),
            launch_time_utc=data["time_utc"],
            max_steering_deg=float(data.get("max_steering_deg", 15.0)),
        )

        return jsonify({
            "guidance_lat": result.guidance_lat,
            "guidance_lon": result.guidance_lon,
            "steering_angle_deg": round(result.steering_angle_deg, 2),
            "steering_bearing_deg": round(result.steering_bearing_deg, 1),
            "feasible": result.feasible,
            "infeasible_reason": result.infeasible_reason,
            "descent_track": [
                {"lat": p.lat, "lon": p.lon, "alt_agl_ft": round(p.alt_agl_ft, 1), "time_s": round(p.time_s, 1)}
                for p in result.descent_track
            ],
            "forward_landing_lat": result.forward_landing_lat,
            "forward_landing_lon": result.forward_landing_lon,
            "landing_error_m": round(result.landing_error_m, 1),
            "total_descent_time_s": round(result.total_descent_time_s, 1),
            "total_drift_m": round(result.total_drift_m, 0),
            "wind_profile": {
                "ground_elev_ft": round(result.wind_profile.ground_elev_ft, 0),
                "layers": [
                    {"alt_ft": round(l.alt_ft, 0), "speed_kts": round(l.speed_kts, 1), "direction_deg": round(l.direction_deg, 0)}
                    for l in result.wind_profile.layers
                ],
            },
        })

    except Exception as e:
        traceback.print_exc()
        return jsonify({"error": str(e)}), 400


# ---------------------------------------------------------------------------
# Main page
# ---------------------------------------------------------------------------

HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Reverse Drift Cast</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://cesium.com/downloads/cesiumjs/releases/1.114/Build/Cesium/Cesium.js"></script>
<link href="https://cesium.com/downloads/cesiumjs/releases/1.114/Build/Cesium/Widgets/widgets.css" rel="stylesheet">
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
         background: #1a1a2e; color: #e0e0e0; height: 100vh; display: flex; flex-direction: column; }

  header { background: #16213e; padding: 10px 20px; display: flex; align-items: center; gap: 16px;
           border-bottom: 2px solid #0f3460; }
  header h1 { font-size: 18px; color: #e94560; font-weight: 700; }
  header .subtitle { font-size: 12px; color: #888; }

  .main { display: flex; flex: 1; overflow: hidden; }

  /* --- Left panel --- */
  .panel { width: 340px; min-width: 340px; background: #16213e; overflow-y: auto;
           padding: 16px; display: flex; flex-direction: column; gap: 12px;
           border-right: 2px solid #0f3460; }

  .section-title { font-size: 13px; font-weight: 700; color: #e94560; text-transform: uppercase;
                   letter-spacing: 1px; margin-bottom: 4px; padding-bottom: 4px;
                   border-bottom: 1px solid #0f3460; }

  .click-mode { display: flex; gap: 8px; }
  .click-mode label { flex: 1; display: flex; align-items: center; justify-content: center;
                      gap: 6px; padding: 8px; border-radius: 6px; cursor: pointer;
                      font-size: 12px; font-weight: 600; border: 2px solid #333;
                      transition: all 0.15s; }
  .click-mode input[type=radio] { display: none; }
  .click-mode input[type=radio]:checked + span { }
  .click-mode label:has(input:checked) { border-color: #e94560; background: rgba(233,69,96,0.15); }
  .click-mode .launch-label { color: #ff6b6b; }
  .click-mode .landing-label { color: #51cf66; }

  .coord-row { display: flex; gap: 6px; align-items: center; }
  .coord-row label { font-size: 11px; color: #888; width: 54px; text-align: right; flex-shrink: 0; }
  .coord-row input { flex: 1; }

  .field-group { display: flex; flex-direction: column; gap: 6px; }
  .field-row { display: flex; gap: 8px; align-items: center; }
  .field-row label { font-size: 12px; color: #aaa; width: 110px; text-align: right; flex-shrink: 0; }
  .field-row input { flex: 1; }

  input[type=number], input[type=text], input[type=datetime-local] {
    background: #1a1a2e; border: 1px solid #333; border-radius: 4px;
    color: #e0e0e0; padding: 6px 8px; font-size: 13px; font-family: 'SF Mono', monospace; }
  input:focus { border-color: #e94560; outline: none; }

  .btn-compute { width: 100%; padding: 12px; background: #e94560; color: white;
                 border: none; border-radius: 6px; font-size: 14px; font-weight: 700;
                 cursor: pointer; transition: background 0.15s; letter-spacing: 0.5px; }
  .btn-compute:hover { background: #d63851; }
  .btn-compute:disabled { background: #555; cursor: wait; }

  /* Results */
  .results { font-size: 12px; }
  .result-row { display: flex; justify-content: space-between; padding: 3px 0;
                border-bottom: 1px solid #0f3460; }
  .result-label { color: #888; }
  .result-value { font-family: 'SF Mono', monospace; color: #e0e0e0; text-align: right; }
  .feasible { color: #51cf66; font-weight: 700; }
  .infeasible { color: #ff6b6b; font-weight: 700; }
  .error-msg { color: #ff6b6b; font-size: 12px; padding: 8px; background: rgba(255,107,107,0.1);
               border-radius: 4px; }

  /* Wind table */
  .wind-table { width: 100%; font-size: 11px; border-collapse: collapse; }
  .wind-table th { color: #888; font-weight: 600; text-align: right; padding: 2px 6px;
                   border-bottom: 1px solid #0f3460; }
  .wind-table td { text-align: right; padding: 2px 6px; font-family: 'SF Mono', monospace; }
  .wind-table tr:nth-child(even) { background: rgba(255,255,255,0.02); }

  /* --- Map --- */
  .map-container { flex: 1; position: relative; }
  #map { width: 100%; height: 100%; }

  /* Legend overlay */
  .legend { position: absolute; bottom: 20px; right: 20px; background: rgba(22,33,62,0.9);
            padding: 10px 14px; border-radius: 8px; z-index: 1000; font-size: 11px; }
  .legend-item { display: flex; align-items: center; gap: 6px; margin: 3px 0; }
  .legend-dot { width: 10px; height: 10px; border-radius: 50%; }

  /* 3D toggle */
  .btn-3d { padding: 6px 16px; background: transparent; color: #4dabf7; border: 2px solid #4dabf7;
            border-radius: 6px; font-size: 12px; font-weight: 700; cursor: pointer;
            transition: all 0.15s; letter-spacing: 0.5px; }
  .btn-3d:hover { background: rgba(77,171,247,0.15); }
  .btn-3d.active { background: #4dabf7; color: #1a1a2e; }

  /* Cesium overrides */
  #cesiumContainer .cesium-viewer-bottom,
  #cesiumContainer .cesium-viewer-animationContainer,
  #cesiumContainer .cesium-viewer-timelineContainer,
  #cesiumContainer .cesium-viewer-fullscreenContainer { display: none !important; }
</style>
</head>
<body>

<header>
  <h1>Reverse Drift Cast</h1>
  <span class="subtitle">Apogee Guidance Point Calculator</span>
  <div style="flex:1"></div>
  <button class="btn-3d" id="btn3d" onclick="toggle3D()">3D View</button>
</header>

<div class="main">
  <!-- Left panel -->
  <div class="panel">

    <!-- Click mode -->
    <div>
      <div class="section-title">Map Click Mode</div>
      <div class="click-mode">
        <label class="launch-label">
          <input type="radio" name="clickMode" value="launch" checked>
          <span>Set Launch</span>
        </label>
        <label class="landing-label">
          <input type="radio" name="clickMode" value="landing">
          <span>Set Landing</span>
        </label>
      </div>
    </div>

    <!-- Coordinates -->
    <div>
      <div class="section-title">Coordinates</div>
      <div class="field-group">
        <div class="coord-row">
          <label style="color:#ff6b6b">Launch lat</label>
          <input type="number" id="launch_lat" step="0.0001" value="37.7608">
        </div>
        <div class="coord-row">
          <label style="color:#ff6b6b">Launch lon</label>
          <input type="number" id="launch_lon" step="0.0001" value="-75.7446">
        </div>
        <div class="coord-row">
          <label style="color:#51cf66">Landing lat</label>
          <input type="number" id="landing_lat" step="0.0001" value="37.7608">
        </div>
        <div class="coord-row">
          <label style="color:#51cf66">Landing lon</label>
          <input type="number" id="landing_lon" step="0.0001" value="-75.7446">
        </div>
      </div>
    </div>

    <!-- Flight parameters -->
    <div>
      <div class="section-title">Flight Parameters</div>
      <div class="field-group">
        <div class="field-row">
          <label>Apogee (ft AGL)</label>
          <input type="number" id="apogee_ft" value="10000" step="100">
        </div>
        <div class="field-row">
          <label>Drogue (fps)</label>
          <input type="number" id="drogue_rate" value="60" step="1">
        </div>
        <div class="field-row">
          <label>Main (fps)</label>
          <input type="number" id="main_rate" value="18" step="1">
        </div>
        <div class="field-row">
          <label>Main deploy (ft)</label>
          <input type="number" id="main_deploy" value="700" step="50">
        </div>
        <div class="field-row">
          <label>Launch time</label>
          <input type="datetime-local" id="time_utc" step="3600">
        </div>
        <div class="field-row">
          <label>Max steer (deg)</label>
          <input type="number" id="max_steering" value="25" step="1">
        </div>
      </div>
    </div>

    <button class="btn-compute" id="btnCompute" onclick="compute()">COMPUTE</button>

    <!-- Results -->
    <div id="resultsSection" style="display:none">
      <div class="section-title">Results</div>
      <div class="results" id="resultsBody"></div>
    </div>

    <!-- Error -->
    <div id="errorSection" style="display:none">
      <div class="error-msg" id="errorMsg"></div>
    </div>

    <!-- Wind profile -->
    <div id="windSection" style="display:none">
      <div class="section-title">Wind Profile</div>
      <table class="wind-table">
        <thead><tr><th>Alt (ft)</th><th>Speed (kts)</th><th>From (deg)</th></tr></thead>
        <tbody id="windBody"></tbody>
      </table>
    </div>

  </div>

  <!-- Map -->
  <div class="map-container">
    <div id="map"></div>
    <div id="cesiumContainer" style="display:none; width:100%; height:100%; position:absolute; top:0; left:0;"></div>
    <div class="legend">
      <div class="legend-item"><div class="legend-dot" style="background:#ff6b6b"></div> Launch Pad</div>
      <div class="legend-item"><div class="legend-dot" style="background:#51cf66"></div> Landing Target</div>
      <div class="legend-item"><div class="legend-dot" style="background:#4dabf7"></div> Guidance Point</div>
      <div class="legend-item"><div style="width:20px;height:3px;background:#9775fa;border-radius:2px"></div> Descent Track</div>
    </div>
  </div>
</div>

<script>
// ---- Map setup ----
const defaultCenter = [37.7608, -75.7446];
const map = L.map('map').setView(defaultCenter, 14);

const osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19, attribution: '&copy; OpenStreetMap'
});
const satLayer = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
  maxZoom: 19, attribution: '&copy; Esri'
});
osmLayer.addTo(map);
L.control.layers({'Street': osmLayer, 'Satellite': satLayer}).addTo(map);

// ---- Markers ----
function makeIcon(color, size) {
  return L.divIcon({
    className: '',
    html: `<div style="width:${size}px;height:${size}px;background:${color};border-radius:50%;border:2px solid white;box-shadow:0 0 6px rgba(0,0,0,0.5);"></div>`,
    iconSize: [size, size],
    iconAnchor: [size/2, size/2],
  });
}

let launchMarker = null;
let landingMarker = null;
let guidanceMarker = null;
let descentLine = null;
let boostLine = null;

function setLaunchMarker(lat, lng) {
  if (launchMarker) map.removeLayer(launchMarker);
  launchMarker = L.marker([lat, lng], {icon: makeIcon('#ff6b6b', 16), draggable: true})
    .addTo(map)
    .bindTooltip('Launch Pad', {permanent: false});
  launchMarker.on('dragend', function(e) {
    const p = e.target.getLatLng();
    document.getElementById('launch_lat').value = p.lat.toFixed(6);
    document.getElementById('launch_lon').value = p.lng.toFixed(6);
  });
  document.getElementById('launch_lat').value = lat.toFixed(6);
  document.getElementById('launch_lon').value = lng.toFixed(6);
}

function setLandingMarker(lat, lng) {
  if (landingMarker) map.removeLayer(landingMarker);
  landingMarker = L.marker([lat, lng], {icon: makeIcon('#51cf66', 16), draggable: true})
    .addTo(map)
    .bindTooltip('Landing Target', {permanent: false});
  landingMarker.on('dragend', function(e) {
    const p = e.target.getLatLng();
    document.getElementById('landing_lat').value = p.lat.toFixed(6);
    document.getElementById('landing_lon').value = p.lng.toFixed(6);
  });
  document.getElementById('landing_lat').value = lat.toFixed(6);
  document.getElementById('landing_lon').value = lng.toFixed(6);
}

// Place initial markers
setLaunchMarker(37.7608, -75.7446);

// ---- Map click ----
map.on('click', function(e) {
  const mode = document.querySelector('input[name=clickMode]:checked').value;
  if (mode === 'launch') {
    setLaunchMarker(e.latlng.lat, e.latlng.lng);
    // Auto-switch to landing mode
    document.querySelector('input[name=clickMode][value=landing]').checked = true;
  } else {
    setLandingMarker(e.latlng.lat, e.latlng.lng);
  }
});

// ---- Default time (next hour UTC) ----
(function() {
  const now = new Date();
  now.setMinutes(0, 0, 0);
  now.setHours(now.getHours() + 1);
  // Format as local datetime-local input value
  const pad = n => String(n).padStart(2, '0');
  const val = `${now.getFullYear()}-${pad(now.getMonth()+1)}-${pad(now.getDate())}T${pad(now.getHours())}:${pad(now.getMinutes())}`;
  document.getElementById('time_utc').value = val;
})();

// ---- Compute ----
async function compute() {
  const btn = document.getElementById('btnCompute');
  btn.disabled = true;
  btn.textContent = 'COMPUTING...';
  document.getElementById('errorSection').style.display = 'none';
  document.getElementById('resultsSection').style.display = 'none';
  document.getElementById('windSection').style.display = 'none';

  // Clear previous results from map
  if (guidanceMarker) { map.removeLayer(guidanceMarker); guidanceMarker = null; }
  if (descentLine) { map.removeLayer(descentLine); descentLine = null; }
  if (boostLine) { map.removeLayer(boostLine); boostLine = null; }

  // Build time string in UTC ISO format from the datetime-local input
  const timeInput = document.getElementById('time_utc').value;
  // datetime-local gives "YYYY-MM-DDTHH:MM" in local time
  const localDate = new Date(timeInput);
  const timeUTC = localDate.toISOString();

  const payload = {
    launch_lat: parseFloat(document.getElementById('launch_lat').value),
    launch_lon: parseFloat(document.getElementById('launch_lon').value),
    landing_lat: parseFloat(document.getElementById('landing_lat').value),
    landing_lon: parseFloat(document.getElementById('landing_lon').value),
    apogee_ft: parseFloat(document.getElementById('apogee_ft').value),
    drogue_rate_fps: parseFloat(document.getElementById('drogue_rate').value),
    main_rate_fps: parseFloat(document.getElementById('main_rate').value),
    main_deploy_ft: parseFloat(document.getElementById('main_deploy').value),
    time_utc: timeUTC,
    max_steering_deg: parseFloat(document.getElementById('max_steering').value),
  };

  try {
    const resp = await fetch('/api/compute', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(payload),
    });
    const data = await resp.json();

    if (data.error) {
      document.getElementById('errorMsg').textContent = data.error;
      document.getElementById('errorSection').style.display = 'block';
      return;
    }

    // ---- Draw results on map ----

    // Guidance marker
    guidanceMarker = L.marker([data.guidance_lat, data.guidance_lon], {
      icon: makeIcon('#4dabf7', 14),
    }).addTo(map).bindTooltip('Guidance Point (apogee)', {permanent: false});

    // Descent track
    if (data.descent_track && data.descent_track.length > 1) {
      const coords = data.descent_track.map(p => [p.lat, p.lon]);
      descentLine = L.polyline(coords, {color: '#9775fa', weight: 3, opacity: 0.8}).addTo(map);
    }

    // Boost line (pad -> guidance)
    boostLine = L.polyline([
      [payload.launch_lat, payload.launch_lon],
      [data.guidance_lat, data.guidance_lon]
    ], {color: '#4dabf7', weight: 2, opacity: 0.5, dashArray: '8 6'}).addTo(map);

    // Fit bounds
    const allPts = [
      [payload.launch_lat, payload.launch_lon],
      [data.guidance_lat, data.guidance_lon],
    ];
    if (landingMarker) allPts.push([payload.landing_lat, payload.landing_lon]);
    if (data.descent_track) data.descent_track.forEach(p => allPts.push([p.lat, p.lon]));
    map.fitBounds(L.latLngBounds(allPts).pad(0.15));

    // ---- Populate results panel ----
    const statusClass = data.feasible ? 'feasible' : 'infeasible';
    const statusText = data.feasible ? 'FEASIBLE' : 'INFEASIBLE';

    let html = `
      <div class="result-row">
        <span class="result-label">Guidance point</span>
        <span class="result-value">${data.guidance_lat.toFixed(6)}, ${data.guidance_lon.toFixed(6)}</span>
      </div>
      <div class="result-row">
        <span class="result-label">Steering angle</span>
        <span class="result-value">${data.steering_angle_deg.toFixed(1)}&deg; off vertical</span>
      </div>
      <div class="result-row">
        <span class="result-label">Steering bearing</span>
        <span class="result-value">${data.steering_bearing_deg.toFixed(0)}&deg;</span>
      </div>
      <div class="result-row">
        <span class="result-label">Status</span>
        <span class="result-value ${statusClass}">${statusText}</span>
      </div>
    `;
    if (data.infeasible_reason) {
      html += `<div style="color:#ff6b6b;font-size:11px;padding:4px 0">${data.infeasible_reason}</div>`;
    }
    html += `
      <div class="result-row">
        <span class="result-label">Descent time</span>
        <span class="result-value">${data.total_descent_time_s.toFixed(0)} s (${(data.total_descent_time_s/60).toFixed(1)} min)</span>
      </div>
      <div class="result-row">
        <span class="result-label">Total drift</span>
        <span class="result-value">${data.total_drift_m.toFixed(0)} m</span>
      </div>
      <div class="result-row">
        <span class="result-label">Verification error</span>
        <span class="result-value">${data.landing_error_m.toFixed(1)} m</span>
      </div>
      <div class="result-row">
        <span class="result-label">Ground elev</span>
        <span class="result-value">${data.wind_profile.ground_elev_ft.toFixed(0)} ft ASL</span>
      </div>
    `;
    document.getElementById('resultsBody').innerHTML = html;
    document.getElementById('resultsSection').style.display = 'block';

    // ---- Wind profile table ----
    let windHtml = '';
    for (const l of data.wind_profile.layers) {
      windHtml += `<tr><td>${l.alt_ft.toFixed(0)}</td><td>${l.speed_kts.toFixed(1)}</td><td>${l.direction_deg.toFixed(0)}</td></tr>`;
    }
    document.getElementById('windBody').innerHTML = windHtml;
    document.getElementById('windSection').style.display = 'block';

  } catch (err) {
    document.getElementById('errorMsg').textContent = 'Network error: ' + err.message;
    document.getElementById('errorSection').style.display = 'block';
  } finally {
    btn.disabled = false;
    btn.textContent = 'COMPUTE';
  }
}

// ======================================================================
// 3D Cesium View
// ======================================================================

let cesiumViewer = null;
let is3D = false;
let lastComputeData = null;  // store last result for 3D rendering

// Store compute result for 3D view
const _origCompute = compute;

// Patch compute to save data
async function computeAndStore() {
  await _origCompute();
}

// Override the compute to capture data
const origFetch = window.fetch;
window.fetch = async function(...args) {
  const resp = await origFetch(...args);
  // Clone response so we can read it twice
  if (args[0] === '/api/compute') {
    const clone = resp.clone();
    clone.json().then(d => {
      if (!d.error) {
        lastComputeData = d;
        lastComputeData._launchLat = parseFloat(document.getElementById('launch_lat').value);
        lastComputeData._launchLon = parseFloat(document.getElementById('launch_lon').value);
        lastComputeData._apogee_ft = parseFloat(document.getElementById('apogee_ft').value);
        // Auto-update 3D if visible
        if (is3D && cesiumViewer) setTimeout(update3D, 200);
      }
    });
  }
  return resp;
};

function toggle3D() {
  const btn = document.getElementById('btn3d');
  const mapDiv = document.getElementById('map');
  const cesiumDiv = document.getElementById('cesiumContainer');

  is3D = !is3D;
  btn.classList.toggle('active', is3D);
  btn.textContent = is3D ? '2D View' : '3D View';

  if (is3D) {
    mapDiv.style.display = 'none';
    cesiumDiv.style.display = 'block';
    initCesium();
    if (lastComputeData) update3D();
  } else {
    cesiumDiv.style.display = 'none';
    mapDiv.style.display = 'block';
    map.invalidateSize();
  }
}

function initCesium() {
  if (cesiumViewer) return;

  // No Ion token needed — we add OSM imagery after construction
  cesiumViewer = new Cesium.Viewer('cesiumContainer', {
    baseLayerPicker: false,
    geocoder: false,
    homeButton: false,
    sceneModePicker: false,
    selectionIndicator: false,
    navigationHelpButton: false,
    animation: false,
    timeline: false,
    fullscreenButton: false,
    infoBox: false,
    scene3DOnly: true,
  });

  // Add ArcGIS World Imagery satellite tiles (no API token required)
  cesiumViewer.imageryLayers.addImageryProvider(
    new Cesium.UrlTemplateImageryProvider({
      url: 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
      credit: 'Esri World Imagery',
    })
  );

  // Fly to default location
  const lat = parseFloat(document.getElementById('launch_lat').value) || 37.7608;
  const lon = parseFloat(document.getElementById('launch_lon').value) || -75.7446;
  cesiumViewer.camera.flyTo({
    destination: Cesium.Cartesian3.fromDegrees(lon, lat, 15000),
    orientation: { heading: 0, pitch: Cesium.Math.toRadians(-50), roll: 0 },
    duration: 1.5,
  });
}

function update3D() {
  if (!cesiumViewer || !lastComputeData) return;
  const d = lastComputeData;

  cesiumViewer.entities.removeAll();

  const groundElev_m = (d.wind_profile.ground_elev_ft || 0) * 0.3048;

  // ---- Launch pad (red pin at ground) ----
  cesiumViewer.entities.add({
    position: Cesium.Cartesian3.fromDegrees(d._launchLon, d._launchLat, groundElev_m),
    point: { pixelSize: 12, color: Cesium.Color.fromCssColorString('#ff6b6b'), outlineColor: Cesium.Color.WHITE, outlineWidth: 2,
             heightReference: Cesium.HeightReference.CLAMP_TO_GROUND },
    label: { text: 'Launch Pad', font: '12px sans-serif', fillColor: Cesium.Color.WHITE,
             style: Cesium.LabelStyle.FILL_AND_OUTLINE, outlineWidth: 2,
             verticalOrigin: Cesium.VerticalOrigin.BOTTOM, pixelOffset: new Cesium.Cartesian2(0, -14) },
  });

  // ---- Landing target (green pin at ground) ----
  cesiumViewer.entities.add({
    position: Cesium.Cartesian3.fromDegrees(d.forward_landing_lon || d._launchLon, d.forward_landing_lat || d._launchLat, groundElev_m),
    point: { pixelSize: 12, color: Cesium.Color.fromCssColorString('#51cf66'), outlineColor: Cesium.Color.WHITE, outlineWidth: 2,
             heightReference: Cesium.HeightReference.CLAMP_TO_GROUND },
    label: { text: 'Landing Target', font: '12px sans-serif', fillColor: Cesium.Color.WHITE,
             style: Cesium.LabelStyle.FILL_AND_OUTLINE, outlineWidth: 2,
             verticalOrigin: Cesium.VerticalOrigin.BOTTOM, pixelOffset: new Cesium.Cartesian2(0, -14) },
  });

  // ---- Guidance point (blue pin at apogee altitude) ----
  const apogee_m = (d._apogee_ft || 10000) * 0.3048;
  cesiumViewer.entities.add({
    position: Cesium.Cartesian3.fromDegrees(d.guidance_lon, d.guidance_lat, groundElev_m + apogee_m),
    point: { pixelSize: 14, color: Cesium.Color.fromCssColorString('#4dabf7'), outlineColor: Cesium.Color.WHITE, outlineWidth: 2 },
    label: { text: 'Guidance Point', font: '12px sans-serif', fillColor: Cesium.Color.fromCssColorString('#4dabf7'),
             style: Cesium.LabelStyle.FILL_AND_OUTLINE, outlineWidth: 2,
             verticalOrigin: Cesium.VerticalOrigin.BOTTOM, pixelOffset: new Cesium.Cartesian2(0, -16) },
  });

  // ---- Descent track (purple polyline with altitude) ----
  if (d.descent_track && d.descent_track.length > 1) {
    const positions = [];
    for (const p of d.descent_track) {
      const alt_m = p.alt_agl_ft * 0.3048 + groundElev_m;
      positions.push(Cesium.Cartesian3.fromDegrees(p.lon, p.lat, alt_m));
    }
    cesiumViewer.entities.add({
      polyline: {
        positions: positions,
        width: 4,
        material: new Cesium.PolylineGlowMaterialProperty({
          glowPower: 0.15,
          color: Cesium.Color.fromCssColorString('#9775fa'),
        }),
      },
    });

    // Vertical drop line from guidance point to ground
    cesiumViewer.entities.add({
      polyline: {
        positions: [
          Cesium.Cartesian3.fromDegrees(d.guidance_lon, d.guidance_lat, groundElev_m + apogee_m),
          Cesium.Cartesian3.fromDegrees(d.guidance_lon, d.guidance_lat, groundElev_m),
        ],
        width: 1,
        material: new Cesium.PolylineDashMaterialProperty({
          color: Cesium.Color.fromCssColorString('#4dabf7').withAlpha(0.4),
          dashLength: 12,
        }),
      },
    });
  }

  // ---- Boost trajectory (dashed blue line from pad to guidance at altitude) ----
  cesiumViewer.entities.add({
    polyline: {
      positions: [
        Cesium.Cartesian3.fromDegrees(d._launchLon, d._launchLat, groundElev_m),
        Cesium.Cartesian3.fromDegrees(d.guidance_lon, d.guidance_lat, groundElev_m + apogee_m),
      ],
      width: 2,
      material: new Cesium.PolylineDashMaterialProperty({
        color: Cesium.Color.fromCssColorString('#4dabf7').withAlpha(0.6),
        dashLength: 16,
      }),
    },
  });

  // ---- Fly camera to view the whole scene from the side ----
  // Collect all entity positions into a bounding sphere, then fly to it
  // with a heading/pitch/range that gives a good 3D perspective.
  const allCartesian = [];
  allCartesian.push(Cesium.Cartesian3.fromDegrees(d._launchLon, d._launchLat, groundElev_m));
  allCartesian.push(Cesium.Cartesian3.fromDegrees(d.guidance_lon, d.guidance_lat, groundElev_m + apogee_m));
  if (d.forward_landing_lat) {
    allCartesian.push(Cesium.Cartesian3.fromDegrees(d.forward_landing_lon, d.forward_landing_lat, groundElev_m));
  }
  if (d.descent_track) {
    for (const p of d.descent_track) {
      allCartesian.push(Cesium.Cartesian3.fromDegrees(p.lon, p.lat, p.alt_agl_ft * 0.3048 + groundElev_m));
    }
  }
  const boundingSphere = Cesium.BoundingSphere.fromPoints(allCartesian);

  // Compute track bearing for a side-view heading
  const dLat = d.guidance_lat - d._launchLat;
  const dLon = d.guidance_lon - d._launchLon;
  const trackBearingDeg = Math.atan2(dLon, dLat) * 180 / Math.PI;
  // Look perpendicular to the track (from the south-ish side)
  const viewHeadingDeg = (trackBearingDeg + 90 + 360) % 360;

  cesiumViewer.camera.flyToBoundingSphere(boundingSphere, {
    offset: new Cesium.HeadingPitchRange(
      Cesium.Math.toRadians(viewHeadingDeg),
      Cesium.Math.toRadians(-25),
      boundingSphere.radius * 2.5
    ),
    duration: 1.5,
  });
}
</script>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    import webbrowser, threading
    port = 5050
    url = f"http://localhost:{port}"
    print(f"Starting Drift Cast GUI at {url}")
    threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    app.run(host="127.0.0.1", port=port, debug=False)


if __name__ == "__main__":
    main()
