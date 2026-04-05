"""Export interactive 3D flight visualization as a self-contained HTML file.

Uses Three.js for 3D rocket rendering and Canvas 2D for telemetry strip charts.
The output is a single HTML file that can be opened in any modern browser.
"""
import json
import numpy as np
import pandas as pd
from pathlib import Path
from typing import Optional

from .rocket_3d import RocketMesh


# ---------------------------------------------------------------------------
# Mesh-to-JSON conversion
# ---------------------------------------------------------------------------

def _mesh_to_json(mesh: RocketMesh) -> dict:
    """Convert RocketMesh to Three.js-compatible geometry JSON.

    Surfaces of revolution are converted to indexed triangle meshes.
    Fins are converted to flat triangle meshes.
    All coordinates are in FRD body frame (nose at +X).
    """
    surfaces = []
    for X, Y, Z, color in mesh.surfaces:
        n_x, n_t = X.shape
        # Flatten grid to vertex list, apply FRD flip
        verts = []
        for i in range(n_x):
            for j in range(n_t):
                bx = -(X[i, j] - mesh.cg_position)
                by = Y[i, j]
                bz = Z[i, j]
                verts.extend([round(bx, 5), round(by, 5), round(bz, 5)])

        # Build triangle indices from grid
        indices = []
        for i in range(n_x - 1):
            for j in range(n_t - 1):
                v00 = i * n_t + j
                v01 = i * n_t + j + 1
                v10 = (i + 1) * n_t + j
                v11 = (i + 1) * n_t + j + 1
                indices.extend([v00, v10, v01])
                indices.extend([v01, v10, v11])

        surfaces.append({
            'vertices': verts,
            'indices': indices,
            'color': color,
        })

    fins = []
    for verts_arr, color in mesh.fins:
        # Apply FRD flip to fin vertices
        body_verts = verts_arr.copy()
        body_verts[:, 0] = -(body_verts[:, 0] - mesh.cg_position)
        # Triangulate polygon (fan from first vertex)
        flat_verts = []
        for v in body_verts:
            flat_verts.extend([round(float(v[0]), 5),
                               round(float(v[1]), 5),
                               round(float(v[2]), 5)])
        indices = []
        n = len(body_verts)
        for i in range(1, n - 1):
            indices.extend([0, i, i + 1])

        fins.append({
            'vertices': flat_verts,
            'indices': indices,
            'color': color,
        })

    return {
        'surfaces': surfaces,
        'fins': fins,
        'cg': round(float(mesh.cg_position), 4),
        'length': round(float(mesh.total_length), 4),
        'radius': round(float(mesh.body_radius), 4),
    }


# ---------------------------------------------------------------------------
# Telemetry preparation
# ---------------------------------------------------------------------------

def _prepare_telemetry(df: pd.DataFrame, fps: int = 60) -> dict:
    """Downsample and package simulation telemetry for embedding."""
    t = df['time'].values
    dt_anim = 1.0 / fps
    t_anim = np.arange(t[0], t[-1], dt_anim)

    def interp(col):
        if col in df.columns:
            return np.interp(t_anim, t, df[col].values).tolist()
        return None

    def interp_round(col, decimals=3):
        v = interp(col)
        if v is not None:
            return [round(x, decimals) for x in v]
        return None

    # Core position/velocity
    data = {
        'time': [round(x, 4) for x in t_anim.tolist()],
        'x': interp_round('x'),
        'y': interp_round('y'),
        'z': interp_round('z'),
        'speed': interp_round('speed', 1),
        'altitude': interp_round('altitude', 1),
        'mach': interp_round('mach', 3),
    }

    # Quaternion (NED/FRD)
    for q in ['true_q0_ned', 'true_q1_ned', 'true_q2_ned', 'true_q3_ned']:
        data[q] = interp_round(q, 5)

    # Attitude
    data['pitch'] = interp_round('true_pitch_ned_deg', 1)
    data['yaw'] = interp_round('true_yaw_ned_deg', 1)
    data['roll'] = interp_round('true_roll_ned_deg', 1)

    # Thrust
    data['thrust'] = interp_round('thrust', 1)

    # Single fin tab (roll control)
    data['fin_tab_cmd'] = interp_round('fin_tab_cmd', 2)
    data['fin_tab_actual'] = interp_round('fin_tab_actual', 2)

    # 4-fin data (guidance mode)
    has_guidance = 'fin_0_cmd' in df.columns
    data['has_guidance'] = has_guidance
    if has_guidance:
        for i in range(4):
            data[f'fin_{i}_cmd'] = interp_round(f'fin_{i}_cmd', 2)
            data[f'fin_{i}_actual'] = interp_round(f'fin_{i}_actual', 2)
        data['guid_pitch_cmd'] = interp_round('guid_pitch_cmd_deg', 2)
        data['guid_yaw_cmd'] = interp_round('guid_yaw_cmd_deg', 2)
        data['guid_lateral_offset'] = interp_round('guid_lateral_offset_m', 2)
        if 'guid_target_alt' in df.columns:
            data['guid_target_alt'] = interp_round('guid_target_alt', 1)

    # Alpha
    data['alpha'] = interp_round('alpha_deg', 1)

    # Flight phase — nearest neighbor interp
    if 'flight_phase' in df.columns:
        phase_idx = np.searchsorted(t, t_anim, side='right') - 1
        phase_idx = np.clip(phase_idx, 0, len(t) - 1)
        data['flight_phase'] = [str(df['flight_phase'].iloc[i]) for i in phase_idx]

    return data


# ---------------------------------------------------------------------------
# HTML template
# ---------------------------------------------------------------------------

def _build_html(geometry: dict, telemetry: dict, title: str) -> str:
    """Build the complete self-contained HTML file."""

    geom_json = json.dumps(geometry, separators=(',', ':'))
    telem_json = json.dumps(telemetry, separators=(',', ':'))

    return f'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>{title}</title>
<style>
* {{ margin: 0; padding: 0; box-sizing: border-box; }}
body {{ background: #1a1a2e; color: #e0e0e0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, monospace; overflow: hidden; height: 100vh; }}
#container {{ display: flex; height: calc(100vh - 52px); }}
#viewport {{ flex: 1; min-width: 0; position: relative; }}
#telemetry {{ width: 320px; background: #16213e; border-left: 1px solid #333; display: flex; flex-direction: column; overflow-y: auto; }}
.chart-container {{ padding: 8px 12px; border-bottom: 1px solid #2a2a4a; }}
.chart-title {{ font-size: 11px; font-weight: 600; color: #8ab4f8; margin-bottom: 4px; text-transform: uppercase; letter-spacing: 0.5px; }}
canvas.chart {{ width: 100%; height: 90px; display: block; cursor: pointer; }}
canvas.chart-tall {{ height: 110px; }}
#controls {{ height: 52px; background: #0f3460; display: flex; align-items: center; padding: 0 16px; gap: 12px; border-top: 1px solid #333; }}
#play-btn {{ background: #e94560; border: none; color: white; width: 36px; height: 36px; border-radius: 50%; cursor: pointer; font-size: 16px; display: flex; align-items: center; justify-content: center; }}
#play-btn:hover {{ background: #ff6b81; }}
#scrubber {{ flex: 1; -webkit-appearance: none; height: 6px; border-radius: 3px; background: #333; outline: none; }}
#scrubber::-webkit-slider-thumb {{ -webkit-appearance: none; width: 16px; height: 16px; border-radius: 50%; background: #e94560; cursor: pointer; }}
#speed-select {{ background: #1a1a2e; color: #e0e0e0; border: 1px solid #444; border-radius: 4px; padding: 4px 8px; font-size: 12px; }}
#time-display {{ font-size: 13px; font-variant-numeric: tabular-nums; min-width: 120px; text-align: right; }}
#info-overlay {{ position: absolute; top: 12px; left: 12px; font-size: 12px; pointer-events: none; }}
#info-overlay div {{ background: rgba(15,52,96,0.85); padding: 4px 10px; border-radius: 4px; margin-bottom: 4px; }}
.phase-badge {{ display: inline-block; padding: 2px 8px; border-radius: 3px; font-weight: 700; font-size: 11px; }}
.phase-PAD {{ background: #555; }}
.phase-BOOST {{ background: #e94560; }}
.phase-COAST {{ background: #0a84ff; }}
.phase-DESCENT {{ background: #30d158; }}
#title-bar {{ position: absolute; top: 12px; right: 12px; font-size: 14px; font-weight: 600; color: #8ab4f8; pointer-events: none; background: rgba(15,52,96,0.85); padding: 6px 14px; border-radius: 6px; }}
#view-cube {{ position: absolute; bottom: 12px; right: 12px; width: 140px; height: 140px; cursor: pointer; z-index: 10; }}
</style>
</head>
<body>
<div id="container">
  <div id="viewport">
    <div id="info-overlay"></div>
    <div id="title-bar">{title}</div>
    <div id="view-cube"></div>
  </div>
  <div id="telemetry"></div>
</div>
<div id="controls">
  <button id="play-btn" title="Play / Pause">&#9654;</button>
  <input type="range" id="scrubber" min="0" max="1000" value="0">
  <select id="speed-select">
    <option value="0.1">0.1x</option>
    <option value="0.25">0.25x</option>
    <option value="0.5">0.5x</option>
    <option value="1" selected>1x</option>
    <option value="2">2x</option>
    <option value="4">4x</option>
    <option value="8">8x</option>
  </select>
  <div id="time-display">T+0.00s</div>
</div>

<script src="https://cdn.jsdelivr.net/npm/three@0.137.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.137.0/examples/js/controls/OrbitControls.js"></script>

<script>
// ── Embedded data ──
const GEOM = {geom_json};
const T = {telem_json};

const nFrames = T.time.length;
const tMax = T.time[nFrames - 1];
const tMin = T.time[0];
const hasQuat = T.true_q0_ned !== null;
const hasGuidance = T.has_guidance === true;

// ── State ──
let playing = false;
let playSpeed = 1.0;
let currentFrame = 0;
let lastTimestamp = null;
let simTime = 0;  // accumulated simulation time for playback

// ── Three.js setup ──
const vpDiv = document.getElementById('viewport');
const renderer = new THREE.WebGLRenderer({{ antialias: true, alpha: false }});
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setClearColor(0x1a1a2e);
vpDiv.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x1a1a2e, 0.0008);

const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 5000);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;

// Lighting
scene.add(new THREE.AmbientLight(0xffffff, 0.5));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(50, 100, 200);
scene.add(dirLight);
const hemiLight = new THREE.HemisphereLight(0x8ab4f8, 0x333366, 0.3);
scene.add(hemiLight);

// ── Build rocket model ──
const rocketGroup = new THREE.Group();

function buildRocket() {{
    // Surfaces
    for (const surf of GEOM.surfaces) {{
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.Float32BufferAttribute(surf.vertices, 3));
        geom.setIndex(surf.indices);
        geom.computeVertexNormals();
        const mat = new THREE.MeshPhongMaterial({{
            color: surf.color,
            shininess: 60,
            side: THREE.DoubleSide,
        }});
        rocketGroup.add(new THREE.Mesh(geom, mat));
    }}
    // Fins
    for (const fin of GEOM.fins) {{
        const geom = new THREE.BufferGeometry();
        geom.setAttribute('position', new THREE.Float32BufferAttribute(fin.vertices, 3));
        geom.setIndex(fin.indices);
        geom.computeVertexNormals();
        const mat = new THREE.MeshPhongMaterial({{
            color: fin.color,
            shininess: 40,
            side: THREE.DoubleSide,
        }});
        rocketGroup.add(new THREE.Mesh(geom, mat));
    }}
}}
buildRocket();

// Scale rocket for visibility: ~5% of max altitude
const maxAlt = Math.max(...T.altitude);
const rocketScale = Math.max(5, maxAlt * 0.05 / GEOM.length);
rocketGroup.scale.set(rocketScale, rocketScale, rocketScale);
scene.add(rocketGroup);

// ── NED→ENU rotation ──
const R_ned_enu = new THREE.Matrix4().set(
    0, 1, 0, 0,
    1, 0, 0, 0,
    0, 0,-1, 0,
    0, 0, 0, 1
);

function quatToMatrix(q0, q1, q2, q3) {{
    // Body FRD → NED rotation matrix
    const n = Math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    const w = q0/n, x = q1/n, y = q2/n, z = q3/n;
    return new THREE.Matrix4().set(
        1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y),   0,
        2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x),   0,
        2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y), 0,
        0,             0,             0,             1
    );
}}

// ── Trajectory trail ──
const trailPositions = new Float32Array(nFrames * 3);
for (let i = 0; i < nFrames; i++) {{
    trailPositions[i*3]   = T.x[i];
    trailPositions[i*3+1] = T.y[i];
    trailPositions[i*3+2] = T.z[i];
}}
const trailGeom = new THREE.BufferGeometry();
trailGeom.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
trailGeom.setDrawRange(0, 0);
const trailMat = new THREE.LineBasicMaterial({{ color: 0xe94560, linewidth: 2 }});
const trailLine = new THREE.Line(trailGeom, trailMat);
scene.add(trailLine);

// ── Ground plane ──
const gridSize = 800;
const grid = new THREE.GridHelper(gridSize, 40, 0x333366, 0x222244);
grid.rotation.x = Math.PI / 2; // XY plane → XZ in Three.js, but we use ENU: X=E, Y=N, Z=Up
// Three.js GridHelper is in XZ plane. We want it in XY plane (Z=0).
// Actually let's just rotate: GridHelper Y is up by default. Our Z is up.
grid.rotation.x = Math.PI / 2;
scene.add(grid);

// Launch pad marker
const padGeom = new THREE.CylinderGeometry(2, 2, 0.5, 16);
const padMat = new THREE.MeshPhongMaterial({{ color: 0xe94560 }});
const padMesh = new THREE.Mesh(padGeom, padMat);
padMesh.rotation.x = Math.PI / 2;
padMesh.position.set(0, 0, 0.25);
scene.add(padMesh);

// Apogee marker
const apogeeIdx = T.altitude.indexOf(Math.max(...T.altitude));
const apogeeSphereGeom = new THREE.SphereGeometry(3, 16, 16);
const apogeeSphereMat = new THREE.MeshPhongMaterial({{ color: 0xffd700, transparent: true, opacity: 0.6 }});
const apogeeSphere = new THREE.Mesh(apogeeSphereGeom, apogeeSphereMat);
apogeeSphere.position.set(T.x[apogeeIdx], T.y[apogeeIdx], T.z[apogeeIdx]);
scene.add(apogeeSphere);

// ── Guidance target marker (0, 0, target_alt) ──
let guidTarget = null;
let guidTargetLine = null;
let guidCmdArrow = null;
if (T.guid_target_alt) {{
    // Find the target altitude (use first valid value)
    let tgtAlt = 0;
    for (let i = 0; i < T.guid_target_alt.length; i++) {{
        if (T.guid_target_alt[i] > 0) {{ tgtAlt = T.guid_target_alt[i]; break; }}
    }}
    if (tgtAlt > 0) {{
        // Diamond/octahedron marker at guidance target point
        const tgtGeom = new THREE.OctahedronGeometry(5);
        const tgtMat = new THREE.MeshPhongMaterial({{
            color: 0x00ff88, transparent: true, opacity: 0.7,
            emissive: 0x00ff88, emissiveIntensity: 0.3
        }});
        guidTarget = new THREE.Mesh(tgtGeom, tgtMat);
        guidTarget.position.set(0, 0, tgtAlt);
        scene.add(guidTarget);

        // Dashed line from pad to target
        const lineGeom = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(0, 0, 0),
            new THREE.Vector3(0, 0, tgtAlt),
        ]);
        const lineMat = new THREE.LineDashedMaterial({{
            color: 0x00ff88, dashSize: 8, gapSize: 4, transparent: true, opacity: 0.4
        }});
        guidTargetLine = new THREE.LineSegments(lineGeom, lineMat);
        guidTargetLine.computeLineDistances();
        scene.add(guidTargetLine);

        // Arrow showing commanded direction (updated per frame)
        const arrowDir = new THREE.Vector3(0, 0, 1);
        guidCmdArrow = new THREE.ArrowHelper(arrowDir, new THREE.Vector3(0,0,0), 30, 0xff4444, 6, 3);
        guidCmdArrow.visible = false;
        scene.add(guidCmdArrow);
    }}
}}

// ── Axes helper near pad ──
const axesLen = 20;
scene.add(new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(0,0,0), axesLen, 0xff4444)); // E
scene.add(new THREE.ArrowHelper(new THREE.Vector3(0,1,0), new THREE.Vector3(0,0,0), axesLen, 0x44ff44)); // N
scene.add(new THREE.ArrowHelper(new THREE.Vector3(0,0,1), new THREE.Vector3(0,0,0), axesLen, 0x4444ff)); // U

// ── Strip charts ──
const telemPanel = document.getElementById('telemetry');

const chartConfigs = [];

// 1. Altitude & Speed
chartConfigs.push({{
    title: 'Altitude (m) / Speed (m/s)',
    datasets: [
        {{ label: 'Alt', data: T.altitude, color: '#8ab4f8', yAxis: 'left' }},
        {{ label: 'Speed', data: T.speed, color: '#ffd700', yAxis: 'right' }},
    ],
    tall: true,
}});

// 2. Thrust
chartConfigs.push({{
    title: 'Thrust (N)',
    datasets: [
        {{ label: 'Thrust', data: T.thrust, color: '#e94560', yAxis: 'left' }},
    ],
}});

// 3. Attitude
chartConfigs.push({{
    title: 'Attitude (deg)',
    datasets: [
        {{ label: 'Pitch', data: T.pitch, color: '#ff6b6b', yAxis: 'left' }},
        {{ label: 'Yaw', data: T.yaw, color: '#51cf66', yAxis: 'left' }},
        {{ label: 'Roll', data: T.roll, color: '#339af0', yAxis: 'left' }},
    ],
    tall: true,
}});

// 4. Fin deflections
if (hasGuidance) {{
    chartConfigs.push({{
        title: 'Fin Deflections (deg)',
        datasets: [
            {{ label: 'Fin0', data: T.fin_0_cmd, color: '#ff6b6b', yAxis: 'left' }},
            {{ label: 'Fin1', data: T.fin_1_cmd, color: '#51cf66', yAxis: 'left' }},
            {{ label: 'Fin2', data: T.fin_2_cmd, color: '#339af0', yAxis: 'left' }},
            {{ label: 'Fin3', data: T.fin_3_cmd, color: '#ffd43b', yAxis: 'left' }},
        ],
        tall: true,
    }});

    chartConfigs.push({{
        title: 'Guidance Commands (deg)',
        datasets: [
            {{ label: 'Pitch', data: T.guid_pitch_cmd, color: '#ff6b6b', yAxis: 'left' }},
            {{ label: 'Yaw', data: T.guid_yaw_cmd, color: '#51cf66', yAxis: 'left' }},
        ],
    }});

    chartConfigs.push({{
        title: 'Lateral Offset (m)',
        datasets: [
            {{ label: 'Offset', data: T.guid_lateral_offset, color: '#cc5de8', yAxis: 'left' }},
        ],
    }});
}} else {{
    chartConfigs.push({{
        title: 'Fin Tab (deg)',
        datasets: [
            {{ label: 'Cmd', data: T.fin_tab_cmd, color: '#ff6b6b', yAxis: 'left' }},
            {{ label: 'Actual', data: T.fin_tab_actual, color: '#339af0', yAxis: 'left' }},
        ],
    }});
}}

// 5. AoA / Mach
chartConfigs.push({{
    title: 'AoA (deg) / Mach',
    datasets: [
        {{ label: 'AoA', data: T.alpha, color: '#ffa94d', yAxis: 'left' }},
        {{ label: 'Mach', data: T.mach, color: '#69db7c', yAxis: 'right' }},
    ],
}});

// Create chart canvases
const charts = [];
for (const cfg of chartConfigs) {{
    const container = document.createElement('div');
    container.className = 'chart-container';
    const titleEl = document.createElement('div');
    titleEl.className = 'chart-title';
    titleEl.textContent = cfg.title;
    container.appendChild(titleEl);

    const canvas = document.createElement('canvas');
    canvas.className = cfg.tall ? 'chart chart-tall' : 'chart';
    container.appendChild(canvas);
    telemPanel.appendChild(container);

    // Click on chart to scrub
    canvas.addEventListener('click', (e) => {{
        const rect = canvas.getBoundingClientRect();
        const frac = (e.clientX - rect.left) / rect.width;
        const margin = 45;
        const plotFrac = (e.clientX - rect.left - margin) / (rect.width - margin - 10);
        const clampedFrac = Math.max(0, Math.min(1, plotFrac));
        currentFrame = Math.round(clampedFrac * (nFrames - 1));
        scrubbing = true;
        updateScrubber();
        drawAllCharts();
        updateRocket();
    }});

    charts.push({{ canvas, cfg }});
}}

function drawChart(canvas, cfg, frame) {{
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    const W = rect.width, H = rect.height;

    const marginL = 45, marginR = 10, marginT = 5, marginB = 20;
    const pW = W - marginL - marginR;
    const pH = H - marginT - marginB;

    // Background
    ctx.fillStyle = '#0d1b2a';
    ctx.fillRect(0, 0, W, H);

    // Compute Y range
    let yMin = Infinity, yMax = -Infinity;
    let yMinR = Infinity, yMaxR = -Infinity;
    for (const ds of cfg.datasets) {{
        if (!ds.data) continue;
        for (const v of ds.data) {{
            if (v === null || v === undefined || isNaN(v)) continue;
            if (ds.yAxis === 'right') {{
                if (v < yMinR) yMinR = v;
                if (v > yMaxR) yMaxR = v;
            }} else {{
                if (v < yMin) yMin = v;
                if (v > yMax) yMax = v;
            }}
        }}
    }}

    // Pad ranges
    function padRange(lo, hi) {{
        if (!isFinite(lo) || !isFinite(hi)) return [0, 1];
        if (hi - lo < 0.01) {{ lo -= 0.5; hi += 0.5; }}
        const pad = (hi - lo) * 0.08;
        return [lo - pad, hi + pad];
    }}
    [yMin, yMax] = padRange(yMin, yMax);
    [yMinR, yMaxR] = padRange(yMinR, yMaxR);

    // Grid lines
    ctx.strokeStyle = '#1e3a5f';
    ctx.lineWidth = 0.5;
    for (let i = 0; i <= 4; i++) {{
        const gy = marginT + pH * i / 4;
        ctx.beginPath();
        ctx.moveTo(marginL, gy);
        ctx.lineTo(marginL + pW, gy);
        ctx.stroke();
    }}

    // Y-axis labels (left)
    ctx.fillStyle = '#667';
    ctx.font = '10px monospace';
    ctx.textAlign = 'right';
    for (let i = 0; i <= 4; i++) {{
        const val = yMax - (yMax - yMin) * i / 4;
        const gy = marginT + pH * i / 4;
        ctx.fillText(val.toFixed(val > 100 ? 0 : 1), marginL - 4, gy + 3);
    }}

    // Time axis labels
    ctx.textAlign = 'center';
    const tRange = tMax - tMin;
    for (let i = 0; i <= 4; i++) {{
        const tVal = tMin + tRange * i / 4;
        const gx = marginL + pW * i / 4;
        ctx.fillText(tVal.toFixed(1) + 's', gx, H - 4);
    }}

    // Data lines
    for (const ds of cfg.datasets) {{
        if (!ds.data) continue;
        const useRight = ds.yAxis === 'right';
        const yLo = useRight ? yMinR : yMin;
        const yHi = useRight ? yMaxR : yMax;
        const yRange = yHi - yLo;

        ctx.strokeStyle = ds.color;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        let started = false;
        // Draw every Nth point for performance
        const step = Math.max(1, Math.floor(nFrames / (pW * 2)));
        for (let i = 0; i < nFrames; i += step) {{
            const x = marginL + (i / (nFrames - 1)) * pW;
            const v = ds.data[i];
            if (v === null || v === undefined || isNaN(v)) continue;
            const y = marginT + pH * (1 - (v - yLo) / yRange);
            if (!started) {{ ctx.moveTo(x, y); started = true; }}
            else ctx.lineTo(x, y);
        }}
        ctx.stroke();
    }}

    // Legend
    ctx.font = '10px monospace';
    let lx = marginL + 4;
    for (const ds of cfg.datasets) {{
        ctx.fillStyle = ds.color;
        ctx.fillRect(lx, marginT + 2, 12, 3);
        ctx.fillStyle = '#aaa';
        ctx.textAlign = 'left';
        ctx.fillText(ds.label, lx + 15, marginT + 8);
        lx += ctx.measureText(ds.label).width + 28;
    }}

    // Cursor line
    const cursorX = marginL + (frame / (nFrames - 1)) * pW;
    ctx.strokeStyle = '#e94560';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(cursorX, marginT);
    ctx.lineTo(cursorX, marginT + pH);
    ctx.stroke();
    ctx.setLineDash([]);

    // Current values at cursor
    ctx.font = 'bold 10px monospace';
    let vy = marginT + 22;
    for (const ds of cfg.datasets) {{
        if (!ds.data || !ds.data[frame]) continue;
        ctx.fillStyle = ds.color;
        const val = ds.data[frame];
        ctx.textAlign = 'right';
        ctx.fillText(ds.label + ': ' + val.toFixed(1), marginL + pW - 2, vy);
        vy += 12;
    }}
}}

function drawAllCharts() {{
    for (const c of charts) {{
        drawChart(c.canvas, c.cfg, currentFrame);
    }}
}}

// ── Info overlay ──
const infoOverlay = document.getElementById('info-overlay');

function updateInfo() {{
    const t = T.time[currentFrame];
    const alt = T.altitude[currentFrame];
    const spd = T.speed[currentFrame];
    const phase = T.flight_phase ? T.flight_phase[currentFrame] : '';

    let html = '';
    if (phase) {{
        html += `<div><span class="phase-badge phase-${{phase}}">${{phase}}</span></div>`;
    }}
    html += `<div>Alt: ${{alt.toFixed(1)}} m &nbsp; Spd: ${{spd.toFixed(1)}} m/s</div>`;
    if (T.mach && T.mach[currentFrame] !== null) {{
        html += `<div>Mach: ${{T.mach[currentFrame].toFixed(2)}}</div>`;
    }}
    infoOverlay.innerHTML = html;
}}

// ── Camera follow ──
let cameraFollow = true;
let scrubbing = false;  // instant snap when scrubbing
const followLerp = 0.08;
// View distance scales with rocket visual size
const viewDist = rocketScale * GEOM.length * 4;

// ── Update functions ──
function updateRocket() {{
    const i = currentFrame;

    // Position (ENU)
    const pos = new THREE.Vector3(T.x[i], T.y[i], T.z[i]);
    rocketGroup.position.copy(pos);

    // Orientation
    if (hasQuat) {{
        const q0 = T.true_q0_ned[i], q1 = T.true_q1_ned[i];
        const q2 = T.true_q2_ned[i], q3 = T.true_q3_ned[i];
        const R_body_ned = quatToMatrix(q0, q1, q2, q3);
        const R_total = R_ned_enu.clone().multiply(R_body_ned);
        rocketGroup.rotation.setFromRotationMatrix(R_total);
    }}

    // Trail
    trailGeom.setDrawRange(0, i + 1);

    // Update guidance target marker position (target alt changes during flight)
    if (guidTarget && T.guid_target_alt) {{
        const tAlt = T.guid_target_alt[i];
        if (tAlt > 0) {{
            guidTarget.position.set(0, 0, tAlt);
            guidTarget.visible = true;
            // Spin the diamond slowly for visibility
            guidTarget.rotation.y += 0.02;
        }} else {{
            guidTarget.visible = false;
        }}
    }}

    // Update guidance command arrow (shows direction rocket is being steered)
    if (guidCmdArrow && T.guid_pitch_cmd && T.guid_yaw_cmd) {{
        const pc = T.guid_pitch_cmd[i] || 0;
        const yc = T.guid_yaw_cmd[i] || 0;
        if (Math.abs(pc) > 0.1 || Math.abs(yc) > 0.1) {{
            // Pitch cmd > 0 = tilt forward (nose up in FRD = toward +Z in ENU)
            // Yaw cmd > 0 = tilt right
            // Show as arrow from rocket toward guidance target
            const tAlt = T.guid_target_alt ? T.guid_target_alt[i] : T.z[apogeeIdx];
            const toTarget = new THREE.Vector3(
                0 - T.x[i], 0 - T.y[i], (tAlt || T.z[apogeeIdx]) - T.z[i]
            ).normalize();
            guidCmdArrow.position.copy(pos);
            guidCmdArrow.setDirection(toTarget);
            guidCmdArrow.setLength(30, 6, 3);
            guidCmdArrow.visible = true;
        }} else {{
            guidCmdArrow.visible = false;
        }}
    }}

    // Camera follow: move orbit target to rocket position
    if (cameraFollow) {{
        const lerpFactor = scrubbing ? 1.0 : followLerp;
        const oldTarget = controls.target.clone();
        controls.target.lerp(pos, lerpFactor);
        // Move camera by same delta to maintain relative view
        const delta = controls.target.clone().sub(oldTarget);
        camera.position.add(delta);
        scrubbing = false;
    }}

    updateInfo();
}}

function updateScrubber() {{
    const scrubber = document.getElementById('scrubber');
    scrubber.value = Math.round((currentFrame / (nFrames - 1)) * 1000);
    const t = T.time[currentFrame];
    document.getElementById('time-display').textContent =
        'T+' + t.toFixed(2) + 's / ' + tMax.toFixed(1) + 's';
}}

// ── View Cube ──
const vcDiv = document.getElementById('view-cube');
const vcRenderer = new THREE.WebGLRenderer({{ antialias: true, alpha: true }});
vcRenderer.setPixelRatio(window.devicePixelRatio);
vcRenderer.setSize(140, 140);
vcDiv.appendChild(vcRenderer.domElement);

const vcScene = new THREE.Scene();
const vcSize = 2.2;
const vcCamera = new THREE.OrthographicCamera(-vcSize, vcSize, vcSize, -vcSize, 0.1, 100);

// Lighting for cube
vcScene.add(new THREE.AmbientLight(0xffffff, 0.6));
const vcDirLight = new THREE.DirectionalLight(0xffffff, 0.5);
vcDirLight.position.set(3, 5, 4);
vcScene.add(vcDirLight);

// Create labeled face texture
function makeFaceTexture(label, bgColor, textColor) {{
    const size = 128;
    const cvs = document.createElement('canvas');
    cvs.width = size; cvs.height = size;
    const c = cvs.getContext('2d');
    // Fill background
    c.fillStyle = bgColor;
    c.fillRect(0, 0, size, size);
    // Border
    c.strokeStyle = 'rgba(255,255,255,0.3)';
    c.lineWidth = 2;
    c.strokeRect(1, 1, size-2, size-2);
    // Text
    c.fillStyle = textColor || '#fff';
    c.font = 'bold 28px -apple-system, BlinkMacSystemFont, sans-serif';
    c.textAlign = 'center';
    c.textBaseline = 'middle';
    c.fillText(label, size/2, size/2);
    const tex = new THREE.CanvasTexture(cvs);
    tex.minFilter = THREE.LinearFilter;
    return tex;
}}

// Face definitions: label, bgColor, camera direction (offset from target)
// ENU: X=East, Y=North, Z=Up
// Three.js BoxGeometry face order: +X, -X, +Y, -Y, +Z, -Z
const faceData = [
    {{ label: 'E',   bg: '#2a5a8a', dir: new THREE.Vector3(1, 0, 0) }},   // +X = East
    {{ label: 'W',   bg: '#2a5a8a', dir: new THREE.Vector3(-1, 0, 0) }},  // -X = West
    {{ label: 'N',   bg: '#2a7a4a', dir: new THREE.Vector3(0, 1, 0) }},   // +Y = North
    {{ label: 'S',   bg: '#2a7a4a', dir: new THREE.Vector3(0, -1, 0) }},  // -Y = South
    {{ label: 'Top', bg: '#5a3a8a', dir: new THREE.Vector3(0, 0, 1) }},   // +Z = Up
    {{ label: 'Btm', bg: '#5a3a8a', dir: new THREE.Vector3(0, 0, -1) }},  // -Z = Down
];

const faceMaterials = faceData.map(f =>
    new THREE.MeshPhongMaterial({{
        map: makeFaceTexture(f.label, f.bg),
        transparent: false,
    }})
);

const cubeGeom = new THREE.BoxGeometry(1.6, 1.6, 1.6);
const cubeMesh = new THREE.Mesh(cubeGeom, faceMaterials);
vcScene.add(cubeMesh);

// Edge wireframe
const edgeGeom = new THREE.EdgesGeometry(cubeGeom);
const edgeMat = new THREE.LineBasicMaterial({{ color: 0xffffff, transparent: true, opacity: 0.35 }});
const edgeLines = new THREE.LineSegments(edgeGeom, edgeMat);
cubeMesh.add(edgeLines);

// Small axis indicators on the cube
const axLen = 1.2;
const vcAxes = new THREE.Group();
vcAxes.add(new THREE.ArrowHelper(new THREE.Vector3(1,0,0), new THREE.Vector3(0,0,0), axLen, 0xff4444, 0.15, 0.08));
vcAxes.add(new THREE.ArrowHelper(new THREE.Vector3(0,1,0), new THREE.Vector3(0,0,0), axLen, 0x44ff44, 0.15, 0.08));
vcAxes.add(new THREE.ArrowHelper(new THREE.Vector3(0,0,1), new THREE.Vector3(0,0,0), axLen, 0x4444ff, 0.15, 0.08));
vcScene.add(vcAxes);

// View transition state
let viewTransition = null; // {{ from: Vector3, to: Vector3, progress: 0..1 }}

function startViewTransition(targetDir) {{
    const dist = camera.position.distanceTo(controls.target);
    const targetPos = controls.target.clone().add(targetDir.clone().normalize().multiplyScalar(dist));
    viewTransition = {{
        fromPos: camera.position.clone(),
        toPos: targetPos,
        fromUp: camera.up.clone(),
        // For top/bottom views, we need to adjust the up vector
        toUp: (Math.abs(targetDir.z) > 0.9) ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(0, 0, 1),
        progress: 0,
    }};
}}

function updateViewTransition() {{
    if (!viewTransition) return;
    viewTransition.progress += 0.06; // ~17 frames to complete
    if (viewTransition.progress >= 1.0) {{
        viewTransition.progress = 1.0;
    }}
    // Ease in-out
    const t = viewTransition.progress;
    const ease = t < 0.5 ? 2*t*t : 1 - Math.pow(-2*t + 2, 2) / 2;

    camera.position.lerpVectors(viewTransition.fromPos, viewTransition.toPos, ease);
    camera.up.lerpVectors(viewTransition.fromUp, viewTransition.toUp, ease).normalize();
    camera.lookAt(controls.target);
    controls.update();

    if (viewTransition.progress >= 1.0) {{
        viewTransition = null;
    }}
}}

// Click handling: raycast on cube faces
const vcRaycaster = new THREE.Raycaster();
vcDiv.addEventListener('click', (e) => {{
    const rect = vcDiv.getBoundingClientRect();
    const mouse = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1
    );
    vcRaycaster.setFromCamera(mouse, vcCamera);
    const hits = vcRaycaster.intersectObject(cubeMesh);
    if (hits.length > 0) {{
        const hit = hits[0];
        const faceIdx = hit.face ? Math.floor(hit.faceIndex / 2) : 0;
        // BoxGeometry has 2 triangles per face, 6 faces: indices 0-1=+X, 2-3=-X, 4-5=+Y, 6-7=-Y, 8-9=+Z, 10-11=-Z
        const dir = faceData[faceIdx].dir.clone();

        // Check if click is near edge/corner for blended views
        const uv = hit.uv;
        if (uv) {{
            const edgeThresh = 0.25;
            const blendDirs = [];
            // Find which other faces this edge is near
            if (uv.x < edgeThresh || uv.x > 1 - edgeThresh || uv.y < edgeThresh || uv.y > 1 - edgeThresh) {{
                // Use the hit normal to find the face, then blend with adjacent face based on world hit position
                const localPoint = hit.point.clone();
                // Normalize the hit point components to find which axes are near the edge
                const ax = Math.abs(localPoint.x);
                const ay = Math.abs(localPoint.y);
                const az = Math.abs(localPoint.z);
                const maxComp = Math.max(ax, ay, az);
                const secondMax = [ax, ay, az].sort((a,b) => b-a)[1];
                // If the second-largest component is close to the max, blend in that direction
                if (secondMax > maxComp * 0.5) {{
                    // Add blend direction from second-largest axis
                    if (ax === secondMax) blendDirs.push(new THREE.Vector3(Math.sign(localPoint.x), 0, 0));
                    if (ay === secondMax) blendDirs.push(new THREE.Vector3(0, Math.sign(localPoint.y), 0));
                    if (az === secondMax) blendDirs.push(new THREE.Vector3(0, 0, Math.sign(localPoint.z)));
                }}
            }}
            if (blendDirs.length > 0) {{
                for (const bd of blendDirs) {{
                    dir.add(bd);
                }}
                dir.normalize();
            }}
        }}
        startViewTransition(dir);
    }}
}});

// Hover effect
vcDiv.addEventListener('mousemove', (e) => {{
    const rect = vcDiv.getBoundingClientRect();
    const mouse = new THREE.Vector2(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1
    );
    vcRaycaster.setFromCamera(mouse, vcCamera);
    const hits = vcRaycaster.intersectObject(cubeMesh);
    // Reset all faces
    for (let i = 0; i < faceMaterials.length; i++) {{
        faceMaterials[i].emissive.setHex(0x000000);
    }}
    if (hits.length > 0) {{
        const faceIdx = Math.floor(hits[0].faceIndex / 2);
        faceMaterials[faceIdx].emissive.setHex(0x333355);
    }}
}});

vcDiv.addEventListener('mouseleave', () => {{
    for (let i = 0; i < faceMaterials.length; i++) {{
        faceMaterials[i].emissive.setHex(0x000000);
    }}
}});

function renderViewCube() {{
    // Sync cube camera with main camera orientation
    const offset = camera.position.clone().sub(controls.target).normalize().multiplyScalar(5);
    vcCamera.position.copy(offset);
    vcCamera.up.copy(camera.up);
    vcCamera.lookAt(0, 0, 0);
    vcRenderer.render(vcScene, vcCamera);
}}

// ── Animation loop ──
function animate(timestamp) {{
    requestAnimationFrame(animate);

    if (playing && lastTimestamp !== null) {{
        const dtReal = (timestamp - lastTimestamp) / 1000;
        const dtSim = dtReal * playSpeed;
        simTime += dtSim;

        // Find new frame matching accumulated simTime
        let newFrame = currentFrame;
        while (newFrame < nFrames - 1 && T.time[newFrame + 1] <= simTime) {{
            newFrame++;
        }}
        if (newFrame >= nFrames - 1) {{
            newFrame = nFrames - 1;
            playing = false;
            document.getElementById('play-btn').innerHTML = '&#9654;';
        }}
        if (newFrame !== currentFrame) {{
            currentFrame = newFrame;
            updateScrubber();
            drawAllCharts();
        }}
    }}
    lastTimestamp = timestamp;

    updateViewTransition();
    updateRocket();
    controls.update();

    // Resize check
    const vpRect = vpDiv.getBoundingClientRect();
    if (renderer.domElement.width !== vpRect.width * window.devicePixelRatio ||
        renderer.domElement.height !== vpRect.height * window.devicePixelRatio) {{
        renderer.setSize(vpRect.width, vpRect.height);
        camera.aspect = vpRect.width / vpRect.height;
        camera.updateProjectionMatrix();
    }}

    renderer.render(scene, camera);
    renderViewCube();
}}

// ── Controls ──
document.getElementById('play-btn').addEventListener('click', () => {{
    playing = !playing;
    if (playing && currentFrame >= nFrames - 1) {{
        currentFrame = 0; // restart
    }}
    simTime = T.time[currentFrame];  // sync simTime to current position
    document.getElementById('play-btn').innerHTML = playing ? '&#9646;&#9646;' : '&#9654;';
    lastTimestamp = null;
}});

document.getElementById('scrubber').addEventListener('input', (e) => {{
    const frac = e.target.value / 1000;
    currentFrame = Math.round(frac * (nFrames - 1));
    simTime = T.time[currentFrame];  // sync simTime to scrubber position
    scrubbing = true;
    drawAllCharts();
    updateRocket();
    updateInfo();
    const t = T.time[currentFrame];
    document.getElementById('time-display').textContent =
        'T+' + t.toFixed(2) + 's / ' + tMax.toFixed(1) + 's';
}});

document.getElementById('speed-select').addEventListener('change', (e) => {{
    playSpeed = parseFloat(e.target.value);
}});

// Keyboard shortcuts
document.addEventListener('keydown', (e) => {{
    if (e.code === 'Space') {{
        e.preventDefault();
        document.getElementById('play-btn').click();
    }} else if (e.code === 'ArrowRight') {{
        currentFrame = Math.min(nFrames - 1, currentFrame + Math.max(1, Math.round(nFrames/200)));
        simTime = T.time[currentFrame];
        scrubbing = true; updateScrubber(); drawAllCharts(); updateRocket();
    }} else if (e.code === 'ArrowLeft') {{
        currentFrame = Math.max(0, currentFrame - Math.max(1, Math.round(nFrames/200)));
        simTime = T.time[currentFrame];
        scrubbing = true; updateScrubber(); drawAllCharts(); updateRocket();
    }} else if (e.code === 'Home') {{
        currentFrame = 0;
        simTime = T.time[currentFrame];
        scrubbing = true; updateScrubber(); drawAllCharts(); updateRocket();
    }} else if (e.code === 'End') {{
        currentFrame = nFrames - 1;
        simTime = T.time[currentFrame];
        scrubbing = true; updateScrubber(); drawAllCharts(); updateRocket();
    }}
}});

// Window resize
window.addEventListener('resize', () => {{
    drawAllCharts();
}});

// ── Initial render ──
updateRocket();
updateScrubber();
drawAllCharts();
updateInfo();
requestAnimationFrame(animate);

// Start camera near the launch pad, looking at origin
controls.target.set(0, 0, 0);
camera.position.set(viewDist * 1.5, -viewDist * 1.2, viewDist * 0.8);
controls.update();
renderViewCube();

</script>
</body>
</html>'''


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def export_flight_html(df: pd.DataFrame, rocket_mesh: RocketMesh,
                       output_path: str = 'flight.html',
                       title: str = 'TinkerRocket Flight',
                       fps: int = 60):
    """Export interactive 3D flight visualization as a self-contained HTML file.

    Args:
        df: Simulation result DataFrame from closed_loop_sim.
        rocket_mesh: RocketMesh instance from build_rocket_mesh().
        output_path: Output HTML file path.
        title: Title shown in the visualization.
        fps: Target animation framerate for telemetry sampling.
    """
    geometry = _mesh_to_json(rocket_mesh)
    telemetry = _prepare_telemetry(df, fps)
    html = _build_html(geometry, telemetry, title)

    out = Path(output_path)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html, encoding='utf-8')
    print(f"Interactive 3D visualization saved to {out} ({len(html) / 1024:.0f} KB)")
