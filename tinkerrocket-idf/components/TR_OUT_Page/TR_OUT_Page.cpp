#include "TR_OUT_Page.h"
#include "RocketComputerTypes.h"

TR_OUT_Page* TR_OUT_Page::instance_ = nullptr;

TR_OUT_Page::TR_OUT_Page(const char* ap_ssid,
                         const char* ap_pass)
: ap_ssid_(ap_ssid),
  ap_pass_(ap_pass),
  server_(80),
  camera_cb_(nullptr),
  state_toggle_cb_(nullptr),
  list_files_cb_(nullptr),
  delete_file_cb_(nullptr),
  download_file_cb_(nullptr)
  {
  }

// -------- Public API --------

bool TR_OUT_Page::begin()
{
  instance_ = this;

  // Start AP
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(ap_ssid_, ap_pass_))
  {
    Serial.println(F("TR_OUT_Page: Failed to start AP!"));
    return false;
  }

  Serial.print(F("AP IP address: "));
  Serial.println(WiFi.softAPIP());

  setupRoutes();
  server_.begin();

  Serial.println(F("TR_OUT_Page: HTTP server started on /, /batteryData, /list, /download, /delete"));
  return true;
}

void TR_OUT_Page::loop()
{
  server_.handleClient();
}

void TR_OUT_Page::updateTelemetry(const Telemetry& t)
{
  telem_ = t;
}

void TR_OUT_Page::updateTelemetry(float soc,
                                  float current,
                                  float voltage,
                                  double latitude,
                                  double longitude,
                                  float gdop,
                                  int num_sats,
                                  RocketState state)
{
  telem_.soc       = soc;
  telem_.current   = current;
  telem_.voltage   = voltage;
  telem_.latitude  = latitude;
  telem_.longitude = longitude;
  telem_.gdop      = gdop;
  telem_.num_sats  = num_sats;
  telem_.state     = state;
}

// -------- Route setup --------

void TR_OUT_Page::setupRoutes()
{
  server_.on("/",            HTTP_GET, handleRootThunk);
  server_.on("/batteryData", HTTP_GET, handleDataThunk);
  server_.on("/list",        HTTP_GET, handleListFilesThunk);
  server_.on("/download",    HTTP_GET, handleDownloadThunk);
  server_.on("/delete",      HTTP_POST, handleDeleteThunk);
  server_.on("/camera/toggle", HTTP_ANY, handleCameraToggleThunk);
  server_.on("/state/toggle", HTTP_POST, handleStateToggleThunk);

  // Simple redirect to /list
  server_.on("/files", HTTP_GET, []()
  {
    if (!instance_) return;
    instance_->server_.sendHeader("Location", "/list");
    instance_->server_.send(303);
  });
}

// -------- Static thunks --------

void TR_OUT_Page::handleRootThunk()
{
  if (instance_) instance_->handleRoot();
}
void TR_OUT_Page::handleDataThunk()
{
  if (instance_) instance_->handleData();
}
void TR_OUT_Page::handleListFilesThunk()
{
  if (instance_) instance_->handleListFiles();
}
void TR_OUT_Page::handleDownloadThunk()
{
  if (instance_) instance_->handleDownload();
}
void TR_OUT_Page::handleDeleteThunk()
{
  if (instance_) instance_->handleDelete();
}

void TR_OUT_Page::handleCameraToggleThunk()
{
  if (!instance_) return;
  instance_->handleCameraToggle();
}

void TR_OUT_Page::handleStateToggle()
{
  // Only allow toggling between READY and PRELAUNCH.
  RocketState current = telem_.state;
  RocketState requested = current;

  if (current == READY) {
    requested = PRELAUNCH;
  } else if (current == PRELAUNCH) {
    requested = READY;
  } else {
    // Other states: just ignore, but respond with current state
    requested = current;
  }

  // Update local telemetry so the UI reacts instantly
  telem_.state = requested;

  // Notify main sketch so it can change rocket_state
  if (state_toggle_cb_) {
    state_toggle_cb_(requested);
  }

  // Respond with JSON including new state string
  String json = F("{\"state\":\"");
  json += rocketStateToString(telem_.state);
  json += F("\"}");
  server_.send(200, F("application/json"), json);
}

void TR_OUT_Page::handleStateToggleThunk()
{
  if (!instance_) return;
  instance_->handleStateToggle();
}

void TR_OUT_Page::handleCameraToggle()
{
  // Toggle desired recording state
  bool newRecording = !telem_.camera_recording;

  // Call back into main sketch (if provided)
  if (camera_cb_)
  {
    camera_cb_(newRecording);
  }

  telem_.camera_recording = newRecording;

  // Respond with small, valid JSON
  String json = F("{\"camera_recording\":");
  json += (telem_.camera_recording ? F("true") : F("false"));
  json += F("}");
  server_.send(200, F("application/json"), json);
}
void TR_OUT_Page::setCameraControlCallback(CameraControlCallback cb) {
  camera_cb_ = cb;
}

void TR_OUT_Page::setStateToggleCallback(StateToggleCallback cb) {
  state_toggle_cb_ = cb;
}

void TR_OUT_Page::setFileCallbacks(ListFilesCallback list_cb,
                                   DeleteFileCallback delete_cb,
                                   DownloadFileCallback download_cb) {
  list_files_cb_ = list_cb;
  delete_file_cb_ = delete_cb;
  download_file_cb_ = download_cb;
}

// -------- Page generation --------

String TR_OUT_Page::generateWebPage() const
{
  const char* stateStr = rocketStateToString(telem_.state);
  const char* css      = rocketStateToCss(telem_.state);

  // Big HTML/JS block as a raw string literal
  String html = R"====(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rocket Access Point</title>
  <style>
    body {
      text-align:center;
      font-family:Arial,sans-serif;
      margin:0;
      padding:16px;
    }
    .box {
      display:inline-block;
      padding:10px 16px;
      margin:5px;
      border-radius:8px;
      color:#fff;
      font-weight:bold;
      min-width:140px;
    }
    .red    { background:#e74c3c; }
    .orange { background:#e67e22; }
    .green  { background:#27ae60; }
    .blue   { background:#3498db; }
    .yellow { background:#f1c40f; color:#000; }
    .cam-red   { background:#e74c3c; color:#fff; }
    .cam-green { background:#27ae60; color:#fff; }
    .flashing-green {
      background:#27ae60;
      animation:flash 1s infinite;
    }
    @keyframes flash {
      0%,100% { opacity:1; }
      50%     { opacity:0; }
    }
    a.btn {
      padding:6px 10px;
      border:1px solid #444;
      border-radius:4px;
      text-decoration:none;
      cursor:pointer;
    }
    a.btn.disabled {
      opacity:0.5;
      pointer-events:none;
    }
    a.btn:not(.cam-red):not(.cam-green) {
      background:#f4f4f4;
      color:#000;
    }
  </style>
</head>
<body>
  <h1>Rocket Access Point</h1>
  <p><a class="btn" href="/list">📁 Files (download / delete)</a></p>

  <h2>Battery Data</h2>
  <p>SOC: <span id="soc">--</span> %</p>
  <p>Current: <span id="current">--</span> mA</p>
  <p>Voltage: <span id="voltage">--</span> V</p>

  <h2>Current State: <span id="state">{{STATE_TEXT}}</span></h2>
  <div id="stateBox" class="box {{STATE_CLASS}}">{{STATE_TEXT}}</div>
  <p><a class="btn disabled" href="#" id="stateToggleBtn">State control not available</a></p>

  <h2>GPS Coordinates</h2>
  <p><span id="latitude">--</span>, <span id="longitude">--</span></p>

  <h2>GNSS Info</h2>
  <p>Satellites: <span id="num_sats">--</span></p>
  <p>GDOP: <span id="gdop">--</span></p>

  <h2>Flight Metrics</h2>
  <p>Max Altitude: <span id="max_alt_m">--</span> m</p>
  <p>Max Speed: <span id="max_speed_mps">--</span> m/s</p>

  <h2>Camera</h2>
  <p><a class="btn cam-red" href="#" id="cameraToggleBtn">Camera Stopped</a></p>

  <script>
    const stateClasses = {
      INITIALIZATION: 'red',
      READY:          'orange',
      PRELAUNCH:      'green',
      INFLIGHT:       'blue',
      LANDED:         'yellow'
    };

    function updateStateUI(state) {
      const box = document.getElementById('stateBox');
      const txt = document.getElementById('state');
      if (!box || !txt || !state) return;

      txt.innerText  = state;
      box.innerText  = state;
      box.className  = 'box ' + (stateClasses[state] || '');
    }

    function updateStateButton(state) {
      const stateBtn = document.getElementById('stateToggleBtn');
      if (!stateBtn) return;

      if (state === 'READY') {
        stateBtn.innerText = 'Go to PRELAUNCH';
        stateBtn.classList.remove('disabled');
      } else if (state === 'PRELAUNCH') {
        stateBtn.innerText = 'Back to READY';
        stateBtn.classList.remove('disabled');
      } else {
        stateBtn.innerText = 'State control not available';
        stateBtn.classList.add('disabled');
      }
    }

    function updateCameraButton(isRecording) {
      const camBtn = document.getElementById('cameraToggleBtn');
      if (!camBtn) return;

      if (isRecording) {
        camBtn.innerText = 'Recording';
        camBtn.classList.remove('cam-red');
        camBtn.classList.add('cam-green');
      } else {
        camBtn.innerText = 'Camera Stopped';
        camBtn.classList.remove('cam-green');
        camBtn.classList.add('cam-red');
      }
    }

    function pollData() {
      fetch('/batteryData?ts=' + Date.now())
        .then(r => r.json())
        .then(d => {
          if (d.soc     !== null && d.soc     !== undefined) document.getElementById('soc').innerText      = d.soc.toFixed(1);
          if (d.current !== null && d.current !== undefined) document.getElementById('current').innerText  = d.current.toFixed(1);
          if (d.voltage !== null && d.voltage !== undefined) document.getElementById('voltage').innerText  = d.voltage.toFixed(2);
          if (d.latitude!== null && d.latitude!== undefined) document.getElementById('latitude').innerText = d.latitude.toFixed(6);
          if (d.longitude!==null && d.longitude!==undefined) document.getElementById('longitude').innerText= d.longitude.toFixed(6);
          if (d.gdop    !== null && d.gdop    !== undefined) document.getElementById('gdop').innerText      = d.gdop.toFixed(1);
          document.getElementById('num_sats').innerText = d.num_sats;
          if (d.max_alt_m !== null && d.max_alt_m !== undefined) document.getElementById('max_alt_m').innerText = d.max_alt_m.toFixed(1);
          if (d.max_speed_mps !== null && d.max_speed_mps !== undefined) document.getElementById('max_speed_mps').innerText = d.max_speed_mps.toFixed(1);

          if (d.state) {
            updateStateUI(d.state);
            updateStateButton(d.state);
          }
          if (typeof d.camera_recording !== 'undefined') {
            updateCameraButton(d.camera_recording);
          }
        })
        .catch(e => console.error('Data Error:', e));
    }

    document.addEventListener('DOMContentLoaded', () => {
      const camBtn   = document.getElementById('cameraToggleBtn');
      const stateBtn = document.getElementById('stateToggleBtn');

      if (camBtn) {
        camBtn.addEventListener('click', e => {
          e.preventDefault();
          fetch('/camera/toggle', { method: 'POST' })
            .then(r => r.json())
            .then(d => {
              if (typeof d.camera_recording !== 'undefined') {
                updateCameraButton(d.camera_recording);
              }
            })
            .catch(err => console.error('Camera toggle error:', err));
        });
      }

      if (stateBtn) {
        stateBtn.addEventListener('click', e => {
          e.preventDefault();
          if (stateBtn.classList.contains('disabled')) return;

          fetch('/state/toggle', { method: 'POST' })
            .then(r => r.json())
            .then(d => {
              if (d.state) {
                updateStateUI(d.state);
                updateStateButton(d.state);
              }
            })
            .catch(err => console.error('State toggle error:', err));
        });
      }

      setInterval(pollData, 500);
      pollData();
    });
  </script>
</body>
</html>
)====";

  // Patch in the current state text and CSS class
  html.replace("{{STATE_TEXT}}",  stateStr);
  html.replace("{{STATE_CLASS}}", css);

  return html;
}

// -------- Handlers --------

void TR_OUT_Page::handleRoot() {
  server_.send(200, F("text/html"), generateWebPage());
}

void TR_OUT_Page::handleData() {
  String json = F("{\"soc\":");
  json += (isnan(telem_.soc) ? F("null") : String(telem_.soc, 3));

  json += F(",\"current\":");
  json += (isnan(telem_.current) ? F("null") : String(telem_.current, 3));

  json += F(",\"voltage\":");
  json += (isnan(telem_.voltage) ? F("null") : String(telem_.voltage, 3));

  json += F(",\"latitude\":");
  json += (isnan(telem_.latitude) ? F("null") : String(telem_.latitude, 6));

  json += F(",\"longitude\":");
  json += (isnan(telem_.longitude) ? F("null") : String(telem_.longitude, 6));

  json += F(",\"gdop\":");
  json += (isnan(telem_.gdop) ? F("null") : String(telem_.gdop, 2));

  json += F(",\"num_sats\":");
  json += String(telem_.num_sats);

  json += F(",\"state\":\"");
  json += rocketStateToString(telem_.state);
  json += F("\"");

  // camera state inside the same JSON object
  json += F(",\"camera_recording\":");
  json += (telem_.camera_recording ? F("true") : F("false"));

  json += F(",\"logging_active\":");
  json += (telem_.logging_active ? F("true") : F("false"));

  json += F(",\"active_file\":\"");
  json += telem_.active_file;
  json += F("\"");

  json += F(",\"rx_kbs\":");
  json += (isnan(telem_.rx_kbs) ? F("null") : String(telem_.rx_kbs, 2));

  json += F(",\"wr_kbs\":");
  json += (isnan(telem_.wr_kbs) ? F("null") : String(telem_.wr_kbs, 2));

  json += F(",\"frames_rx\":");
  json += String(telem_.frames_rx);

  json += F(",\"frames_drop\":");
  json += String(telem_.frames_drop);

  json += F(",\"max_alt_m\":");
  json += (isnan(telem_.max_alt_m) ? F("null") : String(telem_.max_alt_m, 2));

  json += F(",\"max_speed_mps\":");
  json += (isnan(telem_.max_speed_mps) ? F("null") : String(telem_.max_speed_mps, 2));

  json += F("}");   // single closing brace

  server_.send(200, F("application/json"), json);
}

// --- File list (/list) ---
void TR_OUT_Page::handleListFiles() {
  if (!list_files_cb_) {
    server_.send(503, F("text/plain"), F("File listing callback not configured"));
    return;
  }

  FileItem items[64];
  const size_t count = list_files_cb_(items, sizeof(items) / sizeof(items[0]));

  String html =
    F("<!DOCTYPE html><html><head><title>File List</title>"
      "<meta name='viewport' content='width=device-width, initial-scale=1' />"
      "<style>"
      "body{font-family:system-ui,Arial;max-width:800px;margin:20px;}"
      "ul{list-style:none;padding:0}"
      "li{display:flex;gap:10px;align-items:center;justify-content:space-between;margin:6px 0;}"
      ".left{display:flex;gap:10px;align-items:center;}"
      "a.btn,button{padding:4px 8px;border:1px solid #444;background:#f4f4f4;cursor:pointer;border-radius:4px}"
      "button.del{background:#ffe8e8;border-color:#e33;color:#900}"
      "small{opacity:.7;display:block}"
      "form{display:inline}"
      "</style>"
      "<script>"
      "function confirmDel(fn){return confirm('Delete \"'+fn+'\"? This cannot be undone.');}"
      "</script>"
      "</head><body>"
      "<h1>Flash Log Files</h1>"
      "<p><a href='/'>&larr; Back</a></p>"
      "<ul>");

  for (size_t i = 0; i < count; ++i) {
    const String name = String(items[i].name);
    const uint32_t size = items[i].size_bytes;
    html += F("<li>"
              "<div class='left'>");
    html += "<a class='btn' href=\"/download?file=" + name + "\">Download</a>";
    html += "<form method='post' action='/delete' onsubmit=\"return confirmDel('" + name + "')\">"
            "<input type='hidden' name='file' value='" + name + "'/>"
            "<button class='del' type='submit'>Delete</button>"
            "</form></div><div><strong>";
    html += name;
    html += F("</strong><small>");
    html += String(size);
    html += F(" bytes</small></div></li>");
  }

  html += F("</ul></body></html>");
  server_.send(200, F("text/html"), html);
}

// --- Download (/download?file=...) ---
void TR_OUT_Page::handleDownload() {
  if (!server_.hasArg("file")) {
    server_.send(400, F("text/plain"), F("Missing 'file' parameter"));
    return;
  }
  String fn = server_.arg("file");

  // basic sanitization
  if (fn.length() == 0 || fn.indexOf('/') >= 0 || fn == "." || fn == "..") {
    server_.send(400, F("text/plain"), F("Invalid filename"));
    return;
  }

  if (!download_file_cb_) {
    server_.send(503, F("text/plain"), F("Download callback not configured"));
    return;
  }
  if (!download_file_cb_(fn.c_str(), server_)) {
    server_.send(404, F("text/plain"), F("File not found"));
  }
}

// --- Delete (POST /delete) ---
void TR_OUT_Page::handleDelete() {
  if (!server_.hasArg("file")) {
    server_.send(400, F("text/plain"), F("Missing 'file' parameter"));
    return;
  }
  String fn = server_.arg("file");

  // basic sanitization
  if (fn.length() == 0 || fn.indexOf('/') >= 0 || fn == "." || fn == "..") {
    server_.send(400, F("text/plain"), F("Invalid filename"));
    return;
  }

  if (!delete_file_cb_) {
    server_.send(503, F("text/plain"), F("Delete callback not configured"));
    return;
  }

  bool ok = delete_file_cb_(fn.c_str());
  if (!ok) {
    server_.send(423, F("text/plain"), F("Could not delete (file may be in use)"));
    return;
  }

  server_.sendHeader("Location", "/list");
  server_.send(303);
}

// --- MIME helper ---
String TR_OUT_Page::mimeTypeFor(const String& name) {
  String n = name;
  n.toLowerCase();
  if (n.endsWith(".csv"))  return F("text/csv");
  if (n.endsWith(".txt"))  return F("text/plain");
  if (n.endsWith(".json")) return F("application/json");
  if (n.endsWith(".bin"))  return F("application/octet-stream");
  return F("application/octet-stream");
}


const char* TR_OUT_Page::rocketStateToCss(RocketState s) {
  switch (s) {
    case INITIALIZATION: return "red";
    case READY:          return "orange";
    case PRELAUNCH:      return "green";
    case INFLIGHT:       return "blue";
    case LANDED:         return "yellow";
    default:             return "";
  }
}

const char* TR_OUT_Page::rocketStateToString(RocketState s) {
  switch (s) {
    case INITIALIZATION: return "INITIALIZATION";
    case READY:          return "READY";
    case PRELAUNCH:      return "PRELAUNCH";
    case INFLIGHT:       return "INFLIGHT";
    case LANDED:         return "LANDED";
    default:             return "UNKNOWN";
  }
}
