/* Showing a report this browser generated.

   Moved out of app.js so a saved flight opens the same way a fresh report
   does. Only for reports made here, by our own template from the flyer's own
   files: the frame they render in is same-origin with this page (see the
   iframe's comment in index.html), which is acceptable for our own output and
   never for someone else's. A public log's reports are shown from the
   publishing flyer's own GitHub Pages origin instead — see logui.js. */

// Registered once, best-effort. Everything still works without it except the 3D
// globe — see sw.js for why a blob: URL cannot render one.
let swReady = null;
if ("serviceWorker" in navigator) {
  swReady = navigator.serviceWorker
    .register("sw.js")
    .then(() => navigator.serviceWorker.ready)
    .catch(() => null);
}

// Whether the worker is actually in front of our requests. Deliberately checked
// when a report is ready rather than at registration: on a first visit the
// worker installs, activates and calls clients.claim() over several ticks, and
// sampling `controller` before that lands reads false for a worker that is about
// to control the page — which then serves the report from a blob and loses the
// globe for no reason.
async function swControlling() {
  if (!swReady || !(await swReady)) return false;
  if (navigator.serviceWorker.controller) return true;
  await new Promise((resolve) => {
    const done = setTimeout(resolve, 3000);
    navigator.serviceWorker.addEventListener(
      "controllerchange",
      () => { clearTimeout(done); resolve(); },
      { once: true }
    );
  });
  return !!navigator.serviceWorker.controller;
}

// A real same-origin URL for a generated report, or null if the worker is not
// available (private windows and older browsers both land here). CesiumJS builds
// a viewer on a blob: document, fetches its imagery, and then renders a black
// canvas forever; served from a normal path the identical bytes render.
export async function reportUrl(body, stem) {
  try {
    if (!(await swControlling())) return null;
    const url = new URL(`report/${Date.now()}/${stem}.html`, location.href).toString();
    const cache = await caches.open("tinkerrocket-reports");
    await cache.put(
      url,
      new Response(body, { headers: { "Content-Type": "text/html; charset=utf-8" } })
    );
    return url;
  } catch (err) {
    return null;
  }
}

/* Point a frame, an "open" link and a download link at a report.

   The download always comes from a blob: the file is being handed to the
   user, not rendered, so its origin is irrelevant. Returns whether the frame
   got a served URL — without one the globe cannot draw, and the caller says
   so rather than letting the reader wonder why one section is black. */
export async function showLocalReport(body, stem, { frame, open, download }) {
  const blob = body instanceof Blob ? body : new Blob([body], { type: "text/html" });
  const blobUrl = URL.createObjectURL(blob);
  if (download) {
    download.href = blobUrl;
    download.download = `${stem}.html`;
  }
  const served = await reportUrl(blob, stem);
  frame.src = served || blobUrl;
  if (open) open.href = served || blobUrl;
  return !!served;
}
