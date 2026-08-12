/* Serve generated reports from a real URL.
 *
 * The tool used to show a report from a blob: URL — an iframe for the preview,
 * the same URL for "Open full page". Charts survive that; the 3D globe does not.
 * CesiumJS comes up on a blob: document, fetches its imagery and terrain
 * successfully, and then renders nothing at all: a black canvas stuck on
 * "Loading imagery…". The same bytes served from http(s):// or opened from
 * file:// render correctly, so the failure is the document's origin, not the
 * report. Opening the blob in a new tab does not help — it fails there too.
 *
 * So the page writes each report into the Cache under a normal same-origin path
 * and this worker serves it back. Nothing is uploaded and nothing is fetched:
 * the response is assembled in the browser from bytes the browser already has,
 * and the only thing that changes is the URL the document ends up with.
 */

const CACHE = "tinkerrocket-reports";

// Take over on first load rather than waiting for a navigation, so the report
// generated in this very session resolves instead of 404-ing until a reload.
self.addEventListener("install", (ev) => ev.waitUntil(self.skipWaiting()));
self.addEventListener("activate", (ev) => ev.waitUntil(self.clients.claim()));

self.addEventListener("fetch", (ev) => {
  const url = new URL(ev.request.url);
  if (url.origin !== self.location.origin) return;
  // Only our own synthetic path. Everything else — the tool itself, the payload,
  // Pyodide from its CDN — goes to the network exactly as before, so this worker
  // cannot make the site stale.
  if (!url.pathname.includes("/report/")) return;

  ev.respondWith(
    caches.open(CACHE).then((cache) =>
      cache.match(ev.request).then(
        (hit) =>
          hit ||
          new Response(
            "<!doctype html><meta charset=utf-8><title>Report expired</title>" +
              "<p style=\"font:16px system-ui;padding:2rem\">This report is no longer in " +
              "the browser's cache. Generate it again from the tool.</p>",
            { status: 404, headers: { "Content-Type": "text/html; charset=utf-8" } }
          )
      )
    )
  );
});
