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
  if (url.pathname.includes("/public-report/")) {
    ev.respondWith(publicReport(url));
    return;
  }
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

/* Someone else's published report, at a real address, with no origin.

   The tool shows another flyer's report, which is their HTML with their
   scripts. Rendered same-origin with the tool, it could read this browser's
   saved flights and the viewer's GitHub token. Rendered from a blob or a
   srcdoc with no origin, it is safe, but the 3D globe cannot run (CesiumJS
   needs a real address; see the top of this file) and a tab of its own would
   be same-origin again.

   So this worker serves it at a real URL under the tool, and every response
   carries a Content-Security-Policy sandbox without allow-same-origin. That
   header, unlike an iframe's sandbox attribute, holds wherever the document
   ends up: in the tool's frame, and in a tab of its own. The report gets a
   real address and an opaque origin, as it would opened from disk, which is
   where the globe was made to work (cesium_bundle.py).

   The worker is the only way in. Without it the same URL is GitHub's 404, so
   a report can never arrive here unsandboxed. It fetches only from
   raw.githubusercontent.com/<login>/tinkerrocket-flight-log, with the login
   and path checked against the same patterns the page uses.

     public-report/ping                               this worker can do it
     public-report/<login>/flights/<id>/<file>.html   the report
*/
const PUBLIC_RE = /\/public-report\/([A-Za-z0-9](?:[A-Za-z0-9]|-(?=[A-Za-z0-9])){0,38})\/(flights\/[a-z0-9][a-z0-9-]{0,80}\/[\w.\- ]{1,120}\.html)$/;
const PUBLIC_REPO = "tinkerrocket-flight-log";
// No allow-same-origin and no allow-top-navigation. Popups may leave the
// sandbox so the report's links to Google Earth and Maps work as pages.
const PUBLIC_CSP =
  "sandbox allow-scripts allow-popups allow-popups-to-escape-sandbox allow-downloads";

function publicResponse(status, body, extra = {}) {
  return new Response(body, {
    status,
    headers: {
      "Content-Type": "text/html; charset=utf-8",
      "Content-Security-Policy": PUBLIC_CSP,
      "X-Content-Type-Options": "nosniff",
      "Cache-Control": "no-store",
      ...extra,
    },
  });
}

function publicMessage(status, text) {
  return publicResponse(status,
    "<!doctype html><meta charset=utf-8><title>Report unavailable</title>" +
      "<p style=\"font:16px system-ui;padding:2rem\">" + text + "</p>");
}

// A local stand-in for raw.githubusercontent.com, for development only: taken
// from ?raw= when both the tool and the stand-in are on this machine.
function rawBase(url) {
  const local = (host) => host === "localhost" || host === "127.0.0.1";
  const dev = url.searchParams.get("raw");
  if (dev && local(self.location.hostname)) {
    try {
      const d = new URL(dev);
      if (local(d.hostname)) return d.href.replace(/\/$/, "");
    } catch (err) { /* ignored */ }
  }
  return "https://raw.githubusercontent.com";
}

async function publicReport(url) {
  if (url.pathname.endsWith("/public-report/ping")) {
    return new Response(null, { status: 204, headers: { "X-TinkerRocket-Public-Report": "1" } });
  }
  let path;
  try {
    path = decodeURIComponent(url.pathname);
  } catch (err) {
    return publicMessage(400, "That report address is not valid.");
  }
  const m = PUBLIC_RE.exec(path);
  if (!m) return publicMessage(404, "That is not a report in a TinkerRocket flight log.");
  const [, login, file] = m;
  const src = `${rawBase(url)}/${encodeURIComponent(login)}/${PUBLIC_REPO}/HEAD/` +
    file.split("/").map(encodeURIComponent).join("/");
  let resp;
  try {
    resp = await fetch(src, { mode: "cors", credentials: "omit" });
  } catch (err) {
    return publicMessage(502, "The report could not be fetched from GitHub. Check the connection and reload.");
  }
  if (resp.status === 404) return publicMessage(404, "That report is not in the flyer's log any more.");
  if (!resp.ok) return publicMessage(502, `GitHub answered ${resp.status} for that report. Try again shortly.`);
  return publicResponse(200, resp.body);
}
