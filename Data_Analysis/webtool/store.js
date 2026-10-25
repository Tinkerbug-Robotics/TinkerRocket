/* Saved flights, kept in this browser.

   IndexedDB rather than localStorage because a saved flight is megabytes: the
   report alone is ~9 MB (most of it the vendored 3D globe), and the flight's
   own files ride along so it can be published, or analysed again, later.

   Two object stores. `flights` holds the entries — small, read whole to draw
   the log. `blobs` holds the files under "<id>/<name>" keys, read only when a
   flight is opened, so listing a hundred flights never touches a gigabyte.

   Nothing here leaves the machine. Publishing reads from this store and sends
   to the flyer's own repository; see github.js. */

const DB_NAME = "tinkerrocket-flightlog";
const DB_VERSION = 1;

let dbPromise = null;

function db() {
  if (!dbPromise) {
    dbPromise = new Promise((resolve, reject) => {
      const req = indexedDB.open(DB_NAME, DB_VERSION);
      req.onupgradeneeded = () => {
        const d = req.result;
        if (!d.objectStoreNames.contains("flights")) d.createObjectStore("flights", { keyPath: "id" });
        if (!d.objectStoreNames.contains("blobs")) d.createObjectStore("blobs");
      };
      req.onsuccess = () => resolve(req.result);
      req.onerror = () => reject(req.error);
    });
  }
  return dbPromise;
}

function tx(stores, mode, fn) {
  return db().then((d) => new Promise((resolve, reject) => {
    const t = d.transaction(stores, mode);
    let result;
    t.oncomplete = () => resolve(result);
    t.onerror = () => reject(t.error);
    t.onabort = () => reject(t.error || new Error("transaction aborted"));
    result = fn(t);
  }));
}

function request(req) {
  return new Promise((resolve, reject) => {
    req.onsuccess = () => resolve(req.result);
    req.onerror = () => reject(req.error);
  });
}

export async function listFlights() {
  const d = await db();
  return request(d.transaction("flights").objectStore("flights").getAll());
}

export async function getFlight(id) {
  const d = await db();
  return request(d.transaction("flights").objectStore("flights").get(id));
}

export function putFlight(entry) {
  return tx(["flights"], "readwrite", (t) => { t.objectStore("flights").put(entry); });
}

/* An entry and its files in one transaction, so a saved flight never exists
   without the report it points at. `files` is [{name, blob}] where name is
   the entry-relative path ("report.html", "data/flight_….bin"). */
export function saveFlight(entry, files) {
  return tx(["flights", "blobs"], "readwrite", (t) => {
    const blobs = t.objectStore("blobs");
    for (const f of files) blobs.put(f.blob, `${entry.id}/${f.name}`);
    t.objectStore("flights").put(entry);
  });
}

export async function getBlob(id, name) {
  const d = await db();
  return request(d.transaction("blobs").objectStore("blobs").get(`${id}/${name}`));
}

export function deleteFlight(id) {
  return tx(["flights", "blobs"], "readwrite", (t) => {
    t.objectStore("flights").delete(id);
    // Every key under "<id>/". "/" is followed in code-point order by "0",
    // so the range [id/, id0) is exactly that prefix.
    t.objectStore("blobs").delete(IDBKeyRange.bound(`${id}/`, `${id}0`, false, true));
  });
}

/* Ask the browser not to evict the log under storage pressure. Best-effort:
   some browsers grant it silently, some only after a prompt, some never. */
export async function persist() {
  try {
    if (navigator.storage && navigator.storage.persist) return await navigator.storage.persist();
  } catch (err) { /* not offered */ }
  return false;
}
