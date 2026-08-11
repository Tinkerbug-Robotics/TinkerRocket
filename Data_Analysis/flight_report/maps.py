"""Ground-track map specs (Leaflet + OpenStreetMap tiles).

A map is a plain dict — ``{"id", "title", "points", "note"}`` — that render.py
serializes and the template hands to Leaflet. The vendored Leaflet bundle is
inlined only when a report actually contains a map.

Unlike everything else in the report, a map needs the network at *view* time:
the track and markers are drawn from embedded coordinates, but the imagery
underneath comes from a tile server. Opened offline the basemap stays blank and
the track still draws, which is why the track is styled to be legible against
either. Tiles are requested from openstreetmap.org, so viewing a report tells
that server roughly where the flight was.
"""

from __future__ import annotations

import base64
from typing import Any, Optional, Sequence
from xml.sax.saxutils import escape as xml_escape

import numpy as np

# Enough to place a landing in a field without shipping a needlessly long path.
MAX_TRACK_POINTS = 4000


def track_map(
    map_id: str,
    title: str,
    lat: Sequence[float],
    lon: Sequence[float],
    note: str = "",
    alt: Optional[Sequence[float]] = None,
    name: str = "Flight",
) -> Optional[dict[str, Any]]:
    """A flight path with start and landing markers, or None if there's no fix.

    Zeroed and non-finite coordinates are dropped: the receiver reports 0/0
    before it has a fix, and plotting those puts a leg of the track through the
    Gulf of Guinea.
    """
    la = np.asarray(lat, dtype=float)
    lo = np.asarray(lon, dtype=float)
    n = min(la.size, lo.size)
    la, lo = la[:n], lo[:n]

    good = np.isfinite(la) & np.isfinite(lo) & (la != 0) & (lo != 0)
    la, lo = la[good], lo[good]
    if la.size < 2:
        return None

    if la.size > MAX_TRACK_POINTS:
        stride = int(np.ceil(la.size / MAX_TRACK_POINTS))
        la, lo = la[::stride], lo[::stride]

    # 6 decimals is ~0.1 m — well past what the receiver resolves.
    points = [[round(float(a), 6), round(float(o), 6)] for a, o in zip(la, lo)]
    alt_m = None
    if alt is not None:
        a_arr = np.asarray(alt, dtype=float)[good]
        if a_arr.size == la.size:
            alt_m = [round(float(v), 1) for v in a_arr]

    return {
        "id": map_id,
        "title": title,
        "points": points,
        "start": points[0],
        "end": points[-1],
        "note": note,
        "kml": _kml_data_uri(name, points, alt_m),
        # Google's own imagery, without an API key or a per-reader bill: hand the
        # coordinates to Google's viewers instead of embedding their tiles.
        "earth_url": f"https://earth.google.com/web/@{points[-1][0]},{points[-1][1]},1000a,2000d,35y,0h,45t,0r",
        "maps_url": f"https://www.google.com/maps/search/?api=1&query={points[-1][0]},{points[-1][1]}",
    }


def _kml_data_uri(name: str, points: list[list[float]], alt_m: Optional[list[float]]) -> str:
    """The track as a KML download link.

    Opening this in Google Earth puts the flight over Google's imagery in 3D,
    which is both better than a flat basemap for a trajectory and free of the
    key and per-view billing that embedding Google tiles would carry. When
    altitudes are available the track is drawn absolute so the arc is visible;
    otherwise it is clamped to the ground.
    """
    if alt_m is not None:
        coords = " ".join(f"{lon},{lat},{alt}" for (lat, lon), alt in zip(points, alt_m))
        altitude_mode = "absolute"
        extrude = 1
    else:
        coords = " ".join(f"{lon},{lat},0" for lat, lon in points)
        altitude_mode = "clampToGround"
        extrude = 0

    safe = xml_escape(name)
    kml = (
        '<?xml version="1.0" encoding="UTF-8"?>'
        '<kml xmlns="http://www.opengis.net/kml/2.2"><Document>'
        f"<name>{safe}</name>"
        '<Style id="track"><LineStyle><color>ff0077ff</color><width>3</width></LineStyle>'
        '<PolyStyle><color>3f0077ff</color></PolyStyle></Style>'
        f"<Placemark><name>{safe} flight path</name><styleUrl>#track</styleUrl>"
        f"<LineString><extrude>{extrude}</extrude><tessellate>1</tessellate>"
        f"<altitudeMode>{altitude_mode}</altitudeMode>"
        f"<coordinates>{coords}</coordinates></LineString></Placemark>"
        f"<Placemark><name>Pad</name><Point><coordinates>"
        f"{points[0][1]},{points[0][0]},0</coordinates></Point></Placemark>"
        f"<Placemark><name>Landing</name><Point><coordinates>"
        f"{points[-1][1]},{points[-1][0]},0</coordinates></Point></Placemark>"
        "</Document></kml>"
    )
    b64 = base64.b64encode(kml.encode("utf-8")).decode("ascii")
    return f"data:application/vnd.google-earth.kml+xml;base64,{b64}"
