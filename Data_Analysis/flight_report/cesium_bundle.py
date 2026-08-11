"""The vendored CesiumJS bundle, patched so it survives being opened from file://.

A report is a single HTML file that a flyer downloads and double-clicks. That
means `file://`, and `file://` is where Cesium's own single-file strategy breaks.

The IIFE build embeds its entire worker bundle as a base64 string and boots each
worker with a stub that calls `importScripts(<blob: url of that bundle>)`. Over
http that works. From `file://` the page's origin is opaque, so the worker's is
too, and Chrome refuses:

    Not allowed to load local resource: blob:null/<uuid>

Every terrain and geometry worker dies there. Tiles get requested, nothing can
decode them, and the page sits on an empty canvas that still says "loading" —
a failure that looks exactly like a slow network and reports no error at all.

The fix is to stop importing and start inlining: put the worker bundle's source
straight into the worker body. Same code, one origin check fewer.

The bundle on disk is kept byte-identical to upstream so that updating it stays
a file swap (see vendor/README.md). The patch is applied here, at render time,
and is verified to match exactly once — if a future Cesium reshapes the
bootstrap, this raises instead of quietly shipping a report with a blank globe.
"""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

_VENDOR_DIR = Path(__file__).resolve().parent / "vendor"

# Lifted verbatim from cesium.js v1.144.0. The newlines and indentation are part
# of the template literal in the minified source and must be matched exactly.
_WORKER_IMPORT = (
    'if(!n&&typeof CESIUM_WORKERS<"u"){let a=`\n'
    '      importScripts("${UJ(CESIUM_WORKERS)}");\n'
    '      CesiumWorkers["${i}"]();\n'
    '    `;return r=UJ(a),new Worker(r,o)}'
)
_WORKER_INLINE = (
    'if(!n&&typeof CESIUM_WORKERS<"u"){let a=CESIUM_WORKERS+`\n'
    '      /* TinkerRocket: inlined, not importScripts-ed -- see cesium_bundle.py */\n'
    '      CesiumWorkers["${i}"]();\n'
    '    `;return r=UJ(a),new Worker(r,o)}'
)

# The marker a test can look for to prove the patch reached the output.
PATCH_MARKER = "TinkerRocket: inlined, not importScripts-ed"

# Identifies the bundle in the rendered HTML, the way the plotly banner does.
VERSION_BANNER = "Cesium - https://github.com/CesiumGS/cesium"


class CesiumUnavailable(RuntimeError):
    """The vendored bundle is missing or empty."""


class CesiumPatchError(RuntimeError):
    """The bundle no longer matches the worker patch this module applies."""


@lru_cache(maxsize=1)
def cesium_source() -> str:
    """The vendored Cesium IIFE with its worker bootstrap inlined.

    Cached for the process: the file is 5.7 MB and the CLI renders both report
    levels from one run, so re-reading and re-patching per report is pure cost.
    """
    path = _VENDOR_DIR / "cesium.js"
    try:
        src = path.read_text(encoding="utf-8")
    except OSError as e:
        raise CesiumUnavailable(f"cannot read {path}: {e}") from e
    if not src.strip():
        raise CesiumUnavailable(f"{path} is empty")

    found = src.count(_WORKER_IMPORT)
    if found != 1:
        raise CesiumPatchError(
            f"expected exactly one Cesium worker bootstrap in {path.name}, found "
            f"{found}. The vendored bundle has changed shape, so the file:// "
            "worker patch in cesium_bundle.py no longer applies. Re-pin "
            "_WORKER_IMPORT against the new build before shipping — without the "
            "patch the 3D globe renders as a blank canvas that looks like it is "
            "still loading, with nothing in the console to say why."
        )
    return src.replace(_WORKER_IMPORT, _WORKER_INLINE, 1)


@lru_cache(maxsize=1)
def cesium_css() -> str:
    """Cesium's widget stylesheet, or '' if it is absent (widgets then look bare)."""
    try:
        return (_VENDOR_DIR / "cesium-widgets.css").read_text(encoding="utf-8")
    except OSError:
        return ""
