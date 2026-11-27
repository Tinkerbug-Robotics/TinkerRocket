#!/usr/bin/env python3
"""Make the dev edition's app icons: the store icon with a green "D" badge.

The dev and store editions sit side by side on one phone (docs/app-editions.md),
so the dev app's icon carries a small white disc with a bold green D in its
top-right corner. This draws it from the store icons, so re-run it whenever
those change:

    python3 tools/make_dev_icons.py

iOS:     Assets.xcassets/AppIcon.appiconset      -> AppIconDev.appiconset
         (the Debug and Release configurations use AppIconDev; Store keeps AppIcon)
Android: src/main/res/mipmap-*/ic_launcher_{foreground,monochrome}.png
         -> src/dev/res/mipmap-*/  (the dev flavor's copies override main's)

Placement: the top-right sky, clear of the antennae, and inside what each
platform's mask always shows. iOS rounds the corners at about 22% of the side,
so the badge sits in from the corner; an Android adaptive icon only promises the
middle 66 of its 108 dp, so there the badge is drawn inside that circle.
"""
from pathlib import Path
import json

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
IOS_ASSETS = ROOT / "TinkerRocketApp/TinkerRocketApp/Assets.xcassets"
ANDROID_RES = ROOT / "TinkerRocketAndroid/app/src"
DENSITIES = ["mdpi", "hdpi", "xhdpi", "xxhdpi", "xxxhdpi"]

GREEN = (30, 158, 63, 255)      # dark enough to read on white at 60 pt
WHITE = (255, 255, 255, 255)
FONT = "/System/Library/Fonts/Supplemental/Arial Rounded Bold.ttf"

# Badge centre and radius as fractions of the image side.
IOS_BADGE = (0.800, 0.200, 0.135)
# Android: inside the 66 dp safe circle of the 108 dp canvas (centre 0.5, 0.5;
# radius 0.3056): centre 0.233 from the middle, plus a radius of 0.070.
ANDROID_BADGE = (0.665, 0.335, 0.070)


def badge(size: int, cx: float, cy: float, r: float, mono: bool = False) -> Image.Image:
    """A transparent layer, `size` square, with the badge on it.

    mono: for Android's themed-icon layer, which keeps only alpha: an opaque
    disc with the D cut out of it.
    """
    s = 4  # draw large and scale down for smooth edges
    big = size * s
    layer = Image.new("RGBA", (big, big), (0, 0, 0, 0))
    d = ImageDraw.Draw(layer)
    x, y, rad = cx * big, cy * big, r * big

    if not mono:
        # A soft shadow so the white disc lifts off the pale sky.
        sh = Image.new("RGBA", (big, big), (0, 0, 0, 0))
        ImageDraw.Draw(sh).ellipse(
            (x - rad, y - rad + rad * 0.06, x + rad, y + rad + rad * 0.06), fill=(0, 0, 0, 60))
        layer = Image.alpha_composite(layer, sh)
        d = ImageDraw.Draw(layer)

    d.ellipse((x - rad, y - rad, x + rad, y + rad), fill=WHITE)
    if not mono:
        ring = max(1, int(rad * 0.08))
        d.ellipse((x - rad, y - rad, x + rad, y + rad), outline=GREEN, width=ring)

    font = ImageFont.truetype(FONT, int(rad * 1.25))
    left, top, right, bottom = d.textbbox((0, 0), "D", font=font)
    tx = x - (left + right) / 2
    ty = y - (top + bottom) / 2
    if mono:
        mask = Image.new("L", (big, big), 0)
        ImageDraw.Draw(mask).text((tx, ty), "D", font=font, fill=255)
        alpha = layer.getchannel("A")
        alpha.paste(0, mask=mask)
        layer.putalpha(alpha)
    else:
        d.text((tx, ty), "D", font=font, fill=GREEN)
    return layer.resize((size, size), Image.LANCZOS)


def with_badge(src: Path, dst: Path, spec, mono: bool = False) -> None:
    im = Image.open(src).convert("RGBA")
    cx, cy, r = spec
    out = Image.alpha_composite(im, badge(im.size[0], cx, cy, r, mono))
    if mono:
        # The themed layer is all white; only its alpha says anything.
        out = Image.merge("RGBA", (*Image.new("RGB", out.size, (255, 255, 255)).split(),
                                   out.getchannel("A")))
    dst.parent.mkdir(parents=True, exist_ok=True)
    # An iOS app icon must not carry alpha.
    (out.convert("RGB") if dst.suffix == ".png" and "appiconset" in str(dst) else out).save(dst)
    print(f"  wrote {dst.relative_to(ROOT)}")


def ios() -> None:
    src_set = IOS_ASSETS / "AppIcon.appiconset"
    dst_set = IOS_ASSETS / "AppIconDev.appiconset"
    contents = json.loads((src_set / "Contents.json").read_text())
    names = {img["filename"] for img in contents["images"] if "filename" in img}
    for name in sorted(names):
        dev_name = name.replace(".png", "_dev.png")
        with_badge(src_set / name, dst_set / dev_name, IOS_BADGE)
        for img in contents["images"]:
            if img.get("filename") == name:
                img["filename"] = dev_name
    dst_set.mkdir(parents=True, exist_ok=True)
    (dst_set / "Contents.json").write_text(json.dumps(contents, indent=2) + "\n")
    print(f"  wrote {(dst_set / 'Contents.json').relative_to(ROOT)}")


def android() -> None:
    for dpi in DENSITIES:
        for name, mono in (("ic_launcher_foreground", False), ("ic_launcher_monochrome", True)):
            with_badge(ANDROID_RES / f"main/res/mipmap-{dpi}/{name}.png",
                       ANDROID_RES / f"dev/res/mipmap-{dpi}/{name}.png",
                       ANDROID_BADGE, mono)


if __name__ == "__main__":
    ios()
    android()
