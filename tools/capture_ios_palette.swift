// Capture the iOS system colours that design/tokens.json transcribes.
//
// iOS is the design reference and renders these colours LIVE — the iOS app
// calls `.green`/`.red`/… and gets whatever the OS resolves.  Android cannot,
// so tokens.json holds a frozen copy and Android renders that.  The copy goes
// stale whenever Apple retunes the palette, which they did in iOS 26: seven of
// the nine entries moved, some visibly (purple #AF52DE -> #CB30E0).
//
// This script is how the copy gets refreshed.  Run it against the newest
// simulator runtime, paste the two tables into design/tokens.json, and run
// tools/gen_design_tokens.py.
//
//   xcrun --sdk iphonesimulator swiftc \
//       -target arm64-apple-ios26.0-simulator tools/capture_ios_palette.swift -o /tmp/palette
//   xcrun simctl spawn booted /tmp/palette
//
// The increased-contrast columns are printed but deliberately NOT stored.
// tokens.json has two slots per colour (light/dark) and the generated
// `dynamic()` helper reads only `userInterfaceStyle`, so the accessible
// variants have nowhere to live.  They are shown here because that gap is a
// real difference between the platforms -- iOS tracks Increase Contrast for
// free, Android does not -- and anyone refreshing the palette should see the
// size of it (light yellow moves #FFCC00 -> #A16A00).

import UIKit

func hex(_ color: UIColor, _ traits: UITraitCollection) -> String {
    var r: CGFloat = 0, g: CGFloat = 0, b: CGFloat = 0, a: CGFloat = 0
    color.resolvedColor(with: traits).getRed(&r, green: &g, blue: &b, alpha: &a)
    return String(format: "#%02X%02X%02X",
                  Int((r * 255).rounded()),
                  Int((g * 255).rounded()),
                  Int((b * 255).rounded()))
}

let light = UITraitCollection(userInterfaceStyle: .light)
let dark = UITraitCollection(userInterfaceStyle: .dark)
let lightHC = light.modifyingTraits { $0.accessibilityContrast = .high }
let darkHC = dark.modifyingTraits { $0.accessibilityContrast = .high }

/// Palette entry name in tokens.json -> the system colour it transcribes.
let palette: [(String, UIColor)] = [
    ("blue", .systemBlue),
    ("indigo", .systemIndigo),
    ("red", .systemRed),
    ("green", .systemGreen),
    ("orange", .systemOrange),
    ("yellow", .systemYellow),
    ("teal", .systemTeal),
    ("purple", .systemPurple),
    ("gray", .systemGray),
]

/// Surface entry name -> system colour.  Unchanged in iOS 26, but checked
/// every time for the same reason the palette is.
let surfaces: [(String, UIColor)] = [
    ("background", .systemBackground),
    ("card", .systemGray6),
    ("cardSecondary", .systemGray5),
]

print("# iOS \(UIDevice.current.systemVersion) — paste light/dark into design/tokens.json")
print()
print("## palette")
print("name,light,dark,lightIncreasedContrast,darkIncreasedContrast")
for (name, color) in palette {
    print("\(name),\(hex(color, light)),\(hex(color, dark)),\(hex(color, lightHC)),\(hex(color, darkHC))")
}
print()
print("## surfaces")
print("name,light,dark")
for (name, color) in surfaces {
    print("\(name),\(hex(color, light)),\(hex(color, dark))")
}
