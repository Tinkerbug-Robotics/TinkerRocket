//
//  DeviceTypeIcon.swift
//  TinkerRocketApp
//
//  Shared vocabulary for device-list UI: the rocket / base-station glyph
//  every device row renders, and the network-ID copy that Settings and
//  My Devices must state identically.  Extracted from four near-identical
//  view-local copies so the screens can't drift apart.
//

import SwiftUI

/// Rocket / base-station / unknown glyph for a device row.
///
/// Sites differ on three axes, so each is a parameter with the common-case
/// default: `size` bounds the rocket asset (square, aspect-fit), `symbolFont`
/// sizes the SF-symbol types (semantic fonts so Dynamic Type keeps working),
/// and `tint` picks fixed standard colors (blue rocket / orange antenna /
/// secondary unknown), the environment color (DeviceChipBar chips flip
/// white/primary with selection), or an explicit color.
struct DeviceTypeIcon: View {
    enum Tint {
        case standard
        case inherit
        case custom(Color)
    }

    let type: BLEDeviceType
    var size: CGFloat = 24
    var symbolFont: Font = .title3
    var tint: Tint = .standard

    var body: some View {
        switch type {
        case .rocket:
            tinted(standard: .blue) {
                Image("RocketIcon")
                    .resizable()
                    .renderingMode(.template)
                    .aspectRatio(contentMode: .fit)
                    .frame(width: size, height: size)
            }
        case .baseStation:
            tinted(standard: .orange) {
                Image(systemName: "antenna.radiowaves.left.and.right")
                    .font(symbolFont)
            }
        case .unknown:
            tinted(standard: .secondary) {
                Image(systemName: "questionmark.circle")
                    .font(symbolFont)
            }
        }
    }

    @ViewBuilder
    private func tinted<V: View>(standard: Color, _ content: () -> V) -> some View {
        switch tint {
        case .standard: content().foregroundColor(standard)
        case .inherit: content()
        case .custom(let color): content().foregroundColor(color)
        }
    }
}

/// Network-ID copy shared between SettingsView and the My Devices screens —
/// the same condition must be described in the same words everywhere (#150:
/// mismatched wording is how the nid drift stayed undebuggable).
enum NetworkCopy {
    /// The baseline explainer under every network section.
    static let sameNetworkExplainer =
        "Devices only hear each other on the same network ID."

    /// A specific device is on the wrong network ID.  Sites append their own
    /// call to action ("Tap to sync it now.", queueing details, …).
    static let deviceMismatchWarning =
        "This device is on a different network ID than the app expects — it can't hear your other devices."
}
