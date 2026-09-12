//
//  BluetoothAvailability.swift
//  TinkerRocketApp
//
//  #1413: why Bluetooth is unavailable, in a form a view can act on.
//
//  The states were always distinguished — centralManagerDidUpdateState set a
//  different string for each — but a string is not a next step. A first-run
//  user who tapped "Don't Allow" on the Bluetooth prompt got a grey dot,
//  "Bluetooth not authorized", and a Scan button that did nothing, with no
//  hint that the fix lives in Settings › TinkerRocket › Bluetooth.
//
//  The decisions live here rather than in the view so they can be tested
//  without a device: the simulator has no Bluetooth radio, so `.denied` and
//  `.off` cannot be reached there at all.
//

import Foundation
import CoreBluetooth

/// What the Bluetooth stack can do for us right now.
enum BluetoothAvailability: Equatable {
    /// Before the first centralManagerDidUpdateState callback. Says nothing —
    /// the radio may well be fine, and guessing produces a flash of advice
    /// that disappears a moment later.
    case unknown
    case ready
    /// Adapter off. Anyone can fix it; nothing to authorise.
    case off
    /// The user denied this app Bluetooth access (or Screen Time / MDM did).
    case denied
    /// No BLE hardware. In practice: the simulator.
    case unsupported

    init(_ state: CBManagerState) {
        switch state {
        case .poweredOn:    self = .ready
        case .poweredOff:   self = .off
        case .unauthorized: self = .denied
        case .unsupported:  self = .unsupported
        default:            self = .unknown      // .resetting, .unknown
        }
    }

    /// The headline already shown beside the status dot.
    var headline: String {
        switch self {
        case .ready:       return "Bluetooth ready"
        case .off:         return "Bluetooth is off"
        case .denied:      return "Bluetooth not authorized"
        case .unsupported: return "Bluetooth not supported"
        case .unknown:     return "Bluetooth unknown state"
        }
    }

    /// One quiet line under the headline, or nothing. Never a banner and never
    /// a recoloured status dot — an advisory that shouts is worse than the bare
    /// string it replaced.
    ///
    /// `.unsupported` deliberately has none: it only happens in the simulator,
    /// where there is nothing the reader can do and nothing has gone wrong.
    var advice: String? {
        switch self {
        case .ready, .unsupported, .unknown:
            return nil
        case .off:
            // iOS has no supported deep link to the Bluetooth pane, so this
            // has to be an instruction rather than a button.
            return "Turn it on in Control Center, or Settings › Bluetooth."
        case .denied:
            return "TinkerRocket was denied Bluetooth access."
        }
    }

    /// Whether to offer the Settings link. Only the denied case has a
    /// destination: openSettingsURLString lands on the app's own page, which
    /// is exactly where the Bluetooth toggle for this app lives. It would be
    /// the wrong page for a powered-off adapter.
    var offersAppSettings: Bool { self == .denied }

    /// Whether a scan could actually run. The Scan and Add controls gate on
    /// this so they stop starting scans that cannot do anything.
    var canScan: Bool { self == .ready }
}
