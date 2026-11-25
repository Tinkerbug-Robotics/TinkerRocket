//
//  SettingsAccess.swift
//  TinkerRocketApp
//
//  What the Settings sheet may offer for a directly connected rocket, decided
//  in one place so the sheet and its tests agree.  Android twin:
//  `settingsAccess()` in core/protocol SettingsAccess.kt.
//
//  The sheet used to have two separate gates.  "Rocket is off" swapped the
//  config sections out and kept Firmware.  INITIALIZATION laid a full-sheet
//  "Initializing…" overlay over everything, Firmware included, and over the
//  rocket-off view too (a rail-off FC reads INITIALIZATION as often as not).
//  The overlay was the bug reported 2026-10-16: an FC stuck on "Starting
//  sensors" locked out Settings → Firmware → Update firmware…, which is the
//  one way to replace a bad FC image without opening the airframe.
//
//  The rule now is the one the rocket actually imposes.  The settings the
//  flight computer stores and answers need it running.  The out computer's own
//  settings (LoRa telemetry on/off, network ID) and the firmware update need
//  only the Bluetooth link to the out computer, so they stay available whatever
//  the flight computer is doing.  A flight computer update is offered too: the
//  update itself reports whether the flight computer answered.
//

import Foundation

enum SettingsAccess: Equatable {
    /// The flight computer is running its flight software: every setting.
    case full
    /// The flight computer's rail is off.
    case rocketOff
    /// The rail is on, but the flight computer is not running its flight
    /// software: still starting, stuck partway, silent, in rescue mode, or
    /// with no flight loop.
    case flightComputerNotRunning

    /// - Parameters:
    ///   - hasReceivedTelemetry: #377. Before the first frame, telemetry is all
    ///     defaults, and a defaulted pwr_pin_on reads exactly like a rocket that
    ///     is off.
    ///   - bootStepReported: "bs" is present, which the OC forwards only while
    ///     the FC is in setup_fc() (or rescue mode, or has no flight loop).
    static func evaluate(isConnected: Bool,
                         isBaseStation: Bool,
                         hasReceivedTelemetry: Bool,
                         pwrPinOn: Bool,
                         state: String,
                         bootStepReported: Bool) -> SettingsAccess {
        // Offline, profile edits are saved and pushed on a later connect. A base
        // station has its own read-only layout, and its relayed telemetry never
        // carries pwr_pin_on (the LoRa packet has no room), so a gate scoped
        // to it would lock its Settings for good.
        guard isConnected, !isBaseStation else { return .full }
        if hasReceivedTelemetry && !pwrPinOn { return .rocketOff }
        // INITIALIZATION is what the OC's zeroed rocket_state reads until the FC
        // sends its first NonSensorData, i.e. until loop_fc() runs.
        if state == "INITIALIZATION" || bootStepReported { return .flightComputerNotRunning }
        return .full
    }

    /// Settings the flight computer stores and answers: calibrations, IMU
    /// mounting and rate, servo control, PID, roll and guidance, fin layout,
    /// camera, pyro channels and pyro tests.  Edits made while it is not running
    /// would queue in the out computer and land late, out of order or not at
    /// all (#366), so they stay locked.
    var allowsFlightComputerSettings: Bool { self == .full }

    /// The out computer's own settings and the firmware update.  Always on for
    /// a connected rocket: none of them goes through the flight computer.
    var allowsOutComputerSettings: Bool { true }
}
