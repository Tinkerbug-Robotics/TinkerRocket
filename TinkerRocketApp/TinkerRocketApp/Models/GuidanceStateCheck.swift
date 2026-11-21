//
//  GuidanceStateCheck.swift
//  TinkerRocketApp
//
//  Does the flight computer hold the guidance setting the profile shows?
//
//  GTV Blue, 2026-11-01: the app showed station-keeping ON and the flight
//  computer flew with it OFF.  Every screen showed the profile, the one value
//  the out computer reported ("ge") was its own copy, and sending a guidance
//  config wrote "on" into the app's view of the rocket whether or not the
//  flight computer ever applied it.  The flight computer's own on/off now
//  rides config_guid ("gen"), and this compares the profile against that and
//  nothing else.  Pure, so the cases are unit-tested.
//

import Foundation

enum GuidanceStateCheck: Equatable {
    /// The flight computer holds what the profile shows.
    case agrees
    /// Profile says guidance; the flight computer has it off.
    case rocketOff
    /// The flight computer has guidance on; the profile says roll control.
    case rocketOn
    /// Both on, flying different laws (0 = PN, 1 = station-keep).
    case lawDiffers(rocket: UInt8, profile: UInt8)
    /// Profile says guidance and nothing from the flight computer confirms it.
    case unverified(String)
    /// Profile says roll control and the flight computer is not reporting:
    /// nothing to warn about.
    case notApplicable

    static func evaluate(profile: RocketProfile?, config: RocketConfig?,
                         isConnected: Bool, isBaseStation: Bool) -> GuidanceStateCheck {
        guard let profile else { return .notApplicable }
        let wantOn = profile.guidanceEnabled
        func unknown(_ why: String) -> GuidanceStateCheck {
            wantOn ? .unverified(why) : .notApplicable
        }
        guard isConnected else { return unknown("not connected") }
        // Settings never ride LoRa: a base-station link cannot see them.
        guard !isBaseStation else { return unknown("connect to the rocket directly to check") }
        guard let config else { return unknown("rocket settings not read yet") }
        guard let extras = config.guidanceExtras else {
            return unknown("flight computer not reporting (power it on)")
        }
        guard let rocketOn = extras.enabled else {
            return unknown("this rocket's firmware doesn't report it")
        }
        switch (wantOn, rocketOn) {
        case (true, false):  return .rocketOff
        case (false, true):  return .rocketOn
        case (false, false): return .agrees
        case (true, true):
            return extras.guidanceLaw == profile.pnGuidanceLaw
                ? .agrees
                : .lawDiffers(rocket: extras.guidanceLaw, profile: profile.pnGuidanceLaw)
        }
    }

    /// The rocket will not fly what the profile shows.
    var isMismatch: Bool {
        switch self {
        case .rocketOff, .rocketOn, .lawDiffers: return true
        default: return false
        }
    }

    /// One line for the header, Settings and the preflight step; nil when
    /// there is nothing to say.
    var message: String? {
        switch self {
        case .agrees, .notApplicable:
            return nil
        case .rocketOff:
            return "Rocket guidance OFF \u{2014} profile says ON"
        case .rocketOn:
            return "Rocket guidance ON \u{2014} profile says OFF"
        case .lawDiffers(let rocket, let profile):
            return "Rocket flies \(Self.lawName(rocket)) \u{2014} profile says \(Self.lawName(profile))"
        case .unverified(let why):
            return "Rocket guidance unverified \u{2014} \(why)"
        }
    }

    static func lawName(_ law: UInt8) -> String {
        switch law {
        case 0: return "PN"
        case 1: return "station-keep"
        default: return "law \(law)"
        }
    }
}
