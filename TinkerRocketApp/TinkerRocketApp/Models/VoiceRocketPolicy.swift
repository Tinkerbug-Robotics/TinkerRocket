//
//  VoiceRocketPolicy.swift
//  TinkerRocketApp
//
//  Which relayed rocket a base station's voice callouts follow.
//

import Foundation

/// Which relayed rocket the callouts follow on a base-station link: the
/// computer the dashboard is showing.
///
/// Voice used to follow the base station's radio focus and nothing else, and
/// on 2026-11-21 that left Rolly Polly V's flight silent. The 65 mm rocket
/// (rocket 8) had flown through the same base station earlier in the day. A
/// rocket that has flown keeps the focus pin when it goes quiet (so the
/// recovery walk keeps its screen), so the pin was still on rocket 8 when
/// Rolly Polly V's two computers (rockets 1 and 7) flew. Both were on screen,
/// each with a Follow button, while voice waited for a rocket that had already
/// been recovered. The radio pin has good reasons to stay where it is. Voice
/// should follow what is on screen.
///
/// The rule, in order:
/// 1. The followed rocket (the radio focus) keeps voice while it is on the
///    dashboard (heard on this link and not hidden). The exception is a
///    followed rocket gone quiet (landed, or unheard for `quietAfter`) while
///    another rocket on the dashboard is flying: then that flight speaks.
/// 2. A pin applied moments ago (re-seeded on reconnect, or a Follow) holds
///    voice for its grace window before its rocket is heard again, unless
///    the user hid it.
/// 3. Otherwise the first rocket on the dashboard: one flying, else one being
///    heard, else any. With nothing on the dashboard, the focus.
///
/// Where several rockets qualify, the one voice already follows (`current`)
/// keeps it, so two computers in one rocket going INFLIGHT a frame apart do
/// not hand voice back and forth at launch.
///
/// Android twin: `VoiceRocketPolicy.kt`. The two must not diverge.
enum VoiceRocketPolicy {

    /// One relayed rocket as the policy sees it.
    struct Candidate: Equatable {
        let rocketID: UInt8
        /// The user hid it from the dashboard.
        let hidden: Bool
        /// Seconds since its last live frame.
        let silentFor: TimeInterval
        let state: String
        let landed: Bool
        /// A simulated flight never claims voice from another rocket.
        let simulated: Bool
    }

    /// A rocket unheard this long has gone quiet: the focus pin's own
    /// staleness bound.
    static let quietAfter: TimeInterval = BLEDevice.focusPinStaleAfter

    static func isQuiet(_ r: Candidate) -> Bool {
        r.landed || r.state == "LANDED" || r.silentFor >= quietAfter
    }

    static func isFlying(_ r: Candidate) -> Bool {
        !r.simulated && r.state == "INFLIGHT" && !isQuiet(r)
    }

    /// - Parameters:
    ///   - focus: the base station's radio focus (the followed rocket).
    ///   - focusHidden: the user hid the followed rocket from the dashboard.
    ///   - focusPinFresh: the pin was applied within its grace window.
    ///   - current: the rocket voice follows now, if any.
    ///   - rockets: every rocket heard on this link.
    static func choose(focus: UInt8?,
                       focusHidden: Bool,
                       focusPinFresh: Bool,
                       current: UInt8?,
                       rockets: [Candidate]) -> UInt8? {
        let shown = rockets.filter { !$0.hidden }.sorted { $0.rocketID < $1.rocketID }
        let flying = shown.filter(isFlying)

        func pick(_ list: [Candidate]) -> UInt8? {
            if let current, list.contains(where: { $0.rocketID == current }) { return current }
            return list.first?.rocketID
        }

        if let focus, let followed = shown.first(where: { $0.rocketID == focus }) {
            if isQuiet(followed), let flight = pick(flying) { return flight }
            return focus
        }
        if let focus, focusPinFresh, !focusHidden {
            return focus
        }
        return pick(flying)
            ?? pick(shown.filter { $0.silentFor < quietAfter })
            ?? pick(shown)
            ?? focus
    }
}
