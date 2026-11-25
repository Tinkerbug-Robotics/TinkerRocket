//
//  TouchdownPredictor.swift
//  TinkerRocketApp
//
//  When the rocket lands, the base station usually loses it first.  Swift twin
//  of Android's core/session TouchdownPredictor.kt — same model, same
//  constants; the two announcers are a parity pair.
//

import Foundation

/// Predicts touchdown from the last live descent frames, so the announcer can
/// make the landing call when the LoRa link dies before the landed frame gets
/// through.
///
/// **Why.**  The flight computer sets its landed flag within about half a
/// second of touchdown, but at touchdown the antenna reaches the ground and
/// the link usually dies with it.  On four of the five 2026-10-24 BARC
/// flights the last live frame came at most 4 m above pad level, still
/// descending, and the landed frame reached the base station 2–4 minutes
/// later, when someone walked into range.  No landing call was spoken until
/// then.
///
/// **The model.**  Remaining height over the mean descent rate of the last
/// `rateWindow` of live frames.  When the active profile's pyro channels say
/// the rocket fires a main on descent, and it is still falling faster than
/// `mainOpenRatio` times the profile's main rate, the main is taken as still
/// to come: the observed rate down to the main altitude, the profile's main
/// rate below it.  The main altitude comes from the pyro channels, not from
/// the profile's recovery fields: those always hold a value (700 ft by
/// default), so they cannot tell a motor-ejection rocket from a dual-deploy
/// one, while an enabled "altitude on descent" channel is what the flight
/// computer will actually fire.
///
/// **Scored on the flight logs** (base-station LoRa logs against the flight
/// computers' own touchdown, 22 flights from 2026-03 to 2026-10;
/// `Data_Analysis/touchdown_call_eval.py`).  Losing the link at touchdown,
/// the call came 0.4 s early to 3.0 s late on the six flights that really
/// lost it.  Cutting the link at 100 m or lower gives a median error near
/// 1 s.  Above the main on the one dual-deploy flight (Rolly Polly V, main at
/// 800 ft), the plain rate was 14–23 s early; the main leg brought that to
/// 1–7 s.
nonisolated final class TouchdownPredictor {

    // MARK: - Tunables (mirrored in TouchdownPredictor.kt)

    /// Live frames whose rate is averaged: six at the 2 Hz relay.  The
    /// downlink rate is a rounded integer and jitters by a few m/s.
    static let rateWindow: TimeInterval = 3.0
    /// The landing call waits until the link has been silent this long.  Gaps
    /// longer than 1.5 s were under 1 % of the descent gaps in the logs, all
    /// but one of them over 130 m up, where the predicted touchdown is still
    /// far off, so a gap there does not bring the call forward.
    static let silenceBeforeCall: TimeInterval = 1.5
    /// Below this the rocket is not yet really falling (just past apogee) and
    /// height over rate means nothing; the previous prediction stands.
    static let minDescentRateMps = 1.0
    /// Falling faster than this multiple of the profile's main rate means the
    /// main is not out yet.
    static let mainOpenRatio = 2.0

    // MARK: - State

    private var samples: [(at: Date, rateMps: Double)] = []
    /// The last live descent frame.
    private(set) var lastLiveAt: Date?
    /// When the rocket is expected on the ground, or nil before any
    /// prediction could be made.
    private(set) var predictedTouchdownAt: Date?

    func reset() {
        samples = []
        lastLiveAt = nil
        predictedTouchdownAt = nil
    }

    /// One live descent frame.
    ///
    /// - Parameters:
    ///   - altAglM: barometric altitude above the pad, metres.
    ///   - altitudeRateMps: signed, negative descending — as the wire carries it.
    ///   - profile: the active rocket's recovery figures, or nil.
    func observe(now: Date, altAglM: Float?, altitudeRateMps: Float?,
                 profile: RecoveryProfile?) {
        lastLiveAt = now
        if let rate = altitudeRateMps {
            samples.append((now, -Double(rate)))
        }
        samples.removeAll { now.timeIntervalSince($0.at) > Self.rateWindow }
        guard let alt = altAglM, !samples.isEmpty else { return }
        let rate = samples.map(\.rateMps).reduce(0, +) / Double(samples.count)
        guard rate >= Self.minDescentRateMps else { return }
        predictedTouchdownAt = now.addingTimeInterval(
            Self.secondsToTouchdown(altAglM: Double(alt), descentRateMps: rate,
                                    profile: profile))
    }

    /// When the landing call is due: the predicted touchdown, but never before
    /// the link has been silent for `silenceBeforeCall`.  Nil until a
    /// prediction exists.
    var callDueAt: Date? {
        guard let touchdown = predictedTouchdownAt, let last = lastLiveAt else { return nil }
        return max(touchdown, last.addingTimeInterval(Self.silenceBeforeCall))
    }

    /// Seconds from a frame at `altAglM`, falling at `descentRateMps`
    /// (positive), to the ground.  At or below the pad level it is now: the
    /// landing site can sit lower than the pad, and the rocket is on its last
    /// metres either way.
    static func secondsToTouchdown(altAglM h: Double, descentRateMps r: Double,
                                   profile: RecoveryProfile?) -> TimeInterval {
        guard h > 0, r > 0 else { return 0 }
        if let profile, let mainAlt = profile.pyroMainAglM, profile.mainMps > 0,
           r > mainOpenRatio * profile.mainMps {
            return max(h - mainAlt, 0) / r + min(h, mainAlt) / profile.mainMps
        }
        return h / r
    }
}
