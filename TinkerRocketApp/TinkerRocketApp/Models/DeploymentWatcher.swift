//
//  DeploymentWatcher.swift
//  TinkerRocketApp
//
//  Ground-side recovery-event detector (#813).  Swift twin of Android's
//  core/session DeploymentWatcher.kt — same state machine, same constants,
//  same guards.  Kept deliberately parallel: the two announcers are a parity
//  pair, and a detector that drifted between them would have the app tell two
//  flyers different things about the same flight.
//

import Foundation

/// Which canopy a detected deployment is.
enum DeploymentKind {
    /// First canopy, opened above the configured main altitude.
    case drogue
    /// The main: either the second transition, or a first one at/below the
    /// configured main-deploy altitude (single-deploy, or motor ejection).
    case main
    /// A canopy opened but there is no configured main altitude to judge it
    /// against, so which one it is cannot be known.  Spoken as "chute".
    case chute
}

/// The flyer's own numbers for what a good descent looks like, from the active
/// `RocketProfile`.  Absent when no profile is selected — the detector still
/// reports deployments, it just cannot grade them.
struct RecoveryProfile {
    /// Expected descent rate under drogue, m/s.
    let drogueMps: Double
    /// Expected descent rate under main, m/s.
    let mainMps: Double
    /// AGL altitude the main is configured to fire at, metres.
    let mainDeployAglM: Double

    /// The profile stores recovery figures in feet and fps; this is SI.
    private static let metresPerFoot = 0.3048

    init(drogueMps: Double, mainMps: Double, mainDeployAglM: Double) {
        self.drogueMps = drogueMps
        self.mainMps = mainMps
        self.mainDeployAglM = mainDeployAglM
    }

    init(_ p: RocketProfile) {
        self.init(drogueMps: p.drogueRateFps * Self.metresPerFoot,
                  mainMps: p.mainRateFps * Self.metresPerFoot,
                  mainDeployAglM: p.mainDeployAltAglFt * Self.metresPerFoot)
    }
}

/// What `DeploymentWatcher.step` reports, at most one per frame.
enum DeploymentEvent {
    /// A canopy just came out.  Announced immediately.
    case deployed(kind: DeploymentKind, altAglM: Double)
    /// The settled descent rate `verdictWindow` after a deployment.
    /// `nominal` is true when the rate matches the profile's expectation.
    case verdict(kind: DeploymentKind, rateMps: Double, nominal: Bool)
}

/// Drogue and main deployment, detected from the descent-rate curve alone.
///
/// **Why the ground and not the flight computer.**  The FC's own detector
/// (`DeploymentDetector.h`) is far better informed — a 3 ms ejection shock on
/// the accelerometer, a raw baro step at 500 Hz — and none of it reaches the
/// operator.  Its verdict rides `NSF2_DEPLOYED`, which lives in the flight log
/// and nowhere else: the LoRa downlink has no spare bit (`flags_state` and
/// `flags2` are both full, and the 65-byte frame sits at a regulatory
/// ceiling), and the base station zero-fills the per-channel pyro-fired bits
/// when it builds the BLE JSON.  On the link a flyer actually uses at a mile
/// downrange the app receives `altitude_rate` at 2 Hz and nothing else about
/// recovery.  A detector built on that one signal is the only one that works
/// on both links, which is why it is primary rather than a fallback.
///
/// **What it keys on.**  A canopy is a step change in descent rate that holds.
/// The detector tracks the fastest sustained descent in the current phase and
/// waits for the rate to fall below `transitionRatio` of it and *stay* there
/// for `transitionHold`.  The ratio is the same 0.6 the post-flight report
/// uses for a drogue→main transition, so the live callout and the report
/// cannot grade the same flight differently.  The hold is a duration, not a
/// frame count: BLE runs about ten times faster than the 2 Hz downlink, and a
/// frame count would quietly mean two different things on the two links.
///
/// **Deliberately conservative.**  A false "main out" is worse than silence —
/// it tells the flyer to stop watching the sky.  Three cheap guards: nothing
/// is called below `minAglM` (touchdown collapses the rate exactly like a
/// canopy, and nothing real deploys that low); a phase must have reached
/// `minReferenceMps` before any drop counts; and at most two transitions per
/// flight, since a third could only be noise or the ground.
///
/// `nonisolated` because this is pure state-machine logic stepped from
/// telemetry dispatch — its Android twin carries the same fleet-thread-only
/// contract, and main-actor isolation would be wrong on both counts. It also
/// has to be nonisolated to be *destroyable*: a main-actor class gets an
/// isolated deinit, and with the app's iOS 16 deployment target that executor
/// hop comes from the back-deployed shim, which double-frees on iOS 26.2 and
/// aborts the process. Same trap `OfflineTileCache` documents — production
/// never sees it because the announcer outlives the app, so the first code to
/// deallocate one is a test.
nonisolated final class DeploymentWatcher {

    // MARK: - Tunables (mirrored in DeploymentWatcher.kt)

    /// Rate must fall to this fraction of the phase's fastest sustained
    /// descent.  Same constant as `MAIN_TRANSITION_RATIO` in the post-flight
    /// report's deployment module.
    static let transitionRatio = 0.6
    /// How long the slower rate must hold.
    static let transitionHold: TimeInterval = 1.5
    /// A phase must reach this descent rate before a drop can count.
    static let minReferenceMps = 5.0
    /// No deployment is called below this AGL — see the type comment.
    static let minAglM = 15.0
    /// Settling time before the descent rate is worth grading.
    static let verdictWindow: TimeInterval = 4.0
    /// How far past the profile's expectation still counts as nominal.
    static let nominalTolerance = 1.5
    /// Descent under a main faster than this is a partial or failed recovery —
    /// the same threshold the post-flight report warns at.
    static let hardDescentMps = 10.0

    // MARK: - State

    private enum Phase { case ballistic, first, second }

    private var phase: Phase = .ballistic
    /// Fastest sustained descent seen in the current phase, m/s (positive).
    private var reference = 0.0
    /// When the rate first dropped below the ratio, or nil if it has not.
    private var slowSince: Date?

    // Verdict window: open from a deployment until `verdictWindow` later.
    private var verdictKind: DeploymentKind?
    private var verdictDue = Date.distantFuture
    private var verdictSum = 0.0
    private var verdictCount = 0
    /// Expected rate for the canopy currently being graded, m/s.
    private var verdictExpected = 0.0

    func reset() {
        phase = .ballistic
        reference = 0
        slowSince = nil
        verdictKind = nil
        verdictDue = .distantFuture
        verdictSum = 0
        verdictCount = 0
        verdictExpected = 0
    }

    /// Advance by one telemetry frame.
    ///
    /// - Parameters:
    ///   - now: same clock the announcer's cadence gates use.
    ///   - altitudeRateMps: signed vertical rate as the wire carries it —
    ///     negative is descending.  Nil frames are skipped without disturbing
    ///     any accumulated state.
    ///   - altAglM: barometric altitude above the pad, metres.
    ///   - afterApogee: past apogee.  Nothing is detected before.
    ///   - landed: touchdown declared — stops the detector dead.
    ///   - profile: expected rates, or nil when no profile is active.  Absent
    ///     costs the grade, never the deployment call.
    func step(now: Date,
              altitudeRateMps: Float?,
              altAglM: Float?,
              afterApogee: Bool,
              landed: Bool,
              profile: RecoveryProfile?) -> DeploymentEvent? {
        guard !landed, afterApogee,
              let rateSigned = altitudeRateMps,
              let alt = altAglM else { return nil }

        // Positive = descending, which is how the rest of this reads.
        let rate = -Double(rateSigned)
        let altM = Double(alt)

        // A verdict is due first: it is timed from the deployment that opened
        // it, and closing it must not be delayed by the transition logic.
        collectVerdictSample(rate)
        if let v = closingVerdict(now: now) { return v }

        // Below the floor nothing is a deployment — see the type comment.
        guard altM >= Self.minAglM else { return nil }
        // Two transitions is the whole vocabulary; a third could only be noise.
        guard phase != .second else { return nil }

        if rate > reference { reference = rate }

        // Until the vehicle has genuinely been falling there is no baseline to
        // measure a drop against, and near-zero wobble would trip the ratio.
        guard reference >= Self.minReferenceMps else {
            slowSince = nil
            return nil
        }

        guard rate <= reference * Self.transitionRatio else {
            // Back up to speed — whatever slowed it was not a canopy.
            slowSince = nil
            return nil
        }

        guard let since = slowSince else {
            slowSince = now
            return nil
        }
        guard now.timeIntervalSince(since) >= Self.transitionHold else { return nil }

        // Held below the ratio for the full window: a canopy is out.
        let kind = classify(altM: altM, profile: profile)
        phase = (phase == .ballistic) ? .first : .second
        // A first transition already at/below the main altitude IS the main,
        // so there is no second one to wait for.
        if kind == .main { phase = .second }
        reference = rate
        slowSince = nil
        openVerdict(kind: kind, now: now, profile: profile)
        return .deployed(kind: kind, altAglM: altM)
    }

    // MARK: - Classification

    /// First transition above the configured main altitude is the drogue; at
    /// or below it, the main has just fired (or the vehicle is single-deploy
    /// and this is its only canopy).  With no profile there is nothing to
    /// compare against and the honest answer is "a chute".
    private func classify(altM: Double, profile: RecoveryProfile?) -> DeploymentKind {
        if phase == .first { return .main }
        guard let main = profile?.mainDeployAglM else { return .chute }
        return altM > main ? .drogue : .main
    }

    // MARK: - Verdict

    private func openVerdict(kind: DeploymentKind, now: Date, profile: RecoveryProfile?) {
        // No profile, no grade — the deployment is still announced.
        guard let profile else {
            verdictKind = nil
            return
        }
        verdictKind = kind
        verdictDue = now.addingTimeInterval(Self.verdictWindow)
        verdictSum = 0
        verdictCount = 0
        verdictExpected = (kind == .drogue) ? profile.drogueMps : profile.mainMps
    }

    private func collectVerdictSample(_ rate: Double) {
        guard verdictKind != nil else { return }
        verdictSum += rate
        verdictCount += 1
    }

    private func closingVerdict(now: Date) -> DeploymentEvent? {
        guard let kind = verdictKind, now >= verdictDue else { return nil }
        verdictKind = nil
        guard verdictCount > 0 else { return nil }
        let mean = verdictSum / Double(verdictCount)
        return .verdict(kind: kind, rateMps: mean, nominal: isNominal(kind: kind, meanMps: mean))
    }

    private func isNominal(kind: DeploymentKind, meanMps: Double) -> Bool {
        guard verdictExpected > 0 else { return false }
        guard meanMps <= verdictExpected * Self.nominalTolerance else { return false }
        // The report's hard limit is about the FINAL descent — a drogue
        // legitimately falls faster than this, so it only binds under main.
        if kind != .drogue && meanMps > Self.hardDescentMps { return false }
        return true
    }
}
