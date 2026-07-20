//
//  LandingPredictor.swift
//  TinkerRocketApp
//
//  Live in-flight landing-point prediction (issue #156).
//
//  Watches the active rocket's telemetry, classifies flight phase, and runs
//  a forward drift-cast simulation from the current state to estimate where
//  the rocket will land.  Operator-facing goal: if LoRa drops mid-flight,
//  the last published prediction stays pinned on the map so the recovery
//  walk has a target.
//
//  Algorithm mirrors the Python validation tool at
//  `Data_Analysis/landing_predictor.py`.
//  Descent branch (alt_apo flag set, or vu <= 0): drift-cast forward from
//  the current position using the active RocketProfile's drogue/main rates
//  and the cached wind profile.  When a fresh GNSS fix is available, the
//  snapshot position is taken from raw GNSS rather than EKF — bypasses the
//  post-blackout EKF recovery oscillation seen in flight #176 replay.
//  Ascent branch (#191 item 1, post-burnout coast ONLY): integrate
//  ballistic-with-drag on the telemetry EKF velocity to apogee, then the
//  standard descent cast.  Boost is deliberately never predicted — the
//  model has no thrust term, so a motor-burning snapshot is garbage-in;
//  telemetry lost during boost latches the last GNSS fix, as before.
//  Each prediction carries an uncertainty radius, fixed at compute time
//  (#191 item 2, see landingUncertainty / ascentDragSpreadMeters below).
//

import Foundation
import CoreLocation
import Combine

/// Source of the snapshot position — purely informational, for the staleness
/// badge and replay logs.  EKF means the rocket's onboard EKF; GNSS means a
/// fresh telemetry-relayed GPS fix overrode the EKF for descent prediction.
enum LandingSnapshotSource: String {
    case ekf
    case gnss
    case latched   // GNSS lost; predictor is showing a frozen prediction
}

/// The published forecast.  Re-emitted on every telemetry packet while
/// INFLIGHT; latched at the last value when telemetry stops or state
/// reaches LANDED.
struct LandingPrediction: Equatable {
    let landing: CLLocationCoordinate2D
    /// Descent path traced from the snapshot down to landing (for the
    /// dashed-line map overlay).  Each point is (lat, lon, altAglFt, timeS).
    let descentTrack: [TrackPoint]
    /// Where the rocket was when this prediction was computed — used by
    /// the map's "predicted from telemetry T+ago" badge and for the
    /// snapshot-to-landing dashed connector.
    let snapshot: CLLocationCoordinate2D
    let snapshotAltAglFt: Double
    let snapshotSource: LandingSnapshotSource
    let computedAt: Date
    /// `Date()` when the underlying telemetry sample arrived — staleness =
    /// now() - sampleAt.  Frozen at the last live sample once telemetry
    /// stops.
    let sampleAt: Date
    /// Uncertainty radius (m) of `landing`, FIXED at compute time (#191
    /// item 2) — the set of places the rocket could still come down.
    ///
    /// The budget is set by how much modeled descent remains, so it is
    /// widest just after burnout and shrinks as re-predictions run closer
    /// to the ground, bottoming out at `snapshotPositionErrorMeters` (the
    /// fix under the rocket is the last thing left to be unsure about).
    ///
    /// It does NOT grow with staleness once latched: losing packets does
    /// not widen the rocket's options, it only ages our knowledge of them.
    /// The reachable set from the last known state is what it was — the
    /// map's staleness badge, not the radius, carries "how old is this".
    let uncertaintyMeters: Double

    static func == (lhs: LandingPrediction, rhs: LandingPrediction) -> Bool {
        // Equality used only to suppress redundant @Published emissions.
        lhs.landing.latitude == rhs.landing.latitude &&
        lhs.landing.longitude == rhs.landing.longitude &&
        lhs.snapshot.latitude == rhs.snapshot.latitude &&
        lhs.snapshot.longitude == rhs.snapshot.longitude &&
        lhs.snapshotSource == rhs.snapshotSource &&
        lhs.computedAt == rhs.computedAt
    }
}

@MainActor
final class LandingPredictor: ObservableObject {
    @Published private(set) var prediction: LandingPrediction?
    /// Cached wind profile fetched on PRELAUNCH; nil until the first
    /// successful fetch.  Drift-cast falls back to "no wind" when nil.
    @Published private(set) var windProfile: WindProfile?
    @Published private(set) var windFetchError: String?

    private weak var device: BLEDevice?
    private weak var profileStore: RocketProfileStore?
    private var cancellables = Set<AnyCancellable>()

    /// Cached values to suppress redundant work on every BLE notification.
    private var lastWindFetchAt: Date?
    private var lastWindFetchLocation: (lat: Double, lon: Double)?
    private var landed: Bool = false

    /// Freshness window for the GNSS-position substitution during descent.
    /// Matches the Python validation default; tunable per-rocket later.
    private let gnssFreshThresholdS: TimeInterval = 2.0

    /// Refetch the wind profile if older than this and still in PRELAUNCH.
    private let windRefetchAfterS: TimeInterval = 3600.0

    func attach(device: BLEDevice, profileStore: RocketProfileStore) {
        // Drop any prior subscription before re-attaching.
        cancellables.removeAll()
        self.device = device
        self.profileStore = profileStore
        landed = false

        device.$telemetry
            .receive(on: DispatchQueue.main)
            .sink { [weak self] t in
                self?.handleTelemetry(t)
            }
            .store(in: &cancellables)
    }

    func detach() {
        cancellables.removeAll()
        device = nil
        profileStore = nil
        prediction = nil
        windProfile = nil
        windFetchError = nil
        landed = false
        lastWindFetchAt = nil
        lastWindFetchLocation = nil
    }

    // MARK: - Telemetry handling

    private func handleTelemetry(_ t: TelemetryData) {
        // Skip predictions before the rocket has anything resembling a GPS
        // position — the cached lastValidRocketFix is the recovery anchor
        // and shows on the map already, no point synthesizing a prediction
        // from nothing.
        guard let lat = t.latitude, let lon = t.longitude,
              t.num_sats >= 4 else { return }

        let now = Date()

        // Wind prefetch is fire-and-forget; lives off the high-rate path.
        prefetchWindIfNeeded(t: t, lat: lat, lon: lon, now: now)

        // Once LANDED is asserted, freeze the last prediction so the map
        // pin doesn't twitch on noise as the rocket sits on the ground.
        if t.landed_flag {
            if !landed, let last = prediction {
                landed = true
                let frozen = LandingPrediction(
                    landing: last.landing, descentTrack: last.descentTrack,
                    snapshot: last.snapshot, snapshotAltAglFt: last.snapshotAltAglFt,
                    snapshotSource: .latched,
                    computedAt: now, sampleAt: last.sampleAt,
                    uncertaintyMeters: last.uncertaintyMeters
                )
                prediction = frozen
            }
            return
        }

        // Phase: descent if the apogee flag is set OR vertical rate is non-
        // positive; post-burnout coast predicts via the ascent ballistic
        // (#191 item 1).  Boost never predicts — no thrust model.
        let vU = Double(t.altitude_rate ?? 0)
        let descending = t.alt_apo || vU <= 0.5

        guard let profile = profileStore?.activeProfile else { return }

        let snapshot = CLLocationCoordinate2D(latitude: lat, longitude: lon)
        // Altitude AGL: pressure_alt is already MSL/baro per the parser;
        // most operators tare to launch, so treat it as AGL for descent
        // prediction.  Wind profile is also AGL, so units match.
        let altAglFt = mToFt(Double(t.pressure_alt ?? 0))

        let track: [TrackPoint]
        let source: LandingSnapshotSource
        let uncertainty: Double

        if descending {
            track = simulateDescentForLanding(
                startLat: snapshot.latitude,
                startLon: snapshot.longitude,
                currentAltAglFt: altAglFt,
                observedVerticalRateMps: vU,
                profile: profile,
                wind: windProfile
            )
            source = .gnss    // we already required num_sats >= 4
            uncertainty = landingUncertainty(track: track, wind: windProfile)
        } else if t.launch_flag, t.burnout_flag,
                  let ve = t.vel_e, let vn = t.vel_n {
            // Post-burnout coast: position anchors on the GNSS fix (gated
            // fresh above), velocity on the EKF — GNSS velocity is the
            // boost casualty, and pre-#191 firmware sends no ve/vn at all,
            // so this branch simply never runs there.  vu prefers the EKF
            // channel, falling back to the baro-KF rate.
            let vu = Double(t.vel_u.map(Double.init) ?? vU)
            // Apogee straddle (baro says climbing, EKF says not): skip —
            // the next packet classifies as descent.
            guard vu > 0.5 else { return }
            let vel = (e: Double(ve), n: Double(vn), u: vu)
            let k = profile.ballisticDragK

            let cast = simulateAscentThenDescent(
                startLat: snapshot.latitude, startLon: snapshot.longitude,
                currentAltAglFt: altAglFt, velocityENUMps: vel,
                profile: profile, dragK: k, wind: windProfile)
            track = cast.track
            source = .ekf
            // GNSS floor + wind term over the DESCENT segment only (both
            // inside landingUncertainty), plus the drag-model spread
            // (issue spec: k ± 50%) — all fixed at compute time, per the
            // item-2 model.
            uncertainty = landingUncertainty(track: cast.descent, wind: windProfile)
                        + ascentDragSpreadMeters(
                              startLat: snapshot.latitude, startLon: snapshot.longitude,
                              currentAltAglFt: altAglFt, velocityENUMps: vel,
                              profile: profile, dragK: k, wind: windProfile,
                              nominalLanding: cast.track.last)
        } else {
            return
        }

        guard let landing = track.last.map({
            CLLocationCoordinate2D(latitude: $0.lat, longitude: $0.lon)
        }) else { return }

        let newPrediction = LandingPrediction(
            landing: landing, descentTrack: track,
            snapshot: snapshot, snapshotAltAglFt: altAglFt,
            snapshotSource: source,
            computedAt: now, sampleAt: now,
            uncertaintyMeters: uncertainty
        )

        // Suppress no-op republishes (avoids map redraw thrash when nothing
        // material changed).
        if newPrediction != prediction {
            prediction = newPrediction
        }
    }

    // MARK: - Wind prefetch

    private func prefetchWindIfNeeded(t: TelemetryData,
                                      lat: Double, lon: Double,
                                      now: Date) {
        // Only fetch on the pad; never during INFLIGHT — keeps the live
        // path free of network calls.
        let state = t.state
        let preflight = (state == "READY" || state == "PRELAUNCH" ||
                         state == "UNKNOWN")
        guard preflight else { return }

        // Already have a recent fetch at roughly this location?  Skip.
        if let last = lastWindFetchAt,
           let where_ = lastWindFetchLocation,
           now.timeIntervalSince(last) < windRefetchAfterS,
           abs(where_.lat - lat) < 0.01 && abs(where_.lon - lon) < 0.01,
           windProfile != nil {
            return
        }

        lastWindFetchAt = now
        lastWindFetchLocation = (lat, lon)

        Task { [weak self] in
            do {
                let wp = try await fetchWinds(lat: lat, lon: lon,
                                              timeUTC: Date())
                await MainActor.run {
                    self?.windProfile = wp
                    self?.windFetchError = nil
                }
            } catch {
                await MainActor.run {
                    self?.windFetchError = error.localizedDescription
                }
            }
        }
    }
}

// MARK: - DriftCast adapter

/// Wrapper around the global `simulateDescent` for the live predictor:
///   - Drops the rocket from `currentAltAglFt` (not apogee).
///   - When the observed fall rate is non-trivial, uses it in place of
///     the profile rate for the matching layer (drogue if above main
///     deploy alt, main if below).
///
/// Mirrors `landing_predictor.predict_landing`'s descent branch from the
/// Python validation tool.
func simulateDescentForLanding(
    startLat: Double, startLon: Double,
    currentAltAglFt: Double,
    observedVerticalRateMps: Double,
    profile: RocketProfile,
    wind: WindProfile?
) -> [TrackPoint] {
    // Empty wind = "no drift" placeholder.
    let windToUse = wind ?? WindProfile(
        layers: [WindLayer(altFt: 0, speedKts: 0, directionDeg: 0)],
        groundElevFt: 0, fetchTime: "", location: (startLat, startLon)
    )

    var drogueRate = profile.drogueRateFps
    var mainRate = profile.mainRateFps
    let mainAlt = profile.mainDeployAltAglFt

    // Observed-rate override (issue #156 spec).
    let fallRateFps = -observedVerticalRateMps * 3.28084
    if fallRateFps > 0.5 {
        if currentAltAglFt > mainAlt {
            drogueRate = fallRateFps
        } else {
            mainRate = fallRateFps
        }
    }

    return simulateDescent(
        startLat: startLat, startLon: startLon,
        apogeeAglFt: max(currentAltAglFt, 1.0),
        drogueRateFps: drogueRate,
        mainRateFps: mainRate,
        mainDeployAglFt: mainAlt,
        windProfile: windToUse,
        direction: "forward"
    )
}

// MARK: - Uncertainty model (#191 item 2)

/// Fraction of the total applied wind drift charged as uncertainty: the
/// winds-aloft model is the dominant descent error term, and ~20% of
/// mean wind × descent time is the issue-#191 spec.
private let windUncertaintyFraction = 0.2

/// Wind speed (m/s) assumed for the error bound when no wind profile was
/// ever fetched: the cast applied ZERO drift, so the full (unknown) drift
/// is error — bound it at light-breeze scale rather than claiming 0.
private let assumedWindWhenUnknownMps = 2.0

/// Horizontal position error of the snapshot every cast starts from — the
/// FLOOR the radius converges to, never subtracted away by a short descent.
///
/// The circle answers "where could the rocket land"; as the remaining
/// descent (and therefore its wind drift) goes to zero, that set collapses
/// onto the rocket's own reported position — which is itself only known to
/// GNSS accuracy.  Without this term a prediction computed just above the
/// ground draws a ~0 m circle, claiming meter-perfect knowledge the fix
/// does not have.
///
/// Constant rather than live: horizontal accuracy is not on the telemetry
/// wire (the frame carries `num_sats` but no hAcc/pDOP), so this is the
/// typical 3D-fix figure for this GNSS rather than the per-packet value.
/// If hAcc ever joins the frame, read it here instead of this constant.
let snapshotPositionErrorMeters = 3.0

/// Uncertainty radius for a prediction — fixed at compute time.
///
/// Two terms, summed:
///   1. `snapshotPositionErrorMeters` — the GNSS floor, always present.
///   2. Wind drift over the REMAINING descent.  With a wind profile:
///      `windUncertaintyFraction` of the total drift the cast applied (mean
///      wind speed sampled at the track's altitudes × descent duration).
///      Without one: the whole drift is unmodeled, so charge
///      `assumedWindWhenUnknownMps` across the descent instead.
///
/// Term 2 scales with remaining descent time, so the radius shrinks as
/// predictions re-run closer to the ground and converges on term 1 at
/// touchdown.  The ascent branch adds `ascentDragSpreadMeters` on top,
/// which is why the circle is widest just after burnout.
func landingUncertainty(track: [TrackPoint], wind: WindProfile?) -> Double {
    guard track.count >= 2,
          let first = track.first, let last = track.last else {
        return snapshotPositionErrorMeters
    }
    let descentS = max(0, last.timeS - first.timeS)
    guard let wind = wind else {
        return snapshotPositionErrorMeters + assumedWindWhenUnknownMps * descentS
    }
    let speeds = track.map { ktsToMps(wind.interpolate(altAglFt: $0.altAglFt).speedKts) }
    let meanMps = speeds.reduce(0, +) / Double(speeds.count)
    return snapshotPositionErrorMeters + windUncertaintyFraction * meanMps * descentS
}

// MARK: - Ascent ballistic (#191 item 1)

/// Integration step / cap for the coast-to-apogee propagation.  Mirrors
/// `landing_predictor.predict_landing`'s ascent branch (dt 0.05 s, 120 s
/// cap ≈ far beyond any coast this airframe class can fly).
private let ascentDtS = 0.05
private let ascentMaxCoastS = 120.0
private let gMps2 = 9.80665

/// Integrate the post-burnout coast from a position + EKF-velocity snapshot
/// to apogee under gravity + quadratic drag (a = −g·ẑ − k·|v|·v).
/// Returns the ascent track, snapshot → apogee (last point has vU ≤ 0);
/// `dragK` is RocketProfile.ballisticDragK (1/m), 0 = gravity-only.
/// Semi-implicit Euler like the Python reference: velocity first, then
/// position with the updated velocity.
func simulateAscentToApogee(
    startLat: Double, startLon: Double,
    currentAltAglFt: Double,
    velocityENUMps: (e: Double, n: Double, u: Double),
    dragK: Double
) -> [TrackPoint] {
    var lat = startLat, lon = startLon
    var uM = ftToM(currentAltAglFt)
    var (ve, vn, vu) = velocityENUMps
    var t = 0.0
    var track = [TrackPoint(lat: lat, lon: lon,
                            altAglFt: currentAltAglFt, timeS: 0)]
    let maxSteps = Int(ascentMaxCoastS / ascentDtS)
    for _ in 0..<maxSteps {
        let vMag = (ve * ve + vn * vn + vu * vu).squareRoot()
        ve += -dragK * vMag * ve * ascentDtS
        vn += -dragK * vMag * vn * ascentDtS
        vu += (-gMps2 - dragK * vMag * vu) * ascentDtS

        let dE = ve * ascentDtS, dN = vn * ascentDtS
        uM += vu * ascentDtS
        t += ascentDtS
        let dist = (dE * dE + dN * dN).squareRoot()
        if dist > 0 {
            let bearing = atan2(dE, dN) * 180.0 / .pi
            let p = forwardProject(lat: lat, lon: lon,
                                   bearingDeg: bearing, distanceM: dist)
            lat = p.lat; lon = p.lon
        }
        track.append(TrackPoint(lat: lat, lon: lon,
                                altAglFt: mToFt(uM), timeS: t))
        if vu <= 0 { break }
    }
    return track
}

/// Full ascent prediction: coast to apogee, then the standard descent cast
/// from the apogee point (profile rates — there is no observed fall rate at
/// a future apogee).  Returns the stitched track (descent timeS offset so
/// the combined track is time-continuous) plus the raw descent segment,
/// which the uncertainty model needs separately (its wind term charges
/// descent duration only).
func simulateAscentThenDescent(
    startLat: Double, startLon: Double,
    currentAltAglFt: Double,
    velocityENUMps: (e: Double, n: Double, u: Double),
    profile: RocketProfile,
    dragK: Double,
    wind: WindProfile?
) -> (track: [TrackPoint], descent: [TrackPoint]) {
    let ascent = simulateAscentToApogee(
        startLat: startLat, startLon: startLon,
        currentAltAglFt: currentAltAglFt,
        velocityENUMps: velocityENUMps, dragK: dragK)
    guard let apogee = ascent.last else { return ([], []) }

    let descent = simulateDescentForLanding(
        startLat: apogee.lat, startLon: apogee.lon,
        currentAltAglFt: apogee.altAglFt,
        observedVerticalRateMps: 0,
        profile: profile, wind: wind)

    let t0 = apogee.timeS
    let stitched = ascent + descent.dropFirst().map {
        TrackPoint(lat: $0.lat, lon: $0.lon,
                   altAglFt: $0.altAglFt, timeS: $0.timeS + t0)
    }
    return (stitched, descent)
}

/// Drag-model term of the ascent uncertainty (#191 item 2 spec: drag
/// coefficient ± 50%): re-run the full cast at 0.5k and 1.5k and take the
/// largest displacement from the nominal landing.  k = 0 (gravity-only)
/// degenerates to 0 spread, correctly — there is no drag model to be
/// wrong about.
func ascentDragSpreadMeters(
    startLat: Double, startLon: Double,
    currentAltAglFt: Double,
    velocityENUMps: (e: Double, n: Double, u: Double),
    profile: RocketProfile,
    dragK: Double,
    wind: WindProfile?,
    nominalLanding: TrackPoint?
) -> Double {
    guard let nominal = nominalLanding else { return 0 }
    let nominalLoc = CLLocation(latitude: nominal.lat, longitude: nominal.lon)
    var worst = 0.0
    for kVariant in [dragK * 0.5, dragK * 1.5] {
        let cast = simulateAscentThenDescent(
            startLat: startLat, startLon: startLon,
            currentAltAglFt: currentAltAglFt,
            velocityENUMps: velocityENUMps,
            profile: profile, dragK: kVariant, wind: wind)
        guard let l = cast.track.last else { continue }
        let d = CLLocation(latitude: l.lat, longitude: l.lon)
            .distance(from: nominalLoc)
        worst = max(worst, d)
    }
    return worst
}
