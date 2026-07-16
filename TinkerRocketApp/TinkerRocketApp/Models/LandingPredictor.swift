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
//  `Data_Analysis/landing_predictor.py`.  v1 ships the descent branch
//  (alt_apo flag set, or vu <= 0): drift-cast forward from the current
//  position using the active RocketProfile's drogue/main rates and the
//  cached wind profile.  When a fresh GNSS fix is available, the snapshot
//  position is taken from raw GNSS rather than EKF — bypasses the
//  post-blackout EKF recovery oscillation seen in flight #176 replay.
//  Each prediction carries an uncertainty radius (#191 item 2, see
//  landingUncertainty below); ascent ballistic-with-drag lands in a
//  follow-up branch (#191 item 1).
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
    /// item 2).  The error budget is set by the snapshot altitude — how
    /// much modeled descent remains — so it shrinks as re-predictions run
    /// closer to the ground, and it does NOT grow with staleness once
    /// latched: an old prediction is exactly as wrong as when it was made.
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
        // positive.  v1 only predicts during descent.
        let vU = Double(t.altitude_rate ?? 0)
        let descending = t.alt_apo || vU <= 0.5
        guard descending else { return }

        guard let profile = profileStore?.activeProfile else { return }

        let snapshot = CLLocationCoordinate2D(latitude: lat, longitude: lon)
        // Altitude AGL: pressure_alt is already MSL/baro per the parser;
        // most operators tare to launch, so treat it as AGL for descent
        // prediction.  Wind profile is also AGL, so units match.
        let altAglFt = mToFt(Double(t.pressure_alt ?? 0))

        let track = simulateDescentForLanding(
            startLat: snapshot.latitude,
            startLon: snapshot.longitude,
            currentAltAglFt: altAglFt,
            observedVerticalRateMps: vU,
            profile: profile,
            wind: windProfile
        )

        guard let landing = track.last.map({
            CLLocationCoordinate2D(latitude: $0.lat, longitude: $0.lon)
        }) else { return }

        let newPrediction = LandingPrediction(
            landing: landing, descentTrack: track,
            snapshot: snapshot, snapshotAltAglFt: altAglFt,
            snapshotSource: .gnss,    // we already required num_sats >= 4
            computedAt: now, sampleAt: now,
            uncertaintyMeters: landingUncertainty(track: track, wind: windProfile)
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

/// Uncertainty radius for a descent prediction — fixed at compute time.
///
/// Descent branch only.  With a wind profile: `windUncertaintyFraction` of
/// the total drift the cast applied (mean wind speed sampled at the track's
/// altitudes × descent duration).  Without one: the whole drift is
/// unmodeled, so charge `assumedWindWhenUnknownMps` across the descent
/// instead.  Either way the radius scales with REMAINING descent time, so
/// it shrinks as predictions re-run closer to the ground.  The ascent
/// drag-spread term arrives with #191 item 1.
func landingUncertainty(track: [TrackPoint], wind: WindProfile?) -> Double {
    guard track.count >= 2,
          let first = track.first, let last = track.last else { return 0 }
    let descentS = max(0, last.timeS - first.timeS)
    guard let wind = wind else { return assumedWindWhenUnknownMps * descentS }
    let speeds = track.map { ktsToMps(wind.interpolate(altAglFt: $0.altAglFt).speedKts) }
    let meanMps = speeds.reduce(0, +) / Double(speeds.count)
    return windUncertaintyFraction * meanMps * descentS
}
