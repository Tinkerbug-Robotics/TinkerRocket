//
//  GnssVelocityCheck.swift
//  TinkerRocketApp
//
//  #552: how far the flight computer's velocity has drifted from what its own
//  GNSS fixes say — measured on the phone, from the telemetry frame alone.
//
//  The issue is that the EKF throws large transient horizontal-velocity
//  excursions during coast (47 m/s on RIM-66 with twelve satellites locked),
//  and the ascent landing prediction integrates that velocity to apogee, so a
//  wrong velocity becomes a wrong landing point with a lever arm of
//  seconds-to-apogee.  The phone cannot fix the filter, but it can *notice*,
//  because the frame carries the filter's velocity and the receiver's position
//  and those two have to agree.
//
//  The frame carries no GNSS velocity, so this differences successive GNSS
//  positions to make one.  That works — correlation +0.98 to +1.00 against the
//  true disagreement on three of the four 2026-08-29 BARC L2 flights — with
//  one failure mode, and it is the one this file exists to handle.
//
//  ## Why `hacc` sets the differencing interval
//
//  Differencing two positions each uncertain by sigma over an interval dt
//  gives a velocity uncertain by `sqrt(2) * sigma / dt`, which blows up as dt
//  shrinks.  Rolly Polly V flew on four satellites with 29 m fixes; over the
//  0.5 s frame interval that is 82 m/s of pure noise, and its measured
//  "disagreement" read 19.8 m/s where the truth was 9.9 — a 2x over-read that
//  inflated the uncertainty radius to 266 m around a 115 m error.
//
//  The cure is not to subtract the noise (measured: worth almost nothing,
//  because the over-read is driven by outlier fixes rather than by a Gaussian
//  at the sigma level).  It is to difference over a LONGER interval, because
//  the noise falls as 1/dt while a real disagreement does not.  So `hacc`
//  picks the interval: long enough that the differencing noise sits under
//  `targetNoiseMps`, never longer than `maxBaselineS`, past which the average
//  smooths away the excursion we are trying to see.
//
//  Measured over 195 ascent predictions on those four flights, holding
//  everything else fixed, this is worth a 44% tighter radius on the bad-GNSS
//  flight (266 m -> 149 m) at slightly BETTER coverage (68% -> 71%), and
//  changes nothing at all on the three healthy flights, whose `hacc` is 0-1 m
//  and whose interval therefore stays at the frame rate.
//
//  ## When `hacc` is absent
//
//  Legacy firmware and the mini send no `hacc`.  Here, and only here, absent
//  and 0 may be treated alike: both pick the shortest interval, which yields
//  the LARGEST disagreement and so the largest radius.  Being wrong in that
//  direction is safe, and it is exactly the behaviour that shipped before
//  `hacc` existed.
//
//  That equivalence does not generalise.  Anywhere `hacc` is read as a quality
//  bound rather than as an interval input, absent must stay absent — 0 means
//  "better than half a metre", the most trusting value on the scale, so
//  decoding absence as 0 would rate the least trustworthy fixes the most
//  trustworthy.  `TelemetryData.gnss_h_acc_m` stays optional for that reason.
//

import Foundation

enum GnssVelocityCheck {

    /// One telemetry frame's worth of what this needs.
    struct Sample {
        let t: Date
        let latDeg: Double
        let lonDeg: Double
        /// GNSS horizontal accuracy in metres, or nil when not sent.
        let hAccM: Int?
        /// Filter velocity, ENU, m/s.
        let velE: Double
        let velN: Double
    }

    struct Result {
        /// |filter velocity - GNSS-derived velocity|, m/s.
        let disagreementMps: Double
        /// The interval actually differenced over, seconds.
        let baselineS: Double
        /// Differencing noise floor at that interval, m/s.
        let noiseFloorMps: Double
    }

    /// Differencing noise we tolerate before lengthening the interval.
    static let targetNoiseMps: Double = 3.0

    /// Longest interval we will difference over.  Measured: 6 s costs
    /// coverage (68% -> 59% on the flight it is meant to help) because the
    /// chord velocity starts averaging across the excursion instead of
    /// resolving it.
    static let maxBaselineS: Double = 4.0

    /// Sigma contributed by the wire's own rounding.  `lat`/`lon` ship at 5
    /// decimal places (TR_BLE_To_APP.cpp `addDouble("lat", ..., 5)`), a 1.11 m
    /// step, so a uniform rounding error of 1.11/sqrt(12).  It is what stops a
    /// reported `hacc` of 0 from claiming a noiseless difference.
    static let quantizationSigmaM: Double = 1.11 / 12.0.squareRoot()

    /// Drop samples older than this — comfortably past `maxBaselineS` so the
    /// longest interval still has both ends available.
    static let historyS: Double = 12.0

    /// Metres per degree of latitude; good to ~0.1% over any one flight.
    private static let mPerDegLat: Double = 111_320.0

    /// The disagreement, or nil when it cannot honestly be measured — too few
    /// samples, or every sample inside the minimum interval.
    ///
    /// `history` must be in ascending time order; the last entry is "now".
    static func evaluate(history: [Sample]) -> Result? {
        guard history.count >= 2, let latest = history.last else { return nil }

        // Absent hacc falls back to the wire's own rounding alone, which keeps
        // the interval at the frame rate and the estimate deliberately
        // conservative — see "When `hacc` is absent" above for why that is
        // sound here and nowhere else.
        let sigma = (pow(max(Double(latest.hAccM ?? 0), 0), 2)
                     + pow(quantizationSigmaM, 2)).squareRoot()
        let want = 2.0.squareRoot() * sigma / targetNoiseMps

        // Oldest sample that is at least `want` old, but never reaching past
        // maxBaselineS.  Walking back from the newest finds the SHORTEST
        // interval that satisfies the target, so a healthy flight keeps the
        // frame rate and only a noisy one pays the lag.
        var chosen: Sample?
        for s in history.dropLast().reversed() {
            let ageS = latest.t.timeIntervalSince(s.t)
            if ageS > maxBaselineS { break }
            chosen = s
            if ageS >= want { break }
        }
        guard let old = chosen else { return nil }
        let dt = latest.t.timeIntervalSince(old.t)
        guard dt > 0 else { return nil }

        let cosLat = cos(latest.latDeg * .pi / 180.0)
        let dN = (latest.latDeg - old.latDeg) * mPerDegLat
        let dE = (latest.lonDeg - old.lonDeg) * mPerDegLat * cosLat
        let gnssVelE = dE / dt
        let gnssVelN = dN / dt

        // A chord velocity is an AVERAGE over the interval, so it has to be
        // compared with the filter's average over the same interval — not with
        // the instantaneous value at one end.  Comparing an average to an
        // instant reads the excursion's own slope as disagreement.
        let window = history.filter { $0.t >= old.t && $0.t <= latest.t }
        guard !window.isEmpty else { return nil }
        let ekfVelE = window.reduce(0.0) { $0 + $1.velE } / Double(window.count)
        let ekfVelN = window.reduce(0.0) { $0 + $1.velN } / Double(window.count)

        return Result(
            disagreementMps: (pow(ekfVelE - gnssVelE, 2)
                              + pow(ekfVelN - gnssVelN, 2)).squareRoot(),
            baselineS: dt,
            noiseFloorMps: 2.0.squareRoot() * sigma / dt
        )
    }

    /// Append `s` and drop anything older than `historyS`.
    static func trimmed(history: [Sample], appending s: Sample) -> [Sample] {
        let cutoff = s.t.addingTimeInterval(-historyS)
        // Guard against a clock that jumped backwards: a sample stamped later
        // than the newest would otherwise sit in the list forever and be
        // differenced against, producing a nonsense interval.
        return history.filter { $0.t >= cutoff && $0.t <= s.t } + [s]
    }
}
