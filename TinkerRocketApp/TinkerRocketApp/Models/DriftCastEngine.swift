//
//  DriftCastEngine.swift
//  TinkerRocketApp
//
//  Reverse drift cast guidance point calculator.
//  Ports the Python driftcast tool to native Swift for offline field use.
//
//  Algorithm:
//    1. Fetch wind profile from Open-Meteo pressure-level API
//    2. Reverse descent sim: from landing target, walk upwind to find apogee guidance point
//    3. Feasibility check: is steering angle within limits?
//    4. Forward verification: simulate descent from guidance point to confirm landing accuracy
//

import Foundation
import CoreLocation

// MARK: - Data Structures

struct WindLayer {
    let altFt: Double       // altitude AGL (feet)
    let speedKts: Double    // wind speed (knots)
    let directionDeg: Double // wind FROM direction (degrees, 0=N)
}

struct WindProfile {
    let layers: [WindLayer]
    let groundElevFt: Double // ground elevation ASL (feet)
    let fetchTime: String
    let location: (lat: Double, lon: Double)

    /// Interpolate wind speed (kts) and direction (deg) at a given AGL altitude.
    func interpolate(altAglFt: Double) -> (speedKts: Double, directionDeg: Double) {
        guard !layers.isEmpty else { return (0, 0) }

        // Clamp to available range
        if altAglFt <= layers.first!.altFt {
            return (layers.first!.speedKts, layers.first!.directionDeg)
        }
        if altAglFt >= layers.last!.altFt {
            return (layers.last!.speedKts, layers.last!.directionDeg)
        }

        // Find bracketing layers
        for i in 0..<(layers.count - 1) {
            let lo = layers[i]
            let hi = layers[i + 1]
            if lo.altFt <= altAglFt && altAglFt <= hi.altFt {
                let t: Double
                if hi.altFt == lo.altFt {
                    t = 0
                } else {
                    t = (altAglFt - lo.altFt) / (hi.altFt - lo.altFt)
                }
                let speed = lo.speedKts + t * (hi.speedKts - lo.speedKts)
                let direction = interpAngle(lo.directionDeg, hi.directionDeg, t: t)
                return (speed, direction)
            }
        }

        // Fallback
        return (layers.last!.speedKts, layers.last!.directionDeg)
    }
}

struct TrackPoint {
    let lat: Double
    let lon: Double
    let altAglFt: Double
    let timeS: Double
}

struct GuidanceResult {
    // Guidance point at apogee
    let guidanceLat: Double
    let guidanceLon: Double

    // Steering requirements
    let steeringAngleDeg: Double   // off-vertical angle from pad to guidance point
    let steeringBearingDeg: Double // compass heading from pad toward guidance point

    // Feasibility
    let feasible: Bool
    let infeasibleReason: String?

    // Descent track (forward verification: guidance → ground)
    let descentTrack: [TrackPoint]

    // Forward verification landing
    let forwardLandingLat: Double
    let forwardLandingLon: Double
    let landingErrorM: Double

    // Wind profile used
    let windProfile: WindProfile

    // Inputs echoed back
    let launchLat: Double
    let launchLon: Double
    let landingLat: Double
    let landingLon: Double
    let apogeeFt: Double

    // Metrics
    let totalDescentTimeS: Double
    let totalDriftM: Double
}

// MARK: - Geo Utilities

/// Earth radius in meters (WGS-84 mean)
private let earthRadiusM: Double = 6_371_000.0

/// Project a point forward along a great-circle arc (Vincenty direct on sphere).
func forwardProject(lat: Double, lon: Double, bearingDeg: Double, distanceM: Double) -> (lat: Double, lon: Double) {
    let rlat = lat * .pi / 180.0
    let rlon = lon * .pi / 180.0
    let rbrng = bearingDeg * .pi / 180.0
    let dOverR = distanceM / earthRadiusM

    let lat2 = asin(
        sin(rlat) * cos(dOverR)
        + cos(rlat) * sin(dOverR) * cos(rbrng)
    )
    let lon2 = rlon + atan2(
        sin(rbrng) * sin(dOverR) * cos(rlat),
        cos(dOverR) - sin(rlat) * sin(lat2)
    )

    return (lat2 * 180.0 / .pi, lon2 * 180.0 / .pi)
}

/// Linearly interpolate between two angles (degrees), handling wraparound.
private func interpAngle(_ a1: Double, _ a2: Double, t: Double) -> Double {
    var diff = ((a2 - a1 + 180).truncatingRemainder(dividingBy: 360)) - 180
    if diff < -180 { diff += 360 }
    return (a1 + t * diff).truncatingRemainder(dividingBy: 360)
        .addingIfNegative(360)
}

private extension Double {
    func addingIfNegative(_ value: Double) -> Double {
        self < 0 ? self + value : self
    }
}

// Unit conversions
func ftToM(_ ft: Double) -> Double { ft * 0.3048 }
func mToFt(_ m: Double) -> Double { m / 0.3048 }
func ktsToMps(_ kts: Double) -> Double { kts * 0.514444 }

// MARK: - Standard Atmosphere

private let stdP0: Double = 101325.0   // Pa (sea-level pressure)
private let stdT0: Double = 288.15     // K  (sea-level temperature)
private let stdL: Double  = 0.0065     // K/m (lapse rate)
private let stdG: Double  = 9.80665    // m/s²
private let stdM: Double  = 0.0289644  // kg/mol (molar mass of air)
private let stdR: Double  = 8.31447    // J/(mol·K)

/// Convert pressure (hPa) to geometric altitude (m ASL), troposphere only.
private func pressureToAltitudeM(_ pHpa: Double) -> Double {
    let pPa = pHpa * 100.0
    let exponent = stdR * stdL / (stdG * stdM)
    return (stdT0 / stdL) * (1.0 - pow(pPa / stdP0, exponent))
}

// MARK: - Wind API (Open-Meteo)

/// Pressure levels to query from Open-Meteo
private let pressureLevelsHPa: [Int] = [
    1000, 975, 950, 925, 900, 850, 800, 700, 600, 500, 400, 300, 250, 200
]

/// Open-Meteo JSON response structure
private struct OpenMeteoResponse: Codable {
    let elevation: Double?
    let hourly: OpenMeteoHourly
}

private struct OpenMeteoHourly: Codable {
    let time: [String]
    // Dynamic keys for wind_speed_XXXhPa and wind_direction_XXXhPa
    // We decode manually using a custom container
}

/// Fetch wind profile from Open-Meteo pressure-level API.
func fetchWinds(lat: Double, lon: Double, timeUTC: Date, groundElevFt: Double? = nil) async throws -> WindProfile {
    let calendar = Calendar(identifier: .gregorian)
    let comps = calendar.dateComponents(in: TimeZone(identifier: "UTC")!, from: timeUTC)
    let dateStr = String(format: "%04d-%02d-%02d", comps.year!, comps.month!, comps.day!)
    let hour = comps.hour ?? 12

    // Build variable list
    let speedVars = pressureLevelsHPa.map { "wind_speed_\($0)hPa" }.joined(separator: ",")
    let dirVars = pressureLevelsHPa.map { "wind_direction_\($0)hPa" }.joined(separator: ",")

    let urlString = "https://api.open-meteo.com/v1/forecast"
        + "?latitude=\(lat)&longitude=\(lon)"
        + "&hourly=\(speedVars),\(dirVars)"
        + "&wind_speed_unit=kn"
        + "&start_date=\(dateStr)&end_date=\(dateStr)"
        + "&timezone=UTC"

    guard let url = URL(string: urlString) else {
        throw DriftCastError.invalidURL
    }

    var request = URLRequest(url: url)
    request.setValue("TinkerRocket-DriftCast/1.0", forHTTPHeaderField: "User-Agent")
    request.timeoutInterval = 15

    let (data, _) = try await URLSession.shared.data(for: request)

    // Parse JSON manually for dynamic keys
    guard let json = try JSONSerialization.jsonObject(with: data) as? [String: Any],
          let hourlyDict = json["hourly"] as? [String: Any],
          let times = hourlyDict["time"] as? [String] else {
        throw DriftCastError.invalidResponse
    }

    let apiElevM = (json["elevation"] as? Double) ?? 0.0
    let groundElevFtFinal = groundElevFt ?? mToFt(apiElevM)
    let groundElevM = groundElevFtFinal * 0.3048

    // Find hourly index closest to requested hour
    let idx = min(hour, times.count - 1)

    // Build layers
    var layers: [WindLayer] = []
    for p in pressureLevelsHPa {
        let speedKey = "wind_speed_\(p)hPa"
        let dirKey = "wind_direction_\(p)hPa"

        guard let speedArr = hourlyDict[speedKey] as? [Any],
              let dirArr = hourlyDict[dirKey] as? [Any],
              idx < speedArr.count, idx < dirArr.count else { continue }

        guard let speedVal = speedArr[idx] as? Double,
              let dirVal = dirArr[idx] as? Double,
              speedVal >= 0, speedVal < 500,
              dirVal >= 0, dirVal <= 360 else { continue }

        let altMslM = pressureToAltitudeM(Double(p))
        let altAglFt = mToFt(altMslM - groundElevM)

        // Skip layers below ground
        if altAglFt < -100 { continue }

        layers.append(WindLayer(
            altFt: max(0.0, altAglFt),
            speedKts: speedVal,
            directionDeg: dirVal
        ))
    }

    // Sort by altitude
    layers.sort { $0.altFt < $1.altFt }

    // Ensure surface layer exists
    if let first = layers.first, first.altFt > 100 {
        layers.insert(WindLayer(
            altFt: 0.0,
            speedKts: first.speedKts * 0.7,
            directionDeg: first.directionDeg
        ), at: 0)
    }

    return WindProfile(
        layers: layers,
        groundElevFt: groundElevFtFinal,
        fetchTime: "\(dateStr)T\(String(format: "%02d", hour)):00Z",
        location: (lat, lon)
    )
}

// MARK: - Descent Simulation

/// Default layer thickness for descent simulation (feet)
private let layerThicknessFt: Double = 1000.0

/// Generate altitude step boundaries from low to high, splitting at deploy altitude.
private func buildAltSteps(lowFt: Double, highFt: Double, stepFt: Double, deployFt: Double) -> [(low: Double, high: Double)] {
    var steps: [(Double, Double)] = []
    var a = lowFt
    while a < highFt {
        let nextA = min(a + stepFt, highFt)
        if a < deployFt && deployFt < nextA {
            // Split at deploy boundary
            steps.append((a, deployFt))
            steps.append((deployFt, nextA))
        } else {
            steps.append((a, nextA))
        }
        a = nextA
    }
    return steps
}

/// Simulate parachute descent through wind layers.
///
/// - Parameters:
///   - direction: "forward" (apogee → ground, drift downwind) or "reverse" (ground → apogee, walk upwind)
func simulateDescent(
    startLat: Double,
    startLon: Double,
    apogeeAglFt: Double,
    drogueRateFps: Double,
    mainRateFps: Double,
    mainDeployAglFt: Double,
    windProfile: WindProfile,
    direction: String = "forward"
) -> [TrackPoint] {
    // Guard against division by zero in descent rate
    guard drogueRateFps > 0.1 && mainRateFps > 0.1 else {
        return [TrackPoint(lat: startLat, lon: startLon, altAglFt: apogeeAglFt, timeS: 0)]
    }

    var track: [TrackPoint] = []
    var timeS: Double = 0
    var lat = startLat
    var lon = startLon

    let steps = buildAltSteps(lowFt: 0, highFt: apogeeAglFt, stepFt: layerThicknessFt, deployFt: mainDeployAglFt)

    if direction == "forward" {
        // Descend from apogee to ground
        var alt = apogeeAglFt
        track.append(TrackPoint(lat: lat, lon: lon, altAglFt: alt, timeS: timeS))

        for (stepLo, stepHi) in steps.reversed() {
            let step = stepHi - stepLo
            let midAlt = (stepLo + stepHi) / 2.0

            // Pick descent rate
            let descentRate = midAlt > mainDeployAglFt ? drogueRateFps : mainRateFps

            // Time through this layer
            let dt = step / descentRate

            // Wind at mid-layer
            let (speedKts, fromDir) = windProfile.interpolate(altAglFt: midAlt)
            let speedMps = ktsToMps(speedKts)
            let driftM = speedMps * dt

            // Forward: drift downwind (FROM + 180)
            let driftBearing = (fromDir + 180.0).truncatingRemainder(dividingBy: 360.0)

            let projected = forwardProject(lat: lat, lon: lon, bearingDeg: driftBearing, distanceM: driftM)
            lat = projected.lat
            lon = projected.lon
            alt = stepLo
            timeS += dt
            track.append(TrackPoint(lat: lat, lon: lon, altAglFt: alt, timeS: timeS))
        }
    } else {
        // Reverse: start at ground (landing), walk upwind to apogee
        var alt: Double = 0
        track.append(TrackPoint(lat: lat, lon: lon, altAglFt: alt, timeS: timeS))

        for (stepLo, stepHi) in steps {
            let step = stepHi - stepLo
            let midAlt = (stepLo + stepHi) / 2.0

            let descentRate = midAlt > mainDeployAglFt ? drogueRateFps : mainRateFps
            let dt = step / descentRate

            let (speedKts, fromDir) = windProfile.interpolate(altAglFt: midAlt)
            let speedMps = ktsToMps(speedKts)
            let driftM = speedMps * dt

            // Reverse: move upwind (INTO the wind = toward the FROM direction)
            let driftBearing = fromDir

            let projected = forwardProject(lat: lat, lon: lon, bearingDeg: driftBearing, distanceM: driftM)
            lat = projected.lat
            lon = projected.lon
            alt = stepHi
            timeS += dt
            track.append(TrackPoint(lat: lat, lon: lon, altAglFt: alt, timeS: timeS))
        }
    }

    return track
}

// MARK: - Main Orchestration

enum DriftCastError: LocalizedError {
    case invalidURL
    case invalidResponse
    case networkError(String)

    var errorDescription: String? {
        switch self {
        case .invalidURL: return "Invalid API URL"
        case .invalidResponse: return "Invalid response from wind API"
        case .networkError(let msg): return "Network error: \(msg)"
        }
    }
}

/// Compute the apogee guidance point for a desired landing location.
func computeGuidancePoint(
    launchLat: Double,
    launchLon: Double,
    landingLat: Double,
    landingLon: Double,
    apogeeAglFt: Double,
    drogueRateFps: Double,
    mainRateFps: Double,
    mainDeployAglFt: Double,
    launchTimeUTC: Date,
    maxSteeringDeg: Double = 25.0
) async throws -> GuidanceResult {

    // Step 0: Fetch winds
    let windProfile = try await fetchWinds(lat: landingLat, lon: landingLon, timeUTC: launchTimeUTC)

    // Step 1: Reverse descent — landing → apogee guidance point
    let reverseTrack = simulateDescent(
        startLat: landingLat,
        startLon: landingLon,
        apogeeAglFt: apogeeAglFt,
        drogueRateFps: drogueRateFps,
        mainRateFps: mainRateFps,
        mainDeployAglFt: mainDeployAglFt,
        windProfile: windProfile,
        direction: "reverse"
    )

    let guidancePt = reverseTrack.last!
    let guidanceLat = guidancePt.lat
    let guidanceLon = guidancePt.lon

    // Step 2: Feasibility check
    let horizDistM = LocationManager.haversineDistance(
        lat1: launchLat, lon1: launchLon,
        lat2: guidanceLat, lon2: guidanceLon
    )
    let apogeeM = ftToM(apogeeAglFt)
    let steeringAngle = atan2(horizDistM, apogeeM) * 180.0 / .pi
    let steeringBrg = LocationManager.bearing(
        lat1: launchLat, lon1: launchLon,
        lat2: guidanceLat, lon2: guidanceLon
    )
    // Normalize bearing to [0, 360)
    let steeringBrgNorm = ((steeringBrg.truncatingRemainder(dividingBy: 360)) + 360)
        .truncatingRemainder(dividingBy: 360)

    let feasible = steeringAngle <= maxSteeringDeg
    var infeasibleReason: String? = nil
    if !feasible {
        infeasibleReason = String(format:
            "Steering angle %.1f° exceeds maximum %.1f°. " +
            "The guidance point is %.0f m from the pad at bearing %.0f°. " +
            "Try a closer landing point, higher apogee, or increase max steering.",
            steeringAngle, maxSteeringDeg, horizDistM, steeringBrgNorm)
    }

    // Step 3: Forward verification — guidance point → ground
    let forwardTrack = simulateDescent(
        startLat: guidanceLat,
        startLon: guidanceLon,
        apogeeAglFt: apogeeAglFt,
        drogueRateFps: drogueRateFps,
        mainRateFps: mainRateFps,
        mainDeployAglFt: mainDeployAglFt,
        windProfile: windProfile,
        direction: "forward"
    )

    let fwdLanding = forwardTrack.last!
    let landingErrorM = LocationManager.haversineDistance(
        lat1: landingLat, lon1: landingLon,
        lat2: fwdLanding.lat, lon2: fwdLanding.lon
    )

    let totalDriftM = LocationManager.haversineDistance(
        lat1: guidanceLat, lon1: guidanceLon,
        lat2: fwdLanding.lat, lon2: fwdLanding.lon
    )

    let totalTime = forwardTrack.last?.timeS ?? 0

    return GuidanceResult(
        guidanceLat: guidanceLat,
        guidanceLon: guidanceLon,
        steeringAngleDeg: steeringAngle,
        steeringBearingDeg: steeringBrgNorm,
        feasible: feasible,
        infeasibleReason: infeasibleReason,
        descentTrack: forwardTrack,
        forwardLandingLat: fwdLanding.lat,
        forwardLandingLon: fwdLanding.lon,
        landingErrorM: landingErrorM,
        windProfile: windProfile,
        launchLat: launchLat,
        launchLon: launchLon,
        landingLat: landingLat,
        landingLon: landingLon,
        apogeeFt: apogeeAglFt,
        totalDescentTimeS: totalTime,
        totalDriftM: totalDriftM
    )
}
