//
//  LocationManager.swift
//  TinkerRocketApp
//
//  Provides the phone's GPS location and compass heading for
//  computing direction and distance to the rocket.
//

import Foundation
import CoreLocation
import Combine

class LocationManager: NSObject, ObservableObject, CLLocationManagerDelegate {

    @Published var userLocation: CLLocationCoordinate2D?
    @Published var userAltitude: Double?  // Phone GPS altitude in meters (MSL)
    @Published var userAccuracy: Double?  // Horizontal accuracy in meters; nil when invalid
    @Published var heading: Double = 0    // True north compass heading in degrees

    private let manager = CLLocationManager()
    private var isRunning = false

    // Throttle state for reportFix(to:).  Lives here rather than in a global
    // because the fix and its cadence are this object's business, and it keeps
    // the sender free of shared mutable state.
    private var lastFixSentAt: Date?
    private var lastFixSentFrom: CLLocationCoordinate2D?

    /// Minimum wall time between phone-fix pushes while the operator stands
    /// still.  distanceFilter is 5 m, so an unthrottled push would put a BLE
    /// write and a log row on every few steps, competing with telemetry RX.
    private let fixMinInterval: TimeInterval = 30
    /// Movement that forces a push regardless of the interval — actually
    /// repositioning the base station should reach the log promptly.
    private let fixForceDistance: CLLocationDistance = 25

    override init() {
        super.init()
        manager.delegate = self
        manager.desiredAccuracy = kCLLocationAccuracyBest
        manager.distanceFilter = 5       // Update every 5 m
        manager.headingFilter = 2        // Update every 2° (was 1°, reduced to halve heading-driven redraws)
        manager.requestWhenInUseAuthorization()
        // Don't start updates until explicitly requested.
        // Previously started in init(), which caused continuous
        // @Published heading/location updates (and full DashboardView
        // re-renders) even when the data wasn't being used — a major
        // source of steady memory growth.
    }

    /// Start GPS + compass updates.  Call when the data is actually needed
    /// (e.g. connected to a base station that shows direction to rocket).
    func startUpdates() {
        guard !isRunning else { return }
        isRunning = true
        manager.startUpdatingLocation()
        manager.startUpdatingHeading()
    }

    /// Stop GPS + compass updates to conserve battery and prevent
    /// unnecessary @Published changes that trigger SwiftUI view redraws.
    func stopUpdates() {
        guard isRunning else { return }
        isRunning = false
        manager.stopUpdatingLocation()
        manager.stopUpdatingHeading()
    }

    // MARK: - Phone fix -> base station

    /// Push the current fix to `device` as BLE_BS_CMD_SET_PHONE_FIX, throttled.
    ///
    /// The base station is wherever the phone is, and the BS has no GNSS of
    /// its own, so this is the only way the range between the two ends ever
    /// reaches a log file. The BS writes the row only while a logging session
    /// is open, so calling this off-session costs one BLE write and nothing
    /// else. Safe to call on every location update.
    func reportFix(to device: BLEDevice) {
        guard let coord = userLocation, CLLocationCoordinate2DIsValid(coord) else { return }

        let now = Date()
        if let sentAt = lastFixSentAt, let from = lastFixSentFrom {
            let moved = CLLocation(latitude: from.latitude, longitude: from.longitude)
                .distance(from: CLLocation(latitude: coord.latitude, longitude: coord.longitude))
            if now.timeIntervalSince(sentAt) < fixMinInterval && moved < fixForceDistance {
                return
            }
        }
        lastFixSentAt = now
        lastFixSentFrom = coord

        // 47 == BLE_BS_CMD_SET_PHONE_FIX. Literal on purpose — see the note
        // in PhoneFixCodec about the CI parity sweep.
        device.sendRawCommand(47,
                              payload: PhoneFixCodec.encode(coord,
                                                            altitude: userAltitude,
                                                            accuracy: userAccuracy))
    }

    /// Drop the throttle so the next fix is pushed immediately.  Called on
    /// (re)connect: the fix at the START of a log is the one that matters, and
    /// a timestamp left over from the previous session could suppress it.
    func resetFixThrottle() {
        lastFixSentAt = nil
        lastFixSentFrom = nil
    }

    // MARK: - CLLocationManagerDelegate

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        if let loc = locations.last {
            userLocation = loc.coordinate
            if loc.verticalAccuracy >= 0 {
                userAltitude = loc.altitude
            }
            // Negative means the fix is invalid, per CLLocation — publish nil
            // rather than a negative "accuracy" that would log as garbage.
            userAccuracy = loc.horizontalAccuracy >= 0 ? loc.horizontalAccuracy : nil
        }
    }

    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        // Use true heading when available, fall back to magnetic
        let h = newHeading.trueHeading >= 0 ? newHeading.trueHeading : newHeading.magneticHeading
        // Only publish when heading changes by at least the filter amount
        // (CLLocationManager's headingFilter should handle this, but guard
        // here as well to avoid unnecessary objectWillChange triggers).
        if abs(heading - h) >= 1.0 {
            heading = h
        }
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        print("[Location] Error: \(error.localizedDescription)")
    }

    // MARK: - Geo Utilities

    /// Haversine distance in meters between two GPS coordinates
    static func haversineDistance(lat1: Double, lon1: Double,
                                  lat2: Double, lon2: Double) -> Double {
        let R = 6371000.0
        let dLat = (lat2 - lat1) * .pi / 180.0
        let dLon = (lon2 - lon1) * .pi / 180.0
        // Clamp to [0, 1] to prevent NaN from floating-point rounding
        let a = min(1.0, sin(dLat / 2) * sin(dLat / 2) +
                cos(lat1 * .pi / 180.0) * cos(lat2 * .pi / 180.0) *
                sin(dLon / 2) * sin(dLon / 2))
        let c = 2 * atan2(sqrt(a), sqrt(1 - a))
        return R * c
    }

    /// Initial bearing in degrees (0 = north, 90 = east) from point 1 to point 2
    static func bearing(lat1: Double, lon1: Double,
                        lat2: Double, lon2: Double) -> Double {
        let lat1r = lat1 * .pi / 180.0
        let lat2r = lat2 * .pi / 180.0
        let dLon = (lon2 - lon1) * .pi / 180.0

        let y = sin(dLon) * cos(lat2r)
        let x = cos(lat1r) * sin(lat2r) - sin(lat1r) * cos(lat2r) * cos(dLon)
        let rad = atan2(y, x)

        return rad * 180.0 / .pi  // Convert to degrees (may be negative)
    }
}
