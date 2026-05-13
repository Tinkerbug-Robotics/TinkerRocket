//
//  LastValidRocketFix.swift
//  TinkerRocketApp
//
//  Latched copy of the most recent usable rocket GPS fix.  Issue #140:
//  the BS keeps BLE-streaming to the phone even when its LoRa link to
//  the rocket goes quiet or carries a packet with no fresh GPS — those
//  packets land in iOS as a TelemetryData with lat/lon = nil, which
//  blanks the map marker.  We keep a separate latched fix that only
//  advances on a usable solution, so the marker stays parked on the
//  last reported position until a better one arrives.
//

import Foundation
import CoreLocation

struct LastValidRocketFix: Equatable {
    let rocketID: UInt8
    let latitude: Double
    let longitude: Double
    let numSats: Int
    let fixDate: Date

    var coordinate: CLLocationCoordinate2D {
        CLLocationCoordinate2D(latitude: latitude, longitude: longitude)
    }

    /// A 3D GNSS solution needs four satellites; below this the position
    /// can wander by hundreds of metres and isn't worth latching onto as
    /// the rocket's "last known" location.  Tunable — lower if recovery
    /// flights routinely produce sparser fixes.
    static let minSatsForValidFix = 4

    /// Build a fix from a freshly parsed telemetry packet if its GPS
    /// fields look usable.  Returns nil for packets missing GPS, sitting
    /// at the (0, 0) origin, below the sat-count floor, or with no
    /// known rocketID — those must not advance the cache.
    static func fromTelemetry(_ telemetry: TelemetryData,
                              rocketID: UInt8,
                              now: Date = Date()) -> LastValidRocketFix? {
        guard rocketID > 0,
              let lat = telemetry.latitude,
              let lon = telemetry.longitude,
              !(lat == 0 && lon == 0),
              telemetry.num_sats >= minSatsForValidFix else {
            return nil
        }
        return LastValidRocketFix(
            rocketID: rocketID,
            latitude: lat,
            longitude: lon,
            numSats: telemetry.num_sats,
            fixDate: now
        )
    }
}
