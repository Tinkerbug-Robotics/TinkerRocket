import Foundation

/// Virtual Rocket — the iOS twin of Android's demo mode (user request
/// 2026-07-30): a peripheral-less `BLEDevice` driven by a scripted flight
/// through the REAL parse/dispatch/UI stack.  No CoreBluetooth involved, so
/// it runs in the Simulator too.
///
/// Behavior converged with Android's `DemoMode.kt`, frame for frame: the
/// virtual base station relays a "Booster" that idles in READY like a real
/// rocket and flies ONE canned 42 s flight per SIM_START — the standard
/// Simulation sheet is the start control, exactly the ritual a real rocket
/// uses — honoring SIM_STOP mid-flight.  A second relayed rocket sits on the
/// pad so the roster and focus-switching are demonstrable.
///
/// Nothing is recorded, and Files stays real-hardware-only on iOS (Android's
/// demo serves a canned log through its transport fake; iOS has no transport
/// seam — ledger-documented divergence).
@MainActor
final class VirtualRocketDriver {

    let device: BLEDevice
    private var task: Task<Void, Never>?
    private var startRequested = false
    private var stopRequested = false

    /// Test hook: scales every sleep so the whole flight runs in tens of
    /// milliseconds under XCTest (same pattern as OTASession.timeScale).
    var timeScale: Double = 1.0

    init() {
        device = BLEDevice(peripheral: nil, name: "TR-B-Virtual")
        device.commandTap = { [weak self] cmd, payload in
            guard let self else { return }
            // On a base-station link the UI wraps rocket commands in the
            // cmd-50 relay envelope [rid][inner cmd][payload...] (#390) —
            // and the virtual device IS a base station, so unwrap first.
            var effective = Int(cmd)
            if cmd == 50, payload.count >= 2 {
                effective = Int(payload[payload.index(payload.startIndex, offsetBy: 1)])
            }
            if effective == 6 { self.startRequested = true }   // SIM_START
            if effective == 7 { self.stopRequested = true }    // SIM_STOP
        }
    }

    func start() {
        device.onConnect()
        // Identity through the same demux a real readback uses.
        feed("""
            {"type":"config_identity","uid":"virtual1","un":"Virtual Base Station",\
            "nid":7,"dt":"B","fw":"demo+sim"}
            """)
        task = Task { [weak self] in await self?.run() }
    }

    func stop() {
        task?.cancel()
        task = nil
        device.onDisconnect()
    }

    // MARK: - Script (mirror of Android DemoMode.kt)

    private func feed(_ json: String) {
        device.parseTelemetryData(json.data(using: .utf8))
    }

    private func sleep(_ seconds: Double) async {
        try? await Task.sleep(nanoseconds: UInt64(max(0, seconds * timeScale) * 1_000_000_000))
    }

    // OK (01) at baro/imu/ekf/mag/gnss/batt/storage shifts; pyro NA (hidden).
    static let health = 0x100555

    /// Frames per flight (42 s at 5 Hz: 12 s countdown, 10 s up, 16 s down).
    static let flightTicks = 210
    /// Telemetry cadence during the flight, seconds.
    static let flightTickSeconds = 0.2

    /// The focused rocket's frame for `tick` — pure, so the announcer
    /// regression test can replay the exact frames the demo emits
    /// (`testVirtualFlightScript_drivesFullCalloutSequence`).  Android twin:
    /// `VirtualFlightScript.frameJson` — keep them converged frame for frame.
    ///
    /// The script speaks fluent firmware, because every consumer keys off
    /// real conventions (voice-callout outage, 2026-07-30):
    ///  - The countdown is **PRELAUNCH** — the state a real FC arms through,
    ///    and the announcer's reset edge; without it the one-shot callout
    ///    flags stay latched and a SECOND demo flight is mute.
    ///  - Descent is **INFLIGHT** + the apogee flag — firmware has no
    ///    "DESCENT" state, and the announcer gates descent callouts on
    ///    INFLIGHT.
    ///  - **mspd** ramps to a stable max — burnout detection watches
    ///    max-speed stability, and ascent altitude callouts gate on burnout.
    ///  - The rocket **drifts east under canopy** so the landed callout has
    ///    a real horizontal distance (and the bearing arrow moves).
    ///
    /// Callouts per flight: burnout ~15 s, altitude cadence to apogee at
    /// 22 s (400 m), descent at 27 s and 37 s, landed at 38 s.
    static func flightFrameJSON(tick: Int) -> String {
        let t = Double(tick) / 5.0
        let phase: String
        switch t {
        case ..<12: phase = "PRELAUNCH"
        case ..<38: phase = "INFLIGHT"
        default:    phase = "LANDED"
        }
        let alt: Double
        switch t {
        case ..<12: alt = 0
        case ..<22:
            let tt = t - 12
            alt = max(0, 80 * tt - 4 * tt * tt)   // apogee 400 m at t=22
        case ..<38: alt = max(0, 400 - (t - 22) * 25)
        default:    alt = 0
        }
        // Running max the same way the FC reports it.
        let maxAlt = t < 12 ? 0 : (t < 22 ? alt : 400)
        let maxSpeed = t < 12 ? 0 : min(95, (t - 12) * 38)
        var fs = 0x10 | 0x40                      // pwr on + logging
        if t >= 12 { fs |= 0x01 }                 // launch
        if t >= 14.5 { fs |= 0x200 }              // burnout
        if t >= 22 { fs |= 0x02 | 0x04 }          // apogee votes
        if t >= 38 { fs |= 0x08 }                 // landed
        let rate: Double
        switch t {
        case ..<12:   rate = 0
        case ..<14.5: rate = 95
        case ..<22:   rate = 30 - (t - 14.5) * 4
        case ..<38:   rate = -25
        default:      rate = 0
        }
        // ~73 m of easterly drift under canopy → "Landed. 75 meters away".
        let lon = -118.2015 + 0.00005 * (max(22, min(t, 38)) - 22)
        return String(
            format: """
                {"rid":1,"run":"Booster","st":"%@","fs":%d,"ps":%d,"h":%d,\
                "nsat":9,"vol":%.2f,"cur":%.1f,"soc":%.1f,"palt":%.1f,\
                "malt":%.1f,"arate":%.1f,"mspd":%.1f,"lat":%.6f,"lon":%.6f}
                """,
            phase, fs, 0x00A | 0x080, health,
            8.2 - t * 0.004,
            // High-rate logging draw until deployment drops the rate (#623).
            120 + (t >= 12 && t < 22 ? 900.0 : 0.0),
            87 - t * 0.1, alt, maxAlt, rate, maxSpeed,
            34.6572, lon
        )
    }

    private func run() async {
        while !Task.isCancelled {
            // READY idle at 1 Hz until the Simulation sheet starts a flight.
            startRequested = false
            while !startRequested && !Task.isCancelled {
                feed("""
                    {"rid":1,"run":"Booster","st":"READY","fs":16,"ps":\(0x00A | 0x080),\
                    "h":\(Self.health),"nsat":9,"vol":8.20,"soc":87.0,"palt":0.2,\
                    "lat":34.6572,"lon":-118.2015}
                    """)
                feed("""
                    {"rid":2,"run":"Pad Rocket","st":"READY","fs":16,"ps":2,\
                    "h":\(Self.health),"nsat":8,"vol":8.31,"palt":0.4}
                    """)
                await sleep(1.0)
            }
            if Task.isCancelled { return }

            // One flight off the shared script — SIM_STOP aborts to READY.
            stopRequested = false
            for tick in 0..<Self.flightTicks {
                if Task.isCancelled { return }
                if stopRequested { break }
                feed(Self.flightFrameJSON(tick: tick))
                if tick % 5 == 0 {
                    feed("""
                        {"rid":2,"run":"Pad Rocket","st":"READY","fs":16,"ps":2,\
                        "h":\(Self.health),"nsat":8,"vol":8.31,"palt":0.4}
                        """)
                }
                await sleep(Self.flightTickSeconds)
            }
            // Flight over or aborted: loop back to the READY idle, like a
            // real rocket after touchdown + reset.
        }
    }
}
